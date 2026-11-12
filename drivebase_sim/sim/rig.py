"""R3's tick ordering, shared by R4 controllers, training and evaluation."""

import mujoco
import numpy as np

from experiments.r3 import Case, load_scene
from sim.can_dm import CanBus, encode_velocity
from sim.drive import Drive, WHEELS
from sim.motor import DatasheetMotor, ProtectedMotor
from sim.power import DatasheetBattery


class DriveRig:
    """Hold targets for 10 ms: two electrical ticks per 2 ms physics tick.

    No settled-state cache is used. Models may be reused only after restoring
    friction and constructing fresh data, electronics and transport state.
    Call wait_for_simulation before the batch which owns this rig.
    """

    def __init__(self, case=None, *, seed=1, command_drop=0., feedback_drop=0.,
                 model=None, record=False):
        self.case = case or Case(settle_limit=True)
        self.model = load_scene(self.case) if model is None else model
        self.model.geom_friction[:, 0] = self.case.mu
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.drive = Drive.from_scene(self.model, self.data)
        self.drive.slip_multiplier = 1.0
        self.vi = [self.model.joint('drive_' + w).dofadr[0] for w in WHEELS]
        self.qi = [self.model.joint('drive_' + w).qposadr[0] for w in WHEELS]
        self.chassis = self.model.body('chassis').id
        self.gyro_adr = self.model.sensor('gyro').adr[0]
        self.motor_model = DatasheetMotor(kt=self.case.kt,
            current_limit=9.8 if self.case.behaviour == 'limit' else 13,
            power_bound=self.case.power)
        self.motors = [ProtectedMotor(behaviour=self.case.behaviour,
            kt=self.case.kt, timeout=self.case.driver_timeout) for _ in WHEELS]
        if self.case.settle_limit:
            for motor in self.motors:
                motor.behaviour = 'limit'
        self.battery = DatasheetBattery(open_circuit=self.case.supply)
        self.bus = CanBus(seed=seed, drop_rate=command_drop,
                          feedback_drop_rate=feedback_drop)
        self.voltage, self.bus_current = self.case.supply, 0.
        self.torque = np.zeros(6)
        self.step_index = 0
        self.faults = {}
        self.record = False
        self.physics_rows, self.electrical_rows = [], []
        for _ in range(750):
            self._physics(np.zeros(6))
        self.start = self.data.qpos[:3].copy()
        if self.case.settle_limit:
            for motor in self.motors:
                motor.enable(1.5)
                motor.behaviour = self.case.behaviour
        self.record = record
        self.body_ids = [i for i in range(self.model.nbody)
                         if np.any((self.model.geom_bodyid == i) &
                                   (self.model.geom_group != 3))]
        self.body_names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
                           or 'world' for i in self.body_ids]

    @property
    def t(self):
        return self.step_index * .002 - 1.5

    @property
    def gyro(self):
        return self.data.sensordata[self.gyro_adr:self.gyro_adr + 3].copy()

    @property
    def rotation(self):
        return self.data.xmat[self.chassis].reshape(3, 3)

    @property
    def body_velocity(self):
        # mj_objectVelocity returns angular then linear velocity in local axes.
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                self.chassis, velocity, 1)
        return velocity[3:]

    @property
    def failed(self):
        return any(m.error != 1 for m in self.motors) or self.voltage < 15

    def _physics(self, command):
        elapsed = self.t
        tick_torques, electrical = [], []
        for substep in range(2):
            tick = self.step_index * 2 + substep
            now = tick * .001
            if tick % 10 == 0:
                for mid, target in enumerate(command, 1):
                    self.bus.send(encode_velocity(mid, float(target)), now)
            states = {mid: (float((self.data.qpos[self.qi[mid-1]] + np.pi) %
                                  (2*np.pi) - np.pi),
                            float(self.data.qvel[self.vi[mid-1]]),
                            float(self.torque[mid-1]), self.motors[mid-1].error)
                      for mid in range(1, 7)}
            self.bus.advance(now, states)
            speeds = self.data.qvel[self.vi].copy()
            for mid, motor in enumerate(self.motors, 1):
                stamp = self.bus.last_command[mid]
                if stamp is not None and stamp > motor.last_command:
                    motor.command(self.bus.targets[mid], stamp)
            demands = [m.check(w, self.voltage, now)
                       for m, w in zip(self.motors, speeds)]
            self.voltage, self.bus_current, self.torque = self.battery.solve(
                demands, speeds, self.motor_model)
            if self.voltage < 15:
                for motor in self.motors:
                    if motor.error == 1:
                        motor.error = 9
                self.torque[:] = 0
            for i, (motor, speed, torque) in enumerate(zip(self.motors, speeds, self.torque)):
                motor.accept(speed, torque)
                if motor.error != 1 and WHEELS[i] not in self.faults:
                    self.faults[WHEELS[i]] = dict(time_s=now-1.5, error=motor.error)
            tick_torques.append(self.torque.copy())
            electrical.append([now-1.5, self.voltage, self.bus_current,
                               *self.torque, *[m.error for m in self.motors]])
        if self.record:
            q = self.data.qpos[3:7]
            heading = np.arctan2(2*(q[0]*q[3]+q[1]*q[2]),
                                 1-2*(q[2]**2+q[3]**2))
            self.physics_rows.append([elapsed, heading, self.gyro[2],
                *self.data.qpos[:3], *self.body_velocity, self.rotation[2, 2]])
            self.electrical_rows.extend(electrical)
        self.data.ctrl[:] = np.mean(tick_torques, axis=0)
        mujoco.mj_step(self.model, self.data)
        self.step_index += 1
        return electrical

    def advance(self, targets):
        """Advance one host period, returning the ten electrical samples."""
        return np.asarray([row for _ in range(5) for row in self._physics(targets)])

    def replay_row(self):
        # mj_step leaves derived transforms at its input time. Kinematics on
        # separate data gives exact start-of-window poses without touching the
        # R3 sensor/integration ordering of the live rig.
        replay = mujoco.MjData(self.model)
        replay.qpos[:] = self.data.qpos
        mujoco.mj_kinematics(self.model, replay)
        return dict(t=self.t, pos=replay.xpos[self.body_ids].copy(),
            quat=replay.xquat[self.body_ids].copy(),
            yaw_rate=float(np.degrees(self.gyro[2])), current=self.torque/self.case.kt,
            error=np.array([m.error for m in self.motors]), bus_v=self.voltage)
