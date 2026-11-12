# Findings

What the simulation showed, roughly in the order I worked through it. All of it is
simulation: masses, the sizes the team hasn't published and the ground friction are
assumptions or published proxies (see [viz/ROVER_PARAMS.md](../viz/ROVER_PARAMS.md)),
and nothing has run on the real motors.

## 1. First drive tests

With the first motor model, fitted to the catalogue points:

- Driving straight at 0.5 m/s for 10 s, the bus peaked at about 16.7 A and averaged
  about 4.4 A.
- Driving at 0.3 m/s into a rock under the left wheels, all three left wheels went
  over it; the bus peaked at about 6.9 A over the run.
- Holding zero speed on the ramp for 10 s, it drifted about 8 mm. Only the speed loop
  holds it; there is no mechanical brake in the model.
- Turning in place was the problem: a 30 deg/s command reached only about 3.8 deg/s,
  and the rover drifted about 1.7 m.

## 2. Why turning in place is hard

- A simple Coulomb friction model with these wheel positions still predicts about
  22.6 deg/s for the 30 deg/s command, so most of the loss was elsewhere.
- Changing one factor at a time on the terrain: lowering friction from 0.9 to 0.5
  raised the turn from about 4.5 to 16.8 deg/s; flat ground instead of terrain raised
  it to about 10 deg/s; removing torsional friction changed almost nothing.
- The load moves onto one wheel. During the turn the right-rear wheel carries about
  190 N against 103 N at rest, turns backwards against its target, and spends most of
  the 8 s at its torque limit.
- A virtual-work model of the rocker-bogie, fed only the measured wheel ground forces,
  predicts that wheel's mean load (about 187 N in both), and turning the other way
  flips the diagonal. Locking the suspension still leaves about 60 N of diagonal load
  difference that I haven't explained, so this is only partly confirmed.

## 3. Checking the motor model against the datasheet

- The first model lumped every loss into a 2.25 ohm resistance. The manual's key
  parameter table gives 0.33 ohm phase resistance, 305 uH, 14 pole pairs and a 6:1
  gear, so at low speed the limit is the driver's 9.8 A current setting, not the
  supply voltage.
- My first power estimate said a six-wheel stall pulls about 61 A from a full pack.
  With phase copper loss it is about 11.5 A; keeping the old loss coefficient gives
  about 56.5 A. That is a comparison, not a bound, so the real number needs a bench
  measurement. The bus stayed at least 7.9 V above the 15 V undervoltage cutoff.
- With the datasheet motor, the high-friction terrain turn reached about 2.7 deg/s,
  with the right-rear wheel at its current setting for about 6 s.
- If that current setting trips instead of limiting (flat ground, friction 0.9), the
  right-rear driver faults about 2.3 s into the turn and the turn nearly stops
  (about 0.8 deg/s afterwards). Which of the two the real driver does is not confirmed.

## 4. What helps

- A gyro yaw-rate loop (proportional 2.0, integral 1.0 per second, every 10 ms) on
  terrain at friction 0.45, a proxy picked from published values, with the driver
  limiting current, reached about 27.3 deg/s with a peak phase current of about 7.1 A
  and no trips.
- On high friction a torque governor that backs off the turn command cut the turn to
  about 1.7 deg/s, and an arc turn reached only about 3.3 deg/s.
- So: keep the driver limiting below its trip setting, close the loop on yaw, and
  avoid point turns on high-friction ground.

## 5. When commands stop

On the terrain at friction 0.9, if CAN commands stop mid-turn and the driver's 0.5 s
watchdog removes torque, the 8 s run ends about 3.1 m from where the turn started. If
only the host's commands stop and it keeps sending zero speeds, the run ends about
0.27 m away. The driver watchdog is a stop, not a brake, so on a slope the host-side
zero command (or a brake) has to come first.

## 6. A learned controller

A small PPO policy adds up to 5 rad/s to each wheel's speed target on top of the team's
kinematics. It only sees the gyro, the IMU attitude and the motors' CAN feedback, not
the ground friction. It was compared with five hand-designed controllers on 22 test
cases fixed before training, and checkpoints were picked on separate validation
episodes.

- In the first run the policy got worse the longer it trained, and its action noise
  grew from about 0.29 to 0.74. The picked policy averaged about 18 deg/s on the 12
  low- and medium-friction turn cases, against about 15 for the team default and
  27.7 for the gyro yaw loop.
- With the entropy bonus turned off (other settings kept, one million steps), the
  noise stayed near 0.2 and training no longer declined. That policy matched the yaw
  loop's average on the same 12 cases (27.7 deg/s) and had a driver fault in 3 of the
  22 cases against 5 for the yaw-loop controllers, but it tracked worse (yaw-rate RMSE
  about 7.0 against 4.3 deg/s) and on high friction it mostly just stopped turning.
  A different trade-off, not a better controller.
- The float32 C version matches PyTorch to within about 4e-7 per step, but in closed
  loop the turn rates still differed by up to about 6 deg/s on low-friction terrain.
  For a Cortex-M4 it is about 4.9 kB of code and 27 kB of constants; I estimate 150 to
  600 microseconds per call at 170 MHz, not measured on a board.
- The same code, model and seeds gave flat-ground results that agree between Windows
  and Linux, but terrain results that differ by up to about 23 deg/s, likely because
  the many terrain contacts amplify tiny floating-point differences. So comparisons
  stay on one machine, and terrain cases would need several seeds.

The learned-controller runs can be replayed in a browser; see the README.
