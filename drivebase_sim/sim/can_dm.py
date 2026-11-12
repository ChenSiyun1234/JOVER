"""DM V1.0 CAN codecs (manual pp.5-7) and deterministic delayed transport.

Standard 11-bit IDs, 1 Mbps. Simulation uses IDs 1..6 (low nibble in
feedback), master ID 0. Frames each take an assumed fixed 1 ms; arbitration,
bit stuffing and CRC are not simulated. Transport holds the last target and
records accepted-command times; ProtectedMotor optionally enforces a watchdog.
"""

from dataclasses import dataclass
import heapq
import math
import random
import struct

ERRORS = {0: "disabled", 1: "enabled", 8: "overvoltage", 9: "undervoltage",
          10: "overcurrent", 11: "MOS overtemperature", 12: "coil overtemperature",
          13: "communication loss", 14: "overload"}


@dataclass(frozen=True)
class Ranges:
    p_max: float = 12.5
    v_max: float = 30.0
    t_max: float = 12.0

    def __post_init__(self):
        if not all(math.isfinite(x) and x > 0 for x in (self.p_max, self.v_max, self.t_max)):
            raise ValueError("Ranges must be positive and finite")


@dataclass(frozen=True)
class Frame:
    can_id: int
    data: bytes

    def __post_init__(self):
        if not 0 <= self.can_id <= 0x7FF:
            raise ValueError("Standard CAN ID required")
        if len(self.data) > 8:
            raise ValueError("CAN data exceeds 8 bytes")


def to_uint(value, low, high, bits):
    if not math.isfinite(value):
        raise ValueError("Nonfinite command")
    return round((min(high, max(low, value)) - low) * ((1 << bits) - 1) / (high - low))


def to_float(value, low, high, bits):
    return low + value * (high - low) / ((1 << bits) - 1)


def require_length(frame, length):
    if len(frame.data) != length:
        raise ValueError(f"Expected {length} bytes")


def encode_velocity(motor_id, velocity):
    if not math.isfinite(velocity):
        raise ValueError("Nonfinite velocity")
    return Frame(0x200 + motor_id, struct.pack("<f", velocity))


def decode_velocity(frame, motor_id):
    require_length(frame, 4)
    if frame.can_id != 0x200 + motor_id:
        return None
    value, = struct.unpack("<f", frame.data)
    if not math.isfinite(value):
        raise ValueError("Nonfinite velocity")
    return value


def encode_mit(motor_id, position, velocity, kp, kd, torque, ranges=Ranges()):
    fields = [(position, -ranges.p_max, ranges.p_max, 16),
              (velocity, -ranges.v_max, ranges.v_max, 12),
              (kp, 0, 500, 12), (kd, 0, 5, 12),
              (torque, -ranges.t_max, ranges.t_max, 12)]
    packed = 0
    for value, low, high, bits in fields:
        packed = (packed << bits) | to_uint(value, low, high, bits)
    return Frame(motor_id, packed.to_bytes(8, "big"))


def decode_mit(frame, motor_id, ranges=Ranges()):
    require_length(frame, 8)
    if frame.can_id != motor_id:
        return None
    packed = int.from_bytes(frame.data, "big")
    raw = [(packed >> 48) & 65535, (packed >> 36) & 4095,
           (packed >> 24) & 4095, (packed >> 12) & 4095, packed & 4095]
    bounds = [(-ranges.p_max, ranges.p_max, 16), (-ranges.v_max, ranges.v_max, 12),
              (0, 500, 12), (0, 5, 12), (-ranges.t_max, ranges.t_max, 12)]
    return tuple(to_float(v, *b) for v, b in zip(raw, bounds))


def encode_feedback(motor_id, position, velocity, torque, error=1,
                    mos=25, rotor=25, master_id=0, ranges=Ranges()):
    if not 0 <= motor_id <= 15 or not 0 <= error <= 15:
        raise ValueError("Feedback ID and status occupy nibbles")
    p = to_uint(position, -ranges.p_max, ranges.p_max, 16)
    v = to_uint(velocity, -ranges.v_max, ranges.v_max, 12)
    t = to_uint(torque, -ranges.t_max, ranges.t_max, 12)
    return Frame(master_id, bytes([motor_id | (error << 4), p >> 8, p & 255,
                                  v >> 4, ((v & 15) << 4) | (t >> 8), t & 255, mos, rotor]))


def decode_feedback(frame, motor_id=None, master_id=0, ranges=Ranges()):
    require_length(frame, 8)
    b = frame.data
    identity, error = b[0] & 15, b[0] >> 4
    if frame.can_id != master_id or (motor_id is not None and identity != motor_id):
        return None
    return dict(motor_id=identity, error=error, error_name=ERRORS.get(error, "unknown"),
                position=to_float((b[1] << 8) | b[2], -ranges.p_max, ranges.p_max, 16),
                velocity=to_float((b[3] << 4) | (b[4] >> 4), -ranges.v_max, ranges.v_max, 12),
                torque=to_float(((b[4] & 15) << 8) | b[5], -ranges.t_max, ranges.t_max, 12),
                mos=b[6], rotor=b[7])


class CanBus:
    def __init__(self, motor_ids=range(1, 7), drop_rate=0.0, seed=1, feedback_drop_rate=0.0):
        self.ids = tuple(motor_ids)
        if not all(1 <= mid <= 15 for mid in self.ids):
            raise ValueError("Simulated motor IDs must fit the feedback nibble")
        if not 0 <= drop_rate <= 1 or not 0 <= feedback_drop_rate <= 1:
            raise ValueError("Drop rates must be probabilities")
        self.drop_rate, self.feedback_drop_rate = drop_rate, feedback_drop_rate
        self.rng = random.Random(seed)
        self.targets = {mid: 0.0 for mid in self.ids}
        self.last_command = {mid: None for mid in self.ids}
        self.feedback = {}
        self.queue = []
        self.sequence = self.sent = self.dropped = self.received = 0

    def send(self, frame, now, feedback=False):
        self.sent += 1
        if self.rng.random() < (self.feedback_drop_rate if feedback else self.drop_rate):
            self.dropped += 1
            return
        self.sequence += 1
        heapq.heappush(self.queue, (now + 0.001, self.sequence, feedback, frame))

    def advance(self, now, states):
        """Deliver frames; accepted commands trigger one delayed status reply."""
        while self.queue and self.queue[0][0] <= now + 1e-12:
            _, _, reply, frame = heapq.heappop(self.queue)
            if reply:
                decoded = decode_feedback(frame)
                if decoded["motor_id"] in self.targets:
                    self.feedback[decoded["motor_id"]] = decoded
                    self.received += 1
                continue
            mid = frame.can_id - 0x200
            if mid not in self.targets:
                continue
            self.targets[mid] = decode_velocity(frame, mid)
            self.last_command[mid] = now
            position, speed, torque, error = states[mid]
            self.send(encode_feedback(mid, position, speed, torque, error), now, feedback=True)
