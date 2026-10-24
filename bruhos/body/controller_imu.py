"""Read Tony's board-streamed IMU using the action driver's existing connection."""
import math
from bruhos.schemas import Imu


class ControllerIMU:
    def __init__(self, board, fall_deg=55.0, upright_axis=0, forward_axis=2, forward_sign=1, upright_sign=1, pitch_zero_deg=0.0, roll_zero_deg=0.0):
        if upright_axis not in (0, 1, 2) or forward_axis not in (0, 1, 2) or upright_axis == forward_axis:
            raise ValueError('IMU axes must be different indices from 0 to 2')
        if forward_sign not in (-1, 1) or upright_sign not in (-1, 1) or not 10 <= fall_deg <= 85:
            raise ValueError('Invalid fall threshold or orientation sign')
        if not all(math.isfinite(v) and abs(v) <= 20 for v in (pitch_zero_deg, roll_zero_deg)):
            raise ValueError("Upright zero offsets must be finite and within 20 degrees")
        self.pitch_zero_deg, self.roll_zero_deg = pitch_zero_deg, roll_zero_deg
        self.board, self.fall_deg = board, fall_deg
        self.upright_axis, self.forward_axis, self.forward_sign = upright_axis, forward_axis, forward_sign
        self.upright_sign = upright_sign
        self.side_axis = ({0, 1, 2} - {upright_axis, forward_axis}).pop()
        board.enable_reception()

    def read(self):
        sample = self.board.get_imu()
        if sample is None:
            return None
        if len(sample) != 6 or not all(math.isfinite(v) for v in sample):
            raise ValueError('Invalid IMU sample')
        acceleration = sample[:3]
        norm = math.sqrt(sum(v*v for v in acceleration))
        if not .2 <= norm <= 30:
            raise ValueError('IMU acceleration is not usable')
        up = acceleration[self.upright_axis] * self.upright_sign
        forward = acceleration[self.forward_axis] * self.forward_sign
        side = acceleration[self.side_axis]
        pitch = math.degrees(math.atan2(forward, up))
        roll = math.degrees(math.atan2(side, math.hypot(up, forward)))
        pitch = (pitch - self.pitch_zero_deg + 180) % 360 - 180
        roll -= self.roll_zero_deg
        fallen = abs(pitch) > self.fall_deg or abs(roll) > self.fall_deg
        # Side falls are not automatically assigned a front/back recovery.
        direction = (pitch > 0) if fallen and abs(pitch) >= abs(roll) else None
        return Imu(round(roll, 2), round(pitch, 2), fallen, direction), list(sample)
