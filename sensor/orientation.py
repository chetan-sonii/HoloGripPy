from __future__ import annotations

import math
from dataclasses import dataclass

from .udp_receiver import SensorPacket


@dataclass(frozen=True)
class OrientationState:
    """Competition tilt state in degrees relative to the flat calibration pose."""

    roll: float
    pitch: float
    yaw: float
    stationary: bool


class OrientationEstimator:
    """Low-latency, gravity-anchored tilt controller for an MPU6050.

    Physical controller convention used by this project:
      +X = controller right
      +Y = controller/front (towards fingers/front of the hand)
      +Z = up when the controller is flat

    Model convention:
      +X = right
      +Y = front
      +Z = up

    The accelerometer is used as the absolute reference for roll/pitch, so:
      * tilt left  -> model tilts left
      * tilt right -> model tilts right
      * front down -> model pitches down
      * front up   -> model pitches up
      * controller flat -> model returns to zero pose

    The gyro is used for fast short-term response and the accelerometer pulls
    the estimate back to the physically correct tilt. Yaw is intentionally
    locked to zero because an MPU6050 has no absolute heading reference.
    """

    ACCEL_SENSITIVITY = 16384.0  # MPU6050 configured to +/-2 g
    GYRO_SENSITIVITY = 16.4      # MPU6050 configured to +/-2000 deg/s

    # Complementary filter. Higher = more gyro responsiveness. The
    # accelerometer remains the long-term truth for tilt.
    GYRO_WEIGHT = 0.94

    # Small accelerometer low-pass filter to remove high-frequency noise
    # without adding noticeable hand-control latency.
    ACCEL_LPF_ALPHA = 0.38

    # Stationary detection / adaptive gyro bias.
    STATIONARY_GYRO_DPS = 1.8
    STATIONARY_ACCEL_TOLERANCE_G = 0.08
    BIAS_ADAPT_DELAY = 0.30
    BIAS_ADAPT_TIME_SECONDS = 20.0

    # When the controller is genuinely flat, snap the pose to exact zero.
    # This prevents tiny accel/gyro noise from making a stationary flat model
    # wander around its default position.
    LEVEL_SNAP_DEG = 1.4

    MAX_TILT_DEG = 89.0

    def __init__(self) -> None:
        self.gyro_bias = [0.0, 0.0, 0.0]  # raw LSB

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.reference_roll = 0.0
        self.reference_pitch = 0.0

        self.last_timestamp: float | None = None
        self.stationary = False
        self._stationary_time = 0.0

        self._filtered_accel: list[float] | None = None

    # =====================================================
    # Calibration
    # =====================================================

    def calibrate(self, samples: list[SensorPacket]) -> None:
        if not samples:
            raise ValueError("No sensor samples available for calibration.")

        # Average gyro while completely still.
        n = len(samples)
        self.gyro_bias = [
            sum(p.gx for p in samples) / n,
            sum(p.gy for p in samples) / n,
            sum(p.gz for p in samples) / n,
        ]

        # Average accelerometer so the exact physical calibration pose becomes
        # the zero pose. The intended calibration pose is a flat surface.
        ax = sum(p.ax for p in samples) / n
        ay = sum(p.ay for p in samples) / n
        az = sum(p.az for p in samples) / n

        self.reference_roll, self.reference_pitch = self._accel_angles_raw(
            ax, ay, az
        )

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        ref_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if ref_norm > 1e-9:
            self._filtered_accel = [
                ax / ref_norm,
                ay / ref_norm,
                az / ref_norm,
            ]
        else:
            self._filtered_accel = [0.0, 0.0, 1.0]

        self.last_timestamp = samples[-1].received_at
        self.stationary = True
        self._stationary_time = 0.0

    # =====================================================
    # Public API
    # =====================================================

    def reset(self) -> None:
        """Re-zero the current pose without changing gyro calibration."""
        self.reference_roll = self._current_accel_roll()
        self.reference_pitch = self._current_accel_pitch()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def update(self, packet: SensorPacket) -> OrientationState:
        if self.last_timestamp is None:
            self.last_timestamp = packet.received_at
            return self.state()

        dt = packet.received_at - self.last_timestamp
        self.last_timestamp = packet.received_at
        dt = max(0.0005, min(dt, 0.05))

        ax = packet.ax / self.ACCEL_SENSITIVITY
        ay = packet.ay / self.ACCEL_SENSITIVITY
        az = packet.az / self.ACCEL_SENSITIVITY

        gx = (packet.gx - self.gyro_bias[0]) / self.GYRO_SENSITIVITY
        gy = (packet.gy - self.gyro_bias[1]) / self.GYRO_SENSITIVITY
        gz = (packet.gz - self.gyro_bias[2]) / self.GYRO_SENSITIVITY

        # -------------------------------------------------
        # Accelerometer smoothing
        # -------------------------------------------------
        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)

        if accel_norm > 1e-9:
            ax_n = ax / accel_norm
            ay_n = ay / accel_norm
            az_n = az / accel_norm
        else:
            ax_n, ay_n, az_n = 0.0, 0.0, 1.0

        if self._filtered_accel is None:
            self._filtered_accel = [ax_n, ay_n, az_n]
        else:
            a = self.ACCEL_LPF_ALPHA
            self._filtered_accel[0] += a * (
                ax_n - self._filtered_accel[0]
            )
            self._filtered_accel[1] += a * (
                ay_n - self._filtered_accel[1]
            )
            self._filtered_accel[2] += a * (
                az_n - self._filtered_accel[2]
            )

            f_norm = math.sqrt(sum(v * v for v in self._filtered_accel))
            if f_norm > 1e-9:
                self._filtered_accel = [v / f_norm for v in self._filtered_accel]

        fax, fay, faz = self._filtered_accel

        # -------------------------------------------------
        # Correct physical axis mapping
        #
        # Sensor +X = right
        # Sensor +Y = front
        # Sensor +Z = up
        #
        # A positive rotation around sensor Y makes the top move right.
        # Therefore roll is derived from X/Z gravity components.
        # A positive rotation around sensor X makes the front move down.
        # Therefore pitch is derived from -Y/Z gravity components.
        # -------------------------------------------------
        accel_roll = math.degrees(math.atan2(fax, faz)) - self.reference_roll

        accel_pitch = math.degrees(
            math.atan2(
                -fay,
                math.sqrt(fax * fax + faz * faz),
            )
        ) - self.reference_pitch

        accel_roll = self._clamp_tilt(accel_roll)
        accel_pitch = self._clamp_tilt(accel_pitch)

        # -------------------------------------------------
        # Stationary detection
        # -------------------------------------------------
        gyro_norm = math.sqrt(gx * gx + gy * gy + gz * gz)

        self.stationary = (
            abs(accel_norm - 1.0) <= self.STATIONARY_ACCEL_TOLERANCE_G
            and gyro_norm <= self.STATIONARY_GYRO_DPS
        )

        if self.stationary:
            self._stationary_time += dt
        else:
            self._stationary_time = 0.0

        if self.stationary and self._stationary_time >= self.BIAS_ADAPT_DELAY:
            self._adapt_bias(packet, dt)

            # Recalculate the corrected gyro after the online bias update.
            gx = (packet.gx - self.gyro_bias[0]) / self.GYRO_SENSITIVITY
            gy = (packet.gy - self.gyro_bias[1]) / self.GYRO_SENSITIVITY

        # -------------------------------------------------
        # Gyro prediction
        #
        # Physical mounting convention:
        #   gy -> roll around Y
        #   gx -> pitch around X
        # -------------------------------------------------
        gyro_roll = self.roll + gy * dt
        gyro_pitch = self.pitch + gx * dt

        # -------------------------------------------------
        # Accelerometer confidence
        #
        # When linear acceleration is present, |a| is not 1 g and gravity
        # cannot be trusted as strongly. Use the gyro more heavily then.
        # -------------------------------------------------
        deviation = abs(accel_norm - 1.0)

        if deviation <= 0.04:
            accel_confidence = 1.0
        elif deviation >= 0.18:
            accel_confidence = 0.0
        else:
            accel_confidence = 1.0 - (deviation - 0.04) / 0.14

        gyro_weight = 1.0 - (1.0 - self.GYRO_WEIGHT) * accel_confidence
        accel_weight = 1.0 - gyro_weight

        self.roll = gyro_weight * gyro_roll + accel_weight * accel_roll
        self.pitch = gyro_weight * gyro_pitch + accel_weight * accel_pitch

        self.roll = self._clamp_tilt(self.roll)
        self.pitch = self._clamp_tilt(self.pitch)

        # -------------------------------------------------
        # Flat-surface zero behavior
        # -------------------------------------------------
        if (
            self.stationary
            and abs(accel_roll) <= self.LEVEL_SNAP_DEG
            and abs(accel_pitch) <= self.LEVEL_SNAP_DEG
        ):
            self.roll = 0.0
            self.pitch = 0.0

        # Yaw is intentionally not used for the tilt-controlled object.
        self.yaw = 0.0

        return self.state()

    def state(self) -> OrientationState:
        return OrientationState(
            roll=self.roll,
            pitch=self.pitch,
            yaw=0.0,
            stationary=self.stationary,
        )

    def relative_orientation(self) -> tuple[float, float, float]:
        return self.roll, self.pitch, 0.0

    # =====================================================
    # Bias adaptation
    # =====================================================

    def _adapt_bias(self, packet: SensorPacket, dt: float) -> None:
        alpha = 1.0 - math.exp(-dt / self.BIAS_ADAPT_TIME_SECONDS)

        raw = (packet.gx, packet.gy, packet.gz)
        for i in range(3):
            self.gyro_bias[i] += (
                raw[i] - self.gyro_bias[i]
            ) * alpha

    # =====================================================
    # Sensor angles
    # =====================================================

    @classmethod
    def _accel_angles_raw(
        cls,
        ax_raw: float,
        ay_raw: float,
        az_raw: float,
    ) -> tuple[float, float]:
        ax = ax_raw / cls.ACCEL_SENSITIVITY
        ay = ay_raw / cls.ACCEL_SENSITIVITY
        az = az_raw / cls.ACCEL_SENSITIVITY

        roll = math.degrees(math.atan2(ax, az))
        pitch = math.degrees(
            math.atan2(
                -ay,
                math.sqrt(ax * ax + az * az),
            )
        )

        return roll, pitch

    def _current_accel_roll(self) -> float:
        if self._filtered_accel is None:
            return self.reference_roll
        ax, _, az = self._filtered_accel
        return math.degrees(math.atan2(ax, az)) - self.reference_roll

    def _current_accel_pitch(self) -> float:
        if self._filtered_accel is None:
            return self.reference_pitch
        ax, ay, az = self._filtered_accel
        return (
            math.degrees(
                math.atan2(
                    -ay,
                    math.sqrt(ax * ax + az * az),
                )
            )
            - self.reference_pitch
        )

    @classmethod
    def _clamp_tilt(cls, angle: float) -> float:
        return max(-cls.MAX_TILT_DEG, min(cls.MAX_TILT_DEG, angle))
