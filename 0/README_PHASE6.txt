Motion3D Phase 6 — Competition Orientation
============================================

This version replaces direct gyro integration with a quaternion-based Mahony
IMU filter using the MPU6050 accelerometer + gyroscope.

Key changes
-----------
1. Startup gyro bias calibration.
2. Quaternion orientation instead of Euler-angle integration.
3. Accelerometer gravity correction for roll/pitch.
4. Slow gyro-bias adaptation while genuinely stationary.
5. Quaternion-based ZERO reference (press R after startup to re-zero).
6. Sensor timestamps are captured at UDP receive time so queued packets do
   not get integrated using the renderer's frame time.
7. Renderer remains independent and targets 120 FPS.
8. No post-angle smoothing by default, minimizing latency.
9. Explicit sensor-to-Panda3D axis/sign mapping is in renderer.py.

Axis/sign calibration
---------------------
Default mapping follows the old OpenGL project's physical relationship:

  sensor yaw   -> Panda3D Heading
  sensor roll  -> Panda3D Pitch
  sensor pitch -> Panda3D Roll

If an axis moves in the opposite direction, change its sign in renderer.py:

  HEADING_SIGN = +1.0 / -1.0
  PITCH_SIGN   = +1.0 / -1.0
  ROLL_SIGN    = +1.0 / -1.0

Important limitation
--------------------
MPU6050 has no magnetometer. Roll and pitch can be gravity-corrected, but yaw
has no absolute heading reference and can drift over time. A magnetometer can
be added later if competition rules require stable absolute yaw.

Run
---
Connect the PC to ESP32-MPU6050, then:

  python main.py

Keep the sensor still in its intended neutral position during calibration.
Press R later to make the current sensor orientation the new model ZERO.
