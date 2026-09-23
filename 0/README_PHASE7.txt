Motion3D Phase 7 - Accurate Tilt Control

REPLACE THESE FILES IN YOUR CURRENT PROJECT:
  main.py
  renderer.py
  sensor/orientation.py

Keep your existing:
  object_manager.py
  objects.json
  sensor/udp_receiver.py
  assets/

IMPORTANT PHYSICAL SENSOR ORIENTATION
--------------------------------------
For the mapping to match the screen naturally, mount the MPU6050 so that:
  +X points to the user's RIGHT
  +Y points toward the user's FRONT / fingers
  +Z points UP when the sensor is flat.

CONTROL BEHAVIOR
----------------
  Tilt sensor left       -> object tilts left
  Tilt sensor right      -> object tilts right
  Tilt front downward    -> object pitches downward
  Tilt front upward      -> object pitches upward
  Tilt diagonally        -> object follows the diagonal tilt
  Put sensor flat        -> object returns to default zero position

WHY THIS VERSION IS DIFFERENT
------------------------------
The object is NOT controlled by accumulated gyro angle anymore.

The accelerometer supplies the long-term physical tilt reference, while the
gyro provides fast short-term response. This prevents the large drift problem
of direct gyro integration and makes a flat surface a stable zero reference.

YAW IS LOCKED
-------------
The MPU6050 cannot measure absolute yaw without a magnetometer. This version
therefore intentionally does not rotate the object around the vertical axis.
That is preferable for accurate tilt control to allowing an unobservable yaw
angle to drift.

CALIBRATION
-----------
At startup, place the MPU6050 flat and keep it still for approximately 2.5 s.
That physical pose becomes the exact zero pose.

R key:
  Re-zeroes the CURRENT pose.

ESC:
  Exit.

TUNING
------
renderer.py:
  VISUAL_SMOOTHING = 22.0

Higher -> slightly more smoothing.
Lower -> more immediate response.

renderer.py:
  ROLL_SIGN = +1.0
  PITCH_SIGN = +1.0

These are normally correct with the physical mounting described above. If a
single direction is exactly reversed, change only that corresponding sign.
