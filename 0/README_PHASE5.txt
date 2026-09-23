Motion3D Phase 5 - Direct Gyro Control
=======================================

Replace your current renderer.py with this version.

This phase changes the motion algorithm to direct incremental gyro control:

    raw gyro -> bias correction -> deadzone -> light velocity smoothing
              -> dt-based integration -> Panda3D HPR

Important axis mapping:
    GX -> Panda3D Pitch (X rotation)
    GY -> Panda3D Roll  (Y rotation)
    GZ -> Panda3D Heading (Z rotation)

The renderer runs independently at up to 120 FPS while the ESP32 sensor
stream remains around 100 Hz.

Model framing is also reduced:
    target visual size = 1.6 world units
    camera distance = 18 units

Keyboard:
    R      zero the current displayed orientation
    ESC    exit

Do not change main.py or the ESP32 code for this phase.
