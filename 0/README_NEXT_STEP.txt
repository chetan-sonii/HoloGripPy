MOTION3D — MPU6050 SMOOTH RENDERER (PHASE 3)

WHAT CHANGED
------------
- UDP reads in the renderer are now non-blocking.
- Every queued sensor packet is processed; none are intentionally discarded before orientation integration.
- Rendering is decoupled from the ~100 Hz ESP32 sensor stream.
- Panda3D is paced at up to 120 FPS.
- Orientation is smoothed every render frame using exponential angle interpolation.
- A small performance readout shows renderer FPS and sensor connection state.
- Keyboard arrows no longer control object rotation.
- R resets the current sensor orientation to zero.

RUN
---
pip install -r requirements.txt
python main.py

TEST
----
1. Connect the PC to the ESP32-MPU6050 Wi-Fi.
2. Run python main.py.
3. Select an object.
4. Keep the MPU6050 still during calibration.
5. Move the MPU6050 and watch the model.
6. Press R to make the current physical orientation the new zero.

IMPORTANT
---------
The renderer can run at 120 FPS even though the ESP32 sends about 100 sensor
samples per second. Frames between sensor packets reuse the newest orientation
and smoothly interpolate toward it.

CURRENT SENSOR LIMITATION
-------------------------
Yaw is gyro-integrated and will drift because the MPU6050 has no absolute yaw
reference. Roll and pitch are corrected by the accelerometer through the
complementary filter.
