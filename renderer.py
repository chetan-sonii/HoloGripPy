from __future__ import annotations

from pathlib import Path

from direct.gui.DirectGui import DirectLabel
from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    AmbientLight,
    ClockObject,
    DirectionalLight,
    Filename,
    TextNode,
)

from sensor.orientation import OrientationEstimator
from sensor.udp_receiver import SensorPacket, UDPReceiver


class Motion3DRenderer(ShowBase):
    """Low-latency renderer for the gravity-anchored tilt controller."""

    RENDER_FPS = 60

    # Object framing
    MODEL_TARGET_SIZE = 3.5
    CAMERA_DISTANCE = 18.0
    CAMERA_Z = 2.0
    CAMERA_LOOK_Z = 0.0

    # Physical controller -> model direction.
    # +1 is the natural mapping for the required mounting orientation:
    #   MPU +X = right
    #   MPU +Y = front
    #   MPU +Z = up
    ROLL_SIGN = +1.0
    PITCH_SIGN = +1.0

    # Small visual interpolation only. The orientation estimator provides the
    # accuracy; this layer removes the 100 Hz -> 120 FPS visual stepping.
    VISUAL_SMOOTHING = 22.0

    def __init__(
        self,
        model_path: str | Path,
        model_name: str,
        receiver: UDPReceiver,
        orientation: OrientationEstimator,
    ) -> None:
        super().__init__()

        self.model_path = str(Path(model_path).resolve())
        self.model_name = model_name
        self.receiver = receiver
        self.orientation = orientation

        self.model = None

        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.render_roll = 0.0
        self.render_pitch = 0.0

        self.last_packet: SensorPacket | None = None
        self.packet_count = 0
        self.sensor_rate = 0.0
        self.last_packet_time = 0.0
        self.last_hello_time = 0.0
        self._rate_last_time = 0.0
        self._rate_last_count = 0

        self.clock = ClockObject.getGlobalClock()
        self.clock.setMode(ClockObject.MLimited)
        self.clock.setFrameRate(self.RENDER_FPS)

        self.disableMouse()
        self.setBackgroundColor(0.035, 0.04, 0.05, 1.0)

        self._setup_camera()
        self._setup_lights()
        self._load_model()
        self._setup_ui()
        self._setup_controls()

        self.taskMgr.add(
            self._update_sensor,
            "update_sensor",
            sort=-20,
        )
        self.taskMgr.add(
            self._update_render,
            "update_render",
            sort=20,
        )

    # =====================================================
    # CAMERA
    # =====================================================

    def _setup_camera(self) -> None:
        self.camera.setPos(
            0.0,
            -self.CAMERA_DISTANCE,
            self.CAMERA_Z,
        )
        self.camera.lookAt(
            0.0,
            0.0,
            self.CAMERA_LOOK_Z,
        )

    # =====================================================
    # LIGHTS
    # =====================================================

    def _setup_lights(self) -> None:
        ambient = AmbientLight("ambient")
        ambient.setColor((0.50, 0.50, 0.50, 1.0))
        ambient_np = self.render.attachNewNode(ambient)
        self.render.setLight(ambient_np)

        key = DirectionalLight("key")
        key.setColor((0.85, 0.85, 0.85, 1.0))
        key_np = self.render.attachNewNode(key)
        key_np.setHpr(-35, -35, 0)
        self.render.setLight(key_np)

    # =====================================================
    # MODEL
    # =====================================================

    def _load_model(self) -> None:
        model_file = Path(self.model_path)

        if not model_file.exists():
            raise FileNotFoundError(
                f"3D model not found:\n{model_file}"
            )

        print(f"Loading model from: {model_file}")

        filename = Filename.fromOsSpecific(str(model_file))
        self.model = self.loader.loadModel(filename)

        if self.model.isEmpty():
            raise RuntimeError(
                f"Panda3D could not load model:\n{model_file}"
            )

        self.model.reparentTo(self.render)
        self._fit_model_to_view()
        self.model.setHpr(0.0, 0.0, 0.0)

    def _fit_model_to_view(self) -> None:
        min_point, max_point = self.model.getTightBounds()

        if min_point is None or max_point is None:
            return

        size = max_point - min_point
        largest_dimension = max(
            abs(size.x),
            abs(size.y),
            abs(size.z),
        )

        if largest_dimension <= 0.0:
            return

        center = (min_point + max_point) * 0.5
        self.model.setPos(-center)
        self.model.setScale(
            self.MODEL_TARGET_SIZE / largest_dimension
        )

    # =====================================================
    # UI
    # =====================================================

    def _setup_ui(self) -> None:
        self.title = DirectLabel(
            text=self.model_name,
            scale=0.055,
            text_align=TextNode.ACenter,
            pos=(0.0, 0.0, 0.90),
            frameColor=(0, 0, 0, 0),
            text_fg=(0.92, 0.92, 0.92, 1),
        )

        self.orientation_text = DirectLabel(
            text="Left/Right: 0.0°   Front/Back: 0.0°",
            scale=0.037,
            text_align=TextNode.ACenter,
            pos=(0.0, 0.0, 0.80),
            frameColor=(0, 0, 0, 0),
            text_fg=(0.75, 0.90, 0.95, 1),
        )

        self.sensor_text = DirectLabel(
            text="Waiting for ESP32...",
            scale=0.027,
            text_align=TextNode.ACenter,
            pos=(0.0, 0.0, -0.82),
            frameColor=(0, 0, 0, 0),
            text_fg=(0.72, 0.72, 0.72, 1),
        )

        self.status_text = DirectLabel(
            text="Tilt mode | Flat = ZERO",
            scale=0.026,
            text_align=TextNode.ACenter,
            pos=(0.0, 0.0, -0.88),
            frameColor=(0, 0, 0, 0),
            text_fg=(0.60, 0.70, 0.72, 1),
        )

        self.performance_text = DirectLabel(
            text="Renderer: 0 FPS | Sensor: waiting",
            scale=0.024,
            text_align=TextNode.ACenter,
            pos=(0.0, 0.0, -0.94),
            frameColor=(0, 0, 0, 0),
            text_fg=(0.55, 0.65, 0.68, 1),
        )

    # =====================================================
    # CONTROLS
    # =====================================================

    def _setup_controls(self) -> None:
        self.accept("r", self._reset_orientation)
        self.accept("escape", self.userExit)

    def _reset_orientation(self) -> None:
        self.orientation.reset()
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.render_roll = 0.0
        self.render_pitch = 0.0

        if self.model is not None:
            self.model.setHpr(0.0, 0.0, 0.0)

    # =====================================================
    # SENSOR UPDATE
    # =====================================================

    def _update_sensor(self, task):
        packets = self.receiver.drain_all(max_packets=32)

        for packet in packets:
            self.last_packet = packet
            self.packet_count += 1
            self.last_packet_time = packet.received_at

            state = self.orientation.update(packet)

            self.target_roll = state.roll * self.ROLL_SIGN
            self.target_pitch = state.pitch * self.PITCH_SIGN

        if task.time - self.last_hello_time >= 1.0:
            self.receiver.send_hello()
            self.last_hello_time = task.time

        if task.time - self._rate_last_time >= 1.0:
            elapsed = task.time - self._rate_last_time
            count_delta = self.packet_count - self._rate_last_count

            if self._rate_last_time > 0.0:
                self.sensor_rate = count_delta / elapsed

            self._rate_last_time = task.time
            self._rate_last_count = self.packet_count

        return task.cont

    # =====================================================
    # RENDER UPDATE
    # =====================================================

    def _update_render(self, task):
        dt = max(0.0, min(self.clock.getDt(), 0.05))

        factor = min(
            1.0,
            1.0 - pow(2.718281828, -self.VISUAL_SMOOTHING * dt),
        )

        self.render_roll += (
            self.target_roll - self.render_roll
        ) * factor

        self.render_pitch += (
            self.target_pitch - self.render_pitch
        ) * factor

        if self.model is not None:
            # Panda3D HPR:
            #   Heading = Z
            #   Pitch   = X
            #   Roll    = Y
            #
            # We deliberately keep heading at zero. The control objective is
            # tilt, not drift-prone absolute yaw.
            self.model.setHpr(
                0.0,
                self.render_pitch,
                self.render_roll,
            )

        self.orientation_text["text"] = (
            f"Left/Right: {self.render_roll:7.1f}°   "
            f"Front/Back: {self.render_pitch:7.1f}°"
        )

        if self.last_packet is None:
            self.sensor_text["text"] = "Waiting for ESP32..."
            sensor_status = "waiting"
        else:
            packet = self.last_packet
            self.sensor_text["text"] = (
                f"AX {packet.ax:6d}  "
                f"AY {packet.ay:6d}  "
                f"AZ {packet.az:6d}   "
                f"GX {packet.gx:6d}  "
                f"GY {packet.gy:6d}  "
                f"GZ {packet.gz:6d}"
            )
            sensor_status = "connected"

        adaptive = "YES" if self.orientation.stationary else "NO"
        self.status_text["text"] = (
            f"Stationary: {adaptive}  |  "
            f"Flat = ZERO  |  R = RE-ZERO  |  ESC = EXIT"
        )

        fps = self.clock.getAverageFrameRate()
        self.performance_text["text"] = (
            f"Renderer: {fps:5.1f} FPS  |  "
            f"Sensor: {sensor_status}  |  "
            f"Sensor rate: {self.sensor_rate:5.1f} Hz"
        )

        return task.cont
