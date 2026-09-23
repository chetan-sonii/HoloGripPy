from __future__ import annotations

import sys
import time
from pathlib import Path

from object_manager import ObjectManager
from sensor.orientation import OrientationEstimator
from sensor.udp_receiver import UDPReceiver


def print_menu(manager: ObjectManager) -> None:
    print("=" * 60)
    print("                 MOTION3D OBJECT VIEWER")
    print("              ACCURATE TILT CONTROL MODE")
    print("=" * 60)

    print("\nController setup:")
    print("  +X axis -> RIGHT")
    print("  +Y axis -> FRONT / fingers")
    print("  +Z axis -> UP")

    current_category = None
    for index, obj in enumerate(manager.all_objects(), start=1):
        category = obj.get("category", "Other")
        if category != current_category:
            current_category = category
            print(f"\n[{category}]")
        print(f"  {index:2d}. {obj.get('name', obj.get('id', 'Unnamed'))}")

    print("\n  q. Quit")
    print("=" * 60)


def calibrate_sensor(
    receiver: UDPReceiver,
    estimator: OrientationEstimator,
    duration: float = 2.5,
) -> None:
    print("\n==========================================================")
    print("               FLAT-POSITION CALIBRATION")
    print("==========================================================")
    print("Place the MPU6050 FLAT on a level surface.")
    print("Keep it completely still.")
    print("Make sure:")
    print("  +X points RIGHT")
    print("  +Y points FRONT / toward fingers")
    print("  +Z points UP")
    print("The flat position becomes the exact ZERO position.")
    print()

    receiver.send_hello()

    samples = []
    start = time.perf_counter()
    next_hello = start + 0.5
    last_tenth = -1

    while time.perf_counter() - start < duration:
        now = time.perf_counter()

        if now >= next_hello:
            receiver.send_hello()
            next_hello = now + 0.5

        packet = receiver.receive()
        if packet is not None:
            samples.append(packet)

        elapsed = now - start
        percent = int(min(100.0, elapsed / duration * 100.0))
        tenth = percent // 10

        if tenth != last_tenth:
            last_tenth = tenth
            print(
                f"Calibration: {percent:3d}%   "
                f"samples: {len(samples)}"
            )

    if len(samples) < 20:
        raise RuntimeError(
            "Not enough sensor data received during calibration. "
            "Make sure the PC is connected to ESP32-MPU6050 Wi-Fi "
            "and the ESP32 is running the UDP sensor code."
        )

    estimator.calibrate(samples)

    print("Calibration complete.")
    print(
        f"Gyro bias: GX={estimator.gyro_bias[0]:.1f}, "
        f"GY={estimator.gyro_bias[1]:.1f}, "
        f"GZ={estimator.gyro_bias[2]:.1f}"
    )
    print(
        "Control mode: gravity-anchored roll/pitch | "
        "flat = zero | yaw locked"
    )


def main() -> int:
    project_root = Path(__file__).resolve().parent
    manager = ObjectManager(project_root)

    print_menu(manager)

    while True:
        choice = input("Select object: ").strip().lower()

        if choice == "q":
            return 0

        try:
            index = int(choice) - 1
            obj = manager.get_object(index)
            model_path = manager.asset_path(index)
        except (ValueError, IndexError, FileNotFoundError) as exc:
            print(f"Invalid selection: {exc}")
            continue

        print(f"\nLoading: {obj['name']}")
        print(f"Model:  {model_path}")

        receiver = None

        try:
            receiver = UDPReceiver()
            estimator = OrientationEstimator()
            calibrate_sensor(receiver, estimator)

            from renderer import Motion3DRenderer

            app = Motion3DRenderer(
                model_path,
                obj["name"],
                receiver,
                estimator,
            )
            app.run()
            return 0

        except Exception as exc:
            print("\nCould not start Motion3D.")
            print(f"Reason: {exc}")
            print("\nCheck that:")
            print("  1. The PC is connected to ESP32-MPU6050 Wi-Fi.")
            print("  2. The ESP32 is powered and running the UDP sensor code.")
            print("  3. The selected .glb model exists.")
            print("  4. Panda3D and panda3d-gltf are installed.")
            return 1

        finally:
            if receiver is not None:
                receiver.close()


if __name__ == "__main__":
    sys.exit(main())
