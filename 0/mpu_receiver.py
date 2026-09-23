import socket
import json
import time

# =====================================================
# ESP32 NETWORK SETTINGS
# =====================================================

ESP32_IP = "192.168.4.1"
UDP_PORT = 5005

# =====================================================
# Create UDP socket
# =====================================================

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Allow address reuse
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Listen on all network interfaces
sock.bind(("0.0.0.0", UDP_PORT))

# Do not wait forever
sock.settimeout(0.5)

print("=" * 70)
print("MPU6050 PYTHON RECEIVER")
print("=" * 70)

print()
print("Sending connection request to ESP32...")

# =====================================================
# Send HELLO to ESP32
# =====================================================

sock.sendto(
    b"HELLO",
    (ESP32_IP, UDP_PORT)
)

connected = False

# Used for controlling terminal output
packet_counter = 0

print()
print("Waiting for sensor data...")
print()

try:

    while True:

        # -------------------------------------------------
        # Re-send HELLO until ESP32 responds
        # -------------------------------------------------

        if not connected:
            sock.sendto(
                b"HELLO",
                (ESP32_IP, UDP_PORT)
            )

        try:

            data, address = sock.recvfrom(2048)

        except socket.timeout:

            continue

        # -------------------------------------------------
        # Convert bytes -> string
        # -------------------------------------------------

        message = data.decode("utf-8")

        # -------------------------------------------------
        # Parse JSON
        # -------------------------------------------------

        try:

            sensor = json.loads(message)

        except json.JSONDecodeError:

            print("Invalid packet:")
            print(message)

            continue

        # -------------------------------------------------
        # Connection message
        # -------------------------------------------------

        if sensor.get("status") == "ESP32_CONNECTED":

            connected = True

            print("=" * 70)
            print("CONNECTED TO ESP32")
            print("ESP32 address:", address)
            print("=" * 70)

            continue

        connected = True

        packet_counter += 1

        # -------------------------------------------------
        # Extract values
        # -------------------------------------------------

        ax = sensor["ax"]
        ay = sensor["ay"]
        az = sensor["az"]

        gx = sensor["gx"]
        gy = sensor["gy"]
        gz = sensor["gz"]

        roll = sensor["roll"]
        pitch = sensor["pitch"]
        yaw = sensor["yaw"]

        roll360 = sensor["roll360"]
        pitch360 = sensor["pitch360"]
        yaw360 = sensor["yaw360"]

        temperature = sensor["temp"]

        # -------------------------------------------------
        # Display in terminal
        # -------------------------------------------------

        print(
            f"ACCEL  "
            f"X:{ax:7.3f}g  "
            f"Y:{ay:7.3f}g  "
            f"Z:{az:7.3f}g   ||   "

            f"GYRO  "
            f"X:{gx:8.2f}°/s  "
            f"Y:{gy:8.2f}°/s  "
            f"Z:{gz:8.2f}°/s   ||   "

            f"ROTATION  "
            f"Roll:{roll:7.2f}°  "
            f"Pitch:{pitch:7.2f}°  "
            f"Yaw:{yaw:7.2f}°"
        )

        # -------------------------------------------------
        # This is where your 3D object code will go
        # -------------------------------------------------

        # Example:
        #
        # object_rotation_x = roll
        # object_rotation_y = pitch
        # object_rotation_z = yaw
        #
        # Then send these values to your
        # OpenGL / Pygame / Panda3D / Ursina / etc. object.


except KeyboardInterrupt:

    print()
    print("Program stopped by user.")

finally:

    sock.close()

    print("Socket closed.")