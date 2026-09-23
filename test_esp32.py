import socket
import time

# =====================================================
# Configuration
# =====================================================

ESP32_IP = "192.168.4.1"
UDP_PORT = 5005

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Allow socket to reuse the port
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Listen on all network interfaces
sock.bind(("0.0.0.0", UDP_PORT))

# Timeout so we can periodically send HELLO
sock.settimeout(2)

print("======================================")
print("ESP32 MPU6050 DATA TEST")
print("======================================")
print()

# =====================================================
# Send HELLO to ESP32
# =====================================================

print(f"Connecting to ESP32 at {ESP32_IP}:{UDP_PORT}...")

sock.sendto(
    b"HELLO",
    (ESP32_IP, UDP_PORT)
)

print("HELLO sent.")
print("Waiting for sensor data...")
print()

# =====================================================
# Receive data
# =====================================================

packet_count = 0
last_time = time.time()

try:

    while True:

        try:
            data, address = sock.recvfrom(1024)

        except socket.timeout:

            # Send HELLO again if ESP32 didn't respond
            sock.sendto(
                b"HELLO",
                (ESP32_IP, UDP_PORT)
            )

            continue

        # Convert bytes to text
        message = data.decode("utf-8").strip()

        # -------------------------------------------------
        # ESP32 connection confirmation
        # -------------------------------------------------

        if message == "ESP32_CONNECTED":

            print("ESP32 connection confirmed.")
            print()

            continue

        # -------------------------------------------------
        # Parse sensor data
        #
        # Expected:
        # ax,ay,az,gx,gy,gz
        # -------------------------------------------------

        values = message.split(",")

        if len(values) != 6:

            print("Invalid packet:")
            print(message)

            continue

        try:

            ax = int(values[0])
            ay = int(values[1])
            az = int(values[2])

            gx = int(values[3])
            gy = int(values[4])
            gz = int(values[5])

        except ValueError:

            print("Invalid sensor values:")
            print(message)

            continue

        # -------------------------------------------------
        # Packet received successfully
        # -------------------------------------------------

        packet_count += 1

        # Convert to real units for verification

        ax_g = ax / 16384.0
        ay_g = ay / 16384.0
        az_g = az / 16384.0

        gx_dps = gx / 16.4
        gy_dps = gy / 16.4
        gz_dps = gz / 16.4

        # -------------------------------------------------
        # Display
        # -------------------------------------------------

        print(
            f"ACC  "
            f"X: {ax:6d} "
            f"Y: {ay:6d} "
            f"Z: {az:6d}   "
            f"|  "
            f"{ax_g:+.2f}g "
            f"{ay_g:+.2f}g "
            f"{az_g:+.2f}g"
        )

        print(
            f"GYRO "
            f"X: {gx:6d} "
            f"Y: {gy:6d} "
            f"Z: {gz:6d}   "
            f"|  "
            f"{gx_dps:+.2f}°/s "
            f"{gy_dps:+.2f}°/s "
            f"{gz_dps:+.2f}°/s"
        )

        print(
            f"Packets received: {packet_count}"
        )

        print("--------------------------------------")

except KeyboardInterrupt:

    print()
    print("Test stopped.")

finally:

    sock.close()