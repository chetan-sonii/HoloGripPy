from __future__ import annotations

import socket
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class SensorPacket:
    """Raw MPU6050 packet plus local monotonic receive timestamp."""

    ax: int
    ay: int
    az: int
    gx: int
    gy: int
    gz: int
    received_at: float


class UDPReceiver:
    """Receives raw AX,AY,AZ,GX,GY,GZ packets from the ESP32 over UDP."""

    def __init__(
        self,
        esp32_ip: str = "192.168.4.1",
        port: int = 5005,
        timeout: float = 0.05,
    ) -> None:
        self.esp32_ip = esp32_ip
        self.port = port
        self.timeout = timeout

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(timeout)

        self.connected = False

    def send_hello(self) -> None:
        self.sock.sendto(
            b"HELLO",
            (self.esp32_ip, self.port),
        )

    @staticmethod
    def _parse(data: bytes) -> SensorPacket | None:
        message = data.decode("utf-8", errors="ignore").strip()

        if message == "ESP32_CONNECTED":
            return None

        parts = message.split(",")
        if len(parts) != 6:
            return None

        try:
            values = [int(value.strip()) for value in parts]
        except ValueError:
            return None

        return SensorPacket(
            values[0],
            values[1],
            values[2],
            values[3],
            values[4],
            values[5],
            time.monotonic(),
        )

    def receive(self) -> SensorPacket | None:
        """Receive one packet, waiting up to the configured timeout."""
        self.sock.settimeout(self.timeout)

        try:
            data, _ = self.sock.recvfrom(1024)
        except socket.timeout:
            return None

        packet = self._parse(data)
        if packet is not None:
            self.connected = True

        return packet

    def receive_nonblocking(self) -> SensorPacket | None:
        """Receive one packet without blocking the Panda3D render loop."""
        self.sock.setblocking(False)

        try:
            data, _ = self.sock.recvfrom(1024)
        except BlockingIOError:
            return None

        packet = self._parse(data)
        if packet is not None:
            self.connected = True

        return packet

    def drain_all(self, max_packets: int = 32) -> list[SensorPacket]:
        """Drain queued packets without blocking."""
        packets: list[SensorPacket] = []

        for _ in range(max_packets):
            packet = self.receive_nonblocking()
            if packet is None:
                break
            packets.append(packet)

        return packets

    def close(self) -> None:
        self.sock.close()
