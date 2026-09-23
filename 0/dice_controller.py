import math
import socket
import json
import threading
import time
import sys

import numpy as np
import pygame

from OpenGL.GL import *
from OpenGL.GLU import *

# ============================================================
# ESP32 / UDP SETTINGS
# ============================================================

ESP32_IP = "192.168.4.1"
UDP_PORT = 5005

# ============================================================
# CONTROL SETTINGS
# ============================================================

# Overall rotation sensitivity.
SENSITIVITY = 1.0

# Change any of these to -1.0 if that physical axis feels reversed.
INVERT_X = 1.0
INVERT_Y = 1.0
INVERT_Z = 1.0

# Larger = faster response; smaller = smoother.
SMOOTHING_SPEED = 18.0

# Small movements below this are ignored to reduce stationary jitter.
DEADZONE_DEG = 0.25

WINDOW_WIDTH = 1100
WINDOW_HEIGHT = 750
FPS = 120

# ============================================================
# SHARED SENSOR STATE
# ============================================================

sensor_lock = threading.Lock()

sensor_data = {
    "connected": False,
    "ax": 0.0,
    "ay": 0.0,
    "az": 0.0,
    "gx": 0.0,
    "gy": 0.0,
    "gz": 0.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "temp": 0.0,
}

stop_network = False

# ============================================================
# QUATERNION FUNCTIONS
# Format: [w, x, y, z]
# ============================================================

def quat_normalize(q):
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / n


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=np.float64)


def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def quat_from_euler_deg(roll_deg, pitch_deg, yaw_deg):
    """Build quaternion from roll(X), pitch(Y), yaw(Z), ZYX order."""
    r = math.radians(roll_deg) * 0.5
    p = math.radians(pitch_deg) * 0.5
    y = math.radians(yaw_deg) * 0.5

    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    return quat_normalize(np.array([
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
    ], dtype=np.float64))


def quat_slerp(q0, q1, t):
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)

    dot = float(np.dot(q0, q1))

    # q and -q represent the same orientation. Pick the short path.
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    dot = max(-1.0, min(1.0, dot))

    if dot > 0.9995:
        return quat_normalize(q0 + t * (q1 - q0))

    theta0 = math.acos(dot)
    sin_theta0 = math.sin(theta0)
    theta = theta0 * t
    sin_theta = math.sin(theta)

    s0 = math.cos(theta) - dot * sin_theta / sin_theta0
    s1 = sin_theta / sin_theta0

    return quat_normalize(s0 * q0 + s1 * q1)


def quat_to_axis_angle(q):
    q = quat_normalize(q)
    if q[0] < 0.0:
        q = -q

    w = max(-1.0, min(1.0, float(q[0])))
    angle = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))

    if s < 1e-8:
        return 0.0, 1.0, 0.0, 0.0

    axis = q[1:] / s
    return math.degrees(angle), float(axis[0]), float(axis[1]), float(axis[2])


def shortest_angle_deg(a, b):
    return (a - b + 180.0) % 360.0 - 180.0

# ============================================================
# NETWORK THREAD
# ============================================================

def network_thread():
    global stop_network

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(0.20)

    try:
        sock.bind(("0.0.0.0", UDP_PORT))
        last_hello = 0.0

        while not stop_network:
            now = time.monotonic()

            if now - last_hello > 1.0:
                try:
                    sock.sendto(b"HELLO", (ESP32_IP, UDP_PORT))
                    last_hello = now
                except OSError:
                    pass

            try:
                data, _address = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                packet = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue

            with sensor_lock:
                if packet.get("status") == "ESP32_CONNECTED":
                    sensor_data["connected"] = True
                    continue

                required = (
                    "ax", "ay", "az", "gx", "gy", "gz",
                    "roll", "pitch", "yaw"
                )

                if all(k in packet for k in required):
                    sensor_data["connected"] = True
                    for key in required:
                        sensor_data[key] = float(packet[key])
                    sensor_data["temp"] = float(packet.get("temp", 0.0))
    finally:
        sock.close()

# ============================================================
# DICE GEOMETRY
# ============================================================

# Face number -> RGB
COLORS = {
    1: (0.90, 0.035, 0.055),  # Red
    2: (0.07, 0.23, 0.92),    # Blue
    3: (0.06, 0.70, 0.18),    # Green
    4: (0.96, 0.14, 0.07),    # Orange
    5: (0.98, 0.82, 0.03),    # Yellow
    6: (0.54, 0.10, 0.82),    # Purple
}

# Opposite sides sum to 7.
# +Z=1, -Z=6, +X=2, -X=5, +Y=3, -Y=4
FACE_DEFS = [
    ((0, 0, 1), 1),
    ((0, 0, -1), 6),
    ((1, 0, 0), 2),
    ((-1, 0, 0), 5),
    ((0, 1, 0), 3),
    ((0, -1, 0), 4),
]

PIP_PATTERNS = {
    1: [(0, 0)],
    2: [(-1, -1), (1, 1)],
    3: [(-1, -1), (0, 0), (1, 1)],
    4: [(-1, -1), (-1, 1), (1, -1), (1, 1)],
    5: [(-1, -1), (-1, 1), (0, 0), (1, -1), (1, 1)],
    6: [(-1, -1), (-1, 0), (-1, 1),
        (1, -1), (1, 0), (1, 1)],
}

PIP_SPACING = 0.29
PIP_RADIUS = 0.105
GLU_QUADRIC = None


def configure_material(color, shininess=80.0, specular=(0.95, 0.95, 0.95, 1.0)):
    diffuse = (float(color[0]), float(color[1]), float(color[2]), 1.0)
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse)
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular)
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess)


def add_scaled(a, b, scale):
    return (a[0] + b[0] * scale, a[1] + b[1] * scale, a[2] + b[2] * scale)


def face_axes(normal):
    nx, ny, nz = normal
    if nz == 1:
        return (1, 0, 0), (0, 1, 0)
    if nz == -1:
        return (-1, 0, 0), (0, 1, 0)
    if nx == 1:
        return (0, 0, -1), (0, 1, 0)
    if nx == -1:
        return (0, 0, 1), (0, 1, 0)
    if ny == 1:
        return (1, 0, 0), (0, 0, -1)
    return (1, 0, 0), (0, 0, 1)


def face_point(normal, u, v, u_amount, v_amount, depth):
    p = (normal[0] * depth, normal[1] * depth, normal[2] * depth)
    p = add_scaled(p, u, u_amount)
    p = add_scaled(p, v, v_amount)
    return p


def draw_base_cube(size=1.96):
    h = size / 2.0

    faces = [
        ((0, 0, 1),  [(-h, -h, h), (h, -h, h), (h, h, h), (-h, h, h)]),
        ((0, 0, -1), [(-h, -h, -h), (-h, h, -h), (h, h, -h), (h, -h, -h)]),
        ((1, 0, 0),  [(h, -h, -h), (h, h, -h), (h, h, h), (h, -h, h)]),
        ((-1, 0, 0), [(-h, -h, -h), (-h, -h, h), (-h, h, h), (-h, h, -h)]),
        ((0, 1, 0),  [(-h, h, -h), (-h, h, h), (h, h, h), (h, h, -h)]),
        ((0, -1, 0), [(-h, -h, -h), (h, -h, -h), (h, -h, h), (-h, -h, h)]),
    ]

    configure_material((0.045, 0.05, 0.065), shininess=95.0)

    glBegin(GL_QUADS)
    for normal, vertices in faces:
        glNormal3f(*normal)
        for vertex in vertices:
            glVertex3f(*vertex)
    glEnd()


def draw_face(face_number, normal, size=1.0):
    u, v = face_axes(normal)

    outer = 0.98 * size
    inner = 0.84 * size
    outer_depth = 0.995 * size
    inner_depth = 1.045 * size

    color = COLORS[face_number]
    darker = tuple(max(0.0, c * 0.60) for c in color)

    outer_pts = [
        face_point(normal, u, v, -outer, -outer, outer_depth),
        face_point(normal, u, v,  outer, -outer, outer_depth),
        face_point(normal, u, v,  outer,  outer, outer_depth),
        face_point(normal, u, v, -outer,  outer, outer_depth),
    ]
    inner_pts = [
        face_point(normal, u, v, -inner, -inner, inner_depth),
        face_point(normal, u, v,  inner, -inner, inner_depth),
        face_point(normal, u, v,  inner,  inner, inner_depth),
        face_point(normal, u, v, -inner,  inner, inner_depth),
    ]

    # Beveled border.
    configure_material(darker, shininess=65.0)
    glBegin(GL_QUADS)
    for i in range(4):
        j = (i + 1) % 4
        glNormal3f(*normal)
        glVertex3f(*outer_pts[i])
        glVertex3f(*outer_pts[j])
        glVertex3f(*inner_pts[j])
        glVertex3f(*inner_pts[i])
    glEnd()

    # Colored face.
    configure_material(color, shininess=105.0)
    glBegin(GL_QUADS)
    glNormal3f(*normal)
    for p in inner_pts:
        glVertex3f(*p)
    glEnd()

    # Black glossy pips.
    configure_material(
        (0.005, 0.005, 0.007),
        shininess=120.0,
        specular=(0.40, 0.40, 0.40, 1.0)
    )

    for px, py in PIP_PATTERNS[face_number]:
        center = face_point(
            normal,
            u,
            v,
            px * PIP_SPACING * size,
            py * PIP_SPACING * size,
            1.075 * size
        )
        glPushMatrix()
        glTranslatef(*center)
        gluSphere(GLU_QUADRIC, PIP_RADIUS * size, 20, 12)
        glPopMatrix()


def draw_dice():
    draw_base_cube()
    for normal, face_number in FACE_DEFS:
        draw_face(face_number, normal)


def draw_floor():
    # Dark studio floor. Kept below the dice without touching it.
    configure_material(
        (0.025, 0.03, 0.045),
        shininess=35.0,
        specular=(0.12, 0.12, 0.16, 1.0)
    )
    y = -1.45
    glBegin(GL_QUADS)
    glNormal3f(0, 1, 0)
    glVertex3f(-8, y, -8)
    glVertex3f(8, y, -8)
    glVertex3f(8, y, 8)
    glVertex3f(-8, y, 8)
    glEnd()

# ============================================================
# OPENGL SETUP
# ============================================================

def setup_opengl(width, height):
    pygame.display.set_caption("ESP32 MPU6050 — Wireless 3D Dice Controller")
    pygame.display.set_mode((width, height), pygame.DOUBLEBUF | pygame.OPENGL)

    glViewport(0, 0, width, height)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, width / float(height), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)

    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glEnable(GL_NORMALIZE)
    glShadeModel(GL_SMOOTH)

    # Keep culling off; the custom face winding differs by face.
    glDisable(GL_CULL_FACE)

    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
    glClearColor(0.008, 0.012, 0.022, 1.0)

    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glLightfv(GL_LIGHT0, GL_POSITION, (4.0, 5.0, 7.0, 1.0))
    glLightfv(GL_LIGHT0, GL_DIFFUSE, (1.0, 0.92, 0.85, 1.0))
    glLightfv(GL_LIGHT0, GL_SPECULAR, (1.0, 1.0, 1.0, 1.0))

    glEnable(GL_LIGHT1)
    glLightfv(GL_LIGHT1, GL_POSITION, (-4.5, 2.0, 3.5, 1.0))
    glLightfv(GL_LIGHT1, GL_DIFFUSE, (0.22, 0.30, 0.48, 1.0))
    glLightfv(GL_LIGHT1, GL_SPECULAR, (0.12, 0.14, 0.18, 1.0))

    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, (0.09, 0.09, 0.11, 1.0))

    # Do not use GL_COLOR_MATERIAL because materials are set explicitly.
    glDisable(GL_COLOR_MATERIAL)

    global GLU_QUADRIC
    GLU_QUADRIC = gluNewQuadric()
    gluQuadricNormals(GLU_QUADRIC, GLU_SMOOTH)


def draw_overlay(lines):
    """Draw text over the OpenGL image using glDrawPixels."""
    surface = pygame.display.get_surface()

    # Build one transparent 2D surface containing all lines.
    overlay = pygame.Surface(
        (surface.get_width(), surface.get_height()),
        flags=pygame.SRCALPHA
    )

    big = pygame.font.SysFont("consolas", 22, bold=True)
    small = pygame.font.SysFont("consolas", 16)

    y = 18
    for index, line in enumerate(lines):
        font = big if index == 0 else small
        text_surface = font.render(line, True, (235, 240, 250, 255))
        overlay.blit(text_surface, (20, y))
        y += 28 if index == 0 else 22

    # Convert to RGBA and draw as a screen-space bitmap.
    raw = pygame.image.tostring(overlay, "RGBA", True)
    width, height = overlay.get_size()

    glDisable(GL_LIGHTING)
    glDisable(GL_DEPTH_TEST)

    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    glOrtho(0, width, 0, height, -1, 1)

    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()

    glRasterPos2f(0, 0)
    
    # 1. Enable blending and define the alpha function
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    
    # 2. Draw the text surface
    glDrawPixels(width, height, GL_RGBA, GL_UNSIGNED_BYTE, raw)
    
    # 3. Disable blending so the 3D dice renders normally
    glDisable(GL_BLEND)

    glPopMatrix()
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)

    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)

# ============================================================
# MAIN
# ============================================================

def main():
    global stop_network

    pygame.init()

    try:
        pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLEBUFFERS, 1)
        pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLESAMPLES, 4)
    except Exception:
        pass

    setup_opengl(WINDOW_WIDTH, WINDOW_HEIGHT)

    network = threading.Thread(target=network_thread, daemon=True)
    network.start()

    clock = pygame.time.Clock()

    reference = None
    display_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    target_quat = display_quat.copy()

    running = True

    while running:
        dt = min(clock.tick(FPS) / 1000.0, 0.05)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    with sensor_lock:
                        if sensor_data["connected"]:
                            reference = (
                                sensor_data["roll"],
                                sensor_data["pitch"],
                                sensor_data["yaw"],
                            )
                    target_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
                    print("Dice recentered.")

        with sensor_lock:
            current = sensor_data.copy()

        if current["connected"]:
            current_roll = current["roll"]
            current_pitch = current["pitch"]
            current_yaw = current["yaw"]

            if reference is None:
                reference = (current_roll, current_pitch, current_yaw)

            ref_roll, ref_pitch, ref_yaw = reference

            dr = shortest_angle_deg(current_roll, ref_roll) * SENSITIVITY * INVERT_X
            dp = shortest_angle_deg(current_pitch, ref_pitch) * SENSITIVITY * INVERT_Y
            dy = shortest_angle_deg(current_yaw, ref_yaw) * SENSITIVITY * INVERT_Z

            if abs(dr) < DEADZONE_DEG:
                dr = 0.0
            if abs(dp) < DEADZONE_DEG:
                dp = 0.0
            if abs(dy) < DEADZONE_DEG:
                dy = 0.0

            target_quat = quat_from_euler_deg(dr, dp, dy)

        alpha = 1.0 - math.exp(-SMOOTHING_SPEED * dt)
        display_quat = quat_slerp(display_quat, target_quat, alpha)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # Camera.
        glTranslatef(0.0, 0.05, -6.1)

        # Studio floor.
        glPushMatrix()
        draw_floor()
        glPopMatrix()

        # Dice.
        glPushMatrix()
        angle, ax, ay, az = quat_to_axis_angle(display_quat)
        glRotatef(angle, ax, ay, az)
        draw_dice()
        glPopMatrix()

        status = "CONNECTED" if current["connected"] else "WAITING FOR ESP32"
        lines = [
            f"MPU6050 DICE  |  {status}",
            f"Roll {current['roll']:7.2f}°    Pitch {current['pitch']:7.2f}°    Yaw {current['yaw']:7.2f}°",
            f"Gyro X {current['gx']:7.1f}°/s   Y {current['gy']:7.1f}°/s   Z {current['gz']:7.1f}°/s",
            "R = recenter    ESC = exit",
        ]
        draw_overlay(lines)

        pygame.display.flip()

    stop_network = True
    pygame.quit()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        stop_network = True
        pygame.quit()
        sys.exit(0)