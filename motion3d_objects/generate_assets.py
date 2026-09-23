from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / "assets"

# RGBA colors used only for the generated educational models.
WHITE = [220, 220, 225, 255]
GRAY = [120, 125, 135, 255]
RED = [220, 70, 70, 255]
BLUE = [80, 120, 230, 255]
GREEN = [80, 190, 110, 255]
BLACK = [35, 35, 40, 255]
YELLOW = [235, 195, 70, 255]
ORANGE = [235, 135, 60, 255]
HYDROGEN = [245, 245, 245, 255]
OXYGEN = [235, 70, 70, 255]
CARBON = [70, 75, 85, 255]
NITROGEN = [70, 100, 220, 255]
BOND = [160, 160, 165, 255]


def set_color(mesh: trimesh.Trimesh, color: list[int]) -> trimesh.Trimesh:
    mesh.visual.face_colors = np.tile(np.asarray(color, dtype=np.uint8), (len(mesh.faces), 1))
    return mesh


def add_mesh(scene: trimesh.Scene, name: str, mesh: trimesh.Trimesh, position=None, rotation=None) -> None:
    transform = np.eye(4)
    if rotation is not None:
        transform = rotation @ transform
    if position is not None:
        transform[:3, 3] = np.asarray(position, dtype=float)
    scene.add_geometry(mesh, node_name=name, transform=transform)


def cylinder_between(
    start: np.ndarray,
    end: np.ndarray,
    radius: float,
    color: list[int],
    sections: int = 16,
) -> trimesh.Trimesh:
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    vector = end - start
    length = float(np.linalg.norm(vector))
    if length < 1e-8:
        raise ValueError("Cylinder endpoints are too close together")

    direction = vector / length
    rotate = trimesh.geometry.align_vectors(np.array([0.0, 0.0, 1.0]), direction)
    mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    mesh.apply_transform(rotate)
    mesh.apply_translation((start + end) / 2.0)
    return set_color(mesh, color)


def export_scene(scene: trimesh.Scene, relative_path: str) -> None:
    output = ASSETS / relative_path
    output.parent.mkdir(parents=True, exist_ok=True)
    scene.export(output, file_type="glb")
    print(f"Created {output.relative_to(ROOT)}")


def make_geometry() -> None:
    items = [
        ("cube.glb", trimesh.creation.box(extents=[2, 2, 2]), WHITE),
        ("sphere.glb", trimesh.creation.icosphere(subdivisions=3, radius=1), WHITE),
        ("cylinder.glb", trimesh.creation.cylinder(radius=1, height=2, sections=48), WHITE),
        ("cone.glb", trimesh.creation.cone(radius=1, height=2, sections=48), WHITE),
        ("pyramid.glb", trimesh.creation.cone(radius=1.2, height=2, sections=4), WHITE),
    ]
    for filename, mesh, color in items:
        scene = trimesh.Scene()
        add_mesh(scene, Path(filename).stem, set_color(mesh, color))
        export_scene(scene, f"geometry/{filename}")


def make_simple_car() -> None:
    scene = trimesh.Scene()

    # Low-poly educational car.  Unit scale is arbitrary; renderer can rescale later.
    body = trimesh.creation.box(extents=[4.6, 2.0, 0.75])
    add_mesh(scene, "body", set_color(body, RED), position=[0, 0, 0.55])

    cabin = trimesh.creation.box(extents=[2.1, 1.7, 1.0])
    add_mesh(scene, "cabin", set_color(cabin, BLUE), position=[0.25, 0, 1.25])

    front = trimesh.creation.box(extents=[0.7, 1.85, 0.45])
    add_mesh(scene, "front", set_color(front, BLACK), position=[2.25, 0, 0.55])

    # Wheels are cylinders whose default axis is Z; rotate to the car's Y axis.
    wheel_rotation = trimesh.geometry.align_vectors(
        np.array([0.0, 0.0, 1.0]), np.array([0.0, 1.0, 0.0])
    )
    wheel_positions = [
        [1.45, -1.0, 0.15],
        [1.45, 1.0, 0.15],
        [-1.45, -1.0, 0.15],
        [-1.45, 1.0, 0.15],
    ]
    for i, pos in enumerate(wheel_positions, start=1):
        wheel = trimesh.creation.cylinder(radius=0.48, height=0.35, sections=24)
        add_mesh(scene, f"wheel_{i}", set_color(wheel, BLACK), position=pos, rotation=wheel_rotation)

    # Simple lights.
    light = trimesh.creation.box(extents=[0.15, 0.45, 0.2])
    add_mesh(scene, "left_light", set_color(light, YELLOW), position=[2.55, -0.55, 0.7])
    add_mesh(scene, "right_light", set_color(light.copy(), YELLOW), position=[2.55, 0.55, 0.7])

    export_scene(scene, "vehicles/simple_car.glb")


def make_atom(
    filename: str,
    name: str,
    protons: int,
    neutrons: int,
    electron_shells: list[int],
) -> None:
    """Create a stylized Bohr-style educational atom visualization."""
    scene = trimesh.Scene()

    # Nucleons are arranged inside a small central cluster.
    nucleons = []
    for i in range(protons):
        nucleons.append((f"p_{i+1}", RED, [0, 0, 0]))
    for i in range(neutrons):
        nucleons.append((f"n_{i+1}", BLUE, [0, 0, 0]))

    total_nucleons = len(nucleons)
    for i, (part_name, color, _) in enumerate(nucleons):
        # Deterministic spiral placement around the nucleus center.
        if total_nucleons == 1:
            pos = np.zeros(3)
        else:
            angle = i * 2.399963229728653
            radius = 0.28 * np.sqrt(i)
            pos = np.array([radius * np.cos(angle), radius * np.sin(angle), 0.12 * ((i % 3) - 1)])
        sphere = trimesh.creation.icosphere(subdivisions=2, radius=0.18)
        add_mesh(scene, part_name, set_color(sphere, color), position=pos)

    # Electron shells are torus meshes; electrons are small spheres on the shells.
    electron_index = 0
    for shell_index, count in enumerate(electron_shells, start=1):
        shell_radius = 0.8 + shell_index * 0.45
        torus = trimesh.creation.torus(major_radius=shell_radius, minor_radius=0.015)
        add_mesh(scene, f"shell_{shell_index}", set_color(torus, GRAY))

        for j in range(count):
            angle = (2.0 * np.pi * j) / max(count, 1)
            pos = np.array([shell_radius * np.cos(angle), shell_radius * np.sin(angle), 0.0])
            electron = trimesh.creation.icosphere(subdivisions=2, radius=0.08)
            add_mesh(
                scene,
                f"electron_{electron_index+1}",
                set_color(electron, GREEN),
                position=pos,
            )
            electron_index += 1

    export_scene(scene, f"atoms/{filename}")
    return


def make_molecule(
    filename: str,
    parts: list[tuple[str, str, np.ndarray, float]],
    bonds: list[tuple[int, int]],
) -> None:
    scene = trimesh.Scene()

    for index, (symbol, color_name, position, radius) in enumerate(parts):
        color = {
            "H": HYDROGEN,
            "O": OXYGEN,
            "C": CARBON,
            "N": NITROGEN,
        }[symbol]
        atom = trimesh.creation.icosphere(subdivisions=3, radius=radius)
        add_mesh(scene, f"{symbol}_{index}", set_color(atom, color), position=position)

    for index, (a, b) in enumerate(bonds):
        start = parts[a][2]
        end = parts[b][2]
        bond = cylinder_between(start, end, radius=0.07, color=BOND)
        add_mesh(scene, f"bond_{index}", bond)

    export_scene(scene, f"chemistry/{filename}")


def make_molecules() -> None:
    # Water: bent geometry.
    water_parts = [
        ("O", "O", np.array([0.0, 0.0, 0.0]), 0.36),
        ("H", "H", np.array([0.76, 0.0, 0.59]), 0.22),
        ("H", "H", np.array([-0.76, 0.0, 0.59]), 0.22),
    ]
    make_molecule("water.glb", water_parts, [(0, 1), (0, 2)])

    # Methane: tetrahedral carbon + four hydrogens.
    tetra = np.array([
        [1, 1, 1],
        [1, -1, -1],
        [-1, 1, -1],
        [-1, -1, 1],
    ], dtype=float)
    tetra /= np.linalg.norm(tetra[0])
    methane_parts = [("C", "C", np.array([0.0, 0.0, 0.0]), 0.40)]
    for pos in tetra * 0.95:
        methane_parts.append(("H", "H", pos, 0.21))
    make_molecule("methane.glb", methane_parts, [(0, 1), (0, 2), (0, 3), (0, 4)])

    # Carbon dioxide: linear O=C=O educational ball-and-stick model.
    co2_parts = [
        ("O", "O", np.array([-1.15, 0.0, 0.0]), 0.36),
        ("C", "C", np.array([0.0, 0.0, 0.0]), 0.42),
        ("O", "O", np.array([1.15, 0.0, 0.0]), 0.36),
    ]
    make_molecule("carbon_dioxide.glb", co2_parts, [(0, 1), (1, 2)])


def write_manifest() -> None:
    manifest = {
        "objects": [
            {"id": "cube", "name": "Cube", "category": "Geometry", "type": "model", "file": "assets/geometry/cube.glb"},
            {"id": "sphere", "name": "Sphere", "category": "Geometry", "type": "model", "file": "assets/geometry/sphere.glb"},
            {"id": "cylinder", "name": "Cylinder", "category": "Geometry", "type": "model", "file": "assets/geometry/cylinder.glb"},
            {"id": "cone", "name": "Cone", "category": "Geometry", "type": "model", "file": "assets/geometry/cone.glb"},
            {"id": "pyramid", "name": "Pyramid", "category": "Geometry", "type": "model", "file": "assets/geometry/pyramid.glb"},
            {"id": "simple_car", "name": "Simple Car", "category": "Vehicles", "type": "model", "file": "assets/vehicles/simple_car.glb"},
            {"id": "hydrogen_atom", "name": "Hydrogen Atom", "category": "Atoms", "type": "model", "file": "assets/atoms/hydrogen_atom.glb"},
            {"id": "carbon_atom", "name": "Carbon Atom", "category": "Atoms", "type": "model", "file": "assets/atoms/carbon_atom.glb"},
            {"id": "water", "name": "Water (H2O)", "category": "Chemistry", "type": "model", "file": "assets/chemistry/water.glb"},
            {"id": "methane", "name": "Methane (CH4)", "category": "Chemistry", "type": "model", "file": "assets/chemistry/methane.glb"},
            {"id": "carbon_dioxide", "name": "Carbon Dioxide (CO2)", "category": "Chemistry", "type": "model", "file": "assets/chemistry/carbon_dioxide.glb"},
        ]
    }
    (ROOT / "objects.json").write_text(json.dumps(manifest, indent=4), encoding="utf-8")
    print(f"Created {ROOT / 'objects.json'}")


def main() -> None:
    make_geometry()
    make_simple_car()
    make_atom("hydrogen_atom.glb", "Hydrogen", protons=1, neutrons=0, electron_shells=[1])
    make_atom("carbon_atom.glb", "Carbon", protons=6, neutrons=6, electron_shells=[2, 4])
    make_molecules()
    write_manifest()
    print("\nAll initial assets generated successfully.")


if __name__ == "__main__":
    main()
