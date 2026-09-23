from __future__ import annotations

import json
from pathlib import Path
from typing import Any


class ObjectManager:
    """Loads object definitions from objects.json and resolves asset paths."""

    def __init__(self, project_root: Path | None = None) -> None:
        self.project_root = (project_root or Path(__file__).resolve().parent).resolve()
        self.registry_path = self.project_root / "objects.json"
        self.objects: list[dict[str, Any]] = []
        self._load_registry()

    def _load_registry(self) -> None:
        if not self.registry_path.exists():
            raise FileNotFoundError(f"Object registry not found: {self.registry_path}")

        with self.registry_path.open("r", encoding="utf-8") as f:
            data = json.load(f)

        objects = data.get("objects")
        if not isinstance(objects, list):
            raise ValueError("objects.json must contain an 'objects' list")

        self.objects = objects

    def all_objects(self) -> list[dict[str, Any]]:
        return list(self.objects)

    def get_object(self, index: int) -> dict[str, Any]:
        if not 0 <= index < len(self.objects):
            raise IndexError("Object index out of range")
        return self.objects[index]

    def asset_path(self, index: int) -> Path:
        obj = self.get_object(index)
        relative_path = obj.get("file")
        if not relative_path:
            raise ValueError(f"Object '{obj.get('id', index)}' has no file path")

        path = (self.project_root / relative_path).resolve()

        # Keep the resolved asset inside the project directory.
        try:
            path.relative_to(self.project_root)
        except ValueError as exc:
            raise ValueError(f"Asset path escapes project directory: {path}") from exc

        if not path.exists():
            raise FileNotFoundError(f"Asset not found: {path}")

        return path
