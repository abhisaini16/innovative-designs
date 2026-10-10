"""Dimensions of the scanned object."""
from __future__ import annotations

import numpy as np
import trimesh


def measure(mesh: trimesh.Trimesh) -> dict:
    """Return size information in millimetres."""
    obb = mesh.bounding_box_oriented
    ext = np.sort(np.asarray(obb.primitive.extents))[::-1]
    out = {
        "aabb_mm": [round(float(x), 3) for x in mesh.extents],
        "oriented_box_mm": [round(float(x), 3) for x in ext],  # length >= width >= height
        "watertight": bool(mesh.is_watertight),
        "faces": int(len(mesh.faces)),
        "surface_area_mm2": round(float(mesh.area), 2),
    }
    if mesh.is_watertight:
        out["volume_mm3"] = round(float(mesh.volume), 2)
    return out


def distance(p: np.ndarray, q: np.ndarray) -> float:
    """Point-to-point measurement (what a tap-two-points tool in the app computes)."""
    return float(np.linalg.norm(np.asarray(p, float) - np.asarray(q, float)))
