"""Load a scan and make it a clean, watertight, correctly scaled mesh."""
from __future__ import annotations

import numpy as np
import trimesh

# Scan exporters differ: ARKit / Object Capture write metres, most CAD/STL files millimetres.
UNIT_TO_MM = {"mm": 1.0, "cm": 10.0, "m": 1000.0, "in": 25.4}


def load_mesh(path: str, units: str = "mm") -> trimesh.Trimesh:
    """Load OBJ/PLY/STL/GLB/3MF and scale to millimetres. USDZ must be converted to OBJ/PLY first."""
    if units not in UNIT_TO_MM:
        raise ValueError(f"units must be one of {sorted(UNIT_TO_MM)}")
    mesh = trimesh.load(path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
        raise ValueError(f"{path}: no triangle mesh found")
    mesh = mesh.copy()
    mesh.apply_scale(UNIT_TO_MM[units])
    return mesh


def clean(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Merge vertices, drop junk faces/islands, fix normals and close holes."""
    m = mesh.copy()
    m.merge_vertices()
    m.update_faces(m.nondegenerate_faces())
    m.update_faces(m.unique_faces())
    m.remove_unreferenced_vertices()
    parts = m.split(only_watertight=False)
    if len(parts) > 1:  # scans often carry floating noise; keep the object
        m = max(parts, key=lambda p: p.area)
    trimesh.repair.fix_inversion(m)
    trimesh.repair.fix_normals(m)
    trimesh.repair.fill_holes(m)
    if m.is_watertight and m.volume < 0:
        m.invert()
    return m


def scale_to_known_length(mesh: trimesh.Trimesh, measured_mm: float, real_mm: float) -> trimesh.Trimesh:
    """Fix scale of a photogrammetry scan (no LiDAR) from one real-world measurement."""
    if measured_mm <= 0 or real_mm <= 0:
        raise ValueError("lengths must be positive")
    m = mesh.copy()
    m.apply_scale(real_mm / measured_mm)
    return m
