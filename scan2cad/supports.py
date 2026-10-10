"""Pick a print orientation and generate break-away support columns for overhangs."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import trimesh


@dataclass
class SupportParams:
    max_overhang_deg: float = 45.0  # faces leaning out more than this from vertical need support
    spacing: float = 2.5            # mm between support columns
    radius: float = 0.6             # column radius
    gap: float = 0.2                # air gap between tip and part, so supports snap off
    tip_height: float = 1.0
    pad_height: float = 0.3         # wider footing where a column stands on the bed
    min_height: float = 0.6         # skip supports shorter than this


def _overhang_mask(mesh, up_dir, params):
    """Faces that need support when `up_dir` is the build direction, excluding faces lying on the bed."""
    up = np.asarray(up_dir, float)
    nz = mesh.face_normals @ up
    z = mesh.vertices @ up
    zmin = z.min()
    on_bed = z[mesh.faces].max(axis=1) < zmin + 0.05
    return (nz < -np.sin(np.radians(params.max_overhang_deg))) & ~on_bed


def choose_orientation(mesh, params: SupportParams | None = None) -> tuple[np.ndarray, dict]:
    """Return the build-up direction (unit vector, in the mesh frame) with the least overhang area."""
    params = params or SupportParams()
    cands = []
    obb = np.asarray(mesh.bounding_box_oriented.primitive.transform)[:3, :3]
    for ax in list(np.eye(3)) + list(obb.T):
        cands += [ax, -ax]
    # big flat faces make good bed contacts (face normal points down => up = -normal)
    from .primitives import detect_planes
    for pl in detect_planes(mesh)[0][:6]:
        cands.append(-pl.normal)
    best = None
    for up in cands:
        up = np.asarray(up, float) / np.linalg.norm(up)
        area = float(mesh.area_faces[_overhang_mask(mesh, up, params)].sum())
        height = float(np.ptp(mesh.vertices @ up))
        score = area + 0.01 * height  # prefer low overhang, then short prints
        if best is None or score < best[0] - 1e-9:
            best = (score, up, area, height)
    _, up, area, height = best
    return up, {"overhang_area_mm2": round(area, 2), "height_mm": round(height, 2)}


def place_on_bed(mesh, up_dir) -> trimesh.Trimesh:
    """Rotate so up_dir is +Z and the part sits on z=0."""
    m = mesh.copy()
    m.apply_transform(trimesh.geometry.align_vectors(np.asarray(up_dir, float), [0, 0, 1]))
    m.apply_translation([0, 0, -m.bounds[0][2]])
    return m


def generate_supports(mesh, params: SupportParams | None = None) -> tuple[trimesh.Trimesh | None, dict]:
    """`mesh` must already sit on the bed (Z up). Columns drop from each overhang to the bed or the part."""
    params = params or SupportParams()
    over = _overhang_mask(mesh, [0, 0, 1], params)
    stats = {"overhang_area_mm2": round(float(mesh.area_faces[over].sum()), 2), "columns": 0, "volume_mm3": 0.0}
    if not over.any():
        return None, stats

    lo, hi = mesh.bounds
    # tiny offset keeps rays off exactly-axis-aligned triangle edges
    xs = np.arange(lo[0] + params.spacing * 0.0137, hi[0], params.spacing)
    ys = np.arange(lo[1] + params.spacing * 0.0291, hi[1], params.spacing)
    gx, gy = np.meshgrid(xs, ys)
    origins = np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, lo[2] - 1.0)]
    dirs = np.tile([0.0, 0.0, 1.0], (len(origins), 1))
    loc, ray, tri = mesh.ray.intersects_location(origins, dirs, multiple_hits=True)
    if len(loc) == 0:
        return None, stats

    order = np.lexsort((loc[:, 2], ray))
    loc, ray, tri = loc[order], ray[order], tri[order]
    cols = []  # (x, y, z_low, z_high)
    for i in range(len(loc)):
        if not over[tri[i]]:
            continue
        same_ray_prev = i > 0 and ray[i - 1] == ray[i]
        z_low = loc[i - 1, 2] if same_ray_prev else 0.0
        if same_ray_prev and loc[i, 2] - loc[i - 1, 2] < 1e-6:
            continue  # duplicate hit on a shared edge
        z_high = loc[i, 2] - params.gap
        if z_high - z_low >= params.min_height:
            cols.append((loc[i, 0], loc[i, 1], z_low, z_high))
    if not cols:
        return None, stats

    cyl = trimesh.creation.cylinder(radius=1, height=1, sections=12)
    cone = trimesh.creation.cone(radius=1, height=1, sections=12)  # base z=0, apex z=1
    parts = []
    r, th = params.radius, params.tip_height
    for x, y, z0, z1 in cols:
        tip = min(th, z1 - z0)
        body_top = z1 - tip
        if body_top - z0 > 1e-3:
            c = cyl.copy()
            c.vertices = c.vertices * [r, r, body_top - z0] + [x, y, (z0 + body_top) / 2]
            parts.append(c)
        k = cone.copy()
        k.vertices = k.vertices * [r, r, tip] + [x, y, body_top]
        parts.append(k)
        if z0 == 0.0:
            p = cyl.copy()
            p.vertices = p.vertices * [2 * r, 2 * r, params.pad_height] + [x, y, params.pad_height / 2]
            parts.append(p)
    sup = trimesh.util.concatenate(parts)
    stats.update(columns=len(cols), volume_mm3=round(abs(float(sup.volume)), 2))
    return sup, stats
