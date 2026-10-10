"""Find planes and cylinders in a mesh (the building blocks of an editable CAD model)."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import trimesh
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


@dataclass
class Plane:
    normal: np.ndarray
    offset: float  # plane: normal . x = offset
    area: float
    faces: np.ndarray = field(repr=False)


@dataclass
class Cylinder:
    axis: np.ndarray
    point: np.ndarray  # point on the axis closest to the origin
    radius: float
    t0: float  # extent along the axis, as coordinates of (x . axis)
    t1: float
    convex: bool  # True: outside surface of a post/boss; False: wall of a hole
    area: float
    rms: float
    faces: np.ndarray = field(repr=False)


@dataclass
class Segmentation:
    planes: list[Plane]
    cylinders: list[Cylinder]
    coverage: float  # fraction of surface area explained by primitives


def _neighbours(mesh: trimesh.Trimesh) -> list[list[int]]:
    nb: list[list[int]] = [[] for _ in range(len(mesh.faces))]
    for a, b in mesh.face_adjacency:
        nb[a].append(b)
        nb[b].append(a)
    return nb


def detect_planes(mesh, ang_tol_deg=3.0, dist_tol=None, min_area_frac=0.01) -> tuple[list[Plane], np.ndarray]:
    n, c, area = mesh.face_normals, mesh.triangles_center, mesh.area_faces
    dist_tol = dist_tol if dist_tol is not None else 0.003 * float(np.linalg.norm(mesh.extents))
    cos_tol = np.cos(np.radians(ang_tol_deg))
    nb = _neighbours(mesh)
    label = -np.ones(len(n), int)
    planes: list[Plane] = []
    for seed in np.argsort(-area):
        if label[seed] >= 0:
            continue
        ns, cs = n[seed], c[seed]
        region, stack = [seed], [seed]
        label[seed] = -2  # visited
        while stack:
            f = stack.pop()
            for g in nb[f]:
                if label[g] == -1 and n[g] @ ns >= cos_tol and abs((c[g] - cs) @ ns) <= dist_tol:
                    label[g] = -2
                    stack.append(g)
                    region.append(g)
        region = np.array(region)
        if area[region].sum() < min_area_frac * area.sum():
            label[region] = -1  # release so the cylinder pass can use them
            continue
        w = area[region]
        centroid = (c[region] * w[:, None]).sum(0) / w.sum()
        _, _, vt = np.linalg.svd((c[region] - centroid) * np.sqrt(w)[:, None], full_matrices=False)
        nrm = vt[-1]
        if nrm @ (n[region] * w[:, None]).sum(0) < 0:
            nrm = -nrm
        label[region] = len(planes)
        planes.append(Plane(nrm, float(nrm @ centroid), float(w.sum()), region))
    return planes, label


def detect_cylinders(mesh, free_mask, max_rms_frac=0.02, min_area_frac=0.002) -> list[Cylinder]:
    n, area = mesh.face_normals, mesh.area_faces
    idx = np.flatnonzero(free_mask)
    if len(idx) < 8:
        return []
    pos = -np.ones(len(n), int)
    pos[idx] = np.arange(len(idx))
    adj = mesh.face_adjacency
    ok = free_mask[adj[:, 0]] & free_mask[adj[:, 1]]
    ok &= (n[adj[:, 0]] * n[adj[:, 1]]).sum(1) > np.cos(np.radians(45))
    a, b = pos[adj[ok, 0]], pos[adj[ok, 1]]
    g = coo_matrix((np.ones(len(a)), (a, b)), shape=(len(idx), len(idx)))
    _, comp = connected_components(g, directed=False)
    diag = float(np.linalg.norm(mesh.extents))
    out: list[Cylinder] = []
    for k in range(comp.max() + 1):
        faces = idx[comp == k]
        if len(faces) < 8 or area[faces].sum() < min_area_frac * area.sum():
            continue
        w = area[faces]
        N = n[faces]
        evals, evecs = np.linalg.eigh((N * w[:, None]).T @ N)
        if evals[0] / evals.sum() > 0.02 or evals[1] / evals.sum() < 0.1:
            continue  # normals do not lie in one plane / span too little angle
        axis = evecs[:, 0]
        V = mesh.vertices[np.unique(mesh.faces[faces])]
        u = np.cross(axis, [1.0, 0, 0] if abs(axis[0]) < 0.9 else [0, 1.0, 0])
        u /= np.linalg.norm(u)
        v = np.cross(axis, u)
        x, y = V @ u, V @ v
        sol, *_ = np.linalg.lstsq(np.c_[2 * x, 2 * y, np.ones_like(x)], x**2 + y**2, rcond=None)
        cx, cy = sol[0], sol[1]
        r2 = sol[2] + cx**2 + cy**2
        if r2 <= 0:
            continue
        r = float(np.sqrt(r2))
        rms = float(np.sqrt(np.mean((np.hypot(x - cx, y - cy) - r) ** 2)))
        if rms / r > max_rms_frac or r > 2 * diag:
            continue
        t = V @ axis
        centre = cx * u + cy * v
        centre = centre - (centre @ axis) * axis
        cent = mesh.triangles_center[faces]
        radial = cent - (cent @ axis)[:, None] * axis - centre
        convex = float(np.sum(w * np.einsum("ij,ij->i", N, radial))) > 0
        out.append(Cylinder(axis, centre, r, float(t.min()), float(t.max()), convex, float(w.sum()), rms, faces))
    return sorted(out, key=lambda c: -c.area)


def segment(mesh: trimesh.Trimesh) -> Segmentation:
    planes, label = detect_planes(mesh)
    cyls = detect_cylinders(mesh, label < 0)
    explained = sum(p.area for p in planes) + sum(c.area for c in cyls)
    return Segmentation(planes, cyls, float(explained / mesh.area))
