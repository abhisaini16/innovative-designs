"""Write print-ready files."""
from __future__ import annotations

import trimesh


def export_print_files(model: trimesh.Trimesh, supports: trimesh.Trimesh | None, out_dir: str) -> dict:
    """STL for the part, STL for supports, and one 3MF holding both as separate objects."""
    paths = {"model_stl": f"{out_dir}/model_oriented.stl"}
    model.export(paths["model_stl"])
    scene = trimesh.Scene({"model": model})
    if supports is not None:
        paths["supports_stl"] = f"{out_dir}/supports.stl"
        supports.export(paths["supports_stl"])
        scene.add_geometry(supports, node_name="supports", geom_name="supports")
    paths["print_3mf"] = f"{out_dir}/print.3mf"
    scene.export(paths["print_3mf"])
    return paths
