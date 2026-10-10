"""scan2cad scan.obj --units m --out out/"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from . import cad, cleanup, export, measure, primitives, supports


def run(path, units="mm", out="out", real_length_mm=None, params=None, min_iou=0.9) -> dict:
    out_dir = Path(out)
    out_dir.mkdir(parents=True, exist_ok=True)
    params = params or supports.SupportParams()
    mesh = cleanup.clean(cleanup.load_mesh(path, units))
    if real_length_mm:  # photogrammetry scale fix: longest side is really this long
        mesh = cleanup.scale_to_known_length(mesh, float(mesh.extents.max()), real_length_mm)
    report = {"measurements": measure.measure(mesh)}

    seg = primitives.segment(mesh)
    res = cad.reverse_engineer(mesh, seg, str(out_dir), min_iou=min_iou)
    report["cad"] = {
        "mode": res.mode, "iou": round(res.iou, 3), "base": res.base, "features": res.features,
        "planes": len(seg.planes), "cylinders": len(seg.cylinders),
        "surface_explained": round(seg.coverage, 3), "notes": res.notes,
        "step": res.step_path, "script": res.script_path,
    }

    up, orient = supports.choose_orientation(mesh, params)
    placed = supports.place_on_bed(mesh, up)
    sup, sstats = supports.generate_supports(placed, params)
    report["print"] = {"orientation_up": [round(float(x), 4) for x in up], **orient, "supports": sstats,
                       "files": export.export_print_files(placed, sup, str(out_dir))}
    (out_dir / "report.json").write_text(json.dumps(report, indent=2))
    return report


def main(argv=None):
    ap = argparse.ArgumentParser(prog="scan2cad", description=__doc__)
    ap.add_argument("scan", help="OBJ / PLY / STL / GLB / 3MF (convert USDZ to OBJ first)")
    ap.add_argument("--units", default="mm", choices=sorted(cleanup.UNIT_TO_MM),
                    help="units of the file (iPhone Object Capture exports metres)")
    ap.add_argument("--out", default="out")
    ap.add_argument("--real-length-mm", type=float, help="true length of the longest side (fixes non-LiDAR scale)")
    ap.add_argument("--overhang", type=float, default=45.0, help="max printable overhang angle, degrees")
    ap.add_argument("--support-spacing", type=float, default=2.5)
    ap.add_argument("--min-iou", type=float, default=0.9, help="min volume match to accept a parametric model")
    a = ap.parse_args(argv)
    p = supports.SupportParams(max_overhang_deg=a.overhang, spacing=a.support_spacing)
    print(json.dumps(run(a.scan, a.units, a.out, a.real_length_mm, p, a.min_iou), indent=2))


if __name__ == "__main__":
    main()
