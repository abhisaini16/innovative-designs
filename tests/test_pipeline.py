import json
import runpy

import cadquery as cq
import numpy as np
import pytest
import trimesh

from scan2cad import cad, cleanup, measure, primitives, supports
from scan2cad.cli import run


def _stl(wp, path):
    cq.exporters.export(wp, str(path), tolerance=0.02, angularTolerance=0.1)
    return str(path)


@pytest.fixture(scope="module")
def bracket(tmp_path_factory):
    """40x30x20 block, 8 mm through-hole (Z) and a 6 mm post on top."""
    wp = (cq.Workplane("XY").box(40, 30, 20).faces(">Z").workplane().center(10, 0).hole(8)
          .faces(">Z").workplane().center(-20, 0).circle(3).extrude(8))
    return _stl(wp, tmp_path_factory.mktemp("a") / "bracket.stl")


@pytest.fixture(scope="module")
def tee(tmp_path_factory):
    """T shape: wide 60x30x6 top plate on a 10x10x20 stem -> two overhangs when printed stem-down."""
    stem = cq.Workplane("XY").box(10, 10, 20)
    top = cq.Workplane("XY").box(60, 30, 6).translate((0, 0, 13))
    return _stl(stem.union(top), tmp_path_factory.mktemp("b") / "tee.stl")


def test_measure_and_units(bracket):
    m = cleanup.clean(cleanup.load_mesh(bracket))
    r = measure.measure(m)
    assert r["watertight"]
    assert r["oriented_box_mm"][0] == pytest.approx(40, abs=0.2)
    m_m = cleanup.clean(cleanup.load_mesh(bracket, units="m"))
    assert m_m.extents.max() == pytest.approx(40000, rel=0.01)


def test_scale_fix(bracket):
    m = cleanup.scale_to_known_length(cleanup.clean(cleanup.load_mesh(bracket)), 40, 80)
    assert m.extents.max() == pytest.approx(80, abs=0.5)


def test_detects_hole_and_post(bracket):
    seg = primitives.segment(cleanup.clean(cleanup.load_mesh(bracket)))
    holes = [c for c in seg.cylinders if not c.convex]
    posts = [c for c in seg.cylinders if c.convex]
    assert any(c.radius == pytest.approx(4, abs=0.15) for c in holes)
    assert any(c.radius == pytest.approx(3, abs=0.15) for c in posts)
    assert seg.coverage > 0.9


def test_parametric_cad_is_editable(bracket, tmp_path):
    m = cleanup.clean(cleanup.load_mesh(bracket))
    res = cad.reverse_engineer(m, primitives.segment(m), str(tmp_path))
    assert res.mode == "parametric" and res.iou > 0.97
    assert {f["kind"] for f in res.features} == {"hole", "boss"}
    ns = runpy.run_path(res.script_path)
    assert ns["result"].Volume() == pytest.approx(m.volume, rel=0.03)
    # edit a parameter: bigger hole => less material
    ns["FEATURES"][[f["kind"] for f in ns["FEATURES"]].index("hole")]["radius"] = 6.0
    assert ns["build"]().Volume() < ns["result"].Volume() - 500


def test_organic_falls_back_to_faceted(tmp_path):
    blob = trimesh.creation.icosphere(subdivisions=3, radius=10)
    blob.vertices *= [1.0, 1.6, 0.6] * np.ones(3)
    blob.vertices += 0.8 * np.sin(blob.vertices[:, [1, 2, 0]] * 1.3)
    res = cad.reverse_engineer(blob, primitives.segment(blob), str(tmp_path), min_iou=0.97)
    assert res.mode == "faceted" and (tmp_path / "model.step").exists()


def test_supports_generated_for_overhang(tee):
    m = cleanup.clean(cleanup.load_mesh(tee))
    placed = supports.place_on_bed(m, [0, 0, 1])
    sup, stats = supports.generate_supports(placed)
    assert sup is not None and stats["columns"] > 20
    # supports must not intersect the part
    inside = placed.contains(sup.vertices[::7])
    assert inside.mean() < 0.02
    # auto orientation prefers the plate on the bed => no supports needed
    u, info = supports.choose_orientation(m)
    flat, _ = supports.generate_supports(supports.place_on_bed(m, u))
    assert flat is None  # best orientation needs no supports at all
    assert info["overhang_area_mm2"] < stats["overhang_area_mm2"]


def test_full_cli_run(bracket, tmp_path):
    rep = run(bracket, "mm", str(tmp_path))
    assert rep["cad"]["mode"] == "parametric"
    for f in ("model.step", "model.py", "model_oriented.stl", "print.3mf", "report.json"):
        assert (tmp_path / f).exists(), f
    assert json.loads((tmp_path / "report.json").read_text())["measurements"]["watertight"]
