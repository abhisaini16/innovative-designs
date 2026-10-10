# Innovative Designs

## scan2cad: phone scan -> measurements -> editable CAD -> supported 3D print

```
iPhone (LiDAR)  ->  OBJ (metres)  ->  scan2cad  ->  report.json   dimensions, volume
                                                ->  model.step    CAD (STEP)
                                                ->  model.py      editable parametric CadQuery script
                                                ->  print.3mf     oriented part + auto supports
                                                ->  model_oriented.stl / supports.stl
```

```bash
pip install -e '.[dev]'
scan2cad scan.obj --units m --out out/        # iPhone Object Capture exports metres
scan2cad scan.stl --real-length-mm 120        # no LiDAR: fix scale from one real measurement
pytest
```

### What it does
1. **Clean** the mesh (merge, remove noise islands, fix normals, fill holes), scale to mm.
2. **Measure**: bounding box (axis-aligned and oriented), volume, area.
3. **Detect** planes and cylinders (region growing + least-squares fits).
4. **Rebuild as parametric CAD**: a box/cylinder/sphere body plus holes and posts, written as
   `model.py` with named numbers you can edit, and exported to STEP. A feature is kept only if it
   improves volume overlap (IoU) with the scan; the IoU is reported.
5. **Print prep**: chooses the build orientation with the least overhang, then generates
   break-away support columns (default 45 deg limit, 2.5 mm spacing, 0.2 mm air gap) landing on the
   bed or on the part. `print.3mf` holds part and supports as separate objects.

### Limits (be aware)
- "Fully editable CAD" works for machined-style parts (flat faces, round holes, posts). Organic
  shapes fall back to a **faceted STEP**: exact solid, but not feature-editable (`mode: faceted`).
  No extrusions/fillets/chamfers/slots/threads are recognised yet.
- Accuracy is limited by the scan: LiDAR iPhone ~ +-1-3 mm; shiny, transparent or black surfaces scan badly.
  Non-LiDAR iPhones (standard 15) need `--real-length-mm`.
- Supports are straight columns with a tip, not slicer-grade tree supports. For production prints
  you may prefer slicer-generated supports on `model_oriented.stl`.
- USDZ must be exported as OBJ/PLY first. `ios/ScanCapture` is an **untested** capture skeleton.
