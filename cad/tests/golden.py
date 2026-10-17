"""Rewrite tests/golden.json from the current model (after an INTENDED geometry change): python tests/golden.py"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import velvet.hull  # noqa: E402
from velvet.assembly import assemble  # noqa: E402
from test_build import _parts  # noqa: E402

velvet.hull.FRESH = True                                            # never the step cache
rec = {}
for name, s in _parts(assemble()).items():
    b = s.bounding_box()
    rec[name] = dict(vol=round(s.volume, 2), bb=[round(v, 3) for v in (b.min.X, b.min.Y, b.min.Z, b.max.X, b.max.Y, b.max.Z)], valid=bool(s.is_valid))
Path(__file__).with_name("golden.json").write_text(json.dumps(rec, indent=1, sort_keys=True))
print("wrote golden.json", len(rec), "parts")
