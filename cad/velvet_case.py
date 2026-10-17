"""velvet_fold -- a split keyboard, two tented halves (the cadgen model; the CAD lives in velvet/).

Each half: 22 Gateron KS-33 keys on a 10 deg tilted deck (5 columns + a 4-key thumb arc), a hull that hugs its parts behind (the outline is the
parts' footprints + a 3 mm wall, flowed) and is one wedge in front (the rest: a trapezoid with a TPU pad, hinged to the base), a Pico 2 W face up
under an acrylic pane with a printed frame IN THE PALM DECK between the rest and the alphas (plug out the outer wall), an 18650 + Willy's V713
charger in a flat band behind the keys. R carries the Pi Zero 2 W + PiTFT 2.2 stack under a printed
bezel and the Azoteq TPS43 trackpad under a dished skin; L the sunk EC11 knob. Each half sits on a hinged base slab: a flap (stand) under
the hatch drops its nose into one of three sawtooth seats for 28 / 24 / 14 deg of tent. The halves talk over the Picos' radio (alice-kb's).

Package map (velvet/): params (every number, with its reason) -> geo (helpers) -> frames (switches) -> plan (the 2D plan of a half) ->
hull (the half, step by step) / stand (base, flap, seats) / parts (knob) -> assembly (world placement, labels, joints) -> checks, export.
Tests: tests/ (pytest; test_plan is seconds, the rest build the model). History + decisions: PROJECT.md.

Run: python velvet_case.py [--force] -> velvet_case.step. `mkdir .cache` turns on the per-step hull cache (hull.Half.build: a look change at the
end rebuilds in ~2 min, not 10; tests and golden always build fresh).  VELVET_CHECKS=fast|full|off (the cadgen worker ignores env: run
scratchpad full_checks.py for the stand's motion tests), VELVET_STL=1 writes the print STLs to STL/."""
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from build123d import Compound  # noqa: E402
from cadgen import step as _cadgen_step  # noqa: E402

from velvet.assembly import assemble, kinematics, parts  # noqa: E402
from velvet.checks import checks  # noqa: E402
from velvet.export import export  # noqa: E402

CHECKS = os.environ.get("VELVET_CHECKS", "fast")
STL = os.environ.get("VELVET_STL") == "1"
_A = None


def model():
    """The assembled model, built once per process."""
    global _A
    if _A is None:
        _A = assemble()
    return _A


@_cadgen_step(kinematics=kinematics())
def velvet_case():
    A = model()
    if CHECKS != "off":
        checks(A, CHECKS)
    if STL:
        export(A)
    asm = Compound(children=parts(A))
    asm.label = "velvet_case"
    return asm


if __name__ == "__main__":
    velvet_case()
