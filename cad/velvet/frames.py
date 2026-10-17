"""Switch frames per half (from velvet_layout) and the switch + cap solids on them."""
from build123d import Pos, Rot, import_step

from velvet.params import *  # noqa: F401,F403
from velvet.caps import dsa_cap


def half_frames(side):
    """This half's switch frames: (world before SEP, u frame). u = distance from the inner edge, outward; L is the mirror of R."""
    VL.GAP = 2 * X_IN[side]
    s = 1 if side == "R" else -1
    fr = {k: f for k, f in VL.key_frames().items() if k[0] == side}
    fu = {k: Pos(s * f.position.X, f.position.Y, 0) * Rot(0, 0, s * f.orientation.Z) for k, f in fr.items()}
    return fr, fu


def keys(fr):
    """(key, switch, cap) per world frame."""
    mx = import_step(SL.MX_STEP)
    mb = mx.bounding_box()
    mx = Pos(-mb.center().X, -mb.center().Y, -(mb.max.Z - SL.MX_ABOVE)) * mx
    return [(k, f * mx, f * Pos(0, 0, SL.CAP_SKIRT) * dsa_cap(VL.cap_u(k))) for k, f in fr.items()]   # DSA caps on the KS-33's stem, skirt CAP_SKIRT over the plate


# THE LEGENDS (render only, VELVET_LEGENDS=1; Willy 2026-10-17 'get the legends on the keys' for the repo shots): QWERTY on the 3 x 5 + the
# middle/ring extras + 4 thumbs a half; c0 = the inner index column, r0 the top row
LEGENDS = {"L": [["T", "R", "E", "W", "Q"], ["G", "F", "D", "S", "A"], ["B", "V", "C", "X", "Z"]], "R": [["Y", "U", "I", "O", "P"], ["H", "J", "K", "L", ";"], ["N", "M", ",", ".", "/"]]}
LEGEND_X = {"L_r3c2": "ALT", "L_r3c3": "CTRL", "R_r3c2": "-", "R_r3c3": "'", "L_t0": "SPC", "L_t1": "TAB", "L_t2": "ESC", "L_t3": "GUI",
            "R_t0": "ENT", "R_t1": "BSPC", "R_t2": "DEL", "R_t3": "SHFT"}


def legend_text(k):
    if k in LEGEND_X:
        return LEGEND_X[k]
    r, c = int(k.split("_r")[1][0]), int(k.split("c")[1])
    return LEGENDS[k[0]][r][c]


def legend(k):
    """The legend as a 0.12 skin on the cap's dished top (in the cap's frame, skirt on z 0)."""
    from build123d import Text, extrude
    t = legend_text(k)
    cap = dsa_cap(VL.cap_u(k))
    txt = Pos(0, 0, DSA_H - 2.5) * extrude(Text(t, 4.5 if len(t) == 1 else 2.6), 3.0)
    return (txt & (Pos(0, 0, 0.12) * cap)) - cap


def legends(fr):
    """(key, legend) per world frame, placed as keys() places the caps."""
    return [(k, f * Pos(0, 0, SL.CAP_SKIRT) * legend(k)) for k, f in fr.items()]
