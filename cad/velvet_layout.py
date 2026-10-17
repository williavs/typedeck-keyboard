"""velvet_fold layout, THE SHOT'S (Willy 2026-10-11: "lets not marry to my key layout and instead use the one from the picture").
Measured off the black halves of references/velvet-curved-split-reference.jpg (cap top faces segmented, 55 px = 19.05 pitch,
rows foreshortened 0.84 = a 33 deg camera): per hand 6 columns -- inner, index and middle at one height, ring 4.5 lower, pinky
14 and outer 15.5 lower; middle and ring carry a 4th key at the bottom; no per-column fan, the whole block 5 deg tops-OUTWARD
in its half; 3 thumb keys 48-59 below the home row rotated 13-23 deg tops-outward, running inward past the inner column;
knob (L) / 34 mm ball (R) 40 inward of the inner column at the bottom-row height. 23 keys a hand. Gateron KS-33 + ScottoCaps
flat, a Pico 2 W per half, the Pi Zero 2 W + PiTFT 2.2 stack flat on R. The shot is an AI render: these are its numbers, not
ergonomics -- Rempel 2007 wants the tops INWARD (YAW > 0) -- Willy decides after seeing it."""
import itertools, math, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "cyber_split"))
import cyber_display as DP                                     # the plugged Pi + header + socket + PiTFT stack (verified in cyber_split v3.2)
from build123d import Color, Compound, Pos, Rot, Sphere, import_step

SL = DP.SL                                                     # sophie_layout as cyber_layout patched it: KS-33 STEP, MX_ABOVE 6.35, CAP_SKIRT 2.4
REFS = SL.REFS
U = 19.05                                                      # the shot's pitch (standard; its top faces read 12.8 wide at that scale)
COLS = 5                                                       # c0 = inner index .. c4 = pinky (Willy 2026-10-11 21:43: the outer column dropped)
HOME_R, HOME_C = 1, 0                                          # hand x 0 = the inner column's home key, + outward; y up
COL_Y = [1.2, 0.0, 0.0, -4.5, -13.8, -15.5]                     # SHOT: mm each column sits up, home row vs the index column (L/R halves agreed within 0.4)
EXTRA_ROW = {2, 3}                                             # SHOT: middle and ring columns get a 4th key below the bottom row (r3)
THUMB_PIV, THUMB_R0, THUMB_TC = (30.0, -120.0), 80.0, -32.0   # the CMC OUTWARD-below the thumbs (Willy 22:05 'facing the wrong direction': v1 had it inward) ->
                                                               # a tops-INWARD fan, the inner keys lower + turned more (55/40/24/9), as the shot;   # Willy 21:44 "more thumb keys ... angle them right" + "most ergonomic": a row of 4 on an
                                                               # ARC round the thumb's base joint (CMC, GUESS 30 out / 120 below the index home), every key
                                                               # facing away from it, the inner corners 0.6 apart, + 1 on the inner arc under the row's middle.
                                                               # Chosen by a 2D search (scratchpad thumb_arc.py): clear of the trackpad / knob by 2, the row's
                                                               # centre 10 from the old thumb rest. (The shot's straight 13-deg row is retired.)
THUMB_U = (1.0, 1.0, 1.0, 1.0)                                 # cap widths (u) of the arc's 3 (inner -> outer) + the key tucked under the arc: all 1u (Willy 06:21 'leave them all same size')
_H = (U - 0.81) / 2                                            # a DSA cap's half depth (18.24: velvet.caps)
_hw = [U / 2 - 0.405] * 4                                      # the arc's half widths, laid out as the 4-key arc it was
_gap = [math.asin((_hw[k] + 0.3) / (THUMB_R0 - _H)) + math.asin((_hw[k + 1] + 0.3) / (THUMB_R0 - _H)) for k in range(3)]   # centre to centre: the inner corners 0.6 apart
_t = [0.0]
for _g in _gap:
    _t.append(_t[-1] + _g)
_t = [math.degrees(x - (_t[0] + _t[-1]) / 2) + THUMB_TC for x in _t]   # the arc centred on THUMB_TC
THUMBS = tuple((THUMB_PIV[0] + r * math.sin(math.radians(t)), THUMB_PIV[1] + r * math.cos(math.radians(t)), -t)
               for r, t in [(THUMB_R0, t) for t in _t[1:]] + [(THUMB_R0 - 2 * _H - 1.6, THUMB_TC)])   # (x out, y up, deg tops-inward): the arc's inner key is GONE
#                                                               (Willy 08:15 'that outermost key removed so we can draw a line tighter from the edge of peripherals to wrist rest inside edge': it sat at the
#                                                               seam), the other three stay where they were + the key tucked under the arc at THUMB_TC


def cap_u(name):
    """A key's cap width in u (thumbs by THUMB_U, alphas 1)."""
    return THUMB_U[int(name.split("_t")[1])] if "_t" in name else 1.0
YAW = -5.0                                                     # SHOT: deg a half, + = tops inward. The shot's blocks sit 5 deg tops-OUTWARD (every column
                                                               # top edge on both halves); Rempel 2007 found 12 total tops-inward best -- Willy's call
BALL_D = 34.0                                                  # SANWA replacement ball for the M575 (Willy 2026-10-10): spec diameter, a sphere needs no STEP
BALL_X, BALL_Y = -39.5, -19.0                                  # SHOT: ball centre vs the inner column's home (knob mirrored on L): 2 columns in, bottom-row height
FLOOR_Z, CUP_WALL = -10.3, 1.5                                 # cyber_split's key-zone underside (FLOOR_K -8.3 - PLATE_T 2.0); GUESS cup floor
BALL_Z = FLOOR_Z + CUP_WALL + BALL_D / 2                       # 8.2: "drop it down into the frame as much as we can" = the cup floor ON the underside, no bump
GAP = 2 * (-BALL_X + BALL_D / 2 + 4.0 + 2.0)                   # 125: inner columns apart when docked = the ball cup (ball + 4 cup + 2 seam wall) each side.
                                                               # The shot's halves stand 136 apart at the inner columns (loose), so this is its docked minimum
STACK = (0.5 * U, 2.5, SL.CAP_TOP + 0.8)                       # GUESS: Pi+PiTFT stack centre x, its front face this far behind the top row's caps, glass at the rim (9.9)
PICO = (4.0 * U, 12.0, -8.0)                                   # GUESS: Pico centre x, this far behind the top row's caps, z as cyber_layout (flat, under the plate line)
EC11_STEP = REFS / "modules/EC11_smd_5pin_30p_10mm.step"       # Willy's GrabCAD EC11 (shaft +y in the file, pads on y 0), knob for its 6 mm shaft
KNOB_STEP = REFS / "modules/EC11_knob_6mm_shaft.step"          # 14.5 dia; the shot's knob reads ~24 dia in a ~40 housing (a printed knob is OUR part)
SCOTTO = REFS / "scottokeebs"
CAP_C = Color(0.15, 0.9, 0.85)


def _key(r, c, s):
    """One alpha key's frame in hand coords (x outward): straight columns, the shot's stagger."""
    return Pos(s * (c - HOME_C) * U, (HOME_R - r) * U + COL_Y[c], 0)


def _thumb(i, s):
    x, y, a = THUMBS[i]
    return Pos(s * x, y, 0) * Rot(0, 0, s * a)                 # + = tops inward (R turns CCW, L CW)


def half_frame(s):
    return Pos(s * GAP / 2, 0, 0) * Rot(0, 0, s * YAW)


def key_frames():
    """{name: Location} world frames of every switch (plate top at z 0), both hands docked GAP apart and yawed."""
    out = {}
    for s, side in ((1, "R"), (-1, "L")):
        half = half_frame(s)
        for c in range(COLS):
            for r in range(4 if c in EXTRA_ROW else 3):
                out[f"{side}_r{r}c{c}"] = half * _key(r, c, s)
        for i in range(len(THUMBS)):
            out[f"{side}_t{i}"] = half * _thumb(i, s)
    return out


def _centred(part, z_bottom=None):
    bb = part.bounding_box()
    return Pos(-bb.center().X, -bb.center().Y, 0 if z_bottom is None else z_bottom - bb.min.Z) * part


def build():
    frames = key_frames()
    mx = import_step(SL.MX_STEP)
    mb = mx.bounding_box()
    mx = Pos(-mb.center().X, -mb.center().Y, -(mb.max.Z - SL.MX_ABOVE)) * mx                  # centred, plate top at z 0
    cap = _centred(Rot(180, 0, 0) * import_step(SCOTTO / "flat_1u.step"), SL.CAP_SKIRT)       # ScottoCaps are modelled top-down
    parts = []
    for k, f in frames.items():
        for p, name, col in ((f * mx, k, Color(0.55, 0.3, 0.75)), (f * cap, f"{k}_cap", CAP_C)):
            p.label, p.color = name, col
            parts.append(p)
    pico = import_step(SL.PICO_STEP)
    y_top = (HOME_R - 0) * U + max(COL_Y) + U / 2                                               # the top row's highest cap edge, hand y
    for s, side in ((1, "R"), (-1, "L")):
        half = half_frame(s)
        p = half * Pos(s * PICO[0], y_top + PICO[1] + 10.5, PICO[2]) * Rot(0, 0, 90) * _centred(pico)   # long side along the row
        p.label, p.color = f"pico_{side}", Color(0.1, 0.5, 0.2)
        parts.append(p)
        if side == "R":
            ball = half * Pos(s * BALL_X, BALL_Y, BALL_Z) * Sphere(BALL_D / 2)
            ball.label, ball.color = "trackball_34", Color(0.75, 0.05, 0.05)
            parts.append(ball)
            tft, _ = DP.hat_in_F()
            pi, hdr, sock = DP.stack_in_F()
            at = half * Pos(s * STACK[0], y_top + STACK[1] + DP.LID_R, STACK[2] - DP.Z_GLASS)   # F's front face (-LID_R) STACK[1] behind the caps, glass at the rim
            for q, name in ((tft, "pitft_2_2"), (pi, "pi_zero_2_w"), (hdr, "header_2x20"), (sock, "socket_2x20")):
                q = at * q
                q.label = name
                parts.append(q)
        else:
            enc = Rot(90, 0, 0) * import_step(EC11_STEP)                                        # shaft +y -> +z, pads on z 0
            knob = Rot(90, 0, 0) * import_step(KNOB_STEP)
            eb, kb = enc.bounding_box(), knob.bounding_box()
            enc = Pos(-eb.center().X, -eb.center().Y, 0) * enc
            knob = Pos(-kb.center().X, -kb.center().Y, eb.max.Z - 2.0 - kb.min.Z) * knob         # GUESS: the knob 2 mm down the shaft tip
            for q, name, col in ((enc, "ec11", Color(0.3, 0.3, 0.3)), (knob, "knob", Color(0.1, 0.1, 0.1))):
                q = half * Pos(s * BALL_X, BALL_Y, 0) * q                                        # the knob mirrors the ball: pads on the plate top
                q.label, q.color = name, col
                parts.append(q)
    return parts, frames


from cadgen import step as _cadgen_step  # noqa: E402


@_cadgen_step()
def velvet_layout():
    parts, _ = build()
    asm = Compound(children=parts)
    asm.label = "velvet_layout"
    return asm


if __name__ == "__main__":
    parts, frames = build()
    caps = [p for p in parts if p.label.endswith("_cap")]
    for side in "LR":
        bb = Compound(children=[p for p in parts if p.label.startswith(side)]).bounding_box()
        print(f"{side} half: {sum(1 for c in caps if c.label[0] == side)} keys, caps {bb.size.X:.0f} x {bb.size.Y:.0f} mm")
    pairs = [(a, b) for a, b in itertools.combinations(caps, 2) if a.label[0] == b.label[0]]
    near = min(((a.bounding_box().center() - b.bounding_box().center()).length, a.label, b.label) for a, b in pairs)
    clash = [(a.label, b.label) for a, b in pairs if (a.bounding_box().center() - b.bounding_box().center()).length < 1.5 * U and ((a & b) or Compound()).volume > 0.01]
    print(f"closest cap centres {near[1]}/{near[2]} {near[0]:.1f} mm; cap overlaps: {clash or 'none'}")
    by = {p.label: p for p in parts}
    for a, b in (("trackball_34", "R_t0_cap"), ("trackball_34", "R_r2c0_cap"), ("knob", "L_t0_cap"), ("pitft_2_2", "R_r0c2_cap"), ("pico_R", "R_r0c4_cap")):
        print(f"{a} -> {b}: {by[a].distance_to(by[b]):.1f} mm")
    bz = by["trackball_34"].bounding_box()
    print(f"ball top {bz.max.Z:.1f} over the plate ({bz.max.Z - SL.CAP_TOP:.1f} over the caps), bottom {bz.min.Z:.1f}; Pi bottom {by['pi_zero_2_w'].bounding_box().min.Z:.2f} (floor {FLOOR_Z})")
    velvet_layout()
