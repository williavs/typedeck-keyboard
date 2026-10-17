"""THE SLIDE DOORS (build 155): print-in-place sliders in a pocket inside a wall, over an oval port. Wall frame W: origin the oval's centre on the
outer face, +x along the wall, +y INWARD, +z up. Layers from the face: DOOR_SKIN | DOOR_GAP | leaf | DOOR_GAP | (leaf | DOOR_GAP |) DOOR_BACK; the
caller's wall is WALL thick, so a pad is fused on the inside before the pocket is cut (only where nothing else backs the pocket). The halves print
BOTTOM-DOWN (164; 155-163 rim-down): the pocket's model-bottom face is the print floor, every door's bottom edge DOOR_GAP_FLOOR over it, its top edge
DOOR_GAP_BRIDGE under the pocket's roof (by the frame's real up: door_bypass reads it off Wf). The reasons for each number are in params (PICO_*, PI_*, DOOR_*)."""
from shapely import affinity
from shapely.geometry import Point, box as sbox
from build123d import Align, Box, Pos, Rot
from velvet.params import *  # noqa: F401,F403
from velvet.geo import _hit, prism

MID = (Align.CENTER, Align.CENTER, Align.CENTER)


def wbox(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0, align=MID)


def wprism(poly, y0, y1):
    """A plan polygon (x, z) extruded INWARD from depth y0 to y1."""
    return Rot(90, 0, 0) * prism(poly, -y1, -y0)


def ellipse(w, h, x=0.0, z=0.0):
    return affinity.scale(Point(x, z).buffer(1.0, quad_segs=48), w / 2, h / 2, origin=(x, z))


def rounded_rect(w, h, r, x=0.0, z=0.0):
    return sbox(x - w / 2 + r, z - h / 2 + r, x + w / 2 - r, z + h / 2 - r).buffer(r, quad_segs=12)


def tab(x0, x1, z0, z1, y_face):
    """The thumb tab: from DOOR_TAB[2] proud of the wall's face to 0.01 into the door's face at y_face."""
    return wbox(x0, x1, -DOOR_TAB[2], y_face + 0.01, z0, z1)


def _solid_grid(hull, Wf, xs, zs, ys, skip, what):
    """Every (x, z) of the grid not skipped must be inside the hull at each depth y."""
    bad = [(round(x, 1), round(y, 1), round(z, 1)) for x in xs for z in zs if not skip(x, z) for y in ys if not hull.is_inside((Wf * Pos(x, y, z)).position)]
    assert not bad, f"{what}: {len(bad)} probe(s) in air, e.g. {bad[:6]}"


def _vol(q):
    return 0.0 if q is None else q.volume


def _cut(hull, tool, what):
    """hull - tool, checked against what the tool actually overlaps (a boolean can hand the target back unchanged: velvet 83); one fuzzy retry."""
    from velvet.geo import fuzzy_cut
    ex = _vol(hull & tool)                                                        # judged by fresh booleans of the tool's region: the whole hull's GProp
    assert ex > 5.0, f"{what}: the tool overlaps only {ex:.1f} mm3 of the hull"    # volume drifts tens of mm3 (velvet 122)
    r = hull - tool
    left = _vol(r & tool)
    if left > 0.05 * ex + 0.3:
        print(f"   {what}: the plain cut left {left:.1f} of {ex:.0f} mm3, retrying fuzzy", flush=True)
        r = fuzzy_cut(hull, tool); left = _vol(r & tool)
    assert left <= 0.05 * ex + 0.3, f"{what}: the cut left {left:.1f} of {ex:.0f} mm3"
    assert len(r.solids()) == len(hull.solids()), f"{what}: {len(r.solids())} solids after the cut"
    return r, ex


def door_bypass(hull, Wf, bore, side, spec=None, tag="PLUG DOOR"):
    """A two-leaf slide door (the Pico plug's, 155; the charging windows', 162) behind a rounded-rectangle hole: pad, pocket, hole, tab tail cut from
    `hull`; `spec` the numbers (DOOR_SPEC / CHG_SPEC), `bore` a tunnel to keep out of the pad (or None). Returns (hull, (A, B), clear) with the leaves
    CLOSED in the caller's frame and `clear` a box round the pad (for the hatch's fence)."""
    s = spec or DOOR_SPEC
    import numpy as np
    (pk0, pk1), (a0, a1), (b0, b1), (ta, tb) = s["pocket"], s["a"], s["b"], s["leaf"]
    ya0 = DOOR_SKIN + DOOR_GAP; ya1 = ya0 + ta; yb0 = ya1 + DOOR_GAP; yb1 = yb0 + tb; yp1 = yb1 + DOOR_GAP; yback = yp1 + DOOR_BACK
    hz = s["hole"][1] / 2 + 0.05
    up = (Wf * Pos(0, 0, 1)).position.Z - Wf.position.Z                                       # (164) the halves print BOTTOM-DOWN (Willy 01:45): the pocket's model-BOTTOM face is
    pz0, pz1 = (-hz - DOOR_GAP_FLOOR, hz + DOOR_GAP_BRIDGE) if up > 0 else (-hz - DOOR_GAP_BRIDGE, hz + DOOR_GAP_FLOOR)   # the print floor (L's charge frame is upside down: W's -z is up)
    skip = lambda x, z: abs(x) < s["plug_w"] / 2 + 1.0
    xs, zs = np.linspace(pk0 + 0.2, pk1 - 0.2, 20), (pz0 + 0.3, 0.0, pz1 - 0.3)
    _solid_grid(hull, Wf, xs, zs, (1.0, WALL - 0.1), skip, f"{side} {tag}: the wall under the pocket is not a full plate")
    v0 = hull.volume
    pad = Wf * wbox(pk0 - 0.8, pk1 + 0.8, WALL - 0.5, yback, s["pad_z"][0], s["pad_z"][1])
    if bore is not None:
        pad = pad - bore
    clear = Wf * wbox(pk0 - 1.1, pk1 + 1.1, WALL - 0.8, yback + 0.3, s["pad_z"][0] - 0.3, s["pad_z"][1] + 0.3)
    hull = hull + pad
    assert len(hull.solids()) == 1 and hull.volume > v0 + 10.0, f"{side} {tag}: the pad did not fuse ({hull.volume - v0:.0f} mm3, {len(hull.solids())} solids)"
    v1 = hull.volume
    hull, _ = _cut(hull, Wf * wbox(pk0, pk1, DOOR_SKIN, yp1, pz0, pz1), f"{side} {tag}: pocket")
    v2 = hull.volume
    tx0, tx1, tz0, tz1 = s["tab"]
    tail = sbox(s["tail"], tz0 - DOOR_GAP, -s["hole"][0] / 2 + s["hole"][2] + 0.1, tz1 + DOOR_GAP)          # the tab's tail off the hole's top-front corner: through the SKIN only
    hull, _ = _cut(hull, Wf * wprism(rounded_rect(*s["hole"]), -1.0, yback + 0.5), f"{side} {tag}: hole")
    hull, _ = _cut(hull, Wf * wprism(tail, -1.0, DOOR_SKIN + 0.1), f"{side} {tag}: the tab's tail")
    hole = rounded_rect(*s["hole"]).union(tail)
    near = lambda x, z: hole.buffer(1.0).contains(Point(x, z))
    _solid_grid(hull, Wf, xs, zs, (DOOR_SKIN / 2,), near, f"{side} {tag}: outer skin")
    _solid_grid(hull, Wf, xs, zs, (yback - 0.3,), near, f"{side} {tag}: back")
    gx0, gx1, gh, gd = s["groove"]; lx0, lx1, lh = s["lug"]
    assert gx1 - lx1 > 7.0 and lx0 - gx0 >= DOOR_GAP - 0.01 and a1 - gx1 >= 0.79 and ta - gd >= 0.59 and b1 - lx1 >= 0.19, "the lug / groove numbers no longer give the lost motion or the end walls"
    assert tx0 - 0.5 >= gx0 and tz0 - gh >= 0.4, "the tab stands over the groove's band: A's face there is only the 0.6 skin"
    A = wbox(a0, a1, ya0, ya1, -hz, hz) - wbox(gx0, gx1, ya1 - gd, ya1 + 1.0, -gh, gh) + tab(tx0, tx1, tz0, tz1, ya0)   # the tab above the groove's band, through the hole
    B = wbox(b0, b1, yb0, yb1, -hz, hz) + wbox(lx0, lx1, ya1 - gd + DOOR_GAP, yb0 + 0.01, -lh, lh)                   # the lug: across the A-B gap, DOOR_GAP off the groove's floor
    A, B = Wf * A, Wf * B
    for n, q in (("A", A), ("B", B)):
        assert q.is_valid and len(q.solids()) == 1, f"{side} {tag}: leaf {n} is {len(q.solids())} solids, valid {q.is_valid}"
        assert _hit(q, hull) < 0.01, f"{side} {tag}: leaf {n} sits in the hull by {_hit(q, hull):.2f} mm3"
    assert _hit(A, B) < 0.01, f"{side} {tag}: the leaves touch ({_hit(A, B):.2f} mm3)"
    print(f"   {side} {tag}: hole {s["hole"][0]} x {s["hole"][1]} r {s["hole"][2]}, pocket {pk0}..{pk1} x {pz0:.2f}..{pz1:.2f}, {yp1 - DOOR_SKIN:.1f} deep behind {DOOR_SKIN} of skin, the wall padded to {yback:.1f}; "
          f"A {A.volume:.0f} mm3 (grooved), B {B.volume:.0f} mm3 (lugged), A alone for {gx1 - lx1:.1f} of its {s["travel"]} stroke; pad +{v1 - v0:.0f}, pocket -{v1 - v2:.0f}, hole -{v2 - hull.volume:.0f}", flush=True)
    return hull, (A, B), clear
