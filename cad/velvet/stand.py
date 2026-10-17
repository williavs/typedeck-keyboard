"""Tenting: the hinge pin, the flap (stand), its sawtooth seats, the tent angles they give, the base slab, the joints' axes."""
import math

from shapely.geometry import box as sbox
from build123d import Align, Box, Cone, Cylinder, Polyline, Pos, Rot, extrude, make_face, Compound

from velvet.params import *  # noqa: F401,F403
from velvet.geo import band, prism, rounded_prism


def pin(d, length, head, flat, nose, ring=PIN_RING):
    """A printed hinge pin along +y (cyber_split's): chamfered head, shaft, a snap ring, tapered nose, a D-flat on -z (prints flat on it)."""
    hd, ht = head
    q = Rot(-90, 0, 0) * Cone(hd / 2 - 0.6, hd / 2, 0.6, align=BIG)
    q += Pos(0, 0.59, 0) * Rot(-90, 0, 0) * Cylinder(hd / 2, ht - 0.59, align=BIG)
    q += Pos(0, ht - 0.01, 0) * Rot(-90, 0, 0) * Cylinder(d / 2, length - ht - nose + 0.02, align=BIG)
    q += Pos(0, length - nose, 0) * Rot(-90, 0, 0) * Cone(d / 2, d / 4, nose, align=BIG)
    rh, rl = ring                                                                                # THE SNAP RING, 0.5 behind the nose's base: 45 deg on, a flat, 45 deg off
    if rh > 0:
        y_r = length - nose - 0.5 - rl
        q += Pos(0, y_r, 0) * Rot(-90, 0, 0) * Cone(d / 2, d / 2 + rh, rh, align=BIG)
        q += Pos(0, y_r + rh - 0.01, 0) * Rot(-90, 0, 0) * Cylinder(d / 2 + rh, rl - 2 * rh + 0.02, align=BIG)
        q += Pos(0, y_r + rl - rh, 0) * Rot(-90, 0, 0) * Cone(d / 2 + rh, d / 2, rh, align=BIG)
    q -= Pos(0, length / 2, -(d / 2 - flat) - hd / 2) * Box(hd + 2, length + 2, hd, align=MID)
    return q


RING_BACK = PIN_NOSE + 0.5 + PIN_RING[1] / 2   # the ring's centre back from the pin's tip
HEAD_OUT = 8.0                                 # the head envelope runs this far out from the seat: past the face, the case's surface ends it
Z_FLAT = PIN_D / 2 - PIN_FLAT                  # the D-flat's plane under the axis (the shank's and the head's)


def shank(length, ring=PIN_RING):
    """A pin WITHOUT its head, along +y from the seat (y = 0) to the nose tip (y = length): the shaft, the snap ring, the tapered nose, the D-flat on -z.
    The head is the case's own form (hull.hinges / build_base: the caps), fused on in assembly (build 131)."""
    q = Pos(0, -0.01, 0) * Rot(-90, 0, 0) * Cylinder(PIN_D / 2, length - PIN_NOSE + 0.02, align=BIG)
    q += Pos(0, length - PIN_NOSE, 0) * Rot(-90, 0, 0) * Cone(PIN_D / 2, PIN_D / 4, PIN_NOSE, align=BIG)
    rh, rl = ring
    if rh > 0:
        y_r = length - PIN_NOSE - 0.5 - rl
        q += Pos(0, y_r, 0) * Rot(-90, 0, 0) * Cone(PIN_D / 2, PIN_D / 2 + rh, rh, align=BIG)
        q += Pos(0, y_r + rh - 0.01, 0) * Rot(-90, 0, 0) * Cylinder(PIN_D / 2 + rh, rl - 2 * rh + 0.02, align=BIG)
        q += Pos(0, y_r + rl - rh, 0) * Rot(-90, 0, 0) * Cone(PIN_D / 2 + rh, PIN_D / 2, rh, align=BIG)
    q -= Pos(0, length / 2, -Z_FLAT - PIN_HEAD[0] / 2) * Box(PIN_HEAD[0] + 2, length + 4, PIN_HEAD[0], align=MID)
    return q


def head_env(seat, sense, x, z, clr=0.0):
    """The head's envelope on the axis (x, z): a Ø PIN_HEAD[0] (+ 2 clr) cylinder from the seat HEAD_OUT along `sense` (+1 / -1 in y), its D-flat
    Z_FLAT (+ clr) under the axis. clr 0 = the head itself (the case's material inside it); 0.2 = its room."""
    r = PIN_HEAD[0] / 2 + clr
    q = Pos(x, seat, z) * Rot(-90 * sense, 0, 0) * Cylinder(r, HEAD_OUT, align=BIG)
    return q - Pos(x, seat + sense * HEAD_OUT / 2, z - Z_FLAT - clr - r) * Box(2 * r + 2, HEAD_OUT + 2, 2 * r, align=MID)


def back_at_axis(p):
    """K y of the hull's back face on the hinge axis (the tent pin's head ends there)."""
    from shapely import affinity
    from shapely.geometry import LineString
    line = p["K"](LineString([(p["ax_x"], -1000.0), (p["ax_x"], 1000.0)]))
    seg = line.intersection(p["plan"])
    return affinity.rotate(affinity.translate(seg, -p["xin"], 0), -YAW, origin=(0, 0)).bounds[3]      # (plan.half_plan's Ki, inlined: a local lambda there)


def tent_pin_len(p):
    """The SHANK: from the seat (PIN_HEAD[1] inside the back face, on the axis) to the nose 0.5 inside the first knuckle (build 131: the head is the
    case's form from the seat out to the face; before, it sat 4.5 inside the face in a hole)."""
    return back_at_axis(p) - PIN_HEAD[1] - (p["y0"] + 0.5)


def rest_pin_len(p):
    """The SHANK: from the seat (PIN_HEAD[1] inside the FRONT face) to the nose 0.5 inside the last rest knuckle."""
    return p["rslots"][-1] + KNUCKLE - 0.5 - (REST_FRONT + PIN_HEAD[1])


def ring_relief(y_tip, toward, x=0.0, z=0.0):
    """The relief for the snap ring in the knuckle a pin's tip ends in: a short wider bore on the axis (x, z) centred RING_BACK behind the tip
    (toward = the pin's +y sense: +1 / -1). Build 125 left it at (0, y, 0): a stray 5.85 x 2 cylinder at the plate and the ring in a plain bore."""
    return Pos(x, y_tip - toward * RING_BACK, z) * Rot(90, 0, 0) * Cylinder(PIN_BORE / 2 + PIN_RING[0], PIN_RELIEF, align=MID)


def pin_coupon(rh):
    """A fit coupon for the snap ring at ring height rh: a block with the PIN_BORE through it along y and the relief at its far end, and a stub
    pin with the ring, side by side on the bed (the pin flat on its D, the bore horizontal as the knuckles print). Print one per rh, pick the click."""
    blk = Pos(0, 0, 6.0) * Box(14.0, 18.0, 12.0, align=MID)
    blk -= Pos(0, 0, 6.0) * Rot(90, 0, 0) * Cylinder(PIN_BORE / 2, 20.0, align=MID)
    blk -= Pos(0, 0, 6.0) * ring_relief(9.0 - 0.5, 1)
    stub = Pos(16.0, -11.0, PIN_D / 2 - PIN_FLAT) * pin(PIN_D, 9.0 - 0.5 + 11.0, PIN_HEAD, PIN_FLAT, PIN_NOSE, (rh, PIN_RING[1]))
    return blk + stub



def tent_angles():
    """Per notch: (x, tent deg, flap deg from folded, the plate's lean from vertical toward the hinge, nose centre (h, z) off the axis).
    Section through the hinge axis, h toward the hinge, z up. R(a) is the joints' sense (+ lifts the inner edge / swings the flap down):
    (h, z) -> (h cos a + z sin a, -h sin a + z cos a). The flap's nose, in its own frame, sits at (S, E) from the pivot; the nose centre
    rests r over the seat's floor: solve the tent where |nose - pivot| fits, then the flap angle that points it there."""
    R = lambda a, v: (v[0] * math.cos(a) + v[1] * math.sin(a), -v[0] * math.sin(a) + v[1] * math.cos(a))
    piv = (-FLAP_P, -(AX_UP + PIV_DROP))
    L = math.hypot(FLAP_S, FLAP_E)
    cz = -(AX_UP + NOTCH_D) + FLAP_NOSE
    out = []
    for x in NOTCHES:
        gap = lambda th: math.dist(R(th, piv), (-x, cz)) - L
        lo, hi = 0.0, math.radians(60)
        for _ in range(60):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if gap(mid) < 0 else (lo, mid)
        th = (lo + hi) / 2
        pt = R(th, piv)
        ang = lambda v: math.atan2(v[1], v[0])
        ph = (ang((FLAP_S, FLAP_E)) - ang((-x - pt[0], cz - pt[1])) - th) % (2 * math.pi)   # R turns clockwise: angle(R(a)v) = angle(v) - a
        out.append((x, math.degrees(th), math.degrees(ph), math.degrees(th + ph) - 90.0, (-x, cz)))
    return out


def seat_section(x, t):
    """One seat's section as 4 points (a, z): a = distance from the hinge axis along the base, z = height over the axis.
    The stop face (vertical, just past the nose) from the floor to the top, then the ramp (parallel to the flap's lower face,
    NOTCH_CLR off it) back down to the floor: [(stop, floor), (stop, top), (ramp top), (ramp floor)]."""
    r, c, tr = FLAP_NOSE, NOTCH_CLR, math.radians(t)
    zc = -(AX_UP + NOTCH_D) + r
    zf, zt = zc - r, -AX_UP + 1.0
    xa = lambda z: x - (r + c) * math.cos(tr) - math.sin(tr) * (z - zc + (r + c) * math.sin(tr)) / math.cos(tr)   # the ramp line
    xs = x + r + 0.1
    return [(xs, zf), (xs, zt), (xa(zt), zt), (xa(zf), zf)]


def seat(x, t, ax_x, fy, z_ax):
    """One sawtooth seat in K: the section (away from the hinge, z off the axis) = a vertical stop face just past the nose, the floor
    under it, and a ramp parallel to the flap's lower face (NOTCH_CLR off it) up to the top -- the nose drops in from above, the
    load (down + away from the hinge) goes into the floor and the stop face, and every face prints without support."""
    sec = seat_section(x, t)
    c = NOTCH_CLR
    w = FLAP_W + 2 * c + 0.6
    face = make_face(Polyline(*[(ax_x - a, fy - w / 2, z_ax + z) for a, z in sec], close=True))
    return extrude(face, amount=w, dir=(0, 1, 0))


def tent_axes(side, p):
    """The tent and flap joints in WORLD: (origin, direction); + lifts the inner edge / raises the flap. Locations only, no booleans."""
    s = 1 if side == "R" else -1
    Kf = Pos(p["xin"], 0, 0) * Rot(0, 0, YAW)
    W = Pos(s * SEP / 2, 0, -ZB + BASE_T)
    ax_o = (Kf * Pos(p["ax_x"], p["y0"], ZB + AX_UP)).position
    fl_o = (Kf * Pos(p["ax_x"] - FLAP_P, FLAP_Y[side], ZB - PIV_DROP)).position
    ky = (Kf * Pos(0, 1, 0)).position - Kf.position                                 # K's y axis in the u frame
    o = (W * Pos(s * ax_o.X, ax_o.Y, ax_o.Z)).position
    fo = (W * Pos(s * fl_o.X, fl_o.Y, fl_o.Z)).position
    return dict(tent=((o.X, o.Y, o.Z), (ky.X, s * ky.Y, 0.0)), flap=((fo.X, fo.Y, fo.Z), (ky.X, s * ky.Y, 0.0)))   # the flap swings DOWN with +, same sense as the tent


def build_base(side, p, Kf):
    """The tenting base in the half's frame: the half's outline up to the hinge axis, 5 thick under the hull; its knuckles, coves for
    the hull's, the pin bore; the flap's recess (pointing at the hinge), its pivot bore, grooves for the hatch's ridges at tent 0."""
    ax_x, z_ax, fy = p["ax_x"], ZB + AX_UP, FLAP_Y[side]
    slab = p["plan"].intersection(p["K"](sbox(-1000.0, -1000.0, ax_x, 1000.0)))
    pad = p["K"](sbox(-1000.0, fy - FLAP_W / 2 - BASE_PAD, ax_x, fy + FLAP_W / 2 + BASE_PAD)).intersection(slab)   # the pad spans the base's whole width: anchored to the axis it
    #                                                                                                       ended 30 short of R's hinge-end rim (R is the console wider) and
    #                                                                                                       left a window L never had (Willy 22:27 'why is the plate different on each side')
    base = prism(slab, ZB - BASE_T, ZB) - prism(slab.buffer(-BASE_RIM, join_style=1, quad_segs=4), ZB - BASE_T - 1, ZB + 1) + prism(pad, ZB - BASE_T, ZB)   # a rim frame + the stand's pad
    keep = Kf * Pos(ax_x - 20.0, 0, ZB - BASE_T - 1.0) * Box(100.0, 1000.0, 100.0, align=(Align.MIN, Align.CENTER, Align.MIN))   # the hinge side stays square
    for z0, z1, rt, rb in ((ZB - EDGE_R_TOP - 0.5, ZB + 0.5, EDGE_R_TOP, 0.0), (ZB - BASE_T - 0.5, ZB - BASE_T + EDGE_R_BOT + 0.5, 0.0, EDGE_R_BOT)):   # ROUNDED outline edges, top + bottom
        bb = band(slab, z0, z1, (rt or rb) + 0.5) - keep                                   # the band's inner wall 0.5 PAST the round's tangent line: ON it, OCC's common
        piece = rounded_prism(slab, ZB - BASE_T, ZB, r_top=rt, r_bot=rb) & bb               # with the rounded prism came back EMPTY with no error and the plates lost their
        plain = (prism(slab, ZB - BASE_T, ZB) & bb).volume                                 # bands as 1.5-1.8 x 2-2.3 rebates: R's bottom, L's top and bottom (R's top happened
        assert piece is not None and 0.75 * plain < piece.volume < 0.98 * plain, \
            f"{side}: the base's {'top' if rt else 'bottom'} round swap holds {0 if piece is None else piece.volume:.0f} of the band's {plain:.0f} mm3"   # to work), builds 108-122; Willy 23:40 'one
        base = (base - bb) + piece                                                           # has curved edge and other has straight'. ONE fillet face per edge (build 108)
        assert base.is_valid and len(base.solids()) == 1, f"{side}: the base's {'top' if rt else 'bottom'} round left it {'invalid' if not base.is_valid else str(len(base.solids())) + ' solids'}"
        print(f"   {side} BASE {'TOP' if rt else 'BOTTOM'} ROUND r {rt or rb}: the band {plain:.0f} -> {piece.volume:.0f} mm3")
    for y in p["base_kn"]:
        base += Kf * Pos(ax_x, y + KNUCKLE / 2, z_ax) * Rot(90, 0, 0) * Cylinder(TENT_R, KNUCKLE, align=MID)
    for y in p["hull_kn"]:
        base -= Kf * Pos(ax_x, y + KNUCKLE / 2, z_ax) * Rot(90, 0, 0) * Cylinder(TENT_R + TENT_CLR, KNUCKLE + KN_CLR, align=MID)
    seat_t = back_at_axis(p) - PIN_HEAD[1]                                                   # THE HEADS' SEATS, PIN_HEAD[1] inside the end faces (build 131)
    caps = {"tpin": base & (Kf * head_env(seat_t, 1, ax_x, z_ax))}                            # the base's part of the tent pin's head: its own form, taken before the room
    base -= Kf * head_env(seat_t, 1, ax_x, z_ax, clr=0.2)                                     # the head's room, D-keyed
    rs = p["rslots"]
    r0, r1 = rs[0], rs[-1] + KNUCKLE
    base += prism(p["K"](sbox(ax_x - 18.0, r0, ax_x, r1)), ZB - BASE_T, ZB)                   # the rest hinge's base leaf
    base += Kf * Pos(ax_x, rs[1] + KNUCKLE / 2, z_ax) * Rot(90, 0, 0) * Cylinder(TENT_R, KNUCKLE, align=MID)
    for y in (rs[0], rs[2]):
        base -= Kf * Pos(ax_x, y + KNUCKLE / 2, z_ax) * Rot(90, 0, 0) * Cylinder(TENT_R + TENT_CLR, KNUCKLE + KN_CLR, align=MID)
    seat_r = REST_FRONT + PIN_HEAD[1]
    caps["rpin"] = base & (Kf * head_env(seat_r, -1, ax_x, z_ax))                             # the base's part of the rest pin's head
    base -= Kf * head_env(seat_r, -1, ax_x, z_ax, clr=0.2)
    ya, yb = p["y0"] - 0.5, p["y1"] + ROUND + 6.0                                                                                        # the tent pin's bore through the base's knuckles
    base -= Kf * Pos(ax_x, (ya + yb) / 2, z_ax) * Rot(90, 0, 0) * Cylinder(PIN_BORE / 2, yb - ya, align=MID)
    ya, yb = REST_FRONT - 6.0, r1                                                                                                       # the rest pin's (build 127: no through channel)
    base -= Kf * Pos(ax_x, (ya + yb) / 2, z_ax) * Rot(90, 0, 0) * Cylinder(PIN_BORE / 2, yb - ya, align=MID)
    px = ax_x - FLAP_P
    rec = p["K"](sbox(px, fy - FLAP_W / 2 - 0.4, px + FLAP_S + FLAP_NOSE + 0.5, fy + FLAP_W / 2 + 0.4))   # the folded plate
    base -= prism(rec, ZB - PIV_DROP + FLAP_E - FLAP_T / 2 - 0.3, ZB + 1.0)
    ew = FLAP_W / 2 + EAR_GAP + EAR[1] + 0.4
    base -= prism(p["K"](sbox(px - EAR[0] / 2 - 0.4, fy - ew, px + EAR[0] / 2 + 0.4, fy + ew)), ZB - BASE_T - 1.0, ZB + 1.0)   # the barrel + ears: a window through
    for x, th, ph, t, _c in tent_angles():                      # the sawtooth seats
        sk = seat(x, t, ax_x, fy, z_ax)
        foot = p["K"](sbox(ax_x - x - FLAP_NOSE - 0.1, fy - FLAP_W / 2 - 0.5, ax_x - x + 12, fy + FLAP_W / 2 + 0.5))   # the seat's true extent: stop face .. ramp top
        print(f"   seat {x:.0f} inside the base plan: {p['plan'].contains(foot)} (wall to the edge {p['plan'].exterior.distance(foot):.1f})")
        base -= Kf * sk
    return base, caps


def flap():
    """The stand in its own frame: the pivot on the origin, a barrel round it, the plate along +x (toward the hinge when folded) FLAP_E
    over the pivot (its top flush with the barrel's), a full-round nose, the pin bore. Prints on its side (the profile on the bed)."""
    f = Pos(0, 0, FLAP_E) * Box(FLAP_S, FLAP_W, FLAP_T, align=(Align.MIN, Align.CENTER, Align.CENTER))
    f += Rot(90, 0, 0) * Cylinder(FLAP_RB, FLAP_W, align=MID)
    f += Pos(FLAP_S, 0, FLAP_E) * Rot(90, 0, 0) * Cylinder(FLAP_NOSE, FLAP_W, align=MID)
    f -= Rot(90, 0, 0) * Cylinder((FLAP_PIN_D + FLAP_PIN_CLR) / 2, FLAP_W + 2, align=MID)
    (hd, ht), eng, nose, (rh, rl), flat = FLAP_STUB
    for sy in (-1, 1):                                                                       # (164) the stubs' snap-ring grooves in the barrel's bore
        f -= Pos(0, sy * (FLAP_W / 2 - eng + nose + 0.5 + rl / 2), 0) * Rot(90, 0, 0) * Cylinder(FLAP_PIN_D / 2 + rh + FLAP_PIN_CLR / 2, rl + 0.4, align=MID)
    return f


def flap_stub():
    """(164) A flap stub pin along +y from its head (y 0, flush with the ear's outer face) to the nose: through the ear and the gap, FLAP_STUB[1] into the barrel."""
    (hd, ht), eng, nose, ring, flat = FLAP_STUB
    return pin(FLAP_PIN_D, EAR[1] + EAR_GAP + eng, (hd, ht), flat, nose, ring)


def flap_stubs():
    """The two stubs in the flap's frame (the pivot on the origin, the barrel along y), each from its ear's outer face inward."""
    y_out = FLAP_W / 2 + EAR_GAP + EAR[1]
    return Compound(children=[Pos(0, -y_out, 0) * flap_stub(), Pos(0, y_out, 0) * Rot(0, 0, 180) * flap_stub()])
