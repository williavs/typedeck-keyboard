"""The 2D plan's invariants: seconds, no CAD kernel booleans. Every number here is one the 3D build relies on."""
import math

import pytest
from shapely.geometry import Polygon, box as sbox

from velvet.geo import rect
from shapely.ops import unary_union

from velvet import params as P
from velvet.plan import flap_footprint, plan_of
from velvet.stand import seat_section, tent_angles
from velvet.assembly import kinematics


@pytest.fixture(scope="module", params=["R", "L"])
def side(request):
    return request.param


@pytest.fixture(scope="module")
def plan(side):
    return plan_of(side)


def test_every_part_has_its_wall(plan):
    thin = {n: plan["plan"].exterior.distance(f) for n, f in plan["foot"].items() if plan["plan"].exterior.distance(f) < P.WALL - 0.6}
    assert not thin, f"walls thinner than {P.WALL - 0.6}: {thin}"


def test_parts_inside_the_outline(plan):
    outside = [n for n, f in plan["foot"].items() if not plan["plan"].contains(f)]
    assert not outside


def test_tent_hinge_knuckles(plan):
    pitch = P.KNUCKLE + P.KN_CLR
    slots = plan["slots"]
    assert 4 <= len(slots) <= 8 and plan["hull_kn"] == slots[0::2] and plan["base_kn"] == slots[1::2]   # the run = the outer column's wells (4 since the rear Pico bay went, build 79; 7-8 before)
    assert all(math.isclose(b - a, pitch) for a, b in zip(slots, slots[1:]))
    assert slots[0] >= plan["y0"] - 1e-6 and slots[-1] + P.KNUCKLE <= plan["y1"] + 1e-6


def test_rest_hinge_on_the_same_axis(plan):
    """Three knuckles (hull, base, hull) on the tent axis, under the rest's widest point, clear of the main hinge's run."""
    rs = plan["rslots"]
    assert len(rs) == 3 and all(math.isclose(b - a, P.KNUCKLE + P.KN_CLR) for a, b in zip(rs, rs[1:]))
    assert rs[-1] + P.KNUCKLE < plan["y0"] - P.PIN_HEAD[1] - P.PIN_PROUD, "the rest hinge must stay in front of the main hinge's pin head"
    rest = plan["foot"]["rest"]
    leaf = plan["K"](sbox(plan["ax_x"] - 18.0, rs[0], plan["ax_x"] + P.TENT_R, rs[-1] + P.KNUCKLE))
    assert leaf.intersects(rest.buffer(P.WALL + 2.0)), "the leaf must reach the rest"


def test_stand_seats_bear_on_the_base(plan):
    """Each seat's nose line must find base under it for >= 100 mm of the flap's width (the front of the plate overhangs the rest, by design)."""
    slab = plan["plan"].intersection(plan["K"](sbox(-1000.0, -1000.0, plan["ax_x"], 1000.0)))
    fy = P.FLAP_Y["R"]
    for x in P.NOTCHES:
        foot = plan["K"](sbox(plan["ax_x"] - x - P.FLAP_NOSE - 0.1, fy - P.FLAP_W / 2, plan["ax_x"] - x + 12, fy + P.FLAP_W / 2))
        bearing = foot.intersection(slab.buffer(-4.0))
        assert bearing.area / (12 + P.FLAP_NOSE + 0.1) >= 100.0, f"seat {x}: only {bearing.area / 17.6:.0f} mm of bearing"


def test_flat_zone_holds_the_flat_parts(plan, side):
    flat = plan["flat"]
    for name in ("cell", "display" if side == "R" else "v713"):
        assert flat.contains(plan["foot"][name]), f"{name} is not inside the flat zone"


def test_flat_zone_clear_of_the_wells(plan):
    assert not plan["flat"].intersects(plan["wells"]), "the flat zone may not touch a key well"


def test_flat_zone_takes_everything_but_the_key_block(plan, side):
    """Behind the thumbs, every bit of the plan that is not the key block (wells + WALL closed 12, its convex hull, the deck in front of it)
    or a housing is FLAT. Build 63 left the outline's corner fills behind the cell tilting: patches standing 20 mm over the flat band."""
    from velvet.geo import tp_plan
    yp = min(f.position.Y for f in plan["fu"].values()) - 19.0
    from shapely import affinity
    housing = tp_plan(P.TP_CLR).buffer(P.WALL) if side == "R" else plan["foot"]["knob"].buffer(P.WALL)
    kb = unary_union([plan["wells"].buffer(P.WALL).buffer(12.0).buffer(-12.0), housing])
    kb = unary_union([kb, affinity.translate(kb, 0.0, -300.0)]).convex_hull
    due = plan["plan"].intersection(sbox(-1000.0, yp + 6.0, 1000.0, 1000.0)).difference(kb).difference(housing).difference(plan["wells"].buffer(0.5))
    due = due.difference(plan["palm_side"]).difference(plan["pit"])                            # the palm deck in front of the block is the wedge's (flat but not flat ZONE)
    left = due.difference(plan["flat"])
    assert left.area < 1.0, f"{left.area:.0f} mm2 behind the thumbs tilts without being key block: {[round(v, 1) for v in left.bounds]}"
    assert plan["plateau"].contains(plan["flat"].buffer(-0.01)), "the flat zone is not all plateau (a flat patch at deck level is a trench)"
    assert plan["plateau"].difference(plan["flat"]).difference(housing).difference(plan["palm_side"]).area < 0.5, "plateau outside the flat zone, the housing and the palm side: a raised patch on the tilted deck"


def test_seam_ring_has_no_hairlines(plan):
    """The seam wall's ring is prismed 200 tall and capped by the tilted top: a zero-area hairline in it (the flat zone's buffer meeting
    the outline, build 60) becomes a 0.03 mm blade 20 mm tall. Every piece is a real wall, and opening it 0.45 changes nothing."""
    ring = plan["ring"]
    pieces = [g.area for g in getattr(ring, "geoms", [ring])]
    assert min(pieces) >= 1.0, f"a {min(pieces):.4f} mm2 sliver in the seam ring"
    assert ring.buffer(-0.45, join_style=2).buffer(0.45, join_style=2).area >= 0.97 * ring.area, "the seam ring has hairline-thin stretches"


def test_parts_keep_their_distance(plan):
    """No part footprint overlaps another or a key well (the deck between them is what holds the top together)."""
    foot = plan["foot"]
    for a, b in (("cell", "wells"), ("pico", "wells")):
        assert foot[a].distance(foot[b]) >= 2.0, f"{a} to {b}: {foot[a].distance(foot[b]):.2f}"


def test_every_part_clear_of_the_wells(plan):
    """Every non-key part keeps >= 2 mm of deck to the nearest key well (the trackpad's 2.3 is the design minimum)."""
    foot = plan["foot"]
    for n, f in foot.items():
        if n not in ("wells", "rest", "palm", "flap"):
            assert f.distance(foot["wells"]) >= 2.0, f"{n} to the wells: {f.distance(foot['wells']):.2f}"


def test_flap_inside_the_outline(plan, side):
    """The whole stand (plate + ears) lies under the hull: nothing pokes out of the outline (the palm deck's lobe covers its front end)."""
    flap = plan["K"](flap_footprint(plan["ax_x"], P.FLAP_Y[side]))
    assert plan["plan"].buffer(-P.WALL).contains(flap)


def test_palm_deck_holds_the_rest(plan):
    """The palm deck carries the rest and stops WALL short of the TILTED wells (the pit is the tilted walls' foot, 1.7 behind the flat ones)."""
    from velvet.geo import tilt_shadow
    palm, rest = plan["foot"]["palm"], plan["foot"]["rest"]
    assert palm.contains(rest.buffer(-0.01))
    assert palm.distance(tilt_shadow(plan["wells"], plan["yp"], 0.0)) >= P.WALL - 0.2   # the tilted wall is WALL cos 10 in plan; the 4-segment round sits 0.1 inside its arc


def test_palm_side_wraps_the_thumb_arc(plan):
    """From every thumb key toward the arc's pivot the ray leaves the pit into palm deck, and stays deck for the ramp's whole reach: the pit's wall
    is the only thing between the deck and the wells. (Clipped straight at the tilt line, build 70 left 35 mm of bare tilted plate in front of the
    outer thumb and touched the inner one.)"""
    from shapely.geometry import LineString, Point
    palm_side, pit = plan["palm_side"], plan["pit"]
    piv = plan["K"](Point(*P.VL.THUMB_PIV))
    for k, f in plan["fu"].items():
        if "_t" in k:
            c = Point(f.position.X, f.position.Y)
            d = ((piv.x - c.x) / c.distance(piv), (piv.y - c.y) / c.distance(piv))
            ray = LineString([(c.x, c.y), (c.x + 60 * d[0], c.y + 60 * d[1])])
            hit = ray.intersection(pit.exterior)
            q0 = min(getattr(hit, "geoms", [hit]), key=lambda g: g.distance(c))          # where the ray leaves the pit
            band = LineString([(q0.x + 0.1 * d[0], q0.y + 0.1 * d[1]), (q0.x + P.DECK_FILL * d[0], q0.y + P.DECK_FILL * d[1])])
            assert band.difference(palm_side).length < 0.3, f"{k}: {band.difference(palm_side).length:.1f} mm of the ramp's reach in front of the pit is not deck"
            gap = LineString([(c.x, c.y), (q0.x, q0.y)]).difference(pit).difference(plan["wells"]).length
            assert gap < 0.3, f"{k}: {gap:.1f} mm between the well and the pit's wall is neither"
    assert not palm_side.intersects(pit.buffer(-0.01)) and not palm_side.intersects(plan["flat"].buffer(-0.01)) and not palm_side.intersects(plan["housing"].buffer(-0.01))


def test_front_is_a_wedge(plan, side):
    """The half's front is one trapezoid (Willy 07:37): the inner edge is the seam down to the knee then one straight REST_ANGLE diagonal to the
    front corner, TANGENT to the tightest wall (Willy 08:40 'bring the inner edge in closer') with every part's wall on the hull's side; the outer
    edge is straight on the hinge line from the bay's back to the front corner; the rest is that trapezoid inside the walls."""
    from shapely.geometry import LineString, Point
    from shapely.ops import unary_union
    import math
    ext = plan["plan"].exterior
    tn, knee = math.tan(math.radians(P.REST_ANGLE[side])), plan["knee"]
    line = LineString([(0.0, knee), ((knee + 1000.0) * tn, -1000.0)])
    walls = unary_union([g.buffer(P.WALL, join_style=1, quad_segs=4) for g in plan["foot"].values()] + [plan["block"], plan["housing"]])
    assert line.distance(walls) < 0.05, f"the diagonal is {line.distance(walls):.2f} off the nearest wall: not tangent"
    assert all(x + 0.05 >= (knee - y) * tn for g in getattr(walls, "geoms", [walls]) for x, y in g.exterior.coords), "a wall crosses the diagonal"
    end = min((Point(c) for c in plan["wedge"].exterior.coords if line.distance(Point(c)) < 0.01), key=lambda q: q.y)   # the front-inner corner
    for s in range(8, int(knee - end.y) - 8, 4):                                                  # the diagonal (s = the drop below the knee), off its rounded ends
        q = Point(s * tn, knee - s)
        assert ext.distance(q) < 0.3, f"the inner diagonal is {ext.distance(q):.1f} off the outline at {q.wkt}"
    for y in range(int(knee) + 8, int(plan["housing"].bounds[3]) - 6, 5):                        # the seam above the knee
        assert ext.distance(Point(0.0, y)) < 0.3, f"the inner edge leaves the seam at y {y}"
    xo = (P.VL.COLS - 1) * P.VL.U + 9.12 + P.MARGIN
    for y in range(int(P.REST_FRONT) + 6, int(plan["y1"]) - 2, 5):                               # the outer edge, K: the front corner to the tent rail's back
        q = plan["K"](Point(xo + P.WALL, y))
        assert ext.distance(q) < 0.3, f"the outer edge leaves the hinge line at K y {y}"
    rest = plan["foot"]["rest"]
    assert rest.convex_hull.area < 1.01 * rest.area, "the rest is not convex (a trapezoid with rounded corners)"


def test_pico_bay_in_the_deck(plan, side):
    """The Pico bay (Willy 10:13 / 10:23 / 10:48): in the palm deck along K x as close to the thumb keys as the keys allow, its tunnel reaching the
    outer wall's inner face, 2.5+ clear of the pit (the key block's tilted walls) and the housing, inside the walls; the window through the outer
    wall between the two hinges; the pad stops BORDER_W (the border) off the pane's recess and off the deck face at the keys."""
    bay, pit, pad, housing = plan["foot"]["pico"], plan["pit"], plan["pad"], plan["housing"]
    assert plan["inner"].buffer(0.05).contains(bay)
    assert plan["inner"].exterior.distance(bay) < 0.1, "the tunnel's end is off the outer wall"
    for name, g in (("pit", pit), ("housing", housing)):
        assert g.distance(bay) >= 3.5, f"the bay is {g.distance(bay):.2f} from the {name}"
    assert plan["palm_side"].buffer(0.05).contains(bay), "the bay is not all palm deck"
    recess = plan["K"](plan["recess_k"])
    assert plan["rest_full"].contains(recess) and pit.distance(recess) >= 2.9, f"the pane's recess is {pit.distance(recess):.2f} from the keys"
    assert pad.buffer(0.05).contains(recess), "the pad does not cover the pane's recess (160: the hull cuts the window, the D and the tongue out of it)"   # the pad wraps the pane on three sides (the keys' side is deck: the band is too shallow for a rim there)
    assert abs(pad.distance(pit) - P.BORDER_W) < 0.05, f"the pad stops {pad.distance(pit):.2f} from the keys' deck face (the border: {P.BORDER_W})"
    assert pad.area > 0.6 * plan["rest_full"].area
    yc = (plan["pico_k"].bounds[1] + plan["pico_k"].bounds[3]) / 2.0                               # the plug window's centre along the hinge line, K
    assert plan["rslots"][-1] + P.KNUCKLE + 6.0 < yc < plan["y0"] - 6.0, f"the plug window (K y {yc:.1f}) meets a hinge"


def test_flap_ears_under_the_hatch(plan):
    assert plan["hatch_plan"].contains(plan["ear2"])
    assert plan["hatch_plan"].exterior.distance(plan["ear2"]) >= 1.5


def test_plateau_keeps_every_part(plan, side):
    for name, f in plan["foot"].items():
        if name not in ("wells", "flap"):
            assert plan["plateau"].contains(f.buffer(-0.5)), f"{name} is not on the plateau"


def test_tent_angles_descend_with_the_notch():
    ta = tent_angles()
    angles = [th for _, th, *_ in ta]
    assert angles == sorted(angles, reverse=True)
    assert 10.0 < angles[-1] and angles[0] < 35.0
    for _x, th, ph, _lean, _c in ta:
        assert 0.0 < ph < 150.0, "the flap angle must be within its joint's limits"


def test_stop_faces_are_not_undercut():
    """A seat's vertical stop face must keep >= 4 mm of height after the next seat's ramp is cut (the nose would ride over it)."""
    ta = tent_angles()
    secs = [Polygon(seat_section(x, lean)) for x, _th, _ph, lean, _c in ta]
    for i in range(len(secs) - 1):
        xs, zf = seat_section(ta[i][0], ta[i][3])[0]
        zt = seat_section(ta[i][0], ta[i][3])[1][1]
        from shapely.geometry import LineString
        face = LineString([(xs, zf), (xs, zt)]).difference(secs[i + 1].buffer(0.01))
        assert face.length >= 4.0, f"seat {ta[i][0]}: stop face {face.length:.1f} mm after seat {ta[i + 1][0]}'s ramp"


def test_kinematics_every_parent_is_the_root():
    k = kinematics()
    for m in k["mates"]:
        d = m if isinstance(m, dict) else m.__dict__
        assert str(d.get("parent")) == "#velvet_case", d
    assert "flat" in k["poses"] and len(k["poses"]) == 1 + len(P.NOTCHES)


def test_r_charger_clear_of_the_usb_path(plan, side):
    """R: the V713 bay (cut with its fence + 0.2) stays 0.2 inside the HAT's rear-outer column and 0.3 off an 11 mm plug's path to the Pi's OTG port;
    (the back-wall rib flutes and their rib-free panel are gone, Willy 12:33)."""
    if side != "R":
        return
    from shapely.geometry import Point
    cx, cy = P.F2H.position.X + P.HAT_HOLES_F[3][0], P.F2H.position.Y + P.HAT_HOLES_F[3][1]
    col = Point(cx, cy).buffer(P.POST_D / 2)
    bay = plan["v713"].buffer(P.V713_FENCE + 0.2, join_style=2)
    assert bay.distance(col) >= 0.2, bay.distance(col)
    ux = P.F2H.position.X + P.PI_USB_F
    path = sbox(ux - 5.5, 36.0, ux + 5.5, 80.0)
    assert path.distance(plan["v713"].buffer(P.V713_FENCE, join_style=2)) >= 0.3
    assert plan["plan"].contains(path.intersection(sbox(-1000, -1000, 1000, plan["foot"]["display"].bounds[3])))


def test_console_block_is_flat(plan, side):
    """The housing (L: the knob's; R: the trackpad's) is in the flat zone, one piece with the block behind it (the gap closed), and the plateau holds
    no sliver of housing outside the flat zone (build 85's fins)."""
    from shapely.geometry import Point
    flat, housing = plan["flat"], plan["housing"]
    assert flat.contains(housing.buffer(-0.01))
    v = plan["foot"]["v713"] if side == "L" else plan["foot"]["display"]
    between = Point(min((housing.bounds[0] + housing.bounds[2]) / 2, (v.bounds[0] + v.bounds[2]) / 2) + 2.0, (housing.bounds[3] + v.bounds[1]) / 2)
    assert flat.contains(between), f"the gap between the housing and the block behind it is not flat at {between}"
    assert plan["plateau"].difference(flat).difference(plan["palm_side"]).difference(sbox(-1.0, -1000.0, 0.6, 1000.0)).area < 0.01   # (less the plateau's snap strip to the seam face, build 145)
    assert "palm_tuck" not in plan


def test_ring_up_and_low_split(plan):
    """The seam ring splits into where the tilted deck stands above the r2 round's foot (no round) and where it is at least STEP_C below the rim (chamfered)."""
    up, low = plan["ring_up"], plan["ring_low"]
    assert up.area > 10 and low.area > 10 and up.intersection(low).area < 0.01
    assert up.bounds[1] > low.bounds[3] - 0.01


def test_back_zone_covers_the_back_run(plan):
    """The big round's zone holds the plateau's whole back run (build 85: R's band was clipped out of it) and no point of the seam wall's top edge
    below the corner."""
    from shapely.geometry import Point
    run = plan["plateau"].exterior.intersection(plan["back_zone"].buffer(-P.BACK_R - 1.0))
    assert run.length > 100.0, f"the back zone holds only {run.length:.0f} of the plateau's outline"
    yc = min(y for x, y in plan["plan"].exterior.coords if x < 0.6)                                 # the seam wall runs from here up to the back corner
    assert not plan["back_zone"].intersects(Point(1.5, yc + 5.0))



def test_pad_in_its_tray(plan, side):
    """THE BORDER (Willy 22:46 'i wanted like 2mm of border a little raised around the rests'): the pad is the palm side BORDER_W inside the outline
    (its front corners concentric with the hull's), BORDER_W off the pit and the pane's recess, CUT_OFF into the housing's / console's wall foot;
    nothing of the palm side outside those bands is uncovered but necks thinner than 2 (the opening); the TPU 0.2 inside the pad. The border
    is everything else on the palm side (the bands, the landings), never over a wall or the pit."""
    from shapely.geometry import Point
    pad, outline, palm_side, pit = plan["pad"], plan["plan"], plan["palm_side"], plan["pit"]
    recess = plan["K"](plan["recess_k"])
    edge = outline.exterior.intersection(palm_side.buffer(-3.0, join_style=2).buffer(3.5, join_style=2))
    near = edge.intersection(pad.buffer(P.BORDER_W + 1.0))                                                                       # the outline run the pad follows (L's pocket inside the
    assert near.length > 0.7 * edge.length, f"the pad follows only {near.length:.0f} of the palm side's {edge.length:.0f} outline run"   # thumb arc is border, not pad)
    qs = [ln.interpolate(f, normalized=True) for ln in getattr(near, "geoms", [near]) for f in [i / 40 for i in range(1, 40)]]   # not the run's ends, nor the pit's
    ds = [pad.distance(q) for q in qs if pit.distance(q) > P.BORDER_W + 1.2]                                                      # corner at the wall: the pad's r 1 corners
    assert sum(abs(d - P.BORDER_W) < 0.05 for d in ds) >= 0.9 * len(ds), f"the pad's edge is {sorted(set(round(d, 2) for d in ds))[:8]} from the outline (BORDER_W {P.BORDER_W})"
    assert pad.geom_type == "Polygon" and pad.area > 400.0, f"pad pieces {[round(g.area) for g in getattr(pad, 'geoms', [pad])]} (ONE TPU per side)"
    assert palm_side.buffer(P.CUT_OFF + 0.02, join_style=2).contains(pad), "the pad goes past CUT_OFF into a wall"
    for name in ("housing", "flat"):
        assert pad.intersection(plan[name]).area < 0.3 * plan[name].boundary.length, f"the pad is more than CUT_OFF into the {name}"
    assert abs(pad.distance(pit) - P.BORDER_W) < 0.05 and pad.buffer(0.05).contains(recess), f"the pad is {pad.distance(pit):.2f} off the pit; it covers the pane's recess: {pad.buffer(0.05).contains(recess)} (160)"
    missing = palm_side.buffer(-0.02).difference(pad).difference(outline.exterior.buffer(P.BORDER_W + 0.05)).difference(recess.buffer(P.BORDER_W + 0.05)).difference(pit.buffer(P.BORDER_W + 0.05))
    necks = [g for g in getattr(missing, "geoms", [missing]) if g.area > 0.5]
    assert all(g.buffer(-1.0).is_empty or g.area < 400.0 for g in necks), f"palm side not pad, wider than 2 and over 400 mm2 at {[[round(v) for v in g.bounds] for g in necks if not g.buffer(-1.0).is_empty and g.area >= 400.0]}"   # (no pad islands under 400 mm2: the pane's key side, L's pocket inside the thumb arc)
    main = max(getattr(pad, "geoms", [pad]), key=lambda g: g.area)
    b = main.bounds
    for near in ((b[0], b[1]), (b[2], b[1])):                                                   # the two front corners: the hull's corner arc's centre
        v = min(outline.exterior.coords, key=lambda q: math.dist(q, near))
        pts = [q for q in outline.exterior.coords if math.dist(q, v) < 8.0]
        (x1, y1), (x2, y2), (x3, y3) = pts[0], pts[len(pts) // 2], pts[-1]
        a = x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2
        bb = (x1 * x1 + y1 * y1) * (y3 - y2) + (x2 * x2 + y2 * y2) * (y1 - y3) + (x3 * x3 + y3 * y3) * (y2 - y1)
        cc = (x1 * x1 + y1 * y1) * (x2 - x3) + (x2 * x2 + y2 * y2) * (x3 - x1) + (x3 * x3 + y3 * y3) * (x1 - x2)
        c = Point(-bb / (2 * a), -cc / (2 * a))
        assert abs(c.distance(outline.exterior) - P.ROUND) < 0.1, f"the hull's corner at {v} is not an r {P.ROUND} arc"
        assert abs(c.distance(main.exterior) - (P.ROUND - P.BORDER_W)) < 0.15, f"the pad's corner at {v} is {c.distance(main.exterior):.2f} from the hull's corner centre (concentric: {P.ROUND - P.BORDER_W})"
    tpu, border = plan["pad_tpu"], plan["border"]
    assert abs(tpu.distance(pad.exterior) - 0.2) < 0.02 and pad.contains(tpu), f"the TPU is {tpu.distance(pad.exterior):.2f} inside the pad's edge (0.2)"
    want = palm_side.difference(pad)                                                                                           # the border is EVERYTHING on the palm side that is not
    assert border.symmetric_difference(want).area < 0.5, "the border is not the palm side less the pad and the pane's recess"   # pad or the pane's recess
    assert border.intersection(plan["housing"]).area < 0.01 and border.intersection(pit).area < 0.01, "the border runs over a wall or the pit"
    assert outline.exterior.distance(border) < 0.01 and border.distance(pit) < 0.01 and border.distance(recess) > 1.0, "the border does not reach the outline, the pit; or it comes within 1 of the pane's recess (inside the pad since 160)"
    assert not pad.buffer(P.BORDER_W - 0.05).contains(border.buffer(-0.01)), "the border is nowhere wider than BORDER_W"   # (160: the landings round the pane are pad now; the hull's tongue is case)   # (L's thumb side, the pane's key side)


def test_keywell(plan, side):
    """THE KEYWELL (Willy 22:46 'this deep channel', 23:13 'the same coverage ... on the left side'): the key block's wall top -- the pit's margin
    between the deck face / the seam ring / the housings' walls and the caps -- is filled to the border's level all round the block, except beside
    the pitched console, at the housings' / the flat zone's own footprint and along the outer wall; it reaches 0.3 into the palm side (fuses with
    the deck face), stays CAP_CLR off every cap's tilted shadow and out of the wells (between the caps)."""
    from velvet.geo import tilt_shadow
    well, pit, palm_side, yp = plan["keywell"], plan["pit"], plan["palm_side"], plan["yp"]
    assert well.area > 100, f"the keywell is {well.area:.0f} mm2"
    assert pit.buffer(0.31, join_style=2).union(plan["ring_low"].buffer(0.01)).contains(well), "the keywell leaves the pit (and the ring's strip to a block's chamfer foot)"
    skip = [plan["housing"].buffer(0.5), plan["flat"].buffer(0.5), plan["plan"].exterior.buffer(P.WALL + 0.6)] + ([plan["tp_zone"].buffer(0.6)] if plan["tp_zone"] is not None else [])
    if side == "L":
        skip.append(plan["bay"].buffer(0.5))                                              # THE ENCODER BAY (builds 150-151): the floor east of the bay line -- the silo's
        #                                                                                   flank and the flat zone's wall north of it stand on the plate, no fill
    qs = [pit.exterior.interpolate(i / 240, normalized=True) for i in range(240)]
    due = [q for q in qs if not any(z.contains(q) for z in skip)]
    miss = [q for q in due if not well.buffer(0.35).contains(q)]
    assert len(due) > 60 and len(miss) <= 0.05 * len(due), f"the keywell misses {len(miss)} of {len(due)} pit samples it should hug, at {[(round(q.x), round(q.y)) for q in miss[:6]]}"
    for z in (0.0, 5.0, 10.0):
        caps = tilt_shadow(plan["caps"], yp, z)
        assert well.distance(caps) >= P.CAP_CLR - 0.02, f"the keywell is {well.distance(caps):.2f} from a cap's shadow at {z} over the plate (CAP_CLR {P.CAP_CLR})"
        inside = tilt_shadow(plan["wells"], yp, z).buffer(-P.MARGIN - 0.02)
        assert well.intersection(inside).area < 0.01, "the keywell enters a well (between the caps)"
    assert well.intersection(plan["housing"]).area < 0.01 and well.intersection(plan["flat"]).area < 0.01 and well.distance(plan["foot"]["pico"]) > 1.0, "the keywell enters the housing, the flat zone or the Pico bay"
    if plan["tp_zone"] is not None:
        assert well.difference(sbox(-1000.0, -1000.0, 1000.0, plan["tp_front_y"] + 1.0)).distance(plan["tp_zone"]) > 0.4, "the keywell runs beside the pitched console's key-side edge"   # (along its FRONT the fuse band touches it: flush, no chamfer there)
        front = well.intersection(sbox(plan["tp_zone"].bounds[0], plan["tp_front_y"] - 3.0, plan["tp_zone"].bounds[2], plan["tp_front_y"] + 1.0))   # ... and along the console's FRONT the fill
        assert front.area > 15.0 and front.distance(plan["flat"]) < 0.02, f"the keywell stops short of the console's front face ({front.area:.0f} mm2 along it, {front.distance(plan['flat']):.2f} off)"   # runs to the face (Willy 06:21)   # (16.6 since build 147's one-rectangle-per-column keep-out; was 20.x)


def test_bay_line(plan, side):
    """THE ENCODER BAY, TAKE THREE (build 151; Willy 07:46 'remove the incline ... smooth out that second line so there isnt a lip coming out - just
    smooth straightish'): on L the open pit's edge runs STRAIGHT from the first thumb cap's keep-out corner to the silo's wall (within 0.35 of
    the bay line, no lip east of it), meets the wall at 45..80 deg, and north of there the floor runs along the silo's flank and the flat zone's
    wall to where the plate rises through the keywell's ceiling -- no fill between column 0 and the flat zone (150's incline)."""
    if side != "L":
        return
    from shapely.geometry import LineString, Point
    (px, py), (qx, qy), psi = plan["bay_line"]
    o, well = plan["open_pit"], plan["keywell"]
    line = LineString([(px, py), (qx, qy)])
    assert 45.0 <= psi <= 80.0 and line.length > 8.0
    for i in range(2, 10):                                                                            # the line's middle: the pit's edge ON it (one straight edge)
        s = line.interpolate(i / 10.0, normalized=True)
        assert o.exterior.distance(s) < 0.35, f"the open pit's edge is {o.exterior.distance(s):.2f} off the bay line at ({s.x:.1f}, {s.y:.1f})"
    ux, uy = (qx - px) / line.length, (qy - py) / line.length                                         # ... and no vertex of it EAST of the line (the lip): the signed distance
    bump = [q for q in o.exterior.coords if py + 1.0 < q[1] < qy and -(q[0] - px) * uy + (q[1] - py) * ux < -0.35 and line.distance(Point(q)) < 5.0]
    assert not bump, f"the open pit's edge bulges east of the bay line at {[(round(q[0], 2), round(q[1], 2)) for q in bump]}"
    assert o.exterior.distance(Point(qx, qy)) < 0.5, "the open pit's edge does not reach the silo's wall at the line's end"
    mid = Point((px + qx) / 2 + 1.0, (py + qy) / 2)                                                     # 1 east of the line's middle: floor
    assert o.covers(mid) and not well.covers(mid), "the floor east of the bay line is not open"
    stub = sbox(plan["housing"].bounds[2] + 0.6, -10.0, 42.5, 6.0)                                       # between the silo / the flat zone and column 0, north of the silo
    assert well.intersection(stub).area < 0.5, f"{well.intersection(stub).area:.1f} mm2 of fill north of the silo (the incline)"
    assert o.intersection(stub).area > 0.9 * stub.area, "the floor north of the silo is not open"


def test_console_flat(plan, side):
    """R (Willy 15:34 'raise the trackpad bezel and raised area to be flush with pitft bezel'; build 117): the trackpad console is one flat plane
    at RIM_Z + CONSOLE_UP, the bezel's brim top -- no pitch (builds 97-116 pitched it to the pad's surface + TP_LIP at the front); the zone holds
    the housing and reaches the seam; the console's front edge is IN step_low (a chamfered step to the landing). L: no console."""
    if side == "L":
        assert plan["tp_zone"] is None and plan["tp_pitch"] == 0.0
        return
    housing = plan["housing"]
    assert plan["tp_pitch"] == 0.0, f"the console's pitch is {plan['tp_pitch']:.2f} deg"
    front = plan["tp_front_y"]
    assert plan["step_low"].intersects(sbox(housing.bounds[0], front - 0.5, housing.bounds[2], front + 0.5)), "the console's front edge is not in step_low (no chamfer)"
    assert plan["tp_zone"].contains(housing.buffer(-0.01)) and plan["tp_zone"].bounds[0] < 0.0, "the pitched zone does not cover the housing to the seam"
    beside = plan["flat"].intersection(sbox(housing.bounds[2], front - 1.0, 1000.0, P.Y_P0 + 1.0))                              # the seam ring beside the console
    assert beside.area > 50 and plan["tp_zone"].contains(beside.buffer(-0.01)) and abs(plan["tp_zone"].bounds[2] - beside.bounds[2] - P.CUT_OFF) < 0.01, \
        f"the pitched zone stops at x {plan['tp_zone'].bounds[2]:.2f}; the flat zone beside the console runs to {beside.bounds[2]:.2f} (a fin: builds 97-99)"
    # (the corner fill at the knee, x 0..16, is lifted with the console: its front edge steps 5.2 to the landing too and is chamfered with it)


def test_lip_plan(plan, side):
    """THE LIP (build 117): one polygon along the key block's outer wall from the pit's corner to the back band, 3+ wide (the outline to CAP_CLR off
    the pinky column), ON the outline, part of the keywell (filled to the border's level)."""
    from shapely.geometry import Point
    lip = plan["lip"]; xc, yc = plan["pit_corner"]
    assert lip.geom_type == "Polygon" and lip.area > 150.0, f"{side}: the lip is {lip.area:.0f} mm2, {lip.geom_type}"
    assert plan["keywell"].buffer(0.01).contains(lip)
    assert lip.bounds[1] < yc + 1.0 and lip.bounds[3] > 0.0, f"{side}: the lip runs y {lip.bounds[1]:.1f}..{lip.bounds[3]:.1f} (the pit's corner at {yc:.1f})"
    assert lip.distance(plan["plan"].exterior) < 0.01
    run = plan["plan"].exterior.intersection(lip.buffer(0.1, join_style=2)); run = max(getattr(run, "geoms", [run]), key=lambda g: g.length)
    for i in range(1, 10):
        q = run.interpolate(i / 10, normalized=True); w = 0.0
        while w < 8.0 and lip.buffer(0.02).contains(Point(q.x - w - 0.05, q.y)):
            w += 0.05
        assert w >= 3.0, f"{side}: the lip is {w:.2f} wide at y {q.y:.1f}"


def test_border_round(side):
    """THE BORDER'S ROUND (Willy 09:03): the palm side's outline run is ONE piece (one fillet chain), the round is no wider than the border (its tangent
    line is the recess wall), the pad never reaches where the border's level is capped at KEYWELL_MAX (the flush TPU's top follows the plane alone),
    and the keep's straightened hinge face lies within 0.1 of the plan, never inside it (a keep clipped to the plan leaves no step)."""
    from shapely.geometry import Point
    from velvet.geo import hinge_straight
    plan = plan_of(side)
    run = plan["plan"].exterior.intersection(plan["palm_side"].buffer(0.1, join_style=2))
    pieces = sorted((g.length for g in getattr(run, "geoms", [run])), reverse=True)
    assert pieces[0] > 250.0 and (len(pieces) == 1 or pieces[1] < 5.0), f"the palm side's outline run is {len(pieces)} pieces: {[round(v, 1) for v in pieces]}"
    assert P.BORDER_R <= P.BORDER_W, "the round is wider than the border"
    yb = plan["foot"]["rest"].bounds[3]
    y_cap = yb + (P.KEYWELL_MAX - P.BORDER_UP - (P.RIM_Z - P.TILT_DROP)) / math.tan(math.radians(P.REST_SLOPE))
    assert plan["pad"].bounds[3] < y_cap - 1.0, f"the pad reaches y {plan['pad'].bounds[3]:.1f}, past the border's cap at y {y_cap:.1f}: the flush TPU would stand over the border"
    straight = hinge_straight(plan["plan"])
    pts = list(plan["plan"].exterior.coords)
    assert len(pts) - len(straight.exterior.coords) <= 10
    assert max(straight.exterior.distance(Point(q)) for q in pts) < 0.1
    assert straight.covers(plan["plan"].buffer(-0.01)), "the straightened hinge face cuts inside the plan"


def test_no_ring_sliver_at_the_palm_side(side):
    """The seam ring never leaves a sliver against the palm side (L's 5 mm2 wedge at the knee by the knob housing stood as a slot in the inner wall's
    top, build 107): every ring piece is at least 20 mm2 or clear of the palm side by 0.1."""
    plan = plan_of(side)
    for g in getattr(plan["ring"], "geoms", [plan["ring"]]):
        assert g.area >= 20.0 or g.distance(plan["palm_side"]) >= 0.1, f"a {g.area:.1f} mm2 ring piece at {[round(v) for v in g.bounds]} touches the palm side"
    assert plan["ring"].distance(plan["palm_side"]) >= -0.01




def test_back_chain(plan):
    """The back run as a G1 chain of lines and true arcs (velvet.backline, build 118): tangent at every junction, no arc under r 3 (the
    radius law eases to r - 0.5 there), a handful of elements, and the plan's outline in the zone IS the chain (sampled at 1.1)."""
    from shapely.geometry import LineString, Point, box as sbox
    from velvet.backline import BACK_X0, BACK_Y0, R_MIN, kinks
    chain = plan["back_chain"]
    assert kinks(chain) == [], f"kinks in the back chain: {kinks(chain)}"
    arcs = [e for e in chain if e.kind == "arc"]
    assert arcs and min(e.r for e in arcs) >= R_MIN
    assert 8 <= len(chain) <= 30, f"the chain has {len(chain)} elements"
    sampled = LineString([q for e in chain for q in e.samples(0.5)] + [chain[0].p0])
    zone = sbox(BACK_X0 + 0.5, BACK_Y0 + 0.5, 1e4, 1e4)
    far = [q for q in plan["plan"].exterior.coords if zone.contains(Point(q)) and sampled.distance(Point(q)) > 0.05]
    assert not far, f"{len(far)} outline vertices in the zone off the chain by > 0.05, e.g. {far[:3]}"


def test_radius_law(plan):
    """The back round's radius law (velvet.backline, builds 119-120): the corners the round runs through are opened to OPEN_R (their walls
    bulged out by under 1 mm, rejoined by lane-change biarcs), so the law is BACK_R along the whole run -- no dip, no ease -- and the only
    arc under OPEN_R left is the seam corner, which the round never reaches (build 119 eased 6 -> 3.5 through r 4 corners: a pinch)."""
    from shapely.geometry import LineString, Point, box as sbox
    from velvet.backline import EASE, TIGHT_R, radius_law
    from velvet.params import BACK_R
    chain = plan["back_chain"]
    r, spans = radius_law(chain)
    seam = sbox(plan["plan"].bounds[2] - BACK_R - 2.0, -1e4, 1e4, 1e4)
    tight = [e for e in chain if e.kind == "arc" and e.r < TIGHT_R and not e.box]
    assert all(seam.contains(Point(e.mid)) for e in tight), f"corners under {TIGHT_R} off the seam: {[e for e in tight if not seam.contains(Point(e.mid))]}"
    eased = seam.buffer(EASE + 1.0)
    for e, (s0, s1) in zip(chain, spans):
        if not e.box and not eased.intersects(LineString([e.p0, e.p1])):
            assert all(abs(r(s0 + (s1 - s0) * f) - BACK_R) < 1e-9 for f in (0.0, 0.5, 1.0)), f"the law dips on {e}"


def test_flat_zone_has_no_spikes(plan, side):
    """The flat zone and the plateau carry no zero-width spikes or zero-area slits (a 0.01 closing changes nothing): the plateau's edge band
    followed a 9 mm spike into L's flat zone as an r 1.8 groove (Willy 07:07 'a random dip ... north of the encoder knob', build 124)."""
    for k in ("flat", "plateau", "palm_side"):
        g = plan[k]
        closed = g.buffer(0.01, join_style=2).buffer(-0.01, join_style=2)
        n0, n1 = sum(len(q.exterior.coords) for q in getattr(g, "geoms", [g])), sum(len(q.exterior.coords) for q in getattr(closed, "geoms", [closed]))
        assert n0 == n1, f"{side} {k}: a 0.01 closing drops {n0 - n1} outline vertices (zero-width spikes)"
        assert not any(q.interiors for q in getattr(g, "geoms", [g])), f"{side} {k}: interior rings {[round(r.area, 3) for q in getattr(g, 'geoms', [g]) for r in map(__import__('shapely.geometry', fromlist=['Polygon']).Polygon, q.interiors)]}"
