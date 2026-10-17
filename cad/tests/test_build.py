"""The built model: static clashes are 0, parts seat where they should. Minutes: run with --build."""
import json
from pathlib import Path

import pytest

from shapely.ops import unary_union

from velvet import params as P
from velvet.geo import _hit, well_top

pytestmark = pytest.mark.build
GOLDEN = Path(__file__).with_name("golden.json")


def _parts(A):
    out = {}
    for k in "RL":
        h = A["halves"][k]
        for name, val in h.items():
            if name not in ("keys", "local", "W", "tent_axis", "flap_axis") and hasattr(val, "volume"):
                out[f"{k}.{name}"] = val
        for kk, sw, cap in h["keys"]:
            out[f"{k}.key.{kk}.sw"], out[f"{k}.key.{kk}.cap"] = sw, cap
    return out


OURS = ("hull", "hatch", "base", "flap", "fpin", "tpin", "rpin", "bezel", "pdoor_a", "pdoor_b", "cdoor_a", "cdoor_b", "picover", "pane", "tpu", "knob", "pad", "v713", "ledbezel", "crown", "overlay", "guide_cyan", "guide_magenta")   # printed, cut or boxed by us (vendor STEPs may report invalid to OCC: the Pico does)


def test_every_part_of_ours_is_valid(model):
    bad = [n for n, s in _parts(model).items() if n.split(".")[1] in OURS and not s.is_valid]
    assert not bad


def test_hull_is_one_solid(model):
    for k in "RL":
        assert len(model["halves"][k]["hull"].solids()) == 1


@pytest.mark.parametrize("a,b", [("hull", "hatch"), ("pico", "hull"), ("pico", "hatch"), ("pane", "hull"), ("cell", "hull"), ("cell", "hatch"),
                                 ("v713", "hull"), ("v713", "hatch"), ("tpu", "hull"), ("base", "hull"), ("base", "hatch"), ("flap", "hull"), ("flap", "base"),
                                 ("tpin", "hull"), ("tpin", "base"), ("rpin", "hull"), ("rpin", "base"), ("fpin", "hatch"), ("fpin", "base"),
                                 ("crown", "hull"), ("guide_cyan", "hull"), ("guide_magenta", "hull"), ("guide_cyan", "crown"), ("guide_magenta", "crown"), ("fil_cyan", "guide_cyan"),
                                 ("fil_magenta", "guide_magenta"), ("fil_cyan", "hull"), ("fil_magenta", "hull"), ("ledbezel", "hull"), ("ledbezel", "crown"),   # THE LIGHTS (152), THE CROWN (153)
                                 ("overlay", "crown"), ("overlay", "hull"), ("overlay", "pad"), ("pad", "crown"), ("crown", "hat"), ("crown", "pi"), ("crown", "cover"),
                                 ("picover", "pi"), ("picover", "hat"), ("picover", "bezel")])   # (162: the OTG cover; its press into the hull is tested apart)
def test_no_static_clash(model, a, b):
    for k in "RL":
        h = model["halves"][k]
        if h.get(a) is None or h.get(b) is None:                                                 # R-only parts (the diffuser, the overlay, the stack)
            continue
        limit = 7.0 if k == "R" and {a, b} == {"crown", "hull"} else 0.01                    # R's bezel body (part of the crown since 153) presses its BZ_RIBS crush ribs
        assert _hit(h[a], h[b]) < limit, f"{k}: {a} vs {b}: {_hit(h[a], h[b]):.2f}"          # 0.2 into the display pocket's walls by design: 11 x 0.58 mm3 (build 155's chain)


def test_r_stack_and_trackpad(model):
    h = model["halves"]["R"]
    for a in ("pi", "hat", "header", "pad"):   # (161: no socket -- plain headers soldered into the PiTFT)
        assert _hit(h[a], h["hull"]) < 0.01, f"R: {a} vs hull"
    ribs = P.BZ_RIBS * P.RIB_W * (P.RIB_H - P.BZ_CLR) * P.GLASS_SINK                        # the bezel's crush ribs on all four faces: a press fit, this much interference by design
    assert abs(_hit(h["bezel"], h["hull"]) - ribs) < 0.1, f"R: bezel vs hull {_hit(h['bezel'], h['hull']):.2f}, the ribs alone would be {ribs:.2f}"
    assert _hit(h["bezel"], h["hat"]) < 0.01 and _hit(h["pad"], h["hatch"]) < 0.01


def test_l_knob(model):
    h = model["halves"]["L"]
    assert _hit(h["ec11"], h["hull"]) < 0.01 and _hit(h["knob"], h["hull"]) < 0.01 and _hit(h["knob"], h["ec11"]) < 0.01


def test_caps_clear_the_hull(model):
    for k in "RL":
        h = model["halves"][k]
        caps = [c for _, _, c in h["keys"]]
        assert _hit(caps[0].fuse(*caps[1:]), h["hull"]) < 0.01


def test_switches_seat_in_the_deck(model):
    """Each switch's clip tabs sit in the 1.2 lip: the overlap equals the tabs' volume (0.85) on every key."""
    for k in "RL":
        h = model["halves"][k]
        vals = [_hit(sw, h["hull"]) for _, sw, _ in h["keys"]]
        assert max(vals) - min(vals) < 0.02 and 0.5 < min(vals) < 1.2, (k, min(vals), max(vals))


def test_nothing_outside_the_outline(model):
    """The hull lives inside its outline: nothing of it stands outside the plan (past 0.05 of numeric fuzz) except the rest hinge's leaf and
    knuckles at the outer face. Builds 58-61 had the tilted body's lean out there: 0.35 mm lips along the flat walls' backs (128 mm3 at the
    display bay, attached) and a 0.2 mm skin 13 mm tall along the Pico bay's outer wall (detached and silently dropped in 58, attached in 59+)."""
    from build123d import Plane, Pos, mirror
    from shapely.geometry import box as sbox
    from velvet.geo import prism
    from velvet.plan import plan_of
    for k in "RL":
        p = plan_of(k)
        r0, r1 = p["rslots"][0] - 2.0, p["rslots"][-1] + P.KNUCKLE + 2.0
        keep = unary_union([p["plan"].buffer(0.05, join_style=2), p["K"](sbox(p["ax_x"] - 20.0, r0, p["ax_x"] + P.TENT_R + 1.0, r1))])   # (plan + 8) - keep as TWO simple prisms:
        #                                                                                           prism() takes a polygon's exterior only; the holed annulus measured the whole hull (builds 72-78)
        s = 1 if k == "R" else -1
        W = Pos(s * P.SEP / 2, 0, -P.ZB + P.BASE_T)
        mir = (lambda q: q) if k == "R" else (lambda q: mirror(q, about=Plane.YZ))
        hull = model["halves"][k]["hull"]
        c = p["plan"].representative_point()
        inside = _hit(hull, W * mir(prism(sbox(c.x - 10.0, c.y - 10.0, c.x + 10.0, c.y + 10.0), P.ZB, P.RIM_Z)))   # (a prism 0.05 inside the walls was flaky: OCC's common
        assert inside > 500.0, f"{k}: the probe misses the hull ({inside:.0f} mm3): wrong frame"                 #  with near-coincident faces, R 0 / L fine, build 117)
        out = _hit(hull, W * mir(prism(p["plan"].buffer(8.0, join_style=2), P.ZB - 1.0, 200.0))) - _hit(hull, W * mir(prism(keep, P.ZB - 1.0, 200.0)))
        assert out < 0.5, f"{k}: {out:.2f} mm3 of hull outside the outline"


def test_seam_wall_foot_is_solid(model):
    """The 2 mm of plateau along the seam wall's foot is solid from RIM_Z - 2.2 up: the plateau's edge round must not run along the wall
    (build 64 had a 2 mm rounded groove there, the strip 47 / 64 % solid)."""
    from build123d import Plane, Pos, mirror
    from velvet.geo import prism
    from velvet.plan import plan_of
    for k in "RL":
        p = plan_of(k)
        parts = unary_union([g for n, g in p["foot"].items() if n not in ("wells", "rest", "flap", "palm")])
        strip = p["flat"].intersection(p["ring"].buffer(2.0, join_style=2)).difference(p["wells"].buffer(3.0)).difference(parts.buffer(4.0, join_style=2))   # the parts' own pockets (bezel, pry notch, cell contacts) are voids by design
        strip = strip.difference(p["step_low"].buffer(P.STEP_C + 1.0, join_style=2))   # where the deck is BELOW the wall's foot the edge is chamfered STEP_C by design (Willy 15:23)
        strip = strip.difference(p["corner"].buffer(0.5))                                      # THE RIM STRIP (build 140): the plateau's corner behind P / Q is cut to the plate by design
        strip = strip.difference(p["plan"].exterior.buffer(1.5))                          # not the outline's own edge: the back round lives there (and the rib flutes did, builds 84-115) the back round live there (build 88: a flute at the pinky's top read as a 3.9 mm3 groove)
        s = 1 if k == "R" else -1
        W = Pos(s * P.SEP / 2, 0, -P.ZB + P.BASE_T)
        mir = (lambda q: q) if k == "R" else (lambda q: mirror(q, about=Plane.YZ))
        probe = W * mir(prism(strip, P.RIM_Z - 2.2, P.RIM_Z - 0.1))
        full = sum(q.volume for q in probe.solids())
        got = _hit(model["halves"][k]["hull"], probe)
        assert got >= 0.97 * full, f"{k}: the plateau along the seam wall's foot is {100 * got / full:.0f}% solid (a groove)"


def test_seam_wall_tops_at_the_tilted_deck(model):
    """Within the seam ring's footprint (less the keywell, which fills the ring's tilt-shadow strip to the border's level by design) nothing stands
    higher than the tilted DECK at that y (+ 0.6): the ring is capped by the tilted top, which is the deck everywhere round the key block. Build 65
    had a hairline of tilted plateau there reaching 24 where the deck is 14."""
    from build123d import Plane, Pos, Rot, mirror
    from shapely.geometry import box as sbox
    from velvet.geo import prism
    from velvet.plan import plan_of
    for k in "RL":
        p, h = plan_of(k), model["halves"][k]
        yp = min(f.position.Y for f in p["fu"].values()) - 19.0
        T = Pos(0, 0, -P.TILT_DROP) * Pos(0, yp, P.RIM_Z) * Rot(P.KEY_TILT, 0, 0) * Pos(0, -yp, -P.RIM_Z)   # the tilt, as Half.tilt builds it
        ring = p["ring"]
        s = 1 if k == "R" else -1
        W = Pos(s * P.SEP / 2, 0, -P.ZB + P.BASE_T)
        mir = (lambda q: q) if k == "R" else (lambda q: mirror(q, about=Plane.YZ))
        from velvet.geo import tp_plan
        housing = tp_plan(P.TP_CLR).buffer(P.WALL + 0.5) if k == "R" else p["foot"]["knob"].buffer(P.WALL + 0.5)   # its plateau rides on the tilted deck, legitimately higher
        y0, y1 = ring.bounds[1], ring.bounds[3]
        y = y0
        while y < y1:
            piece = ring.intersection(sbox(-1000.0, y, 1000.0, y + 5.0)).difference(housing).difference(p["keywell"].buffer(0.05)).difference(p["swell"].buffer(0.05))   # (the keywell / the swell fills the ring's shadow strip by design)
            if piece.area > 1.0:
                z_deck = (T * Pos(0, min(y + 5.0, y1), 0)).position.Z                 # the tilted deck's top at the slice's back edge
                over = _hit(h["hull"], W * mir(prism(piece, z_deck + 0.6, 200.0)))
                assert over < 0.5, f"{k}: {over:.2f} mm3 stands over the seam wall at y {y:.0f}..{y + 5:.0f} (deck there {z_deck:.1f})"   # (< 0.5: build 125 read 0.12, a hairline
                #                                                                                     x 68.5..70.7, y 1.3..2.9 along the flat's corner edge in the ring_low -- the step chamfer's standoff, item 64)
            y += 5.0


def test_deck_meets_the_plate(model):
    """From every thumb key out through the pit's edge: 1.5 inside the pit the hull's top is the KEYWELL's fill (the border's level, BORDER_UP over
    the rest plane; the tilted PLATE where the fill stops short), 1 mm outside it the BORDER's top (the same level; no ramp, Willy 10:13), past the border
    (BORDER_W + 0.6) the pad's recess floor PAD_DEPTH under the plane where the pad covers, and the deck is solid at its face (a column 1 out
    from the edge reaches the plate)."""
    import math
    from build123d import Align, Box, Pos
    from shapely.geometry import LineString, Point
    from velvet.checks import wall_drop
    from velvet.plan import plan_of
    a = math.radians(P.KEY_TILT)
    for k in "RL":
        p, h = plan_of(k), model["halves"][k]
        yp, sx = p["yp"], (1.0 if k == "R" else -1.0)
        W = model["halves"][k]["W"]
        piv = p["K"](Point(*P.VL.THUMB_PIV))
        yb = p["foot"]["rest"].bounds[3]
        for name, f in p["fu"].items():
            if "_t" not in name:
                continue
            c = Point(f.position.X, f.position.Y)
            d = ((piv.x - c.x) / c.distance(piv), (piv.y - c.y) / c.distance(piv))
            hit = LineString([(c.x, c.y), (c.x + 60 * d[0], c.y + 60 * d[1])]).intersection(p["pit"].exterior)
            q0 = min(getattr(hit, "geoms", [hit]), key=lambda g: g.distance(c))
            def col(s_):
                x, y = q0.x + s_ * d[0], q0.y + s_ * d[1]
                q = h["hull"] & Pos(sx * (x + P.SEP / 2), y, 0) * Box(0.4, 0.4, 400, align=(Align.CENTER, Align.CENTER, Align.CENTER))
                sol = q.solids() if q is not None else []
                return (max(s.bounding_box().max.Z for s in sol) - W.position.Z, min(s.bounding_box().min.Z for s in sol) - W.position.Z, y) if sol else (None, None, y)
            at = lambda s_: Point(q0.x + s_ * d[0], q0.y + s_ * d[1])
            rest = lambda y_: P.RIM_Z - P.TILT_DROP + (y_ - yb) * math.tan(math.radians(P.REST_SLOPE))
            top_in, _, y_in = col(-1.5)
            plate = (y_in - yp - P.RIM_Z * math.sin(a)) * math.tan(a) + P.RIM_Z * (1 - math.cos(a)) - P.TILT_DROP
            want = (well_top(p, y_in) - wall_drop(p, at(-1.5), 0.4)) if p["keywell"].contains(at(-1.5)) else plate   # the fill's top (its three ceilings, build 148) less the key wall's round (build 147)
            assert top_in is not None and abs(top_in - want) < 0.5, f"{k} {name}: 1.5 inside the pit the top is {top_in} vs {want:.2f} ({'the keywell' if want != plate else 'the plate'})"
            top_out, bot_out, y_out = col(1.0)
            deck = min(rest(y_out) + P.BORDER_UP, P.KEYWELL_MAX) if p["border"].contains(at(1.0)) else rest(y_out)
            assert not p["pad"].contains(at(1.0)), f"{k} {name}: the pad reaches 1 mm from the pit (the border is {P.BORDER_W})"
            assert top_out is not None and abs(top_out - deck) < 0.5, f"{k} {name}: 1 mm out of the pit the top is {top_out} vs {deck:.2f} (the border)"
            top_pad, _, y_pad = col(P.BORDER_W + 0.6)
            if p["pad"].contains(at(P.BORDER_W + 0.6)):
                floor_ = P.RIM_Z - P.TILT_DROP + (y_pad - yb) * math.tan(math.radians(P.REST_SLOPE)) - P.PAD_DEPTH
                assert top_pad is not None and abs(top_pad - floor_) < 0.5, f"{k} {name}: past the rim the top is {top_pad} vs the recess floor {floor_:.2f}"
            plate_out = (y_out - yp - P.RIM_Z * math.sin(a)) * math.tan(a) + P.RIM_Z * (1 - math.cos(a)) - P.TILT_DROP
            assert bot_out < plate_out + 0.6, f"{k} {name}: the deck's face is hollow: material only from {bot_out:.2f}, the plate at {plate_out:.2f}"


def test_pi_otg_oval(model):
    """R (155, the door dropped in 161): the Pi's OTG port faces the back wall's oval; an 11 x 7 plug overmold runs from the port's face out through
    it touching nothing; a box 0.65 of the oval's axes is open through the wall."""
    from build123d import Align, Box, Pos
    h = model["halves"]["R"]
    ux, uy, uz = h["local"]["pi_usb"]
    wx, wy, wz = h["local"]["usb_win"]
    assert abs(ux - wx) < 0.01 and abs(uz - wz) < 0.01, (ux, uz, wx, wz)
    L = wy + P.WALL / 2 + 6.0 - uy
    path = h["W"] * Pos(ux, uy + L / 2, uz) * Box(11.0, L, 7.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    for name in ("hull", "hat", "hatch", "v713", "bezel", "pi"):
        assert _hit(path, h[name]) < 0.01, f"the plug path hits {name}: {_hit(path, h[name]):.2f}"
    assert h["local"]["pi_door"] is None
    inner = h["W"] * Pos(ux, wy + P.WALL / 2 - 10.0, uz) * Box(0.65 * P.PI_OVAL[0], 20.0, 0.65 * P.PI_OVAL[1], align=(Align.CENTER, Align.CENTER, Align.CENTER))   # inside the oval (an ellipse holds 0.71 of its axes)
    assert _hit(inner, h["hull"]) < 0.01, _hit(inner, h["hull"])
    crush = 4 * P.RIB_W * (P.WALL - 0.5) * (P.RIB_H - P.COVER_CLR)                                     # THE OTG COVER (162): four crush ribs pressed into the oval's wall
    assert 0.5 * crush < _hit(h["picover"], h["hull"]) < 1.5 * crush, f"the OTG cover presses {_hit(h['picover'], h['hull']):.2f} mm3 (four ribs: {crush:.2f})"
    assert h["picover"].is_valid and len(h["picover"].solids()) == 1
    for c in h["caps"]:                                                                              # THE BUTTON CAPS (162): valid, clear of the bezel and of the HAT (on its plungers)
        assert c.is_valid and len(c.solids()) == 1 and _hit(c, h["bezel"]) < 0.01 and _hit(c, h["hat"]) < 0.01, (_hit(c, h["bezel"]), _hit(c, h["hat"]))


def test_charge_doors(model):
    """Both halves (162): the charging window's two-leaf slide door -- leaves valid, clear of the hull and the hatch closed and open, an 11 x 7 plug path
    through the hole clear of the hull and covered by the closed leaves."""
    from build123d import Align, Box, Pos
    sp = P.CHG_SPEC
    for k in "RL":
        hl = model["halves"][k]["local"]
        A_, B_ = hl["chg_doors"]; Wf = hl["chg_Wf"]; Wi = Wf.inverse()
        lost = sp["groove"][1] - sp["lug"][1]
        oA, oB = Wf * Pos(-sp["travel"], 0, 0) * (Wi * A_), Wf * Pos(-(sp["travel"] - lost), 0, 0) * (Wi * B_)
        plug = Wf * Pos(0, 5.0, 0) * Box(11.0, 14.0, 7.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))   # W: +y inward; 2 outside the face to 12 in
        for n, q in (("A", A_), ("B", B_), ("A open", oA), ("B open", oB)):
            assert q.is_valid and len(q.solids()) == 1, f"{k}: charge door leaf {n} invalid"
            assert _hit(q, hl["hull"]) < 0.01 and _hit(q, hl["hatch"]) < 0.01, f"{k}: charge door leaf {n} hits the hull / hatch: {_hit(q, hl['hull']):.2f} / {_hit(q, hl['hatch']):.2f}"
        assert _hit(A_, B_) < 0.01 and _hit(oA, oB) < 0.01, f"{k}: the charge door's leaves touch"
        assert _hit(oA, plug) < 0.01 and _hit(oB, plug) < 0.01, f"{k}: the open charge door sits in the plug's way: {_hit(oA, plug):.2f} / {_hit(oB, plug):.2f}"
        assert _hit(A_, plug) > 20.0 and _hit(B_, plug) > 20.0, f"{k}: the closed charge door does not cover the hole"
        assert _hit(plug, hl["hull"]) < 0.01 and _hit(plug, hl["hatch"]) < 0.01, f"{k}: the charging plug path hits the hull / hatch: {_hit(plug, hl['hull']):.2f} / {_hit(plug, hl['hatch']):.2f}"


def test_sd_card_relief(model):
    """R: a microSD card standing 2 proud of the Pi's outer edge sits in the wall's relief."""
    from build123d import Align, Box, Pos
    h = model["halves"]["R"]
    sr = h["local"]["sd_relief"]
    card = h["W"] * Pos(sr[0] + (sr[1] - sr[0] - 0.5) / 2, (sr[2] + sr[3]) / 2, (sr[4] + sr[5]) / 2) * Box(sr[1] - sr[0] - 0.5, sr[3] - sr[2] - 2 * P.SD_RELIEF[1] + 0.2, 1.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    assert _hit(card, h["hull"]) < 0.01


def test_back_is_rounded(model):
    """The back run's top edge is a BACK_R round (OCC's evolutive fillet on the backline chain, easing only beside the tight corners): at the
    run's middle the hull's top 1 and 3 in from the outline sits on the R6 arc, 7 in it is the rim."""
    from shapely.geometry import Point
    from velvet.checks import back_profile, back_round_z
    from velvet.lights import foot_plan, nets
    from velvet.plan import plan_of
    for k in "RL":
        p, h = plan_of(k), model["halves"][k]
        grooves = unary_union([foot_plan(l) for l in nets(k, p["K"]).values()])                  # THE PLATEAU LIGHTS (build 152): 7 in sits in a groove
        tops, qs = back_profile(p, h["hull"], h["W"], k, pts=True)
        for d, t, q in zip((1.0, 3.0, 7.0), tops, qs):
            want = back_round_z(d) - (P.GROOVE_D if grooves.contains(Point(q)) else 0.0)
            assert t is not None and abs(t - want) < 0.3, f"{k}: {d} in from the back the top is {t}, the round says {want:.2f} ({'a groove' if want < back_round_z(d) else 'the rim'})"


def test_plate_fused_to_the_deck(model):
    """The key plate's front edge is one solid with the deck's face: 0.5 inside the pit's front the top solid runs the plate's full thickness down
    (before build 85 the plate's leaning face left a wedge under its top edge: the segment was 1.4 tall), and so across 0.5..2.5 inside; where
    the keywell sits on the plate the one solid runs from under the plate up to the border's level."""
    from shapely.geometry import Point
    from velvet.checks import column_tops, pit_crossing, pit_front_probes
    from velvet.geo import rest_z
    from velvet.plan import plan_of
    for k in "RL":
        h, p = model["halves"][k], plan_of(k)
        pts = pit_front_probes(p, h["local"]["bosses"])
        for q, segs in zip(pts, column_tops(h["hull"], h["W"], k, pts)):
            assert segs, f"{k}: no hull at {q}"
            if p["keywell"].contains(Point(q)):                                                   # THE KEYWELL over the plate: one solid from under the plate
                lvl = well_top(p, q[1])                                                           # up to the fill's top: the border's level, KEYWELL_MAX, the caps' ceiling (build 148)
                assert abs(segs[0][1] - lvl) < 0.5 and segs[0][1] - segs[0][0] >= 2.9, f"{k}: under the keywell the plate's edge segment is {segs[0]} (its top should be {lvl:.2f}, at least 2.9 tall)"
            else:
                assert 2.9 <= segs[0][1] - segs[0][0] <= 4.5, f"{k}: the plate's edge segment {segs}"   # 3.1 the plate; 3.8 at the thumb block's front, its wall under the plate; 5-13 = the deck's face grazed
        cross = pit_crossing(p, h["hull"], h["W"], k, h["local"]["bosses"])
        assert all(t >= 2.9 for t in cross), f"{k}: the plate across the pit's edge {cross} (a slot: build 87)"   # (thicker than the plate where the keywell sits on it)


def test_console_edge_is_one_round(model):
    """Where the console steps down (to the keys, to the palm deck: plan.step_low) its edge is ONE EDGE_R_TOP round like every other top edge
    (build 122; builds 90-121 cut a 45 deg STEP_C chamfer there, whose end at the console's front corner stood as a post): 0.4 in from the
    edge the top sits on the r 1.8 arc (top - 0.7), 1.2 in it is within 0.1 of the rim, 2 in it IS the rim (the keep's outline is grown 0.3
    and simplified 0.1-0.3: +- 0.3 along the edge). Builds 85-89 had a layered 4 mm round there, a bare stretch and a valley beside the knob."""
    import math
    from shapely.ops import nearest_points
    from velvet.checks import column_tops
    from velvet.plan import plan_of
    r = P.EDGE_R_TOP
    arc = lambda d: P.RIM_Z if d >= r else -r + math.sqrt(r * r - (r - d) ** 2)     # the round's drop below the top, d in from the edge (negative)
    for k in "RL":
        p, h = plan_of(k), model["halves"][k]
        edge = p["flat"].boundary.intersection(p["step_low"].buffer(-0.02))
        lines = sorted(getattr(edge, "geoms", [edge]), key=lambda g: -g.length)[:2]
        qs = [ln.interpolate(f, normalized=True) for ln in lines for f in (0.3, 0.5, 0.7)]
        ins = lambda q, d: nearest_points(p["flat"].buffer(-d, join_style=1).boundary, q)[0]
        top = lambda q: P.RIM_Z                                                               # (build 153: the console sits at the rim under THE CROWN, no lift, no raised badge)   # R's console is raised; L's badge is the housing's step edge (build 136)
        for d, tol in ((0.4, 0.35), (1.2, 0.15), (2.0, 0.1)):
            zs = [s[0][1] if s else None for s in column_tops(h["hull"], h["W"], k, [(ins(q, d).x, ins(q, d).y) for q in qs], w=0.2)]
            for v, q in zip(zs, qs):
                if "seat" in h["local"] and h["local"]["seat"].contains(ins(q, d)):              # under the brim's flange the hull is the seat (the rim): the strip's round beside
                    continue                                                                    # it is the keep's fillet's tail (build 143)
                if k == "L" and math.hypot(ins(q, d).x - P.KNOB_U, ins(q, d).y - P.KNOB_Y) < P.KNOB_BEZ_R + 0.3:   # THE RING'S SKIRT (build 154): inside KNOB_BEZ_R the silo ring's
                    continue                                                                    # top sits SILO_SINK under the rim, under the cover's ring
                want = top(q) + (arc(d) if d < r else 0.0)
                assert v is not None and abs(v - want) < tol, f"{k}: {d} in from the console's step edge at ({q.x:.0f},{q.y:.0f}) the top is {None if v is None else round(v, 2)}, the round says {want:.2f}"


def test_border_and_keywell(model):
    """THE BORDER (Willy 22:46 'like 2mm of border a little raised around the rests not this deep channel'; 09:03 'raise the tpu and get rid of that
    lip ... case needs to be rounded over there'): along the rest's outline run the border's top is one r BORDER_R round down from BORDER_UP over the
    rest plane (0.5 / 1 / 1.9 in from the case's face on the circle), BORDER_W + 0.6 in it is the pad's recess floor (-PAD_DEPTH) with the TPU's top
    FLUSH with the border (PAD_PROUD = BORDER_UP); along the pit's run in the palm side, 0.5 inside the pit the hull's top is the border's level too (THE KEYWELL: the slot filled,
    one surface from the TPU's edge to the caps) and 1 outside it the border's; the TPU's top is on the plane everywhere."""
    import math
    from shapely.ops import nearest_points
    from velvet.checks import column_tops, inward, round_profile, wall_drop
    from velvet.geo import rest_z, swell_z
    from velvet.plan import plan_of
    for k in "RL":
        p, h = plan_of(k), model["halves"][k]
        top = lambda part, q, w=0.2: (c[0][1] - rest_z(p, q.y)) if (c := column_tops(part, h["W"], k, [(q.x, q.y)], w=w)[0]) else None
        edge = p["plan"].exterior.intersection(p["palm_side"].buffer(-3.0, join_style=2).buffer(3.5, join_style=2))
        run = max(getattr(edge, "geoms", [edge]), key=lambda g: g.length)
        for f in (0.3, 0.5, 0.7):
            q = run.interpolate(f, normalized=True)
            ins = lambda d: nearest_points(p["plan"].buffer(-d, join_style=1).boundary, q)[0]
            s = math.tan(math.radians(P.REST_SLOPE)) * inward(p["plan"], q)[1]                      # the top's slope along the inward normal (the front wall: 95 deg dihedral)
            for d in (0.5, 1.0, P.BORDER_R - 0.1):                                                   # THE ROUND: the border's top d in from the face is on the rolling ball's section
                want = round_profile(d, s)                                                           # (a 0.05 column: the round climbs 1.6/mm at 0.5 in; the 0.2
                got = top(h["hull"], ins(d), 0.05)                                                     # column's far edge read 0.13 over the circle, verify105)
                assert got is not None and abs(got - want) < 0.1, f"{k}: {d} in from the face the hull's top is {got} over the rest plane (the border's round: {want:.2f})"
            if p["pad"].contains(ins(P.BORDER_W + 0.6)):
                got, tgot = top(h["hull"], ins(P.BORDER_W + 0.6)), top(h["tpu"], ins(P.BORDER_W + 0.6))
                assert got is not None and abs(got + P.PAD_DEPTH) < 0.1, f"{k}: inside the border the hull's top is {got} off the rest plane (the recess floor: -{P.PAD_DEPTH})"
                assert tgot is not None and abs(tgot - P.PAD_PROUD) < 0.05, f"{k}: inside the border the TPU's top is {tgot} off the rest plane"
        pit = p["pit"]
        prun = pit.exterior.intersection(p["palm_side"].buffer(0.3, join_style=2))
        prun = max(getattr(prun, "geoms", [prun]), key=lambda g: g.length)
        for f in (0.3, 0.5, 0.7):
            q = prun.interpolate(f, normalized=True)
            qi, qo = nearest_points(pit.buffer(-0.5, join_style=1).exterior, q)[0], nearest_points(pit.buffer(1.0, join_style=1).exterior, q)[0]
            lvl = lambda q_: (swell_z(p, q_.x, q_.y) - rest_z(p, q_.y)) if p["swell"].buffer(0.1, join_style=2).contains(q_) else min(P.BORDER_UP, (P.RIM_Z if p["lip"].buffer(0.1, join_style=2).contains(q_) else P.KEYWELL_MAX) - rest_z(p, q_.y))   # the border's level, capped at KEYWELL_MAX
            cap_in = lambda q_: lvl(q_) if p["swell"].buffer(0.1, join_style=2).contains(q_) or p["lip"].buffer(0.1, join_style=2).contains(q_) else well_top(p, q_.y) - rest_z(p, q_.y)   # the keywell's fill: its three ceilings (build 148)
            if p["keywell"].contains(qi):
                got = top(h["hull"], qi)
                want = cap_in(qi) - wall_drop(p, qi, 0.2)                                                          # less the key wall's round (build 147)
                tol = 0.15 if wall_drop(p, qi, 0.2) > 0.0 else 0.1                                               # (on the round a 0.2 column reads a 0.9 slope: build 148's R read 0.107 off)
                assert got is not None and abs(got - want) < tol, f"{k}: 0.5 inside the pit the hull's top is {got} over the rest plane (the keywell at the border's level: {want:.2f})"
            if p["border"].contains(qo):
                got = top(h["hull"], qo)
                assert got is not None and abs(got - lvl(qo)) < 0.1, f"{k}: 1 outside the pit the hull's top is {got} over the rest plane (the border: {lvl(qo):.2f})"
        skip = [p["housing"].buffer(0.5), p["flat"].buffer(0.5), p["plan"].exterior.buffer(P.WALL + 0.6)] + ([p["tp_zone"].buffer(0.6)] if p["tp_zone"] is not None else [])
        due = [q for q in (pit.exterior.interpolate(i / 24, normalized=True) for i in range(24)) if not any(z.contains(q) for z in skip) and not p["palm_side"].buffer(0.3).contains(q)]
        a = math.radians(P.KEY_TILT)                                                                         # where the tilted plate itself is higher than the
        plate = lambda q_: (q_.y - p["yp"] - P.RIM_Z * math.sin(a)) * math.tan(a) + P.RIM_Z * (1 - math.cos(a)) - P.TILT_DROP - rest_z(p, q_.y)   # fill's cap the top is the plate
        for q in due[:6]:                                                                                   # round the block's other sides too (L 23:13)
            qi = nearest_points(pit.buffer(-0.5, join_style=1).exterior, q)[0]
            if p["keywell"].contains(qi):
                got, want = top(h["hull"], qi), max(cap_in(qi), plate(qi)) - wall_drop(p, qi, 0.2)
                assert got is not None and abs(got - want) < 0.5, f"{k}: 0.5 inside the pit at ({qi.x:.0f}, {qi.y:.0f}) the hull's top is {got} over the rest plane ({want:.2f}: the fill's cap or the plate)"
        tb = h["tpu"].bounding_box()
        flush = tb.max.Z - h["W"].position.Z - rest_z(p, tb.max.Y)
        assert abs(flush - P.PAD_PROUD) < 0.05, f"{k}: the TPU's top is {flush:.2f} over the rest plane (flush with the border: {P.PAD_PROUD})"


def test_keywell_meets_the_console(model):
    """R (Willy 06:21 'only half width of the key wall actually reaches the trackpad wall'): the keywell's fill runs to the console's front face at
    the border's level -- 0.5 south of the face at x 57 / 59 / 61 the hull's top is the fill (builds 121-123 had the pit's floor there: the tooth
    fix had taken the fill 4.5 off the whole console, so the 2 mm border alone reached the face). The fill ends at x 61.5 there since build 147: the
    thumbs' keep-out diagonal runs straight to the face -- 146's 2 x 4 tab east of it was the mouth of the fill needle between t0 and t1, which the
    open pit's closing swallows (fill between caps is never wanted)."""
    from velvet.checks import column_tops
    from velvet.geo import rest_z
    from velvet.plan import plan_of
    p, h = plan_of("R"), model["halves"]["R"]
    y = p["tp_front_y"] - 0.5
    for x in (57.0, 59.0, 61.0):
        got = column_tops(h["hull"], h["W"], "R", [(x, y)], w=0.2)[0]
        want = well_top(p, y)
        assert got and abs(got[0][1] - want) < 0.1, f"R: 0.5 south of the console's face at x {x:.0f} the hull's top is {got[0][1] if got else None} (the keywell's fill: {want:.2f})"


def test_plateau_top_is_flat(model):
    """The flat zone's top IS flat: on a 0.5 mm height map of the hull's mesh, every cell 2.5 inside the zone (off the housing, R's console
    and the display's pocket) is within 0.1 of RIM_Z. Build 124's L carried an r 1.8 groove 1.8 deep north of the knob (a plan spike)."""
    import numpy as np
    from build123d import Plane, mirror
    from shapely.geometry import Point
    from shapely.prepared import prep
    from velvet.checks import top_map
    from velvet.plan import plan_of
    for k in "RL":
        p, h = plan_of(k), model["halves"][k]
        hull = h["W"].inverse() * h["hull"]
        if k == "L":
            hull = mirror(hull, about=Plane.YZ)
        zone = p["flat"].buffer(-2.5, join_style=2).difference(p["housing"].buffer(1.0)).difference(p["swell"].buffer(1.0)).difference(p["corner"].buffer(0.5)).difference(model["halves"][k]["local"]["led_tray"].buffer(1.5))   # (the swell: its own level, build 132; the LED tray, 134)
        if "badge" in model["halves"][k]["local"]:                                              # THE BADGE (build 136): L's plate stands BADGE_T over the flat zone
            zone = zone.difference(model["halves"][k]["local"]["badge"].buffer(1.5))
        zone = zone.difference(model["halves"][k]["local"]["lid_plan"].buffer(1.0))              # THE PLATEAU LIGHTS (build 152): grooves, vias and pin holes under the lid
        for g in ([p["tp_zone"].buffer(0.6)] if p["tp_zone"] is not None else []) + [p["foot"][n].buffer(1.0) for n in ("display",) if n in p["foot"]]:
            zone = zone.difference(g)
        x0, y0, x1, y1 = zone.bounds
        H, pz = top_map(hull, x0, x1, y0, y1, 0.5), prep(zone)
        off = [(round(x0 + i * 0.5, 1), round(y0 + j * 0.5, 1), round(float(H[j, i]), 2)) for j in range(H.shape[0]) for i in range(H.shape[1])
               if not np.isnan(H[j, i]) and abs(H[j, i] - P.RIM_Z) > 0.1 and pz.contains(Point(x0 + i * 0.5, y0 + j * 0.5))]
        assert not off, f"{k}: {len(off)} cells of the flat zone's top are off RIM_Z {P.RIM_Z}: {off[:8]}"


def test_pins_go_in_and_out(model):
    """THE HINGE PINS can be assembled (Willy 07:02 'make sure it functions, and can be assembled easy'): each 5 mm pin slid out along the axis
    toward its head (5 .. 80 mm) meets nothing but its snap ring's squeeze (hull, base, hatch). (Build 125-126's through channel for a push-out rod
    is gone: it broke into the plug tunnel's floor; the pins come out by their heads.) Build 124's rest pin sat 13 mm inside a blind pocket; the tent pin had no way out."""
    from build123d import Plane, Pos, Rot, Vector, mirror
    from velvet.plan import plan_of
    for k in "RL":
        p, h = plan_of(k), model["halves"][k]
        Kf = Pos(p["xin"], 0, 0) * Rot(0, 0, P.YAW)
        W = h["W"]
        mir = (lambda q: mirror(q, about=Plane.YZ)) if k == "L" else (lambda q: q)
        d = (W * Kf * Pos(0, 1, 0)).position - (W * Kf).position                           # K's +y (the back) in world ...
        if k == "L":
            d = Vector(-d.X, d.Y, d.Z)                                                       # ... mirrored about YZ with the half (build 126: unmirrored, L's pins slid 15 deg off the axis)
        for name, sense in (("tpin", 1.0), ("rpin", -1.0)):                                    # the tent pin's head is at the back, the rest pin's at the front
            for s_ in (5.0, 20.0, 40.0, 80.0):
                moved = Pos(d.X * s_ * sense, d.Y * s_ * sense, d.Z * s_ * sense) * h[name]
                lim = 25.0 * (P.PIN_RING[0] - P.PIN_CLR / 2)                                    # the snap ring's own squeeze in a plain bore, by design: 0.075 a side ~1.2 mm3, 0.225 (Willy's 0.4 coupon) ~3.1
                for other in ("hull", "base", "hatch"):
                    assert _hit(moved, h[other]) < lim, f"{k}: the {name} slid {s_} out hits the {other} by {_hit(moved, h[other]):.2f} mm3"



def test_wall_over_the_plug_window(model):
    """The outer wall keeps >= 1 mm between the Pico's plug window and the pad's recess floor (PAD_DEPTH under the rest plane; the TPU sits above)."""
    from shapely.geometry import Point
    from shapely.ops import nearest_points
    from velvet.checks import column_tops
    from velvet.plan import plan_of
    for k in "RL":
        p, h = plan_of(k), model["halves"][k]
        ux, uy, uz = h["local"]["pico_plug"]
        q = nearest_points(p["plan"].buffer(-0.6, join_style=2).boundary, Point(ux, uy))[0]
        seg = column_tops(h["hull"], h["W"], k, [(q.x, q.y)], w=0.2)[0]
        assert seg and abs(seg[0][0] - (uz + P.PLUG_H / 2)) < 0.3, f"{k}: the top segment over the plug window is {seg[0] if seg else None}, the window's top {uz + P.PLUG_H / 2:.2f}"
        assert seg[0][1] - seg[0][0] >= 1.0, f"{k}: the wall over the plug window is only {seg[0][1] - seg[0][0]:.2f} tall"


def test_no_sheets_no_crack(model):
    """Nothing of the hull stands over the rim in front of y 15 or in the pit over the tilted plate, and the flat zone / palm side seam is solid
    through the plate. Builds 85-88 had zero-thickness sheets of the housing's and the inner wall's faces (the tilt cut's faces lay ON them)
    standing up to 5.5 over the rim as fins round the knob, and a 0.001-0.02 hairline crack along the housing between the two kept prisms."""
    from velvet.checks import sheets
    from velvet.plan import plan_of
    for k in "RL":
        h = model["halves"][k]
        sh = sheets(h["hull"], h["W"], k, plan_of(k), h["local"]["badge"], h["local"]["strip"][4] if "strip" in h["local"] else None)   # (R's strip beside the brim, build 143)
        assert not sh["above"], f"{k}: {len(sh['above'])} piece(s) over the rim in front of y 15: {sh['above'][:5]}"
        assert not sh["pit"], f"{k}: {len(sh['pit'])} piece(s) in the pit over the plate: {sh['pit'][:5]}"
        assert not sh["edge"], f"{k}: {len(sh['edge'])} piece(s) over the border at the pit's edge (the blades of build 103): {sh['edge'][:5]}"
        assert sh["crack"] < 0.01, f"{k}: {sh['crack']} mm3 empty along the zone / palm seam through the plate (a crack)"


def test_crowns(model):
    """THE CROWNS (build 153; Willy 10:44 'one single cap piece ... cohesive and secure', 10:54 'flush with case'): on both halves the crown's top is
    RIM_Z + CROWN_T (+ RIDGE_H over the ridges), each guide's top flush with it (within 0.05), each filament inside its net's guide (its bbox within the
    guide's + 0.15 in plan, under the guide's top), the crown's plan inside the outline's edge-round margin except along the seam face; R's overlay lies
    OVERLAY_SINK under the crown's top, on the pad."""
    from build123d import Plane, mirror
    from shapely.geometry import box as sbox
    from velvet.plan import plan_of
    for k in "RL":
        h = model["halves"][k]
        Wi = h["W"].inverse()
        un = (lambda q: Wi * q) if k == "R" else (lambda q: mirror(Wi * q, about=Plane.YZ))
        crown = un(h["crown"])
        cb = crown.bounding_box()
        assert abs(cb.max.Z - (P.RIM_Z + P.CROWN_T + (P.RIDGE_H if P.RIDGES else 0.0))) < 0.05, f"{k}: the crown's top is at {cb.max.Z:.2f}"   # (157: RIDGES off -- one plane)
        for n in ("cyan", "magenta"):
            gb = un(h[f"guide_{n}"]).bounding_box()
            assert abs(gb.max.Z - (P.RIM_Z + P.CROWN_T)) < 0.05, f"{k}: the {n} guide's top is at {gb.max.Z:.2f}, the crown's {P.RIM_Z + P.CROWN_T}"
        for n, f in h["fils"].items():
            gb, fb = un(h[f"guide_{n.split('_')[0]}"]).bounding_box(), un(f).bounding_box()
            assert gb.min.X - 0.15 < fb.min.X and fb.max.X < gb.max.X + 0.15 and gb.min.Y - 0.15 < fb.min.Y and fb.max.Y < gb.max.Y + 0.15 and fb.max.Z < gb.max.Z - 0.5, f"{k}: the {n} filament is not inside its guide"
        pl, cp = plan_of(k), h["local"]["crown_plan"]
        out = cp.difference(pl["plan"].buffer(-(P.EDGE_R_TOP + 0.4), join_style=2)).difference(sbox(-1.0, -1000.0, P.EDGE_R_TOP + 0.5, 1000.0))   # the seam face: R's hook, L's
        #                                                                                                                     bezel ring at 1.9 off it (KNOB_BEZ_R = the silo round's tangent + 0.1, 154)
        if h["local"].get("crown_hook") is not None:                                             # R: THE LIP'S HOOK fills the round under the crown's edge wherever its plan reaches the
            out = out.difference(h["local"]["crown_hook"].buffer(P.EDGE_R_TOP + 0.4 - P.BZ_HOOK + 0.1, join_style=2))   # outline (the seam face + the front diagonal, build 156)
        assert out.area < 0.05, f"{k}: the crown reaches the outline's edge round by {out.area:.2f} mm2 at {[round(v, 1) for v in out.bounds]}"
        if k == "R":
            ob = un(h["overlay"]).bounding_box()
            assert abs(ob.max.Z - (P.RIM_Z + P.CROWN_T - P.OVERLAY_SINK)) < 0.05 and abs(ob.min.Z - P.TP_TOP) < 0.05, f"R: the overlay spans z {ob.min.Z:.2f}..{ob.max.Z:.2f}"


def test_regression_against_golden(model):
    """Every part's volume and bbox vs tests/golden.json (regenerate with tests/golden.py after an intended change)."""
    if not GOLDEN.exists():
        pytest.skip("no golden.json")
    gold = json.loads(GOLDEN.read_text())
    diffs = []
    for name, s in _parts(model).items():
        g = gold.get(name)
        if g is None:
            diffs.append(f"{name}: new"); continue
        b = s.bounding_box()
        bb = [b.min.X, b.min.Y, b.min.Z, b.max.X, b.max.Y, b.max.Z]
        if abs(s.volume - g["vol"]) > max(0.5, 1e-4 * g["vol"]) or max(abs(u - v) for u, v in zip(bb, g["bb"])) > 0.02:
            diffs.append(f"{name}: vol {s.volume:.2f} vs {g['vol']}, bb {[round(v, 2) for v in bb]} vs {g['bb']}")
    assert not diffs, "\n".join(diffs)


def test_pico_plug_tunnel_and_cover(model):
    """Both halves (Willy 21:06 'reliable tunnel for all sized usb + cover for bay'): a 12 x 8 plug overmold runs from the Pico's connector face out
    through the bore and the case's face touching nothing (hull, hatch, TPU, pane, frame); the bore leaves >= 1.8 of skin under the pad's recess
    floor; the cover presses into the window on its four crush ribs alone, its BRIM alone outside the case's face (LIP_T proud, Willy 10:02)."""
    from build123d import Align, Box, Pos
    from velvet.checks import column_tops
    from velvet.geo import prism, rest_z
    from velvet.plan import plan_of
    for k in "RL":
        p, hl = plan_of(k), model["halves"][k]["local"]
        x0, x1 = hl["pico_face"], hl["pico_bore"][1]                                     # the overmold starts at the connector's face
        Lp = x1 + 15.0 - x0
        path = hl["bay_Lf"] * Pos(x0 + Lp / 2, 0, hl["usb_z"]) * Box(Lp, P.PLUG_W - 1.0, P.PLUG_H - 1.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
        for name in ("hull", "hatch", "tpu", "pane"):
            assert _hit(path, hl[name]) < 0.01, f"{k}: the plug path hits the {name}: {_hit(path, hl[name]):.2f}"
        q = (hl["bay_Lf"] * Pos((x0 + x1) / 2, 0, hl["usb_z"])).position                   # mid-tunnel: the skin between the bore's roof and the pad's floor
        seg = column_tops(hl["hull"], Pos(0, 0, 0), "R", [(q.X, q.Y)], w=0.2)[0]
        skin = seg[0][1] - seg[0][0]
        assert abs(seg[0][1] - (rest_z(p, q.Y) - P.PAD_DEPTH)) < 0.1 and skin >= 1.8, f"{k}: over the tunnel the skin is {seg[0]} ({skin:.2f} thick; the pad's floor at {rest_z(p, q.Y) - P.PAD_DEPTH:.2f})"
        A_, B_ = hl["pico_doors"]; Wf = hl["pico_Wf"]; Wi = Wf.inverse()                 # THE PLUG DOOR (build 155)
        lost = P.PICO_GROOVE[1] - P.PICO_LUG[1]                                                # A alone for this much of its stroke, then it drags B
        oA, oB = Wf * Pos(-P.PICO_TRAVEL, 0, 0) * (Wi * A_), Wf * Pos(-(P.PICO_TRAVEL - lost), 0, 0) * (Wi * B_)
        plug = path                                                                          # the same 12 x 8 overmold runs out through the rounded-rectangle hole (build 155, Willy 16:27)
        for n, q in (("A", A_), ("B", B_), ("A open", oA), ("B open", oB)):
            assert q.is_valid and len(q.solids()) == 1, f"{k}: leaf {n} invalid"
            assert _hit(q, hl["hull"]) < 0.01, f"{k}: leaf {n} hits the hull: {_hit(q, hl['hull']):.2f}"
        assert _hit(A_, B_) < 0.01 and _hit(oA, oB) < 0.01, f"{k}: the leaves touch"
        assert _hit(oA, plug) < 0.01 and _hit(oB, plug) < 0.01, f"{k}: the open leaves sit in the plug's way: {_hit(oA, plug):.2f} / {_hit(oB, plug):.2f}"
        assert _hit(A_, plug) > 20.0 and _hit(B_, plug) > 20.0, f"{k}: the closed leaves do not cover the hole"


def test_outer_wall_flat(model):
    """The outer wall along the key block is ON the outline up to its top round (build 113: the tilted body's wall read 0.2-0.3 inside it from z -8
    to the plate, the plate's rotated deck round a diagonal crease; Willy 10:58 'it should be flat')."""
    import math
    from build123d import Vector
    from shapely.geometry import Point
    from velvet.plan import plan_of
    for k in "RL":
        p, h = plan_of(k), model["halves"][k]["local"]                  # the half's parts in its own u frame
        plan = p["plan"]; ext = plan.exterior; xmax = plan.bounds[2]
        sol = max(h["hull"].solids(), key=lambda s: s.volume)
        a = math.radians(P.KEY_TILT)
        plate = lambda y: (y - p["yp"] - P.RIM_Z * math.sin(a)) * math.tan(a) + P.RIM_Z * (1 - math.cos(a)) - P.TILT_DROP
        qs = [ext.interpolate(i / 300, normalized=True) for i in range(300)]
        qs = [q for q in qs if q.x > xmax - 12 and p["tilted"].buffer(0.5, join_style=2).distance(q) < 0.1 and p["palm_side"].distance(q) > 3.0 and p["flat"].distance(q) > 3.0]
        assert len(qs) >= 5, f"{k}: only {len(qs)} outer-wall stations along the tilted zone"
        for q in qs[::max(1, len(qs) // 8)]:
            b0 = ext.interpolate(ext.project(q) - 0.5); b1 = ext.interpolate(ext.project(q) + 0.5)
            tx, ty = b1.x - b0.x, b1.y - b0.y; n = math.hypot(tx, ty); nx, ny = -ty / n, tx / n
            if not plan.contains(Point(q.x + nx, q.y + ny)):
                nx, ny = -nx, -ny
            from velvet.geo import rest_z
            on_lip = p["lip"].buffer(0.1, join_style=2).contains(Point(q.x + 1.0 * nx, q.y + 1.0 * ny))
            from velvet.geo import lip_z
            top = (lip_z(p, q.x + 1.0 * nx, q.y + 1.0 * ny) - P.BORDER_R) if on_lip else (plate(q.y) - P.EDGE_R_TOP)   # up the lip to its round (build 117; the swell's level, 132), else the plate's edge
            slots = [z0 for _, x0, y0, z0 in P.SW805_AT[k] if ext.interpolate(ext.project(Point(x0, y0))).distance(q) < P.SW805_SLOT[0] / 2 + P.SW805_TRAVEL + 2.0]   # an 805's knob slot here (build 133)
            z = P.ZB + 3.0
            while z < top - 0.6:
                if not any(abs(z - z0) < P.SW805_SLOT[1] / 2 + 1.0 for z0 in slots):
                    assert sol.is_inside(Vector(q.x + 0.15 * nx, q.y + 0.15 * ny, z), 1e-3), f"{k}: the outer wall at ({q.x:.1f}, {q.y:.1f}) z {z:.1f} is not within 0.15 of the outline"
                z += 2.0


def test_lip(model):
    """THE LIP (build 117; Willy 12:29 'that sharp cutoff near the z and / keys', 14:16 'the lip coming off the wrist rests ... smoothness'): along
    the key block's outer wall the rim runs on from the rest at the border's level (capped RIM_Z: flush into the back band) -- no cliff past the pit's corner -- its
    outer top edge rounded BORDER_R (0.55 down 0.5 in), CAP_CLR off the pinky column's caps."""
    import math
    from build123d import Align, Box, Pos
    from shapely.geometry import Point
    from velvet.geo import lip_z
    from velvet.plan import plan_of
    for k in "RL":
        p, h = plan_of(k), model["halves"][k]["local"]
        plan, lip = p["plan"], p["lip"]; ext = plan.exterior
        assert lip.geom_type == "Polygon" and lip.area > 150.0, f"{k}: the lip is {lip.area:.0f} mm2, {lip.geom_type}"
        run = ext.intersection(lip.buffer(0.1, join_style=2)); run = max(getattr(run, "geoms", [run]), key=lambda g: g.length)
        assert run.length > 50.0, f"{k}: the lip's outline run is {run.length:.0f} mm"
        def top(x, y):
            c = h["hull"] & Pos(x, y, 0) * Box(0.3, 0.3, 400, align=(Align.CENTER, Align.CENTER, Align.CENTER)); sol = c.solids() if c else []
            return max(s.bounding_box().max.Z for s in sol) if sol else None
        for i in range(1, 8):
            q = run.interpolate(i / 8, normalized=True)
            b0, b1 = run.interpolate(max(0.0, run.project(q) - 0.5)), run.interpolate(min(run.length, run.project(q) + 0.5))
            tx, ty = b1.x - b0.x, b1.y - b0.y; n = math.hypot(tx, ty); nx, ny = -ty / n, tx / n
            if not plan.contains(Point(q.x + nx, q.y + ny)):
                nx, ny = -nx, -ny
            want = lip_z(p, q.x + 2.5 * nx, q.y + 2.5 * ny)                                    # the border's level (to the rim) or THE SWELL (build 132: the lip follows the plate)
            want_e = lip_z(p, q.x + 0.5 * nx, q.y + 0.5 * ny)
            t_in, t_edge = top(q.x + 2.5 * nx, q.y + 2.5 * ny), top(q.x + 0.5 * nx, q.y + 0.5 * ny)
            assert t_in is not None and abs(t_in - want) < 0.3, f"{k}: the lip at ({q.x:.1f}, {q.y:.1f}) tops at {t_in} vs the lip's level {want:.2f}"
            assert t_edge is not None and t_in - 1.3 < t_edge < t_in - 0.35, f"{k}: the lip's edge at ({q.x:.1f}, {q.y:.1f}) is {t_edge} vs the lip's top {t_in:.2f} there ({want_e:.2f} by lip_z; an r {P.BORDER_R} round)"
        assert lip.distance(p["caps"]) > P.CAP_CLR - 0.05, f"{k}: the lip is {lip.distance(p['caps']):.2f} from the caps"


def test_covers_clear_the_hull_and_the_pad(model):
    """Every cover's BRIM sits ON the wall, not in it (the brim is the cover outside the outline; the plug's crush ribs overlap the bore on purpose),
    and the pane frame's brim (the frame over the deck) stays off the hull and the TPU: it rests on the hull's border round the recess."""
    from build123d import Align, Box, Pos
    from velvet.geo import _hit, prism
    from velvet.plan import plan_of
    for k in "RL":
        p, h = plan_of(k), model["halves"][k]["local"]
        inside = prism(p["plan"], P.ZB - 1.0, P.RIM_Z + 10.0)
        for name in ("pico_cover", "cover"):
            if name in h:
                brim = h[name] - inside
                assert brim.volume > 100.0, f"{k}: {name} has no brim outside the outline ({brim.volume:.0f} mm3)"
                assert _hit(brim, h["hull"]) < 0.05, f"{k}: {name}'s brim overlaps the hull by {_hit(brim, h['hull']):.2f} mm3"
        assert h["frame"] is None, f"{k}: a pane frame is back (159 dropped it: the chin and the pad lap the pane)"



def test_base_plate_edges_are_rounded(model):
    """The base plates' outline edges are the EDGE_R_TOP / EDGE_R_BOT rounds (stand.build_base's band swaps), top and bottom, all round except the
    hinge side: 0.4 in from the outline the top sits on the r 1.8 arc (-0.67) and the underside on the r 1.5 arc (+0.48), 1.2 in within 0.15 of
    the faces, 2 in ON them. Builds 108-122 carried 1.5-1.8 x 2-2.3 mm REBATES there instead -- the swap's band ended ON the round's tangent line
    and OCC's common with the rounded prism came back EMPTY, no error (R's top alone survived; Willy 23:40 'one has curved edge and other has
    straight')."""
    import math
    from shapely.geometry import box as sbox
    from shapely.ops import nearest_points
    from velvet.checks import column_tops
    from velvet.plan import plan_of
    rt, rb = P.EDGE_R_TOP, P.EDGE_R_BOT
    arc = lambda r, d: 0.0 if d >= r else r - math.sqrt(r * r - (r - d) ** 2)      # the round's drop from the face, d in from the edge
    for k in "RL":
        p, h = plan_of(k), model["halves"][k]
        slab = p["plan"].intersection(p["K"](sbox(-1000.0, -1000.0, p["ax_x"], 1000.0)))
        hinge_side = p["K"](sbox(p["ax_x"] - 20.0, -1000.0, 1000.0, 1000.0))
        ring = slab.exterior
        qs = [q for q in (ring.interpolate((i + 0.5) / 20, normalized=True) for i in range(20)) if not hinge_side.buffer(2.0).contains(q)]
        ref = [nearest_points(slab.buffer(-3.0, join_style=1).boundary, q)[0] for q in qs]                      # 3 in: the plain faces -- or the flap's recess /
        flat = column_tops(h["base"], h["W"], k, [(q.x, q.y) for q in ref], w=0.2)                               # the ears' window, open at the front diagonal
        qs = [q for q, sg in zip(qs, flat) if sg and abs(sg[0][1] - P.ZB) < 0.1 and abs(sg[-1][0] - (P.ZB - P.BASE_T)) < 0.1]   # (L (32,-112): -4.2; R (58,-115))
        assert len(qs) >= 6, f"{k}: only {len(qs)} outline stations on the plain faces off the hinge side"
        for d, tol in ((0.4, 0.35), (1.2, 0.15), (2.0, 0.1)):
            pts = [nearest_points(slab.buffer(-d, join_style=1).boundary, q)[0] for q in qs]
            segs = column_tops(h["base"], h["W"], k, [(q.x, q.y) for q in pts], w=0.2)
            for q, sg in zip(pts, segs):
                assert sg, f"{k}: no base under ({q.x:.0f},{q.y:.0f}) {d} in from the outline"
                top, bot = sg[0][1] - P.ZB, sg[-1][0] - (P.ZB - P.BASE_T)
                assert abs(top + arc(rt, d)) < tol, f"{k}: {d} in from the base's outline at ({q.x:.0f},{q.y:.0f}) the top is {top:+.2f} off the face, the r {rt} round says {-arc(rt, d):+.2f}"
                assert abs(bot - arc(rb, d)) < tol, f"{k}: {d} in from the base's outline at ({q.x:.0f},{q.y:.0f}) the underside is {bot:+.2f} off the face, the r {rb} round says {arc(rb, d):+.2f}"


def test_hull_bottom_edge_is_rounded(model):
    """The hull's bottom outline edge is the EDGE_R_BOT round all round except the hinge side (hull.hinges' band swap): 0.4 in from the outline
    the underside sits on the r 1.5 arc (+0.48 over ZB), 1.2 in within 0.15 of it, 2 in ON it. L's was a 1.5 x 2 rebate from build 108 to 123:
    the swap's band ended ON the round's tangent line and OCC's common came back EMPTY, no error, and nothing probed the underside's edge."""
    import math
    from shapely.geometry import box as sbox
    from shapely.ops import nearest_points
    from velvet.checks import column_tops
    from velvet.plan import plan_of
    rb = P.EDGE_R_BOT
    arc = lambda d: 0.0 if d >= rb else rb - math.sqrt(rb * rb - (rb - d) ** 2)
    for k in "RL":
        p, h = plan_of(k), model["halves"][k]
        plan = p["plan"]
        hinge_side = p["K"](sbox(p["ax_x"] - 20.0, -1000.0, 1000.0, 1000.0))
        ring = plan.exterior
        qs = [q for q in (ring.interpolate(f, normalized=True) for f in (0.05, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95)) if not hinge_side.buffer(2.0).contains(q)]
        assert len(qs) >= 6, f"{k}: only {len(qs)} outline stations off the hinge side"
        for d, tol in ((0.4, 0.35), (1.2, 0.15), (2.0, 0.1)):
            pts = [nearest_points(plan.buffer(-d, join_style=1).boundary, q)[0] for q in qs]
            segs = column_tops(h["hull"], h["W"], k, [(q.x, q.y) for q in pts], w=0.2)
            for q, sg in zip(pts, segs):
                assert sg, f"{k}: no hull under ({q.x:.0f},{q.y:.0f}) {d} in from the outline"
                bot = sg[-1][0] - P.ZB
                assert abs(bot - arc(d)) < tol, f"{k}: {d} in from the hull's outline at ({q.x:.0f},{q.y:.0f}) the underside is {bot:+.2f} over ZB, the r {rb} round says {arc(d):+.2f}"


def test_outline_round_runs_unbroken(model):
    """THE TOP EDGE ROUND ROUND THE WHOLE OUTLINE (Willy 07:50 'messed up edges near the outer battery corner, both sides', 07:55 'random sharp
    edges on the curve too'): every 1 mm along the outline -- the rest's border, the console and the seam face aside, they have their own tests --
    the top 0.3 .. 1.5 in matches the r 1.8 arc on the local top plane (the level at 2 in where the top is flat there; else 3 / 4 in, or 2 / 2.5
    along the 3.3 wide lip) within 0.2: no run of three stations square (above the arc), notched (below) or stepped (off the arc further in). Build 125: 11 mm square round the outer-back corner's
    arc (the back band stopped R + 1 off the pit's ring), 6 mm square up the outer wall between the lip's run and the back zone, both sides."""
    import math
    import numpy as np
    from build123d import Plane, mirror
    from shapely.ops import nearest_points
    from velvet.checks import top_map
    from velvet.geo import rest_z, swell_z
    from velvet.plan import plan_of
    r, D = P.EDGE_R_TOP, (0.3, 0.6, 0.9, 1.2, 1.5)
    flat = lambda d: r - math.sqrt(max(r * r - (r - d) ** 2, 0.0))                      # the r 1.8 round's drop d in from the edge on a level top
    for k in "RL":
        p, h = plan_of(k), model["halves"][k]
        hull = h["W"].inverse() * h["hull"]                                                      # back in the plan's frame
        hull = hull if k == "R" else mirror(hull, about=Plane.YZ)
        plan = p["plan"]
        bx0, by0, bx1, by1 = plan.bounds
        H = top_map(hull, bx0 - 2.0, bx1 + 2.0, by0 - 2.0, by1 + 2.0, 0.1)
        at = lambda x, y: H[int(round((y - by0 + 2.0) / 0.1)), int(round((x - bx0 + 2.0) / 0.1))]
        skip = [p["palm_side"].buffer(3.5), p["housing"].buffer(2.0), h["local"]["badge"].buffer(1.0)] + ([p["tp_zone"].buffer(2.0)] if p["tp_zone"] is not None else [])   # (L's badge into the seam wall, build 144)
        inner, ext = plan.buffer(-1.0, join_style=1).exterior, plan.exterior
        bad = []                                                                                  # (x, y, kind, worst)
        for i in range(int(ext.length)):
            q = ext.interpolate(float(i))
            if q.x < 0.6 or any(z.contains(q) for z in skip):
                continue
            qi = nearest_points(inner, q)[0]
            nx, ny = qi.x - q.x, qi.y - q.y
            nn = math.hypot(nx, ny)
            if nn < 1e-6:
                continue
            nx, ny = nx / nn, ny / nn
            z = {d: at(q.x + d * nx, q.y + d * ny) for d in D + (2.0, 2.5, 3.0, 4.0)}
            if any(np.isnan(z[d]) for d in D + (2.0, 2.5)):
                continue
            arc = flat
            if p["swell"].buffer(1.0, join_style=2).contains(q):                                # THE SWELL's two planes (build 132): the design surface itself, FLOORED at the
                on_lip = p["lip"].buffer(0.1, join_style=2).contains(q)                         # plateau's rim (the swell only ADDS: where plane B is under the rim the rim stays)
                floor = lambda y, on_lip=on_lip: min(rest_z(p, y) + P.BORDER_UP, P.RIM_Z) if on_lip else P.RIM_Z   # or at the lip's border level (build 135)
                plane = lambda d, q=q, nx=nx, ny=ny, floor=floor: max(swell_z(p, q.x + d * nx, q.y + d * ny), floor(q.y + d * ny))
                m = plane(0.0) - plane(1.0)                                                     # the top's fall inward per mm: a round on a SLOPED top drops MORE at the edge
                arc = lambda d, m=m: max(0.0, m * (r - d) + r * math.sqrt(1.0 + m * m) - math.sqrt(max(r * r - (r - d) ** 2, 0.0)))   # (1.08 vs 0.81 at 0.3 in on
                #                                                                                plane B's 11 deg along the back run: build 135's 'notched 0.25-0.33', strip_end135)
            elif abs(z[2.0] - z[2.5]) < 0.05:                                                  # a FLAT top at 2..2.5 in: that level (the corner's 2.2 wide strip: 3..4 in is off it)
                plane = lambda d, z=z: z[2.0]
            elif abs(z[2.0] - P.RIM_Z) < 0.05:                                                    # the RIM at 2 in: the round is on it (THE RIM STRIP, build 140: 3.3 wide; the pit's
                plane = lambda d: P.RIM_Z                                                        # diagonal 2.6 in at R (160, 20) read as a sloped top from 3 / 4 in)
            elif np.isnan(z[3.0]) or np.isnan(z[4.0]) or abs(z[4.0] - z[3.0]) > 0.6:            # the lip: 3.3 wide, the pit's wall drops inside it
                plane = lambda d, z=z: z[2.5] + (z[2.0] - z[2.5]) * (2.5 - d) * 2.0
            else:
                plane = lambda d, z=z: z[4.0] + (z[3.0] - z[4.0]) * (4.0 - d)
            dev = [z[d] - (plane(d) - arc(d)) for d in D]
            kind = "square" if dev[0] > 0.2 else ("notched" if dev[0] < -0.2 else ("stepped" if max(abs(v) for v in dev[1:]) > 0.2 else None))
            if kind:
                bad.append((q.x, q.y, kind, max(abs(v) for v in dev)))
        runs = [(a, b, c) for a, b, c in zip(bad, bad[1:], bad[2:]) if a[2] == b[2] == c[2] and math.hypot(a[0] - c[0], a[1] - c[1]) < 4.5]   # 3 stations in a row: a 3 mm run
        assert not runs, f"{k}: the top edge round breaks at " + "; ".join(f"({a[0]:.0f},{a[1]:.0f}) {a[2]} {max(a[3], b[3], c[3]):.2f}" for a, b, c in runs[:6]) + f" ({len(runs)} runs of 3 stations)"


def test_key_wall_under_the_caps(model):
    """THE KEY WALL'S CEILING (build 148; Willy 21:50 'a rule for all keys yeah - keep it consistent and simple'): beside every cap -- the ring 1..2.5
    out from its footprint inside the keywell fill -- the hull's top is at least KEY_PROUD under the caps' top plane (CAP_TOP_T over the tilted
    plate). The mesh's top map at 0.5 (a boolean column misses a round's shoulder); 0.05 for the raster."""
    import math
    import numpy as np
    from shapely.geometry import Point
    from velvet.checks import top_map
    from velvet.geo import plate_z
    from velvet.plan import plan_of
    ca = math.cos(math.radians(P.KEY_TILT))
    for k in "RL":
        p, h, sx = plan_of(k), model["halves"][k], (1.0 if k == "R" else -1.0)
        kw, W = p["keywell"], model["halves"][k]["W"]
        wx = lambda x: sx * (x + P.SEP / 2)
        b = kw.bounds; d = 0.5
        X0, X1 = sorted((wx(b[0]) - 3.0, wx(b[2]) + 3.0)); Y0, Y1 = b[1] - 3.0, b[3] + 3.0
        H = top_map(h["hull"], X0, X1, Y0, Y1, d) - W.position.Z
        cells = [(sx * (X0 + i * d) - P.SEP / 2, Y0 + j * d, H[j, i]) for j in range(H.shape[0]) for i in range(H.shape[1]) if not np.isnan(H[j, i])]
        high = []
        for name, c in p["cap_by"].items():
            ring = c.buffer(2.5, join_style=1).difference(c.buffer(1.0, join_style=1)).intersection(kw)
            if ring.is_empty:
                continue
            over = [z - (plate_z(p, y) + (P.CAP_TOP_T - P.KEY_PROUD) / ca) for x, y, z in cells if ring.contains(Point(x, y))]   # vs the ceiling AT the cell
            assert over, f"{k} {name}: no hull top in the keywell beside the cap"                                                 # (the plate tilts: the cap's
            if max(over) > 0.05:                                                                                                  # top is a plane, not a height)
                high.append(f"{name} {max(over):.2f} over")
        assert not high, f"{k}: the wall stands within KEY_PROUD {P.KEY_PROUD} of the cap top beside " + ", ".join(high)
