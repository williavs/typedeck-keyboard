"""The 2D plan of a half (u frame): footprints, the flowed outline (its front one wedge), the tent hinge line and knuckle slots, the rest hinge slots."""
import math

import numpy as np

from shapely import affinity
from shapely.geometry import LineString, Point, Polygon, box as sbox
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

from velvet.params import *  # noqa: F401,F403
from velvet.geo import flow, hinge_straight, plate_z, rect, tilt_shadow, tp_plan
from velvet.backline import back_chain
from velvet.frames import half_frames
from velvet.caps import cap_size


def flap_footprint(ax_x, fy):
    """The stand's footprint in K, folded: the plate points AT the hinge from its pivot (barrel to nose), the two ears beyond its ends."""
    plate = sbox(ax_x - FLAP_P - FLAP_RB, fy - FLAP_W / 2, ax_x - FLAP_P + FLAP_S + FLAP_NOSE, fy + FLAP_W / 2)
    ears = [sbox(ax_x - FLAP_P - EAR[0] / 2, fy + sy * (FLAP_W / 2 + EAR_GAP + EAR[1] / 2) - EAR[1] / 2, ax_x - FLAP_P + EAR[0] / 2, fy + sy * (FLAP_W / 2 + EAR_GAP + EAR[1] / 2) + EAR[1] / 2) for sy in (-1, 1)]
    return unary_union([plate] + ears)


def back_run(plan, K, Ki, xo, y1):
    """The outline's BACK RUN: the one chain of edges behind the rail's back end (K y > y1), off the seam wall and the hinge line -- from the
    seam's back corner round every jog and fillet to the hinge corner -- as a LineString (the back round's zone; the rib flutes that once
    stood on it are gone, Willy 12:33 'remove to simplify')."""
    ring = orient(plan, 1.0).exterior.coords
    edges = []
    for a, b in zip(ring, ring[1:]):
        L = math.dist(a, b)
        if L < 1e-6:
            continue
        m = Point((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        mk = Ki(m)
        edges.append((a, b, L, mk.y > y1 - 0.5 and m.x > 0.6 and mk.x < xo + WALL - 0.6))
    i0 = next(i for i, e in enumerate(edges) if not e[3])                                      # start off the run: the ring may begin inside it
    edges = edges[i0:] + edges[:i0]
    runs, cur = [], []
    for e in edges:
        if e[3]:
            cur.append(e)
        elif cur:
            runs.append(cur)
            cur = []
    run = max(runs + [cur], key=lambda r: sum(e[2] for e in r))
    return LineString([run[0][0]] + [e[1] for e in run])


def shoulder(plan, side, foot, r_cv=12.0, r_cc=FLOW):
    """THE SHOULDER (build 134; Willy 16:03 'why does the edge around the encoder travel almost to the back with nice curve then sharp angle
    before it meets the back - ugly'): the back outline's step between the seam-side bay's back wall (L: the charger's, y 39; R: the display's,
    73.6) and the cell bay's was the flow's r 18 closing sweep running straight into an r 4 (ROUND) opening corner -- 75 deg of curve then an
    80 deg kink. Re-drawn as an OGEE: a concave r_cc arc (= FLOW, so the closing keeps it) into a convex r_cv arc, G1 at the inflection, tangent
    to both walls. The convex arc is tangent to the outer wall right over the bay's corner (L: the cell bore's NW corner; R: the display pocket's
    NE corner) -- the one spot where a bigger radius keeps WALL (the arc's centre sits WALL + r inside, straight over the corner); the concave
    arc's tangent point is solved from the circles' external tangency. The notch between the walls fills (+114 / +106 mm2), nothing else moves.
    Both halves: L line - r 18 - r 12 - line, R line - r 12 - r 18 - line (the chain fit reads them back as two arcs)."""
    bay = foot["v713"] if side == "L" else foot["display"]
    ya = bay.bounds[3] + WALL                                                                   # the seam-side bay's back wall: y = ya, eastward
    cp = list(foot["cell"].exterior.coords)[:4]
    w, e = sorted(sorted(cp, key=lambda q: -q[1])[:2])                                         # the bore's north edge: its NW, NE corners
    d = np.array(e) - np.array(w); d /= np.hypot(*d)
    nb = np.array([-d[1], d[0]])
    if nb[1] < 0:
        nb = -nb                                                                               # outward: north
    A = dict(p=np.array([0.0, ya]), d=np.array([1.0, 0.0]), n=np.array([0.0, 1.0]))
    B = dict(p=np.array(w) + nb * WALL, d=d, n=nb)                                             # the cell bay's back wall (K-yawed), WALL off the bore
    if side == "L":
        hi, lo, corner, sense = B, A, np.array(w), -1.0                                        # the convex arc on the cell wall over the bore's NW corner; the concave west of it
    else:
        hi, lo, corner, sense = A, B, np.array([bay.bounds[2], bay.bounds[3]]), 1.0            # the convex arc on the display wall over the pocket's NE corner; the concave east
    t_hi = hi["p"] + hi["d"] * np.dot(corner - hi["p"], hi["d"])                               # the foot of the perpendicular from the corner
    c_hi = t_hi - hi["n"] * r_cv
    q = lo["p"] + lo["n"] * r_cc - c_hi
    qd, R = np.dot(q, lo["d"]), r_cv + r_cc
    disc = qd * qd - np.dot(q, q) + R * R
    assert disc > 0, f"{side}: no ogee r {r_cc} / {r_cv} between the bays' back walls"
    s = max((-qd - math.sqrt(disc), -qd + math.sqrt(disc)), key=lambda s_: sense * (lo["p"] + lo["d"] * s_ - t_hi)[0])   # the root on the ogee's side
    t_lo = lo["p"] + lo["d"] * s
    c_lo = t_lo + lo["n"] * r_cc
    I = c_lo + (c_hi - c_lo) * r_cc / R                                                        # the inflection: where the circles touch
    def arc(c, r, a, b, n):
        a0, a1 = math.atan2(*(a - c)[::-1]), math.atan2(*(b - c)[::-1])
        da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
        return [(c[0] + r * math.cos(a0 + da * i / n), c[1] + r * math.sin(a0 + da * i / n)) for i in range(n + 1)]
    pts = arc(c_lo, r_cc, t_lo, I, 24)[:-1] + arc(c_hi, r_cv, I, t_hi, 16)                    # ~0.85 mm chords, like the plan's
    far = t_hi - sense * hi["d"] * 8.0                                                         # on along the outer wall past the old corner (the fill's edge ON the old outline there:
    fill = Polygon(pts + [tuple(far), tuple(far - hi["n"] * 20.0), tuple(t_lo - lo["n"] * 20.0)])   # the 0.01 closing below dissolves the slit, as geo.prism does)
    assert fill.is_valid, f"{side}: the shoulder's fill polygon is invalid"
    new = unary_union([plan, fill]).buffer(0.01, join_style=2, mitre_limit=20.0).buffer(-0.01, join_style=2, mitre_limit=20.0)
    assert new.geom_type == "Polygon" and len(new.interiors) == len(plan.interiors), f"{side}: the shoulder's union is {new.geom_type} with {len(new.interiors)} holes"
    x0, x1 = sorted((t_lo[0], t_hi[0]))
    old = plan.exterior.intersection(sbox(x0 + 0.01, min(t_lo[1], t_hi[1]) - 0.5, x1 - 0.01, 1000.0))
    assert fill.buffer(0.05).contains(old), f"{side}: the old outline sticks out of the shoulder's ogee"
    print(f"   {side} SHOULDER: ogee r {r_cc:.0f} / {r_cv:.0f} from ({t_lo[0]:.1f}, {t_lo[1]:.1f}) through ({I[0]:.1f}, {I[1]:.1f}) to ({t_hi[0]:.1f}, {t_hi[1]:.1f}), +{new.area - plan.area:.0f} mm2", flush=True)
    return new


def half_plan(side):
    """Plan geometry in the u frame: every part's footprint, the flowed outline. K rects are given in the key frame."""
    fr, fu = half_frames(side)
    xin = X_IN[side]
    cap_by = {k: Polygon(rect(f, *cap_size(VL.cap_u(k)))) for k, f in fu.items()}
    cap_list = list(cap_by.values())
    caps = unary_union(cap_list)
    wells = caps.buffer(MARGIN + 2.5, join_style=1, quad_segs=4).buffer(-2.5, join_style=1, quad_segs=4).simplify(0.3)   # one well per cluster (alphas; thumbs)

    def K(poly):
        return affinity.translate(affinity.rotate(poly, YAW, origin=(0, 0)), xin, 0)
    Lc = CELL_L + 2 * LEAD
    r = CELL_R + CELL_CLR
    cell_k = sbox(CELL_X0, TOPS[0] + WALL - 0.3, CELL_X0 + Lc, TOPS[0] + WALL - 0.3 + 2 * r)       # the bore's footprint in K
    xo = (VL.COLS - 1) * VL.U + cap_size(1.0)[0] / 2 + MARGIN                                      # the outer column's outer well edge in K (the cap's half width: 9.0 left the DSA's wells 0.12 past it)
    ax_x = xo + WALL - TENT_R - 0.05                                                              # the tent axis in K (the hinge barrel 0.05 inside the outer face: flush, the barrel's
    #                                                                                               face was TANGENT to the wall along z_ax and OCC left every knuckle's seam a free edge -- the hull invalid, build 74)
    foot = dict(wells=wells, cell=K(cell_k))
    foot["flap"] = K(flap_footprint(xo + WALL - TENT_R, FLAP_Y[side])).buffer(1.6, join_style=2)   # the folded stand under the hatch: the hull covers it (it poked out beside the thumbs); 1.6: the hatch
    #                                                                                               keeps 1.5 round the ears (R's 30 deg diagonal passed 4.35 from an ear: 1.35 of hatch)
    # THE KEY BLOCK: the wells + WALL, closed 12 (the alphas, the thumb arc and the deck between them); the housings (the trackpad's / the knob's)
    yp = min(f.position.Y for f in fu.values()) - 13.0 - 6.0                                   # the tilt line: 6 in front of the front-most cap
    sweep = lambda g: unary_union([tilt_shadow(g, yp, 0.0), tilt_shadow(g, yp, 10.0)]).convex_hull   # a cap's keep-out footprint: its shadows at 0 and 10 over the plate
    block = wells.buffer(WALL, join_style=1, quad_segs=4).buffer(12.0, join_style=1).buffer(-12.0, join_style=1)
    housing = (tp_plan(TP_CLR) if side == "R" else Point(KNOB_U, KNOB_Y).buffer(KNOB_R + KNOB_CLR, quad_segs=8)).buffer(WALL, join_style=1)   # + its own plateau wall
    if side == "R":
        foot["pad"] = tp_plan(TP_CLR)
        foot["display"] = sbox(U_PK0, Y_P0, U_PK1, Y_P0 + PK_D + WIRE)
        v713 = sbox(V713_R[0], V713_R[1], V713_R[0] + V713[0] + 2 * V713_CLR, V713_R[1] + V713[1] + 2 * V713_CLR)   # inside the display pocket's footprint
    else:
        foot["knob"] = Point(KNOB_U, KNOB_Y).buffer(KNOB_R + KNOB_CLR, quad_segs=8)
        v713 = sbox(V713_L[0], V713_L[1], V713_L[0] + V713[1] + 2 * V713_CLR, V713_L[1] + V713[0] + 2 * V713_CLR)
        foot["v713"] = v713.buffer(V713_FENCE, join_style=2)                                      # the bay holds the fence too
    # THE WEDGE (Willy 07:37 'a straight line from the peripherals to the end of the wrists', 'rectangular with splayed sides', 'sort of a wedge';
    # 08:40 'bring the inner edge in closer'): the half's front is ONE trapezoid -- the outer edge straight on the hinge line, the front edge square
    # to it (K), the inner edge the seam down to the KNEE then REST_ANGLE outward to the front corner. The knee is where the diagonal TANGENT to the
    # tightest wall (every part + WALL, the key block, the housing) meets the seam: a line x = (knee - y) tan A clears a wall point (x, y) iff
    # knee <= y + x / tan A. It reaches back to the housing's back (or the knee); the parts' walls and the key block shape everything behind it.
    tn = math.tan(math.radians(REST_ANGLE[side]))
    walls = unary_union([g.buffer(WALL, join_style=1, quad_segs=4) for g in foot.values()] + [block, housing])
    knee = min(y + x / tn for g in getattr(walls, "geoms", [walls]) for x, y in g.exterior.coords)
    diag = Polygon([(0.0, 1000.0), (0.0, knee), ((knee + 1000.0) * tn, -1000.0), (1000.0, -1000.0), (1000.0, 1000.0)])
    wedge = K(sbox(-1000.0, REST_FRONT, xo + WALL, 1000.0)).intersection(diag).intersection(sbox(-1000.0, -1000.0, 1000.0, max(housing.bounds[3], knee)))
    outer = affinity.rotate(affinity.translate(wells, -xin, 0), -YAW, origin=(0, 0)).intersection(sbox(xo - VL.U, -1000.0, xo + 1.0, 1000.0))   # the outer column's wells, in K
    face = K(sbox(xo - WALL, outer.bounds[1], xo, outer.bounds[3]))                               # a straight strip along the HINGE FACE, the outer column's lowest well to its highest
    plan = flow(unary_union([f.buffer(WALL, join_style=1, quad_segs=4) for f in foot.values()] + [face.buffer(WALL, join_style=2), wedge]))   # the face strip: the flow's closing bridged the gap between the pinky's top well and the bay with a scallop (build 63)
    plan = shoulder(plan, side, foot)                                                        # THE SHOULDER (build 134): the ogee between the bays' back walls
    plan = hinge_straight(plan)                                                                   # the hinge face ONE straight line (the flow's buffers leave it a 0.08 zigzag of 6
    #                                                                                               vertices; OCC's fillet refuses the border's round with it, and a keep straightened
    #                                                                                               on its own simplifies the corner beside it differently from the hull: build 105's
    #                                                                                               one bad fillet face at L's inner front corner, scratchpad invalid_where)
    plan, back = back_chain(plan)                                                               # THE BACK CHAIN (build 118): the back run re-drawn as lines + true arcs, G1, so OCC rounds it in one pass (velvet.backline)
    thin = {n: round(plan.exterior.distance(f), 2) for n, f in foot.items() if plan.exterior.distance(f) < WALL - 0.6}
    assert not thin, f"{side}: walls thinner than {WALL - 0.6}: {thin}"
    inner = plan.buffer(-WALL, join_style=1, quad_segs=4)                                       # inside the walls
    foot["rest"] = wedge.intersection(sbox(-1000.0, -1000.0, 1000.0, yp - 2.0)).intersection(inner)   # the REST: the wedge in front of the tilt line, inside its walls (the TPU pad sits in it)
    # THE TENT HINGE along the outer face (a straight line in K at xo + WALL): its run (the outer column's lowest well to the bay's back; the wedge's
    # outer side is on the line too but that stretch is the rest hinge's), the knuckle slots, the rail
    planK = affinity.rotate(affinity.translate(plan, -xin, 0), -YAW, origin=(0, 0))
    ys = [y for x, y in planK.exterior.coords if abs(x - (xo + WALL)) < 0.6]
    y0, y1 = max(min(ys) + ROUND, outer.bounds[1]), max(ys) - ROUND
    pitch = KNUCKLE + KN_CLR
    n = int((y1 - y0 + KN_CLR) // pitch)
    slots = [y0 + i * pitch for i in range(n)]
    strip = lambda a, b: K(sbox(ax_x - TENT_R - RAIL_W + 4.0, a - ROUND - 6.0, ax_x + 10.0, b + ROUND + 6.0)).buffer(0.3, join_style=2)   # the hatch stops short of a hinge's rail + knuckles
    restK = affinity.rotate(affinity.translate(foot["rest"], -xin, 0), -YAW, origin=(0, 0))   # the REST HINGE (Willy 23:40 'hinges on the rests to base'): 3 knuckles
    yc = (restK.bounds[1] + restK.bounds[3]) / 2.0                                              # under the rest's middle (its outer side is on the axis line all along)
    rslots = [yc - 1.5 * pitch + KN_CLR / 2 + i * pitch for i in range(3)]                     # hull, base, hull
    rest_rail = K(sbox(xo + WALL - RAIL_W, rslots[0], xo + WALL, rslots[-1] + KNUCKLE))         # its rail, like the tent hinge's (the knuckles reach 5.9 into the hollow at the hatch's level: build 72 clashed)
    rail = K(sbox(xo + WALL - RAIL_W, rslots[0], xo + WALL, y1))                                # ONE RAIL from the rest hinge's front knuckle to the tent rail's back end (build 125): the
    #                                                                                           pin bore runs through it from the front face to the back face, so a rod from either end
    #                                                                                           pushes the other hinge's pin out (both bores were blind: the rest pin's head sat 13 mm
    #                                                                                           inside a pocket it could never enter, the tent pin's 2.5 inside the back face with no way out)
    # the hatch's outline: inside the walls, stopped at both hinges' rails, off the axis line past the rails' ends (it would dip 0.1 into the base when tented)
    hatch_plan = inner.difference(strip(min(rslots[0], REST_FRONT + ROUND + 5.0), y1))          # the rest's strip from past the front edge to the tent rail's back end: one strip, the
    #                                                                                           bore between the hinges runs through solid hull (build 125; before, the hatch reached the
    #                                                                                           axis line between the rails)
    #                                                                                           front edge: ending 0.23 before inner's rounded corner it left a sliver of hatch
    #                                                                                           outline there, a 0.1 slit in the wall face, R's hull invalid (builds 73-76)
    # the flap's ears under the hatch (the whole span, and the two ears themselves)
    fy = FLAP_Y[side]
    ears = K(sbox(ax_x - FLAP_P - EAR[0] / 2, fy - FLAP_W / 2 - EAR_GAP - EAR[1], ax_x - FLAP_P + EAR[0] / 2, fy + FLAP_W / 2 + EAR_GAP + EAR[1]))
    ear2 = unary_union([K(sbox(ax_x - FLAP_P - EAR[0] / 2, fy + sy * (FLAP_W / 2 + EAR_GAP + EAR[1] / 2) - EAR[1] / 2, ax_x - FLAP_P + EAR[0] / 2, fy + sy * (FLAP_W / 2 + EAR_GAP + EAR[1] / 2) + EAR[1] / 2)) for sy in (-1, 1)])
    behind, ahead = sbox(-1000.0, yp + 6.0, 1000.0, 1000.0), sbox(-1000.0, -1000.0, 1000.0, yp + 6.0)
    keyblock = unary_union([block, housing])                                                   # the block HUGS the keys (Willy 10:13: the Picos go in the deck in front of the alphas; builds 73-78's convex hull ate it)
    kbf = unary_union([affinity.translate(keyblock, 0.0, -d) for d in range(0, 301, 6)])        # the block + the housing and everything IN FRONT of them (swept down: their own front boundary, not a hull)
    kb = kbf.convex_hull                                                                       # ... and the hull of that (the slanted back edge kept)
    # THE PALM DECK: everything in the wedge that is in front of the key block / the housing, plus the corner beside the inner thumb in front of the
    # housing, inside the walls; FLAT (the tilt leaves it), its top the rest plane, down a RAMP into the pit all round. Not the walls beside the
    # block (no key lip) and never the flat zone
    inboard = sbox(-1000.0, -1000.0, keyblock.bounds[0] + 0.5, housing.bounds[1])
    wedge_palm = wedge.intersection(unary_union([kbf, inboard]))
    # the FLAT zone (low profile, Willy 23:41 'lower the battery ... and the plastic for the charger lower than the keys'): EVERYTHING behind
    # the thumbs that is not the KEY BLOCK (the wells + WALL, closed 12, its convex hull, and the deck in front of it) or a housing (the trackpad's / the
    # knob's ride on the tilted deck) or the palm deck stays flat on the hatch at the rim; the keys alone tilt. (parts + WALL left the outline's corner
    # fills behind the cell tilting: 50-70 mm2 patches standing 20 mm over the flat band with their seam walls, build 63)
    ff = [foot["cell"], foot["display"] if side == "R" else foot["v713"]]
    flat = plan.intersection(behind).difference(kb).difference(wedge_palm).union(unary_union([g.buffer(WALL, join_style=1, quad_segs=4) for g in ff]))
    flat = flat.difference(wells.buffer(0.5, join_style=1, quad_segs=4)).difference(housing)
    # THE CONSOLE BLOCK (Willy 12:39 'where the dial is has way too much going on - simplify'; 13:22 'get rid of tilt on trackpad and make it streamline
    # with everything'): the housing (L: the knob's; R: the trackpad's) is FLAT, one plateau with the block behind it (L: the charger bay; R: the display
    # pocket + its wire bay), the gap between them closed by 8; its pocket vertical -- not a tilted drum beside a flat box with a seam wall between.
    # The housing's OWN polygon goes into flat (a re-approximated copy left 0.04 crescents of housing in the plateau but outside the flat zone: tilted,
    # they stood 3-6 over the rim round the knob as a picket fence of fins, build 85)
    blk = unary_union([housing, (foot["v713"] if side == "L" else foot["display"]).buffer(WALL, join_style=1, quad_segs=4)])
    bridge = blk.buffer(8.0, join_style=1).buffer(-8.0, join_style=1).difference(blk).intersection(plan).difference(block)   # (not the key block's wall: the pit must stay the tilted walls' foot)
    flat = unary_union([flat, blk, bridge])
    if side == "L":                                                                             # THE WEB TO THE SEAM FACE (build 145; Willy 11:34 "lets go all the way to the side wall and
        xt, yt = min(housing.exterior.coords, key=lambda q: math.dist(q, (KNOB_U - HOUSING_R / math.sqrt(2.0), KNOB_Y - HOUSING_R / math.sqrt(2.0))))   # the circle's SW vertex
        web = Polygon([(0.0, KNOB_Y + BADGE_SEAM_Y), (0.0, yt + xt), (xt, yt), (9.0, -25.0), (9.0, KNOB_Y + BADGE_SEAM_Y)])   # follow it down"): the knob's circle kisses the face (x 0) at KNOB_Y; the cusps between its
        #                                                                                       THE WEB'S SOUTH END (build 154; Willy 13:52 'smooth out that corner where the encoder ring
        #                                                                                       should smoothly transition into the side slope'): the circle's own 45 deg TANGENT from its SW
        #                                                                                       vertex to the seam face (one line, no corner against the circle; 145-153's square web end
        #                                                                                       at KNOB_Y - BADGE_SEAM_Y stood as a block beside the silo once nothing covered it)
        housing, flat = unary_union([housing, web]), unary_union([flat, web])                  # polyline and the face were in NO zone (key_deck cut one to the plate: a 0.15 x 1.7 slot 7
        #                                                                                       deep in the face) or the lower deck's (a slot under the badge). The housing and the flat zone
        #                                                                                       run to the face BADGE_SEAM_Y either way of the knob (the circle 2.5 off it there); the badge
        #                                                                                       rides it. After the bridge: closed 8 with the web's corner, blk grew a 1.5 x 6 crescent of
        #                                                                                       flat zone south of the web that the badge's band then cut to its floor (the test's 7.1)
    # THE CORNER BESIDE THE CONSOLE (Willy 15:23 'lots of holes in the encoder area ... simplicity and fluidity'): the strip between the console
    # block and the key block's wall (the notch between the key block and its convex hull, kb) tilted down to the plate: a valley beside the
    # knob, a slot and blades at its corners (build 89). It is FLAT: the zone's edge there is the key block + WALL, the seam ring one WALL band.
    gap = plan.intersection(behind).difference(block.buffer(WALL, join_style=1, quad_segs=4)).difference(wedge_palm).difference(flat)
    gap = unary_union([g for g in getattr(gap, "geoms", [gap]) if g.area > 1.0 and g.buffer(0.1).intersects(blk)])
    flat = unary_union([flat, gap])
    flat = flat.union(unary_union(ff).buffer(0.5, join_style=2)).intersection(plan)   # every flat part whole, whatever the margins took
    if side == "R":
        # THE KNEE WEDGE (build 117): between the housing's rounded SW corner and the seam strip, over the console's front line, the deck was
        # palm side (border) -- a wedge 2.5 wide, 3 long. Level with the pitched console it never showed; with the console flat at RIM_Z +
        # CONSOLE_UP it was a 5 mm notch into the console's corner (top_map R x 6-22 y -46..-34: 6.1 at (17, -40)) and the console round's band
        # swap stood a fin over it at the outline (-6 mm3 'off' the edge). It is console: flat, lifted, chamfered along the straight front line.
        fy = tp_plan(TP_CLR).bounds[1] - WALL                                                      # = tp_front_y, the console's front line
        flat = flat.union(plan.intersection(sbox(-1000.0, fy, housing.bounds[0] + 6.0, Y_P0)))      # to past the housing's rounded corner (r 5.3: a 0.5 jog at + 3)
        flat = flat.difference(sbox(-1000.0, -1000.0, housing.bounds[0] + 6.0, fy))                  # and no spike of the seam strip's old tip south of the line (0.5 x 0.8:
        #                                                                                           OCC refused the step chamfer's keep on it, scratchpad chamfer_knee)
    flat = flat.buffer(0.01, join_style=2).buffer(-0.01, join_style=2)                           # CLOSED 0.01: the unions above leave ZERO-WIDTH SPIKES in the outline (shapely keeps both
    #                                                                                           traversals of a shared edge, [[feedback_shapely_union_slits]]); the plateau's edge band
    #                                                                                           followed one 9 mm into L's flat zone north of the knob as an r 1.8 groove 1.8 deep (Willy
    #                                                                                           07:07 'a random dip ... right north of the encoder knob', build 124; L 178 -> 171 vertices)
    # the SEAM WALL's plan: a WALL ring round the flat zone, opened 0.1 (where the zone meets the outline the difference leaves zero-area
    # hairlines; prismed and capped by the tilted top they are blades in waiting); where it faces the palm deck it IS deck (below)
    ring_raw = flat.buffer(WALL, join_style=1, quad_segs=4).difference(flat).intersection(plan)
    ring_raw = ring_raw.buffer(-0.1, join_style=2).buffer(0.1, join_style=2)
    palm_raw = wedge_palm.intersection(inner).difference(keyblock).difference(flat)              # (not minus the ring: where the flat zone faces the palm deck the ring IS deck, no seam wall)
    palm_raw = max(getattr(palm_raw, "geoms", [palm_raw]), key=lambda g: g.area)
    bay = bay_line = None
    if side == "L":
        # THE ENCODER BAY, TAKE THREE (build 151; Willy 07:46 'remove the incline thats unnecessary' -- 150's fill north of the silo between column 0
        # and the flat zone -- 'and smooth out that second line so there isnt a lip coming out - just smooth straightish' -- 150's R 2.8 turn + end
        # line into the silo): THE LINE from the first thumb cap's keep-out corner nearest the knob (bay_P: where the thumb wall's face turns north)
        # straight to the silo's wall (bay_Q), heading BAY_HEAD. Everything east of it from bay_P north is the key bay's floor: the palm deck's
        # corner (cut WALL + BRIDGE_KW west of the line here, so its band ends BRIDGE_KW short of the line and the deck face's strip ON it), the
        # seam ring beside the silo, the keywell's fill (fills) -- the stub north of the silo too. The wall from the thumbs runs the line as a fill
        # wedge against the silo's flank, its top the border's level (well_top = the rest plane + BORDER_UP there), and dies on the silo's wall at
        # bay_Q: the chain's last edge ends on the wall (hull.wall_round keeps it). The wedge's tip angle is the line's angle to the wall, bay_psi.
        bay_t1 = sweep(cap_by["L_t1"]).minimum_rotated_rectangle.buffer(CAP_CLR, join_style=2)        # (= rects['L_t1'] + CAP_CLR below: the keep-out as drawn)
        bay_P = np.array(min(bay_t1.exterior.coords[:-1], key=lambda q: math.hypot(q[0] - KNOB_U, q[1] - KNOB_Y)))
        bay_u = np.array([math.cos(math.radians(BAY_HEAD)), math.sin(math.radians(BAY_HEAD))]); bay_w = np.array([-bay_u[1], bay_u[0]])   # along the line (north); its west normal
        bay_east = lambda s, t0: Polygon([tuple(bay_P + s * bay_w + t0 * bay_u), tuple(bay_P + s * bay_w + 80.0 * bay_u), tuple(bay_P + (s - 40.0) * bay_w + 80.0 * bay_u),
                                          tuple(bay_P + (s - 40.0) * bay_w + t0 * bay_u)]).intersection(sbox(25.0, -1000.0, 50.0, 1000.0))   # east of the line offset s west of it, from t0 along it
        bay_hit = LineString([tuple(bay_P), tuple(bay_P + 60.0 * bay_u)]).intersection(housing.exterior)
        bay_q = min(getattr(bay_hit, "geoms", [bay_hit]), key=lambda g: g.distance(Point(*bay_P)))
        bay_Q = np.array([bay_q.x, bay_q.y])
        bay_psi = abs((BAY_HEAD - (math.degrees(math.atan2(bay_Q[1] - KNOB_Y, bay_Q[0] - KNOB_U)) + 90.0) + 180.0) % 360.0 - 180.0)   # the line's angle to the silo's wall at bay_Q
        assert 8.0 < np.linalg.norm(bay_Q - bay_P) < 14.0 and -32.0 < bay_Q[1] < -26.0, f"L: the bay line {bay_P} -> {bay_Q}"
        assert 45.0 <= bay_psi <= 80.0, f"L: the bay line meets the silo's wall at {bay_psi:.1f} deg (the fill wedge's tip: 45 at least)"
        bay_line = (tuple(bay_P), tuple(bay_Q), bay_psi)
        palm_raw = palm_raw.difference(bay_east(WALL + BRIDGE_KW, -1.0))
    # The TILTED DECK's footprint: the key block, the walls beside it and the outline's fills round it -- everything behind the tilt line that is not
    # palm deck, flat zone, seam ring or housing; all of it at the plate level (no key lip). The PIT is its shadow once tilted (the plate-level
    # points move 1.7 back): the palm side ends there and the ramp climbs from there -- at the tilt line too (the walls beside the block stood 9.7
    # under the palm deck's walls, builds 35d7602-70)
    seam = ring_raw.difference(block)                                                       # the seam wall OUTSIDE the key block: inside it the block's own wall stays tilted
    if side == "L":                                                                             # THE ENCODER BAY (builds 150-151): no seam strip east of the silo / the flat zone
        seam = seam.difference(sbox(25.0, bay_P[1] - 1.0, 50.0, 1000.0))                         # from the bay line north -- the ring there is tilted deck: the plate runs to the
        #                                                                                           silo's wall and the flat zone's foot (the ring's strip north of the silo stood
        #                                                                                           out of the pit as a trench once its fill was gone, build 151's first plan)
    #                                                                                         (the pit is its shadow; cut out, the palm deck came 2.1 from the tilted wells, build 87's plan)
    tilted = plan.intersection(behind).difference(palm_raw.buffer(WALL, join_style=1)).difference(flat).difference(seam).difference(housing)
    tilted = unary_union([max(getattr(tilted, "geoms", [tilted]), key=lambda g: g.area), block]).difference(flat).difference(seam).difference(housing)
    tilted = max(getattr(tilted, "geoms", [tilted]), key=lambda g: g.area)
    pit = tilt_shadow(tilted, yp, 0.0)                                                          # the tilted walls' foot at the plate: the deck's face stands there
    palm = palm_raw.difference(pit)
    foot["palm"] = max(getattr(palm, "geoms", [palm]), key=lambda g: g.area)
    # THE PALM SIDE: the palm deck + its walls + everything in front of the tilt line, to the pit; the flat body. It ends at the housing (flat, in the flat zone)
    palm_side = plan.intersection(unary_union([sbox(-1000.0, -1000.0, 1000.0, yp + 6.0 + 2.0), foot["palm"].buffer(WALL, join_style=1)]))   # 2 past the tilt line: up to the pit
    palm_side = palm_side.difference(pit).difference(flat).difference(housing).buffer(-0.05, join_style=2).buffer(0.05, join_style=2)
    if side == "L":                                                                             # THE ENCODER BAY (builds 150-151): the gap between the band's buffer limit and
        bay_gap = bay_east(2.0, -1.0).difference(bay_east(0.0, -1.0)).difference(pit).difference(flat).difference(housing)   # the pit's edge along the line (the pit is
        assert bay_gap.difference(palm_side).area < 3.0, f"L: {bay_gap.difference(palm_side).area:.2f} mm2 in no zone along the bay line"   # the tilted region's SHADOW,
        palm_side = unary_union([palm_side, bay_gap]).buffer(0.01, join_style=2).buffer(-0.01, join_style=2)   # 0.6 north of it) is palm side, as 150's box was
        cusp = plan.intersection(sbox(-1.0, KNOB_Y - 13.0, 4.0, KNOB_Y - 6.0)).difference(flat).difference(housing).difference(pit)   # THE CUSP AT THE WEB'S 45 DEG END (build 154):
        assert cusp.difference(palm_side).area < 3.0, f"L: {cusp.difference(palm_side).area:.2f} mm2 in no zone at the web's end"          # between the palm side's round NW corner (palm + WALL)
        palm_side = unary_union([palm_side, cusp]).buffer(0.01, join_style=2).buffer(-0.01, join_style=2)                               # and the web's tangent line a 0.5 x 3 crescent at the seam
        #                                                                                                                                  face sat in NO zone: a fin at the rim (fin154.py). Palm side.
    if side == "R":                                                                             # THE CONSOLE'S CORNER SLIVER (build 140; Willy 22:54 "weirdness near where the
        yf = housing.bounds[1]                                                                  # trackpad meets the sidewall"): at the console's front-west corner, between the
        xw = plan.exterior.intersection(LineString([(-1000.0, yf - 1.0), (1000.0, yf - 1.0)])).bounds[0]   # knee's diagonal wall and the palm side's round corner (palm.buffer(WALL)'s
        sliver = plan.intersection(sbox(xw - 1.0, yf - 3.0, xw + 1.5, yf)).difference(flat).difference(palm_side)   # r 3 join) lies a 0.7 x 2 crescent in NO zone: key_deck cut it to the
        assert 0.2 < sliver.area < 2.5, f"R: the console corner's sliver is {sliver.area:.2f} mm2"   # plate, a 6 deep slot (scratchpad tpcorner140). It is palm side
        palm_side = unary_union([palm_side, sliver]).buffer(0.01, join_style=2).buffer(-0.01, join_style=2)
    ring = ring_raw.difference(palm_side.buffer(0.05, join_style=2)).buffer(-0.1, join_style=2).buffer(0.1, join_style=2)
    ring = unary_union([g for g in getattr(ring, "geoms", [ring]) if g.area >= 1.0])            # opened again: the palm's edge leaves hairlines
    slivers = [g for g in getattr(ring, "geoms", [ring]) if g.area < 20.0 and g.distance(palm_side) < 0.1]   # THE SEAM WALL'S SLIVERS wedged between the palm side and a flat
    if slivers:                                                                                #   block's rounded corner (L: 5 mm2 at the knee by the knob housing) are PALM SIDE, at the border's
        palm_side = unary_union([palm_side] + slivers).buffer(0.05, join_style=2).buffer(-0.05, join_style=2)   # level: as ring their top was the tilted deck's (2.4), a 1.6 x 4
        ring = ring.difference(unary_union(slivers).buffer(0.05, join_style=2))                 #   slot 5 deep in the inner wall's top between the border (7.4) and the housing
        ring = unary_union([g for g in getattr(ring, "geoms", [ring]) if g.area >= 1.0])        #   (9.9) (build 107; Willy 08:47 'weirdness on the inner left side near the encoder')
    # where the seam wall meets a tilted deck HIGHER than the block's edge it goes UP (ring_up: no edge treatment -- a round there carved a groove at
    # its foot, build 64); everywhere else the block's edge is a step DOWN to the deck and gets ONE STEP_C chamfer (ring_low; Willy 13:22 'smooth
    # coming from front also', 15:23 'simplicity and fluidity': builds 85-89 had a layered 4 mm round, then a bare middle stretch, then the wall).
    # The tilted plate's top at pre-tilt dy: dy sin a + RIM_Z (1 - cos a) - TILT_DROP; the deck meeting the wall came from ~1.7 further forward
    a = math.radians(KEY_TILT)
    dy_up = (RIM_Z - STEP_C - RIM_Z * (1.0 - math.cos(a)) + TILT_DROP) / math.sin(a) + 1.7
    ring_up = ring.intersection(sbox(-1000.0, yp + dy_up, 1000.0, 1000.0))                    # the deck within STEP_C of the rim or above it: the wall goes up
    ring_low = ring.difference(sbox(-1000.0, yp + dy_up, 1000.0, 1000.0))                     # the deck at least STEP_C below the rim: chamfered,
    ring_low = unary_union([g for g in getattr(ring_low, "geoms", [ring_low]) if g.intersects(blk.buffer(WALL + 1.0))])   # beside the CONSOLE (the outer back corner by the hinge keeps EDGE_R_TOP)
    # ... and the console's FRONT, where it meets the palm deck (the rest plane, 4.7+ under the rim there): the same chamfer, one edge all round
    front = flat.boundary.intersection(palm_side.buffer(0.3)).buffer(0.3, join_style=2).intersection(blk.buffer(WALL + 1.0))
    # THE CONSOLE PITCH (R; Willy 21:17 'angle it with the kb without raising it'): the console's top in front of the display pocket -- the housing,
    # the corner fill, the seam wall -- is one FLAT plane at RIM_Z + CONSOLE_UP (build 117; builds 97-116 pitched it down to the pad's surface + TP_LIP
    # at the housing's front wall). Its front and key-side edges carry the STEP_C chamfer (hull.step_chamfer). The zone reaches CUT_OFF past every wall.
    if side == "R":
        tp_front_y = tp_plan(TP_CLR).bounds[1] - WALL                                              # the console's front line (= housing.bounds[1])
        tp_pitch = 0.0   # THE CONSOLE FLAT (Willy 15:34 'raise the trackpad bezel and raised area to be flush with pitft bezel'): one plane at RIM_Z + CONSOLE_UP,
        #                  the bezel's brim top, over the whole island. The pitch of builds 97-116 (5.3 -> 6.39 deg from the hinge line down to the border's level
        #                  at the front) is gone; its frame (hull.inner_bay's Tt) is now the lift alone. The front edge is a STEP_C-chamfered step to the landing.
        tp_x1 = flat.intersection(sbox(tp_plan(TP_CLR).bounds[2], tp_front_y - 1.0, 1000.0, Y_P0 + 1.0)).bounds[2] + CUT_OFF   # to the FLAT zone's edge beside the
        tp_zone = plan.buffer(CUT_OFF, join_style=2).intersection(sbox(-1000.0, tp_front_y - 1.0, tp_x1, Y_P0 + 1.0))         # console: the seam ring too (it stood
        #                                                                                 1.7 wide at RIM_Z beside the pitched console, a fin up to 3 proud, that the pitched
        #                                                                                 chamfer cut sliced -- the two slivers finish dropped in builds 97-99)
        # THE CONSOLE'S FRONT EDGE, the whole of it, in step_low: flat at RIM_Z + CONSOLE_UP it steps 5.2 down to the landing, the chamfer's foot 3 over
        # the landing's top (the 2 mm notch of the pitched console, whose chamfer foot met the landing's top -- Willy 12:25; build 116 cut the front
        # out of step_low for it -- cannot recur). `front` alone is the flat zone's palm-side edge within WALL + 1 of the key block: the thumb pit's
        # stretch; the stretch along the pad's landing is added here
        front = unary_union([front, flat.boundary.intersection(palm_side.buffer(0.3)).buffer(0.3, join_style=2).intersection(sbox(-1000.0, tp_front_y - 2.0, tp_x1, tp_front_y + 2.0))])
    else:
        tp_front_y, tp_pitch, tp_zone = None, 0.0, None
    step_low = unary_union([ring_low, front]).buffer(0.05, join_style=2)                   # (joined where the ring meets the palm side)
    # THE PICO BAY in the palm deck (Willy 10:13 / 10:23 / 10:48 'closer to the thumb keys'): along K x, as far toward the thumbs as the keys allow
    # (the smallest inner end that keeps 3.5 to the pit and the housing inside the walls; then the most clearance along y), the plug reaching the
    # outer wall's window through the hollow under the pad (the 'tunnel' = the bay's strip on to xo, kept free of bosses and fill)
    full = lambda xi, yb: K(sbox(xi, yb, xo, yb + BAY_W))                                       # bay + tunnel, u
    clear = lambda g: min(pit.distance(g), housing.distance(g))
    wall_free = lambda yb: rslots[-1] + KNUCKLE + 6.0 < yb + BAY_W / 2 < y0 - 6.0                 # the window between the two hinges' knuckles
    found = None
    for d in range(0, 161):                                                                     # the inner end from xo - BAY_L inward, 0.5 steps
        xi = xo - BAY_L - 80.0 + d / 2.0
        ok = [(clear(full(xi, yb)), yb) for yb in (v / 2.0 for v in range(-200, -100)) if wall_free(yb) and inner.buffer(0.05).contains(full(xi, yb)) and full(xi, yb).bounds[1] >= yp - 2.5]   # in the DECK, behind the rest's back edge
        ok = [o for o in ok if o[0] >= 3.5]                                                      # 3.5: the pane's recess runs 0.5 past the bay -> 3 of deck to the keys' walls
        if ok:
            found = (xi, max(ok)[1]); break
    assert found, f"{side}: no room for the Pico bay in the deck"
    xi, yb = found
    pico_k = sbox(xi, yb, xi + BAY_L, yb + BAY_W)                                              # the bay proper (K)
    foot["pico"] = full(xi, yb)                                                                  # bay + tunnel (u): the keep-out, the fill's hole
    cx, cy = xi + BAY_L / 2.0, yb + BAY_W / 2.0
    recess_k = sbox(cx + 0.675 - 21.375 - LEDGE - PANE_CLR, cy - 12.5, cx + 0.675 + 21.025 + LEDGE + PANE_CLR, cy + 12.5)   # the pane's recess (K): the board's
    #                                                                                           window 4.8..47.2 along x, 21.6 wide, + ledge + clearance (hull.pico_bay builds the same)
    # THE PAD fills the palm side (Willy 10:48 'fill in more space with wrist rest') inside its BORDER (22:46 'like 2mm of border a little raised
    # around the rests'): BORDER_W inside the outline (corners concentric with the hull's), BORDER_W off the pane's recess and off the pit, CUT_OFF
    # INTO the housing's / the console's wall foot (a recess face ON a wall face would be an OCC sheet, and the undercut keeps the TPU's edge
    # captive); opened 1 (no necks thinner than 2). The pit's offset is ROUND: a mitre spiked 7 along the outer wall at the pit's acute corner
    rest_full = unary_union([foot["rest"], foot["palm"]]).intersection(inner)
    pad = plan.buffer(-BORDER_W, join_style=1, quad_segs=6).intersection(palm_side.buffer(CUT_OFF, join_style=2))
    pad = pad.difference(pit.buffer(BORDER_W, join_style=1, quad_segs=6))                   # (159-160: the pad COVERS the pane's recess; hull.pico_bay cuts the window, the ring's D and the tongue out of it)
    pad = pad.buffer(-1.0, join_style=1).buffer(1.0, join_style=1)
    pad = unary_union([g for g in getattr(pad, "geoms", [pad]) if g.area > 400.0])   # ONE TPU per side: L's 349 mm2 island inside the thumb arc beside the knob (the deck strip, cut off at the neck where
    #                                                                                     thumb arc, cut off at the neck where the diagonal is tangent to the thumb wall) is border
    # THE PIT'S OUTER-FRONT CORNER (the keywell's fin up the outer wall ends here, below; Willy 12:29 'that sharp cutoff near the z and / keys'):
    # the rim's end at the key block IS the keywell wall's end face at the outline, 7.4 over the tilted plate; a quarter-round sweep of it over
    # the band past the pad's edge (build 116 attempt 5) left a 2 wide trench 3-7 deep between the band and the landing beside it (the arc is
    # steepest at the pit's corner, where the landing is) -- reverted; the choice (a wall along the pinky column, or the crisp end) is Willy's
    near = [q for q in pit.exterior.coords if plan.exterior.distance(Point(q)) < 1.5]                 # the pit reaches the outline along the outer wall
    xc, yc = min(near, key=lambda q: q[1])
    pad_tpu = pad.buffer(-0.2, join_style=1, quad_segs=6)                              # the TPU: 0.2 off the tray's walls all round
    # THE BORDER: everything on the palm side that is not pad or the pane's recess -- the BORDER_W band inside the outline, round the pane, between
    # the pad and the pit, the landings where the pad is far from the pit -- stands BORDER_UP over the rest plane (hull.wrist_rest keeps the palm
    # side's top there; the recess is cut below). Never over a wall (the palm side ends at the housing's / console's face: the console's pitched
    # front meets this level, TP_LIP)
    border = palm_side.difference(pad)                                                          # (the pane's recess is inside the pad since 160)
    # THE KEYWELL: the pit's margin on the pad's side -- the key block's wall top, at the plate, 2 wide between the deck face and the caps, 6-8 deep
    # (Willy 22:46 'this deep channel') -- filled to the border's level (hull.deck_edge) CAP_CLR off the caps, never inside the wells (between caps).
    # The caps and wells are tilted: each cap's sweep from the plate to 10 over it (the convex hull of its two shadows: the union of a turned
    # rectangle's two copies has notches 1.4 deep at the thumbs), the wells' shadows every 2.5; 0.3 into the palm side so it fuses with the deck face
    # THE KEEP-OUT IS ONE CLEAN POLYGON (build 147): one swept RECTANGLE per COLUMN of alphas (the column's caps are collinear: the rectangle
    # round their swept hulls' union -- the gaps between caps and the wells inside it are keep-out by construction) and one per thumb cap, each
    # + CAP_CLR with MITRE corners; adjacent columns' rectangles overlap 0.6 (pitch 19 vs 18.4 + 1.2), the thumbs' meet their neighbours'.
    # Every corner of the union is a real one (a column's end, a stagger step, a thumb's edge); smooth_open draws the wall round them
    strip = palm_side.buffer(BRIDGE_KW, join_style=2).simplify(0.15)                            # THE DECK FACE'S STRIP: BRIDGE_KW of fill before the face, ONE line
    #                                                                 (the bridge's limit, the open pit's hard limit; 0.15-simplified: the pit polyline's 0.26-0.34 chords at 3-6 deg)
    # THE ENCODER BAY, TAKE THREE (build 151): the deck face's strip stays west of the bay line (in 150 the band's buffer corner bulged 0.23 past the
    # face; the strip bounds the keep-out below and clips the open pit), and `bay` -- the floor east of the line from bay_P north -- is cut out of
    # the keywell (fills): the open pit runs to the line and on along the silo's flank and the flat zone's wall
    if side == "L":
        strip = strip.difference(bay_east(0.0, -1.0)).buffer(0.01, join_style=2).buffer(-0.01, join_style=2)
        bay = bay_east(0.0, 0.0)
    cols, col_of = {}, lambda k: k.split("_")[1].split("c")[1] if "c" in k.split("_")[1] else k
    for k, c in cap_by.items():
        cols.setdefault(col_of(k), []).append(sweep(c))
    rects = {k: unary_union(g).convex_hull.minimum_rotated_rectangle for k, g in cols.items()}
    keepout = caps_ko = unary_union([g.buffer(CAP_CLR, join_style=2) for g in rects.values()])   # (caps_ko: the caps' own, before the bridges)
    # THE RED LINE (build 146; Willy 13:22 "the key wall should follow this line instead of digging into the key area"): where the thumb arc curves away
    # from the inner column's bottom alpha the fill ran in between the two caps (a tongue to (50, -33) on L, its wall 10 into the key area); the pair's
    # sweeps' convex hull joins their keep-outs, so the wall runs cap corner to cap corner, BRIDGE_KW off the deck's face
    for a, b in {"L": (("L_r2c0", "L_t1"),), "R": ()}[side]:
        end = rects[col_of(a)].intersection(sweep(cap_by[a]).buffer(0.5, join_style=2))          # the column's rectangle at its bottom cap: the hull's chord
        bridge = unary_union([end.buffer(CAP_CLR, join_style=2), rects[col_of(b)].buffer(CAP_CLR, join_style=2)]).convex_hull   # leaves the column's own edge
        #                                                                 and meets the caps' keep-outs AT their corners (the hull of the bare rectangles buffered after put its
        #                                                                 mitre corner 0.29 off the thumb's: a 0.29 step + a 0.003 vertex that the closing's mitre dilation turned
        #                                                                 into a 0.14 tooth on the strip -- the one spot that refused the wall round's chain; the sweeps' own
        #                                                                 facets had put 0.3-1.3 edges at 12-29 deg on the hull, build 147)
        keepout = unary_union([keepout, bridge.difference(strip)])                              # ... but never to the deck's face: the chord ran 0..0.5
        #                                                                 off it (a 0.4 pilaster of keywell at each end, and between them the plate's edge 0.1-0.25 short of the
        #                                                                 face -- tilt's spare covers the pit's x-running edges only -- open to the cavity): BRIDGE_KW of keywell stays
    # ... EVERYWHERE round the key block -- along the palm side (3 past it), beside the seam ring and the knob / display housings (on L the alphas'
    # inner side faces the seam ring and the knob: an 8 deep slot, Willy 23:13 'missing the same coverage of the channel on the left side') -- but
    # not beside the pitched console (its chamfer runs down to the plate; R's inner side, Willy 23:13 'right side is now correct') nor along the
    # outer wall (the block reaches it: the wall stays low); it ends at the housings' faces and the flat zone (hull caps the fill at KEYWELL_MAX)
    def fills(cut):
        """The pit's margin fills -- the keywell, the lip, the rim strip's corner -- with `cut` taken out: the caps' keep-out on the first pass,
        THE OPEN PIT on the second (build 147: the wall is that one line and nothing else draws it)."""
        zone = unary_union([palm_side.buffer(3.0, join_style=2), inner.buffer(-0.5, join_style=2)])
        keywell = pit.buffer(0.3, join_style=2).intersection(zone).intersection(plan).difference(cut).difference(housing).difference(flat)   # inside the outline (the pit's corner at the outer wall: 6 mm3 outside, build 103)
        # ... and the RING'S STRIP between a flat block's chamfer foot and the pit (L's inner column: the housing's step came down 1.25 to a 2.5 wide
        # trench at the plate, then the fill stood 1.4 over it 2.5 mm off -- Willy 12:25 'an extra key well wall near the b and g key that feels hanging
        # and disconnected from the wall system'; scratchpad fine_profile / topmap_L_gb): the fill runs to the foot, the block steps once onto it
        strip = ring_low.difference(flat).difference(housing).intersection(pit.buffer(WALL + 1.5, join_style=2)).intersection(plan)
        keywell = unary_union([keywell, strip.difference(cut)])
        # THE LIP (Willy 14:16 'the lip coming off the wrist rests going to the rest of the board offers nice opp for smoothness'; 12:29 'that sharp
        # cutoff near the z and / keys'): the rest's rim runs ON along the key block's outer wall -- the pit's margin between the caps and the outline
        # (the block reaches the outline there: a 3.3 wide wall, CAP_CLR off the pinky column) is keywell too, filled to the border's level from the
        # pit's corner to the back band, its outer top edge rounded like the rest's (hull.lip_round). Before: the wall stayed low (the tilted plate's
        # edge, 6.8 under the rim at the corner), the fill ran 1.6 short of the outline as a 0.8 fin (builds 103-115), and the rim ended in the cliff.
        lip = pit.buffer(0.3, join_style=2).intersection(plan).intersection(plan.exterior.buffer(WALL + MARGIN + CAP_CLR + 0.5))
        lip = lip.intersection(sbox(0.55 * plan.bounds[2], yc - 1.0, 1000.0, 1000.0)).difference(cut).difference(housing).difference(flat)
        lip = max(getattr(lip, "geoms", [lip]), key=lambda g: g.area)   # the run's strip alone: the band also catches the pit's margin at the back wall's corner (18 mm2, 17 from the run)
        # THE RIM STRIP (build 140; Willy 23:14 "this ugly block was exposed with that change fix it - i love the curve now"): with the swell off (item 88)
        # the plateau's corner behind P / Q -- the flat between the pit's diagonal and the outline -- and the lip's tab in the pinky stagger's notch stood
        # 1 .. 2 over the tilted plate as a block. The rim round the corner is a STRIP_W strip (the lip's own width); the flat and the lip beyond it, north
        # of the lip's run beside P, are `corner`: left out of the lip's and the keywell's fills and cut down to the plate (hull.deck_edge), so the lip's
        # inner wall runs on round the corner and dies out into the back band where the plate rises through the rim
        corner = unary_union([flat, lip]).intersection(sbox(pit.bounds[2] - 15.0, 7.0, 1000.0, 1000.0)).difference(plan.exterior.buffer(STRIP_W))
        corner = unary_union([g for g in getattr(corner, "geoms", [corner]) if g.geom_type == "Polygon" and g.area > 1.0])
        assert 20.0 < corner.area < 40.0 and corner.distance(pit) < 0.5, f"{side}: the rim strip's corner is {corner.area:.1f} mm2, {corner.distance(pit):.2f} off the pit"
        lip = lip.difference(corner)
        keywell = unary_union([keywell, lip])
        keywell = keywell.difference(corner)
        if tp_zone is not None:                                                                            # only beside the console's KEY-SIDE edge (the ring's strip there too), from 1 north of its front
            north = sbox(-1000.0, tp_front_y + 1.0, 1000.0, 1000.0)                                        # face line: along the FRONT the fill runs INTO the console's face and round its front-east corner
            keywell = keywell.difference(tp_zone.buffer(WALL + 1.5, join_style=2).intersection(north))   # to 1 off the alpha well's clearance (Willy 06:21 'only half width of the key wall actually
            #                                                                                                reaches the trackpad wall': build 121's tooth fix (the band from y -1000 for x > 15) had taken
            #                                                                                                the thumb wall's fill 4.5 off the console and the 2 mm border alone reached it; builds 117-120's
            #                                                                                                band from y = tp_front_y - 1 left a 1 mm slot along the face and the 6 x 2 tooth at x 58-64)
            #                                                                        nor beside the console's HOUSING WALL (Willy 10:53 'the extra little wall next to the h and n
            #                                                                          keys'): the fill there rose to KEYWELL_MAX, a 1 mm rib 1.4-2.8 over the tilted plate from y -8 to 6 with the
            #                                                                          seam's trench behind it (scratchpad seam_map / rib_steps: deck_edge's fill); the console's edge meets the plate on its own
        if bay is not None:                                                                             # THE ENCODER BAY (build 150): the silo's flank on the floor
            keywell = keywell.difference(bay)
        keywell = unary_union([g for g in getattr(keywell, "geoms", [keywell]) if g.area > 1.0 and g.distance(pit.exterior) < 0.5])   # only pieces ON the pit's
        #                                                                          wall: the zone's `inner - 0.5` reaches the whole pit, and every gap between key groups
        #                                                                          wider than two caps' clearances filled to the border's level -- an 8 x 8 tower 6 over the
        #                                                                          plate between R's N key and the thumb keys (Willy 13:50 'the leftover wall between the n
        #                                                                          key and the thumb keys'; topmap: 6.5-7.1 at x 76-85, y -36..-28 over a 0.1-1.4 plate)
        return keywell, lip, corner
    keywell, lip, corner = fills(keepout)
    # THE OPEN PIT (build 147; Willy 13:22 'use the same smooth round edges we have around the outside on the key wall', 18:57 'B!!'): ONE polygon, the
    # region cut to the plate, re-drawn so the wall's top edge takes OCC's EDGE_R_TOP round -- every corner an arc of WALL_R >= the round
    # (smooth_open), inside the pit, off the housings / the flat zone / the deck's face (BRIDGE_KW); the fills are derived from it
    open_raw = pit.difference(unary_union([keywell, lip, palm_side, flat, housing])).union(corner)   # (+ the rim strip's corner: cut to the plate by deck_edge)
    open_raw = max(getattr(open_raw, "geoms", [open_raw]), key=lambda g: g.area)
    polys = lambda g: unary_union([q for q in getattr(g, "geoms", [g]) if q.geom_type == "Polygon"])   # (polygons only: the bay line's end on the silo's wall left a
    #                                                                                                  0.1 mm hairline in the clip, a GeometryCollection, build 151)
    clip = lambda g: polys(g.intersection(pit.union(corner)).difference(housing).difference(flat.difference(corner)).difference(strip))   # (the hard limits: a disc at
    #                                                                 a cap corner BRIDGE_KW off the face pokes 1.0 past the corner's edges, the closing's fill at a 90 deg deck-face
    #                                                                 corner comes 0.4 from the face -- the strip stays)
    wall_zone = pit.buffer(1.0, join_style=2).difference(lip.buffer(0.5, join_style=2)).difference(flat.buffer(1.5, join_style=2)).difference(housing.buffer(1.5, join_style=2))   # THE KEY WALL'S RUN: the
    if bay is not None:                                                                        # (builds 150-151: L's run ends where the bay line meets the silo's wall --
        wall_zone = wall_zone.difference(sbox(25.0, bay_Q[1] + 0.5, 50.0, 1000.0))                 # north of it the pit's edge is the silo's flank and the flat zone's wall)
    if tp_zone is not None:                                                                    # deck's face, the thumbs' bowl, the inner side -- not the lip (two
        wall_zone = wall_zone.difference(tp_zone.buffer(WALL + 2.0, join_style=2))                 # rounds on 3.3), the 0.9 back strip by the flat zone, R's console
    #                                                                 (its own round); ONE stretch per side (a gap in the run is two chain ends 4 mm apart: OCC refused the pair)
    s_zone = wall_zone.intersection(pit.union(corner)).difference(housing).difference(flat.difference(corner)).difference(strip)   # (where an S may lie whole)
    open_pit = clip(smooth_open(clip(open_raw), WALL_R, s_zone)).simplify(0.015)   # (0.015: the unions' collinear junk; 0.03 shaved a cap's clearance by 0.03)
    open_pit = drop_short(max(getattr(open_pit, "geoms", [open_pit]), key=lambda g: g.area), caps_ko.intersection(pit))   # (the caps' keep-out guards
    #                                                                 the drops, not the bridges': the red line's vertices on the strip are the flat ones to drop)
    missing = keepout.intersection(pit).difference(housing).difference(flat).difference(strip).difference(open_pit)
    assert missing.buffer(-0.03, join_style=2).is_empty, f"{side}: the open pit does not hold the caps' keep-out ({missing.area:.2f} mm2 at {[round(v, 1) for v in missing.bounds]})"
    keywell, lip, corner = fills(open_pit)
    open_pit = pit.difference(unary_union([keywell, lip, palm_side, housing, flat.difference(corner)])).union(corner)   # AS BUILT: the fills' complement
    open_pit = max(getattr(open_pit, "geoms", [open_pit]), key=lambda g: g.area)                                    # (deck_edge fills keywell + lip, cuts corner)
    open_pit = dedup_ring(open_pit.simplify(0.015))                                                                   # (the fills' crossings put collinear and 0.003 vertices on the line)
    assert pad.area > 0.6 * rest_full.area, f"{side}: the pad is only {pad.area / rest_full.area:.0%} of the rest + deck"
    Ki = lambda g: affinity.rotate(affinity.translate(g, -xin, 0), -YAW, origin=(0, 0))
    run = back_run(plan, K, Ki, xo, y1)
    # the BACK ROUND's zone (hull.key_deck): BACK_R + 2 round the back run, not past the seam corner (the seam wall's top edge keeps EDGE_R_TOP)
    # ... and ON DOWN THE OUTER WALL to the lip's run (build 126: between the run's end at the corner arc and the lip's run, y 7..13, the wall's top
    # edge had neither round -- the lip's fill (deck_edge) re-squared it, 0.67 proud of the arc at 0.4 in; Willy 07:50 'messed up edges near the
    # outer battery corner, both sides'). wall_run is the outline there: hull.lip_round's run reaches it, 1 short of the corner arc's start.
    # (Build 128 put the wall in back_zone too: at key_deck the lip is still pit, void, and the band bridged it; the tilt swung the bridge off as a floater.)
    lip_top = plan.exterior.intersection(lip.buffer(0.1, join_style=2)).bounds[3]                     # the lip's run ends here, up the outer wall
    x_top = max(plan.exterior.coords, key=lambda q: q[0])                                             # the wall's top end: the corner arc starts
    wall_run = plan.exterior.intersection(sbox(plan.bounds[2] - 10.0, lip_top - 0.5, 1000.0, x_top[1] - 1.0))
    wall_run = max(getattr(wall_run, "geoms", [wall_run]), key=lambda g: g.length)
    assert wall_run.length > 3.0 and lip_top < x_top[1] - 3.0, f"{side}: the wall run between the lip ({lip_top:.1f}) and the corner ({x_top[1]:.1f}) is {wall_run.length:.1f} long"
    back_zone = run.buffer(BACK_R + 2.0).difference(sbox(-10.0, -1000.0, BACK_R + 2.5, run.coords[-1][1] - 0.01))   # (a clip at the corner's y killed R's band: it
    #                                                                                                                      sits 20 lower than the display's back, build 85)
    # THE SWELL (build 132; Willy 12:37 'the p key and q are not sitting in secure slots', 13:22 '2. lip follows the plate'): the outer-back corner's
    # zone -- the lip, the pinky stagger's notch behind P / Q and the plateau's corner, from 0.5 inside plane B's foot (x_b: 1.7 inside the cell bay's
    # wall, where the swell is at the rim) to the outline, CAP_CLR off the caps (keepout) -- filled by hull.deck_edge up to the lower of planes A and B
    # plane B's foot: where the A / B crease crosses the outline is set to the MIDDLE of the back run's straight beside the corner -- a crease at a
    # junction of the back chain split an arc into a 0.5 mm edge and OCC refused the round (build 132); it lands ~1.7 inside the cell bay's wall either way
    near = [e for e in back if not e.box and min(e.p0[1], e.p1[1]) > 15.0 and max(e.p0[0], e.p1[0]) > plan.bounds[2] - 25.0 and max(e.p0[0], e.p1[0]) < plan.bounds[2] - 2.0]
    straight = max(near, key=lambda e: max(e.p0[0], e.p1[0]))                                       # the back run's element right beside the corner arc (a shallow arc, ~7 mm, nearly level)
    xm, ym = (straight.p0[0] + straight.p1[0]) / 2, (straight.p0[1] + straight.p1[1]) / 2
    x_b = xm - (plate_z({"yp": yp}, ym) + LIP_UP_LEGACY - RIM_Z) / math.tan(math.radians(SWELL_FALL_LEGACY))   # THE ROLL (build 139): the cylinder's foot, tangent to the rim where
    #                                                                                               builds 132-138's plane B had its foot (the zones unchanged: R 151.7 / L 124.0)
    h_x = plate_z({"yp": yp}, ym) + LIP_UP - RIM_Z                                                # plane A over the rim at the crease crossing (xm, ym)
    swell_R = ((xm - x_b) ** 2 + h_x ** 2) / (2.0 * h_x)                                          # the cylinder through the crossing, tangent at x_b: R = (d^2 + h^2) / 2h
    assert 0.5 < h_x < 6.0 and 20.0 < swell_R < 300.0, f"{side}: the roll from x_b {x_b:.1f} to the crossing ({xm:.1f}, {ym:.1f}) {h_x:.2f} over the rim needs R {swell_R:.0f}"
    assert 1.0 < x_b - foot["cell"].bounds[2] < 8.0, f"{side}: plane B's foot {x_b:.1f} vs the cell bay's wall {foot['cell'].bounds[2]:.1f} (the crease on {straight.kind} {straight.p0} -> {straight.p1}; candidates {[(e.kind, round(e.p0[0]), round(e.p0[1]), round(e.p1[0]), round(e.p1[1]), round(e.length(), 1)) for e in near]})"
    swell = plan.intersection(sbox(x_b - 0.5, -10.0, 1000.0, 1000.0)).difference(open_pit).difference(housing)
    swell = unary_union([g for g in getattr(swell, "geoms", [swell]) if g.geom_type == "Polygon" and g.area > 1.0])
    swell_pit = swell.intersection(pit.buffer(0.3, join_style=2))                                   # its part on the tilted plate (filled from the plate; the plateau's from 2 under the rim)
    swell_pit = unary_union([g for g in getattr(swell_pit, "geoms", [swell_pit]) if g.geom_type == "Polygon" and g.area > 1.0])
    if not SWELL:                                                                                   # THE SWELL OFF (build 139): the zones empty, the corner is plateau
        swell, swell_pit = Polygon(), Polygon()
    plateau = unary_union([flat, palm_side]).intersection(plan).buffer(0.01, join_style=2).buffer(-0.01, join_style=2)   # closed 0.01 too (14 zero-area slits on L, build 124)
    plateau = unary_union([g for g in getattr(plateau, "geoms", [plateau]) if g.geom_type == "Polygon" and g.area > 1.0])   # (polygons only: the knee wedge's union left two zero-area slivers, build 117)   # the PLATEAU (the housings are in flat) (no key lip, Willy 21:44): the top keeps the rim on the flat zone, the housing and the palm side (wrist_rest cuts its slope from it); a flat patch at deck level would be a 9.9 mm trench
    skin = plan.intersection(sbox(-1.0, -1000.0, 0.6, 1000.0))                                 # THE PLATEAU SNAPS TO THE SEAM FACE (build 145): a zone opened 0.05 / 0.1 at a corner ON the
    plateau = unary_union([plateau, skin.intersection(plateau.buffer(0.6, join_style=2))]).buffer(0.01, join_style=2).buffer(-0.01, join_style=2)   # (closed 0.01: two zero-width spikes, test_flat_zone_has_no_spikes)     # face leaves a 0.02-0.16 hairline between it and the face; outside the plateau, key_deck cut
    plateau = unary_union([g for g in getattr(plateau, "geoms", [plateau]) if g.geom_type == "Polygon" and g.area > 1.0])   # it to the plate (L's face at y -31..-25: a 0.02 recess under the
    #                                                                                           tilted deck's foot). Within 0.6 of the face the plateau reaches it. (The whole outline: the
    #                                                                                           lip's run on the outer wall split the hull in lip_round, 3 ghost shells)
    return dict(open_pit=open_pit, wall_zone=wall_zone, bay=bay, bay_line=bay_line, cap_by=cap_by, fr=fr, fu=fu, caps=caps, wells=wells, xin=xin, foot=foot, cell_k=cell_k, v713=v713, plan=plan, K=K, inner=inner, hatch_plan=hatch_plan, plateau=plateau, ears=ears, ear2=ear2, flat=flat, ring=ring, housing=housing, block=block, tilted=tilted, pit=pit, palm_side=palm_side, wedge=wedge, yp=yp, ring_up=ring_up, ring_low=ring_low,
                xo=xo, y0=y0, y1=y1, slots=slots, hull_kn=slots[0::2], base_kn=slots[1::2], ax_x=ax_x, rail=rail, rslots=rslots, rest_rail=rest_rail, knee=knee, pad=pad, pit_corner=(xc, yc), lip=lip, corner=corner, back_run=run, pad_tpu=pad_tpu, border=border, keywell=keywell, pico_k=pico_k, recess_k=recess_k, rest_full=rest_full, back_zone=back_zone, wall_run=wall_run, back_chain=back, step_low=step_low, tp_pitch=tp_pitch, tp_zone=tp_zone, tp_front_y=tp_front_y, swell=swell, swell_pit=swell_pit, x_b=x_b, swell_R=swell_R)


_PLANS = {}


def ysweep(g, dy):
    """g swept along +y by dy -- the exact Minkowski sum with the segment: g plus one parallelogram per edge. The convex hull of two shadows for
    a cap; the true envelope for a concave shape (the wells: a stack of five shadows was a 0.4 staircase along every slanted edge, build 147)."""
    parts = [g]
    for poly in getattr(g, "geoms", [g]):
        for ring in [poly.exterior] + list(poly.interiors):
            cs = list(ring.coords)
            for (ax, ay), (bx, by) in zip(cs, cs[1:]):
                q = Polygon([(ax, ay), (bx, by), (bx, by + dy), (ax, ay + dy)])
                if q.area > 1e-9:
                    parts.append(q.buffer(0))
    return unary_union(parts)


def dedup_ring(poly, eps=0.02):
    """The polygon's exterior with consecutive vertices closer than eps merged (simplify keeps a ring's first and last point, 0.003 apart)."""
    pts = []
    for q in list(poly.exterior.coords)[:-1]:
        if not pts or math.dist(q, pts[-1]) > eps:
            pts.append(tuple(q))
    if len(pts) > 1 and math.dist(pts[0], pts[-1]) <= eps:
        pts.pop()
    return Polygon(pts, [list(h.coords) for h in poly.interiors])


def drop_short(o, protect, maxlen=0.4, flat=8.0):
    """The loop's CONCAVE vertices on an edge shorter than maxlen dropped, and its NEAR-COLLINEAR ones (turning under `flat` deg, off the line by
    under 0.3) either way -- unless within 0.05 of `protect` (the caps' keep-out). Dropping a concave vertex only ADDS open area; a flat one moves
    the line under 0.3, off the caps. The closing's kinks: where it swallows a fanned thumb pair's fill needle the dilation's notch at the mouth is
    not rho from the edges round it, so its fan meets them at a corner (a 0.18 edge between -36 and +22 deg, build 147); an S arc meeting the
    strip's line leaves 0.25. The flat ones: a 5 deg bend between two y-running edges of the strip's line under the rest's sloped top refused the
    wall round's chain at every radius while the same loop under a flat top took it, and so did the loop with that vertex dropped or moved 0.3
    (OCC's corner blend at a nearly straight vertex is ill-conditioned there; scratchpad wallcheck147n)."""
    o = orient(o, 1.0)
    cs = [tuple(q) for q in o.exterior.coords[:-1]]
    guard = protect.buffer(0.05, join_style=2)
    while True:
        n = len(cs); best = None
        for i in range(n):
            p, a, b = cs[i - 1], cs[i], cs[(i + 1) % n]
            u, v = (a[0] - p[0], a[1] - p[1]), (b[0] - a[0], b[1] - a[1])
            cross = u[0] * v[1] - u[1] * v[0]
            turn = abs(math.degrees(math.atan2(cross, u[0] * v[0] + u[1] * v[1])))
            off = LineString([p, b]).distance(Point(a))
            short = min(math.dist(p, a), math.dist(a, b))
            if ((cross < 0.0 and short < maxlen) or (turn < flat and off < 0.3)) and not guard.covers(Point(a)):
                if cross > 0.0 and protect.intersection(Polygon([p, a, b]).buffer(0)).area > 0.01:   # a convex drop cuts the open side: not a cap's clearance
                    continue
                if best is None or short < best[1]:
                    best = (i, short)
        if best is None:
            return Polygon(cs, [list(h.coords) for h in o.interiors])
        cs.pop(best[0])


def s_wedge(a, b, c, d, R, sgn, where):
    """The open area under a tangent S of two R arcs across a STEP of the open pit's line: V = b (convex) and C = c (concave) with the riser b-c
    between them, shorter than 2R. The S leaves V ALONG V's own edge (the convex arc tangent there: no clearance cut) and lands tangent on C's
    edge dx on: a wedge of material dies beside the shorter end. sgn -1 reads the loop backwards (the step the other way round)."""
    e = (b[0] - a[0], b[1] - a[1]); le = math.hypot(*e)
    u = (d[0] - c[0], d[1] - c[1]); lu = math.hypot(*u)
    if le < 0.3 or lu < 0.3:
        return None
    e = (e[0] / le, e[1] / le); u = (u[0] / lu, u[1] / lu); n = (-sgn * e[1], sgn * e[0])              # n: into the open region
    tv = sgn * math.degrees(math.atan2(e[0] * (c[1] - b[1]) - e[1] * (c[0] - b[0]), e[0] * (c[0] - b[0]) + e[1] * (c[1] - b[1])))
    w = (c[0] - b[0], c[1] - b[1]); lw = math.hypot(*w)
    tc = sgn * math.degrees(math.atan2(w[0] * u[1] - w[1] * u[0], w[0] * u[0] + w[1] * u[1]))
    h = w[0] * n[0] + w[1] * n[1]; sx = w[0] * e[0] + w[1] * e[1]
    if not (tv > 25.0 and tc < -25.0 and e[0] * u[0] + e[1] * u[1] > 0.98 and 0.3 < h < 2.0 * R - 0.3):
        return None
    dx = math.sqrt(4.0 * R * h - h * h) - sx                                                   # the two arcs tangent: centres 2R apart
    if not (0.5 < dx < lu - 0.3):
        return None
    A = (b[0] + R * n[0], b[1] + R * n[1]); T = (b[0] + (sx + dx) * e[0] + h * n[0], b[1] + (sx + dx) * e[1] + h * n[1]); B = (T[0] - R * n[0], T[1] - R * n[1])
    def arc(ctr, p0, p1, ccw):
        a0, a1 = math.atan2(p0[1] - ctr[1], p0[0] - ctr[0]), math.atan2(p1[1] - ctr[1], p1[0] - ctr[0])
        sweep = (a1 - a0) % (2.0 * math.pi) if ccw else -((a0 - a1) % (2.0 * math.pi))
        k = max(1, math.ceil(abs(sweep) / math.radians(22.5)))
        return [(ctr[0] + R * math.cos(a0 + sweep * j / k), ctr[1] + R * math.sin(a0 + sweep * j / k)) for j in range(1, k + 1)]
    M = ((A[0] + B[0]) / 2.0, (A[1] + B[1]) / 2.0)
    w = Polygon([b] + arc(A, b, M, sgn > 0) + arc(B, M, T, sgn < 0) + [c]).buffer(0)
    return w if where.buffer(0.05).contains(w) else None                                       # (whole: an S across the deck face's strip is a clipped S)


def smooth_open(o, rho, where):
    """THE OPEN PIT'S LOOP made of arcs no tighter than rho, so the wall's top edge takes OCC's round (build 147: a chain fillet of radius r needs
    every edge L >= r (tan a/2 + tan b/2) between its end turns a and b -- arcs need radius >= r, jogs, steps and tabs under 2r refuse the whole
    chain; a 90 deg corner between long edges is fine): every STEP shorter than 2 rho inside `where` becomes a tangent S of two rho + 0.1 arcs
    (s_wedge; the column staggers at the keywell: 1.2 to 4.4), then a closing of rho -- MITRE dilation, round erosion at 22.5 deg: the concave
    corners filled with rho arcs, the convex ones come back EXACT (a round dilation's arc fans eroded leave a 0.2 / 0.2 nick at every convex
    vertex, on a lone rectangle too). Discs at the convex corners (a 'bay' round each cap corner) crossed the edges at 36 deg and left 0.4-0.5
    chords there: dropped. Not inside `where` (the lip's inner edge keeps its stagger steps: two rounds on a 3.3 wide top is a later chapter) the
    steps stay. All of it only ADDS open area: the caps' clearance is never cut."""
    o = orient(dedup_ring(o), 1.0)                                                              # (0.02-deduped, not simplified: 0.05 off the line is 0.05 off a cap's clearance)
    cs = list(o.exterior.coords)[:-1]
    n, wedges = len(cs), []
    for i in range(n):
        a, b, c, d = cs[i - 1], cs[i], cs[(i + 1) % n], cs[(i + 2) % n]
        for seq, sgn in (((a, b, c, d), 1.0), ((d, c, b, a), -1.0)):
            w = s_wedge(*seq, rho + 0.1, sgn, where)
            if w is not None:
                wedges.append(w)
    return unary_union([o] + wedges).buffer(rho, join_style=2).buffer(-rho, join_style=1, quad_segs=4)


def plan_of(side):
    if side not in _PLANS:
        _PLANS[side] = half_plan(side)
    return _PLANS[side]
