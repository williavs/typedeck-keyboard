"""THE LIGHTS AND THE CROWNS (builds 152-153).

Build 152 (Willy 2026-10-15 23:50 'holes to feed the anode and cathode through the body', 23:51 '30mm cob leds - 2mm in diameter', 23:54 'the
tracks should be longer than the led, the led just provides light to the track ... really cool trace like pcb design and the bezel over top make it
line up perfect - the cover for the light fixture can double as the bezel'): on each battery plateau two lit NETS -- PCB-trace light guides printed
clear, each a T-section (TRACE_W up through the cover's slot, flush with its top; a GROOVE_W foot in a GROOVE_D groove in the plateau, captive under
the cover), lit by a 30 x 2 COB filament in a channel in the guide's underside on a long straight run; the filament's leads run in a 1 x 1 channel
under the guide to the net's VIAS -- a PAD_D pad over a HOLE_D lead hole. THE FILAMENT: no catalog STEP (step.parts: 0 hits in 4 searches; KiCad
packages3D: none) -- a plain cylinder from Willy's spec, labelled GUESS.

Build 153 (Willy 2026-10-16 10:44-11:13: 'bring the pitft bezel and new bezel together to make one single cap piece so its more cohesive and
secure - traces on tft bezel should continue same raised and filled styling'; 'same with the encoder plateau side and lefts new bezel - bring it
all together'; 'the trace design should carry into the encoder plateau also and replace the two 3mm leds in the left side'; 'replace old design
with diffuser fixture that fits the 3mm leds on the right side and bring the bezel together with the pitft one, should be flush with case not
raised'; 'if we bring the trackpad border into the bezel too, we can create support on the bottom so i can place it from top and then put thin
piece of different colored pla to indicate the trackpad area. we could bezelfy all the components on right into one separate piece'; 11:13 'all
lids become one piece', 'trackpad overlay sunken in, same approach and fit as the bezel lid', 'pegs on cover and holes in case'): THE CROWNS -- one
printed plate per half, CROWN_T thick on the rim, its top edges r CROWN_R, silkscreen RIDGES, LID_PIN pegs into holes in the case.
  R: the PiTFT bezel's body + its plate + the LED wing (the 3 mm LEDs' tray in the seat floor, a clear DIFFUSER in it whose pegs run on as ARMS
     onto the bezel plate: 'raised and filled') + the TRACKPAD BORDER (a window the pad drops into from the top, a collar + a LEDGE under its two
     side edges, a thin OVERLAY of another colour sunk in the window) + the plateau lid -- FLUSH with the raised console: the hull under all of it
     is cut back to the rim (hull: the console's lift leaves the crown's seat out).
  L: the badge plate + the plateau lid in one plate on the rim (the hull flat under it, the knob well through it); the two 3 mm LEDs GONE -- the
     magenta and cyan nets run on from the lid down the shoulder corridor at 45 deg, a filament on each (the ENCODER RUNS), their vias drilled to
     the cavity.
The lid face (the 152 nets, ridges, pins) stays in K (the key frame: x along the top row, the same on both halves); the encoder runs, the arms and
the crowns are in the half's PLAN frame (crowns153.png: drawn over build 152's hull and checked there)."""
import math

from build123d import Align, Box, Cylinder, Pos, Rot, extrude, fillet, make_face, Polyline
from shapely import affinity
from shapely.geometry import LineString, Point, Polygon, box as sbox
from shapely.ops import unary_union

from velvet.params import *  # noqa: F401,F403
from velvet.geo import prism, tp_plan

# THE LID FACE, K frame (lights_v2.png): the two nets via -> via (the vias in the bore's lead rooms, x -13..-10 and 55..58), 45 deg jogs, serpentines
NETS_K = {"cyan": LineString([(-11.2, 40.0), (-5.0, 40.0), (-2.0, 43.0), (38.0, 43.0), (41.0, 40.0), (45.5, 40.0), (47.0, 41.5), (50.5, 41.5), (52.0, 40.0), (56.2, 40.0)]),
          "magenta": LineString([(-11.2, 48.3), (-9.9, 47.0), (56.2, 47.0)])}   # (156: magenta STRAIGHT on y 47 to its east via -- 152-155 jogged out to 48.5 / 48.3 for 5.4 + 12.9 mm, the
#                                 groove 2.0 from the crown's back edge: Willy 18:21 "too close to the outside edges"; the pin (57.2, 44.2) between the vias gone with it) (154: the
#                                 y 47 run 39 long -- the 30 filament + FIL_TAB each side straight; 152's ran -7.2..24: 0.5 past the filament's ends)
FILAMENTS_K = {"cyan": LineString([(3.0, 43.0), (33.0, 43.0)]), "magenta": LineString([(-5.4, 47.0), (24.6, 47.0)])}   # on each net's long straight run, FIL_TAB + 0.5 off its corners
RIDGES_K = [LineString([(-2.0, 38.7), (35.0, 38.7)]), LineString([(-5.0, 49.4), (23.0, 49.4)])]                        # the silkscreen: ground traces ...
RINGS_K = [((-2.0, 38.7), 1.2, 0.6), ((35.0, 38.7), 1.2, 0.6), ((-5.0, 49.4), 0.8, 0.5), ((23.0, 49.4), 0.8, 0.5)]      # ... their ring pads (centre, outer r, width)
DOTS_K = [(47.5, 44.2), (50.5, 44.2)]                                                                                  # ... two via dots
PINS_K = [(4.0, 37.4), (20.0, 37.4), (30.0, 37.4), (-2.0, 50.7), (12.0, 50.7), (22.0, 50.7), (-12.0, 44.2)]   # the lid's press pins: off every
#                                                                                                                       groove, the two end ones into the lead rooms (163: 36 -> 30: the cyan E lead way at 36)
LEADS_K = {"cyan": (-0.5, 36.0), "magenta": (-7.5, 40.0)}   # THE LEAD WAYS (163, R): the x where each net's two leads leave south for the key bay -- in the tab room (cyan both, magenta W),
#                                 magenta E 11 east of its tab room (the way must cross the cyan groove where it has NO filament: the cyan trench runs -1.5..37.5 on y 43)
# THE ENCODER RUNS (build 153, L, plan frame; crowns153.png): from the nets' west ends (m0 / c0: the K ends mapped to the plan) a short west leg,
# then 45 deg SW down the shoulder corridor -- magenta on y = x + 4.6, cyan on y = x - 2.9 (5.3 apart, 2.7+ off the crown's edges, 12+ off the
# knob well) -- a 30 mm filament on each diagonal, its ends the net's new vias (drilled to the cavity)
D45, E45 = FIL_L / math.sqrt(2.0), FIL_TAB / math.sqrt(2.0)
SE1, SE2 = ENC_SE / math.sqrt(2.0), ENC_SE * math.sqrt(2.0)   # THE RUNS ENC_SE further from the shoulder (build 156): every diagonal point + SE1 in x, - SE1 in y; the NE corners + SE2 in x
SW1 = ENC_SW / math.sqrt(2.0)                                  # ... and the filaments + their vias ENC_SW down the diagonals (- SW1 in x and y): off the cell bore's end wall
ENC_FIL = {"magenta_enc": lambda m0: [(40.0 + SE1 - SW1 - E45, m0[1] - 4.5 - SE1 - SW1 - E45), (40.0 + SE1 - SW1 - E45 - D45, m0[1] - 4.5 - SE1 - SW1 - E45 - D45)],   # the filaments FIL_TAB down the diagonals
           "cyan_enc": lambda c0: [(39.3 + SE1 - SW1 - E45, c0[1] - 4.4 - SE1 - SW1 - E45), (39.3 + SE1 - SW1 - E45 - D45, c0[1] - 4.4 - SE1 - SW1 - E45 - D45)]}       # from 153's spots: their NE tab rooms end
#                                                                                                                                    THERE (the deep vias of 153, open); 4.5 further
#                                                                                                                                    NE the magenta via met the cell bore's end wall
ENC_RUN = {"magenta": lambda m0: [m0, (44.5 + SE2, m0[1]), (40.0 + SE1 - SW1 - D45 - 2.0 * E45, m0[1] - 4.5 - SE1 - SW1 - D45 - 2.0 * E45)],     # the runs end at the SW tab rooms' ends:
           "cyan": lambda c0: [c0, (43.7 + SE2, c0[1]), (39.3 + SE1 - SW1 - D45 - 2.0 * E45, c0[1] - 4.4 - SE1 - SW1 - D45 - 2.0 * E45)]}         # the pad + the deep via there (154)
STRAP_L, STRAP_W = ((50.0 + SE1, 50.85 - SE1), (4.5 + SE1, 5.35 - SE1)), 20.0   # THE STRAP (156: ENC_SE with the runs) (build 154, L, plan frame; slim154.png: Willy 13:07 'you dont need to go all the way to the edge anymore, the new
#                                                          cover should go straight down to encoder'): the cover = the lid band + this 20 wide strap along the two runs' axis
#                                                          (y = x + 0.85, between the diagonals) from the band down to the knob's bezel ring; the hull's flat zone either side
RIDGES_PLAN = {"L": [[(41.0 + SE1, 41.85 - SE1), (21.0 + SE1, 21.85 - SE1)]],                                                                   # between the diagonals (154: the strap only)
               "R": [[(20.5, 10.5), (40.0, 10.5), (42.0, 8.5), (60.0, 8.5)], [(6.0, 24.5), (6.0, 50.0)], [(3.0, 3.0), (3.0, 60.0)]]}   # the bezel plate's south and west borders
# THE ARMS (build 153, R, plan frame): the 3 LEDs' clear pegs run north out of the tray onto the bezel plate -- the middle LED straight, the others
# jogged 3.0 either way (1.2 between the arms' slots), the two outer ones with a 45 deg jog to their pads; 11 / 23 / 40 mm long
ARMS_R = lambda leds: [[leds[0], (11.8, 6.5)], [leds[1], (14.8, -6.7), (14.8, 8.0), (17.3, 10.5)], [leds[2], (8.8, -12.2), (8.8, 2.0), (6.0, 4.8), (6.0, 22.0)]]
MID = (Align.CENTER, Align.CENTER, Align.CENTER)
BIG = (Align.CENTER, Align.CENTER, Align.MIN)


def chain_ratio(poly, r):
    """The chain-fillet rule on a polygon's edges for a round r: min over edges of L / (r (tan a/2 + tan b/2)), a and b the turns at its ends (memory
    occ-chain-fillet-condition); the worst edge too."""
    cs = list(poly.exterior.coords)[:-1]
    n = len(cs)
    def turn(i):
        a, b, c = cs[i - 1], cs[i], cs[(i + 1) % n]
        v1, v2 = (b[0] - a[0], b[1] - a[1]), (c[0] - b[0], c[1] - b[1])
        return abs(math.atan2(v1[0] * v2[1] - v1[1] * v2[0], v1[0] * v2[0] + v1[1] * v2[1]))
    worst = (99.0, None)
    for i in range(n):
        L, need = math.dist(cs[i], cs[(i + 1) % n]), r * (math.tan(turn(i) / 2) + math.tan(turn((i + 1) % n) / 2))
        if need > 1e-9 and L / need < worst[0]:
            worst = (L / need, (round(cs[i][0], 1), round(cs[i][1], 1), round(L, 2), round(math.degrees(turn(i))), round(math.degrees(turn((i + 1) % n)))))
    return worst


def clean_outline(poly, r=CROWN_R, margin=1.07, tag="the crown"):
    """A crown plan the top round takes: needles out (a vertex turning 150+ deg), then the lightest simplify whose every edge clears the chain rule by
    `margin`; raises with the worst edge if none does (no fallback round)."""
    cs = list(poly.exterior.coords)[:-1]
    n = len(cs)
    keep = []
    for i in range(n):
        a, b, c = cs[i - 1], cs[i], cs[(i + 1) % n]
        v1, v2 = (b[0] - a[0], b[1] - a[1]), (c[0] - b[0], c[1] - b[1])
        if abs(math.degrees(math.atan2(v1[0] * v2[1] - v1[1] * v2[0], v1[0] * v2[0] + v1[1] * v2[1]))) < 150.0:
            keep.append(b)
    poly = Polygon(keep).buffer(0)
    poly = max(getattr(poly, "geoms", [poly]), key=lambda g: g.area)
    for tol in (0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4):
        g = poly.simplify(tol)
        ratio, worst = chain_ratio(g, r)
        if ratio >= margin:
            print(f"   {tag}: {len(g.exterior.coords) - 1} edges (simplify {tol}), the chain rule's worst edge {ratio:.2f}x at {worst}", flush=True)
            return g
    raise AssertionError(f"{tag}: no simplification up to 0.4 clears the chain rule for r {r}: worst {ratio:.2f}x at {worst}")


def lid_plan():
    """The 152 lid's box in K -- the plateau part of the crown."""
    return sbox(*LIGHT_LID)


def lid_plan_r():
    """... with its plan corners r LID_CORNER."""
    return lid_plan().buffer(-LID_CORNER, join_style=2).buffer(LID_CORNER, join_style=1, quad_segs=8)


def nets(side, K):
    """The side's nets in the plan frame: the K nets through K, L's run on with the encoder runs (the west end first)."""
    out = {n: K(l) for n, l in NETS_K.items()}
    if side == "L":
        for n in out:
            k0 = out[n].coords[0]
            out[n] = LineString(list(ENC_RUN[n](k0))[::-1] + list(out[n].coords[1:]))
    return out


def filaments(side, K):
    """The side's filament runs (plan frame): one per net on the plateau, L's two encoder runs too. A filament's net is its name's first word."""
    out = {n: K(l) for n, l in FILAMENTS_K.items()}
    if side == "L":
        ks = {n: K(l).coords[0] for n, l in NETS_K.items()}
        for n, f in ENC_FIL.items():
            out[n] = LineString(f(ks[n.split("_")[0]]))
    return out


def fil_ext(seg, d=FIL_TAB):
    """A filament's segment run d past both ends: the tab room (the trench, the guide's channel)."""
    (xa, ya), (xb, yb) = seg.coords[0], seg.coords[-1]
    L = math.dist((xa, ya), (xb, yb)); ux, uy = (xb - xa) / L, (yb - ya) / L
    return LineString([(xa - d * ux, ya - d * uy), (xb + d * ux, yb + d * uy)])


def vias(side, K):
    """[(x, y, deep)] in the plan frame: the 152 nets' ends (into the bore's lead rooms, VIA_DEPTH; L only since 163 -- R's leads take the lead ways) and L's
    encoder filament ends (deep: to the cavity)."""
    out = [] if side == "R" else [(q.x, q.y, False) for l in NETS_K.values() for q in (K(Point(l.coords[0])), K(Point(l.coords[-1])))]
    if side == "L":
        out += [(x, y, True) for n, f in filaments(side, K).items() if n.endswith("_enc") for x, y in (fil_ext(f).coords[0], fil_ext(f).coords[-1])]   # at the tab rooms' ends
    return out


def leads(side, K):
    """THE LEAD WAYS (163, R; plan frame): per net two strips from 0.5 north of the net's line straight south (K) to LEAD_Y at LEADS_K's x, and the 45 deg
    hole's start + its plan direction (unit): [(net, strip, (x, y), (dx, dy))]. L: none (its plateau leads still drop into the lead rooms)."""
    if side != "R":
        return []
    out = []
    for n, xs in LEADS_K.items():
        l = NETS_K[n]
        for x in xs:
            q = l.intersection(LineString([(x, LEAD_Y), (x, 60.0)]))
            q = min(getattr(q, "geoms", [q]), key=lambda g: g.y)
            assert q.geom_type == "Point" and q.y > LEAD_Y + 3.0, f"the {n} lead way at x {x} meets no net north of y {LEAD_Y} ({q})"
            a, b = K(Point(x, LEAD_Y)), K(Point(x, LEAD_Y - 1.0))
            out.append((n, K(LineString([(x, q.y + 0.5), (x, LEAD_Y)])), (a.x, a.y), (b.x - a.x, b.y - a.y)))
    return out


def visible(line):
    """The lit part of a net: the line VIA_HIDE short of its EAST via (its last vertex on both halves) -- the slot and the guide's stem stop there with the
    end pad, the groove, the foot and the lead channel run on under the crown to the via (build 156; Willy 18:21 "the traces reach a little too far")."""
    from shapely.ops import substring
    return substring(line, 0.0, line.length - VIA_HIDE)


def slot_plan(line):
    """A net's slot in the crown = the guide's stem: the VISIBLE trace TRACE_W wide + a PAD_D pad at each end."""
    line = visible(line)
    return unary_union([line.buffer(TRACE_W / 2, cap_style=2, join_style=2)] + [Point(q).buffer(PAD_D / 2) for q in (line.coords[0], line.coords[-1])])


def foot_plan(line):
    """A net's groove in the plateau = the guide's foot (+ GROOVE_CLR): GROOVE_W wide, the pads' feet PAD_D + 1."""
    return unary_union([line.buffer(GROOVE_W / 2, cap_style=2, join_style=2)] + [Point(q).buffer(PAD_D / 2 + 0.5) for q in (line.coords[0], line.coords[-1])])


def ridge_plan(side, K):
    """The silkscreen (plan frame): the 152 ridges, rings and dots through K + the side's own lines."""
    parts = [K(r).buffer(RIDGE_W / 2, cap_style=2) for r in RIDGES_K]
    parts += [K(Point(c)).buffer(rr).difference(K(Point(c)).buffer(rr - w)) for c, rr, w in RINGS_K]
    parts += [K(Point(q)).buffer(RIDGE_W) for q in DOTS_K]
    parts += [LineString(r).buffer(RIDGE_W / 2, cap_style=2) for r in RIDGES_PLAN[side]]
    return unary_union(parts)


def pick_pins(crown, avoid, have, n, step=1.0, tight=()):
    """THE PEGS (Willy 11:13 'pegs on cover and holes in case'): LID_PIN pins on a step grid inside the crown's footprint -- 2.2 inside its edge, 1.0 + r
    off everything in `avoid` (slots, grooves, wells, the bezel body), 0.6 + r off the `tight` ones (R's LED tray and trackpad pocket: 0.5 of wall between
    the pin hole and theirs), 12 apart from each other and from the pins already `have`; the first n."""
    x0, y0, x1, y1 = crown.bounds
    inner = crown.buffer(-2.2)
    out = list(have)
    picked = []
    y = y0 + step
    while y < y1 and len(picked) < n:
        x = x0 + step
        while x < x1 and len(picked) < n:
            q = Point(x, y)
            if inner.contains(q) and all(a.distance(q) >= 1.0 + LID_PIN[0] / 2 for a in avoid) and all(a.distance(q) >= 0.6 + LID_PIN[0] / 2 for a in tight) and all(math.dist((x, y), o) >= 12.0 for o in out):
                out.append((x, y)); picked.append((x, y))
            x += step
        y += step
    return picked


def crown_pins(side, K, crown, avoid, n=8, tight=()):
    """The lid's 8 pins (K, as 152) + up to n more picked over the rest of the crown (R: 3 on the LED wing, the rest of it being the bezel's body, the
    trackpad's border and the lid -- the bezel's legs on the HAT's pegs and its crush ribs hold that part)."""
    have = [(q.x, q.y) for q in (K(Point(p)) for p in PINS_K)]
    return have + pick_pins(crown, avoid, have, n, tight=tight)


def cutters(side, K, nets_, vias_, pins_, fils_, leads_=()):
    """What the hull loses under the crown, in the plateau frame (z 0 = RIM_Z): the grooves (GROOVE_D), the via holes (HOLE_D: VIA_DEPTH from the groove's
    floor into the bore's lead room, or DEEP to VIA_DEEP for the encoder runs: into the cavity), the pin holes (LID_PIN + 0.2, LID_PIN deep + 0.1), and
    R's lead ways (163): a LEAD_W channel GROOVE_D deep along each strip, a HOLE_D hole 45 deg down along the strip's direction from 0.3 over the floor."""
    cuts = [prism(foot_plan(l), -GROOVE_D, 1.0) for l in nets_.values()]
    cuts += [prism(fil_ext(f).buffer(FIL_D / 2 + 0.2, cap_style=2), -GROOVE_D - FIL_SINK, -GROOVE_D + 0.5) for f in fils_.values()]   # THE FILAMENT TRENCHES (FIL_SINK under the groove's floor), FIL_TAB past each end
    for x, y, deep in vias_:
        z0 = VIA_DEEP - RIM_Z if deep else -GROOVE_D - VIA_DEPTH
        cuts.append(Pos(x, y, z0) * Cylinder(HOLE_D / 2, -GROOVE_D - z0 + 0.5, align=BIG))
    cuts += [Pos(x, y, -LID_PIN[1] - 0.1) * Cylinder(LID_PIN[0] / 2 + 0.1, LID_PIN[1] + 1.1, align=BIG) for x, y in pins_]
    for n, strip, (sx, sy), (dx, dy) in leads_:
        cuts.append(prism(strip.buffer(LEAD_W / 2, cap_style=2), -GROOVE_D, 1.0))
        cuts.append(lead_hole(sx, sy, dx, dy, -GROOVE_D + 0.3, HOLE_D / 2, LEAD_VIA))
    return cuts


def lead_hole(sx, sy, dx, dy, z0, r, length):
    """A cylinder r from (sx, sy, z0) running `length` 45 deg down along the plan direction (dx, dy) (unit): R's lead way holes and their probes."""
    return Pos(sx, sy, z0) * Rot(0, 0, math.degrees(math.atan2(dx, -dy))) * Rot(-45, 0, 0) * Cylinder(r, length, align=(Align.CENTER, Align.CENTER, Align.MAX))


def guide_part(line, fils, t=CROWN_T, notches=()):
    """A NET's clear GUIDE (plateau frame): the stem (the slot's outline) from the plateau top to the crown's top, the foot (the groove less GROOVE_CLR)
    from GROOVE_CLR over the groove's floor to the plateau top; a ROUND channel (FIL_D / 2 + 0.2 about the filament's axis, open through the foot's
    underside, 0.41 of stem wall a side at z 0) along each of the net's filaments + their tab rooms, and the lead channel (LEAD_CH square along the
    whole net) cut up from the foot's underside. Print it upside down (the stem on the bed)."""
    zf = -GROOVE_D + GROOVE_CLR
    g = prism(slot_plan(line), -0.5, t) + prism(foot_plan(line).buffer(-GROOVE_CLR, join_style=1), zf, 0.0)   # the stem 0.5 into the foot: fused; nothing over z 0 outside the slot
    for f in fils:
        g -= fil_bore(fil_ext(f), FIL_D / 2 + 0.2)                                                           # the round channel about the filament (+ the tab room past each end)
    g -= prism(line.buffer(LEAD_CH / 2, cap_style=2, join_style=2), zf - 0.01, zf + LEAD_CH)
    for s in notches:                                                                                # (163) the lead ways' notches through the foot, in the lead channel's layer
        if s.buffer(LEAD_W / 2).intersects(foot_plan(line)):
            g -= prism(s.buffer(LEAD_W / 2, cap_style=2), zf - 0.01, zf + LEAD_CH)
    assert g.is_valid and len(g.solids()) == 1, f"a guide is {'invalid' if not g.is_valid else str(len(g.solids())) + ' solids'}"
    return g


def fil_bore(seg, r):
    """A cylinder r about a filament segment's axis (plateau frame): the filament itself (r FIL_D / 2) or its channel in the guide (+ 0.2)."""
    (xa, ya), (xb, yb) = seg.coords[0], seg.coords[-1]
    return Pos((xa + xb) / 2, (ya + yb) / 2, -GROOVE_D - FIL_SINK + FIL_D / 2) * Rot(0, 0, math.degrees(math.atan2(yb - ya, xb - xa))) * Rot(0, 90, 0) * Cylinder(r, math.dist((xa, ya), (xb, yb)), align=MID)


def filament_part(seg):
    """THE COB FILAMENT (GUESS: Willy's 30 x 2 spec, no catalog STEP), on its trench's floor FIL_SINK under the groove's floor, inside the guide's channel (plateau frame)."""
    (xa, ya), (xb, yb) = seg.coords[0], seg.coords[-1]
    return fil_bore(seg, FIL_D / 2)


def round_top(plate, r=CROWN_R):
    """The crown's top edge round: OCC's fillet on the top face's outer wire (every edge, else the ones over 3 long -- the brim's way since build 114)."""
    face = max(plate.faces(), key=lambda f: f.area if f.normal_at().Z > 0.99 else 0.0)
    for es in (list(face.outer_wire().edges()), [e for e in face.outer_wire().edges() if e.length > 3.0]):
        try:
            return fillet(es, radius=r), len(es)
        except ValueError:
            continue
    raise AssertionError(f"OCC refuses the r {r} round on the crown's top edge")


def crown_part(foot, holes, slots, ridges, pins_, t=CROWN_T, before_round=None):
    """THE CROWN (plateau frame, z 0 = the rim = its underside): the footprint prism t thick, the holes through it (the knob well; R's display window,
    keyholes and trackpad window), the slots (+ LIGHT_CLR: the guides' stems and R's arms, snug), `before_round` (R's bevels and its own raised traces),
    the top edge round, the ridges on top, the pins underneath. Print it top-down."""
    plate = prism(foot, 0.0, t)
    for h in holes:
        plate -= prism(h, -1.0, t + 1.0)
    for s in slots:
        plate -= prism(s.buffer(LIGHT_CLR, join_style=1), -1.0, t + 1.0)
    if before_round is not None:
        plate = before_round(plate)
    plate, ne = round_top(plate)
    if not ridges.is_empty:                                                                     # (157: RIDGES off -- nothing proud of the top)
        plate += prism(ridges, t - 0.01, t + RIDGE_H)
    for x, y in pins_:
        plate += Pos(x, y, -LID_PIN[1]) * Cylinder(LID_PIN[0] / 2, LID_PIN[1] + 0.01, align=BIG)
    assert plate.is_valid and len(plate.solids()) == 1, f"the crown is {'invalid' if not plate.is_valid else str(len(plate.solids())) + ' solids'}"
    return plate, ne


def arms(leds):
    return [LineString(a) for a in ARMS_R(list(leds))]


def arm_slots(leds):
    """The arms' plans: ARM_W wide, a TCAP_PAD pad at the far end."""
    return [a.buffer(ARM_W / 2, cap_style=2, join_style=2).union(Point(a.coords[-1]).buffer(TCAP_PAD / 2)) for a in arms(leds)]


def diffuser_part(tray, leds, t=CROWN_T):
    """THE DIFFUSER (R), in the tray's frame (z 0 = the tray's floor): a clear plate TP_IND thick, flush with the seat floor, its top edge bevelled 45 deg
    into the tray's flared mouth, a pocket over each dome's tip in its underside, and THE PEGS = THE ARMS (Willy 10:08 'standoffs like pegs that line
    up with and fit snuggly into the bezeled cover'; 11:13 LGTM on the arms): each LED's arm rises from the plate t through the crown's slot, flush with
    its top, and runs on over the seat floor and the bezel plate -- the lit trace IS the peg. Print it top up."""
    clr = LED_BEZEL[0]
    inner = tray.buffer(-clr, join_style=2)
    face = lambda z: make_face(Polyline(*[(x, y, z) for x, y in inner.exterior.coords[:-1]], close=True))
    part = extrude(face(0.0), amount=TP_IND - TRAY_FLARE) + extrude(face(TP_IND - TRAY_FLARE), amount=TRAY_FLARE, taper=-45)
    for sl in arm_slots(leds):                                                                 # rooted in the plate inside the tray, then the whole arm from the seat floor up
        part += prism(sl.intersection(inner.buffer(-0.05, join_style=2)), TP_IND - TRAY_FLARE - 0.01, TP_IND + 0.01) + prism(sl, TP_IND, TP_IND + t)
    for x, y in leds:
        part -= Pos(x, y, -0.01) * Cylinder(DOME_POCKET[0] / 2, DOME_POCKET[1] + 0.01, align=BIG)
    assert part.is_valid and len(part.solids()) == 1, f"the diffuser is {'invalid' if not part.is_valid else str(len(part.solids())) + ' solids'}"
    return part


def tp_window():
    """The trackpad's window through the crown: the pad + TP_CLR (the pocket's plan, as today)."""
    return tp_plan(TP_CLR)


def collar_plan():
    """THE COLLAR under the crown round the trackpad window: TP_COLLAR thick on the pad's two side edges and its front (round front corners), not on the
    display side (1.8 of hull between the pockets there: the pad locates on the hull's wall)."""
    w = tp_window()
    return w.buffer(TP_COLLAR, join_style=1).difference(w).intersection(sbox(-1000.0, -1000.0, 1000.0, w.bounds[3] - 0.5))


def collar_room():
    """What the hull's pocket widens by for the collar (+ 0.15 a side), the display side wall kept."""
    w = tp_window()
    return w.buffer(TP_COLLAR + 0.15, join_style=1).intersection(sbox(-1000.0, -1000.0, 1000.0, w.bounds[3]))


def ledge_plan():
    """THE LEDGE the pad sits on: TP_LEDGE in from the window under the pad's two SIDE edges (its connection edges, north and south, stay open)."""
    w = tp_window()
    x0, _, x1, _ = w.bounds
    return w.difference(w.buffer(-TP_LEDGE, join_style=2)).intersection(unary_union([sbox(-1000.0, -1000.0, x0 + TP_LEDGE + 0.01, 1000.0), sbox(x1 - TP_LEDGE - 0.01, -1000.0, 1000.0, 1000.0)]))


def overlay_part():
    """THE OVERLAY (Willy: 'thin piece of different colored pla to indicate the trackpad area', 'sunken in, same fit as the bezel lid'): a plate OVERLAY_T
    thick, the window less LIGHT_CLR (snug), lying on the pad OVERLAY_SINK under the crown's top (z 0 = the pad's top)."""
    return prism(tp_plan(TP_CLR - LIGHT_CLR), 0.0, OVERLAY_T)
