"""Validation on the assembled model: static clashes, seating, the stand's motion tests. Prints a report; the tests/ suite asserts on the same numbers."""
import math

from build123d import Align, Axis, Box, Compound, Plane, Pos, Rot, mirror

from velvet.params import *  # noqa: F401,F403
from velvet.geo import _clip, _hit, _where, prism, rest_plane, rest_z, tessellate
from velvet.stand import tent_angles, tent_pin_len
from velvet.parts import knob
from shapely.geometry import Point
from shapely.ops import nearest_points, unary_union


def round_profile(d, s=0.0, r=BORDER_R):
    """The border's top d in from the case's face, over the rest plane there: the r round (a rolling ball) between the vertical wall and the sloped
    top, whose slope along the inward normal is s (tan, + rising inward: tan REST_SLOPE on the front wall, 0 on the outer wall). The section square
    to the edge is a circle tangent to both: centre r from the wall, r under the top; on the front wall (95 deg dihedral) it sits 0.11 higher at
    0.5 in than a 90 deg circle (test_border_and_keywell's first expectation, build 107)."""
    if d >= r:
        return BORDER_UP
    return BORDER_UP + r * s - r * math.sqrt(1.0 + s * s) - d * s + math.sqrt(max(r * r - (r - d) ** 2, 0.0))


def wall_drop(p, q, w=0.0, r=EDGE_R_TOP):
    """THE KEY WALL'S ROUND (build 147): how far under the fill's flat top the hull's top is at q -- on the r round along the open pit's edge where
    the run is (plan.wall_zone), d = q's distance to the edge; a w-wide column reads its highest point, w / 2 farther from the edge."""
    if p["wall_zone"] is None or not p["wall_zone"].contains(q):
        return 0.0
    d = q.distance(p["open_pit"].boundary) + w / 2.0
    return r - math.sqrt(r * r - (r - d) ** 2) if d < r else 0.0


def inward(plan, q, d=1.0):
    """The inward unit normal of the outline at q (on it): toward the nearest point of the outline d in."""
    qi = nearest_points(plan.buffer(-d, join_style=1).boundary, q)[0]
    n = math.hypot(qi.x - q.x, qi.y - q.y)
    return ((qi.x - q.x) / n, (qi.y - q.y) / n)


def column_tops(part, W, k, pts, z0=-200.0, z1=200.0, w=0.4):
    """For each (x, y) in the half's u frame: the part's solid segments (z lo, z hi) along a w x w column, highest first."""
    pu = W.inverse() * part
    if k == "L":
        pu = mirror(pu, about=Plane.YZ)
    out = []
    for x, y in pts:
        col = pu & (Pos(x, y, (z0 + z1) / 2) * Box(w, w, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.CENTER)))
        segs = sorted(((s.bounding_box().min.Z, s.bounding_box().max.Z) for s in col.solids()), key=lambda s: -s[1]) if col is not None else []
        if not segs:                                                   # EMPTY: OCC's common fails silently on the base plates' fillet faces over the shoulder
            key = (id(part), k)                                        # ogee (12-24 of 63 stations, deterministic, 0.4 columns too; is_inside misreads 0.25-0.5
            if key not in _MESH:                                       # bumps there while the MESH is flat to 0.000: scratchpad base_probe/base_mesh135, build
                _MESH[key] = mesh_of(pu)                               # 135). The mesh ray answers: what the slicer prints.
            segs = ray_segs(_MESH[key], x, y, z0, z1)
        out.append(segs)
    return out


_MESH = {}


def mesh_of(part, tol=0.02):
    """The part's mesh as numpy triangle corners (A, B, C) plus each triangle's plan-projection determinant (NaN for the vertical ones), for ray_segs."""
    import numpy as np
    verts, tris = tessellate(part, tol, 0.1)                                              # (geo.tessellate: every face, build 148)
    V, T = np.array([(v.X, v.Y, v.Z) for v in verts]), np.array(tris)
    A, B, C = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    det = (B[:, 1] - C[:, 1]) * (A[:, 0] - C[:, 0]) + (C[:, 0] - B[:, 0]) * (A[:, 1] - C[:, 1])
    return A, B, C, np.where(np.abs(det) > 1e-12, det, np.nan)


def ray_segs(mesh, x, y, z0=-200.0, z1=200.0):
    """A vertical ray at (x, y) through the mesh: the z's where it pierces triangles, paired into (lo, hi) solid segments within z0..z1, highest
    first (an odd count -- a grazing hit -- collapses to one segment). BOP-free: the honest column where OCC's common comes back empty."""
    import numpy as np
    A, B, C, det = mesh
    with np.errstate(invalid="ignore"):
        l1 = ((B[:, 1] - C[:, 1]) * (x - C[:, 0]) + (C[:, 0] - B[:, 0]) * (y - C[:, 1])) / det
        l2 = ((C[:, 1] - A[:, 1]) * (x - C[:, 0]) + (A[:, 0] - C[:, 0]) * (y - C[:, 1])) / det
        l3 = 1.0 - l1 - l2
        hit = (l1 >= -1e-6) & (l2 >= -1e-6) & (l3 >= -1e-6)
    za = np.sort(l1[hit] * A[hit, 2] + l2[hit] * B[hit, 2] + l3[hit] * C[hit, 2])
    zs = [float(z) for i, z in enumerate(za) if z0 <= z <= z1 and (i == 0 or z - za[i - 1] > 1e-4)]   # a ray through a shared edge hits both triangles
    segs = [(zs[i], zs[i + 1]) for i in range(0, len(zs) - 1, 2)] if len(zs) % 2 == 0 else ([(zs[0], zs[-1])] if zs else [])
    return sorted(segs, key=lambda s: -s[1])


def top_map(part, x0, x1, y0, y1, d=0.5):
    """The part's top surface as a height grid (rows y, cols x, NaN where nothing): its mesh rasterized, max z per cell. The honest flatness probe:
    a 0.4 column reads a round's shoulder, a boolean sees no 0.1 ripple; this sees what the viewer draws (L's groove north of the knob, build 124)."""
    import numpy as np
    verts, tris = tessellate(part, 0.02, 0.1)                                             # (geo.tessellate: every face, build 148)
    V, T = np.array([(v.X, v.Y, v.Z) for v in verts]), np.array(tris)
    nx, ny = int((x1 - x0) / d) + 1, int((y1 - y0) / d) + 1
    H = np.full((ny, nx), np.nan)
    for a, b, c in T:
        Pt = V[[a, b, c]]
        if Pt[:, 0].max() < x0 or Pt[:, 0].min() > x1 or Pt[:, 1].max() < y0 or Pt[:, 1].min() > y1:
            continue
        i0, i1 = max(0, int((Pt[:, 0].min() - x0) / d)), min(nx - 1, int((Pt[:, 0].max() - x0) / d) + 1)
        j0, j1 = max(0, int((Pt[:, 1].min() - y0) / d)), min(ny - 1, int((Pt[:, 1].max() - y0) / d) + 1)
        gx, gy = np.meshgrid(x0 + np.arange(i0, i1 + 1) * d, y0 + np.arange(j0, j1 + 1) * d)
        (ax, ay, az), (bx, by, bz), (cx, cy, cz) = Pt
        det = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(det) < 1e-9:
            continue
        l1 = ((by - cy) * (gx - cx) + (cx - bx) * (gy - cy)) / det
        l2 = ((cy - ay) * (gx - cx) + (ax - cx) * (gy - cy)) / det
        l3 = 1.0 - l1 - l2
        z = l1 * az + l2 * bz + l3 * cz
        sub = H[j0:j1 + 1, i0:i1 + 1]
        upd = (l1 >= -1e-6) & (l2 >= -1e-6) & (l3 >= -1e-6) & (np.isnan(sub) | (z > sub))
        sub[upd] = z[upd]
    return H



def back_profile(plan, hull, W, k, pts=False):
    """The back run's top edge in section at the run's middle: the hull's top 1, 3 and 7 in from the outline (BACK_R: 7.22, 9.10, RIM_Z);
    pts=True hands back the probe points too (the lights' grooves, build 152)."""
    run = plan["back_run"]
    q, a, b = (run.interpolate(f, normalized=True) for f in (0.5, 0.49, 0.51))
    tx, ty = b.x - a.x, b.y - a.y
    L = math.hypot(tx, ty)
    nx, ny = -ty / L, tx / L
    if plan["plan"].contains(Point(q.x + nx, q.y + ny)):                                         # outward
        nx, ny = -nx, -ny
    x, y = q.x, q.y
    qs = [(x - d * nx, y - d * ny) for d in (1.0, 3.0, 7.0)]
    tops = column_tops(hull, W, k, qs, w=0.1)                                                    # a 0.4 column's inner edge read an r 6 round as 7.5
    tops = [(t[0][1] if t else None) for t in tops]
    return (tops, qs) if pts else tops


def back_round_z(d, r=None):
    r = BACK_R if r is None else r
    return RIM_Z if d >= r else RIM_Z - r + math.sqrt(r * r - (r - d) ** 2)


def pit_front_probes(plan, bosses):
    """Three points 0.5 inside the pit where it faces the palm deck (its edges within 0.3 of palm_side; left, middle, right by x), off the
    bosses: the plate's edge, where the deck's face meets it. (Build 85's probe took the pit's lowest vertices and landed on a boss: 12.7 tall;
    0.25 inside, a 0.4 column grazed the deck's face, which sits within 0.2 of the pit's edge -- the fill's outline is simplified 0.2 -- and read
    the deck, 5-13 tall, build 88.)"""
    from shapely.geometry.polygon import orient
    ring = list(orient(plan["pit"], 1.0).exterior.coords)
    pts = []
    for (x0, y0), (x1, y1) in zip(ring, ring[1:]):
        L = math.dist((x0, y0), (x1, y1))
        if L < 0.5:
            continue
        mx, my, nx, ny = (x0 + x1) / 2, (y0 + y1) / 2, -(y1 - y0) / L, (x1 - x0) / L        # inward normal of a CCW ring
        q = Point(mx + 0.5 * nx, my + 0.5 * ny)
        if ny > 0.7 and plan["palm_side"].distance(Point(mx, my)) < 0.3 and plan["inner"].buffer(-2.0).contains(q) \
                and all(q.distance(b) > BOSS_D / 2 + 1.5 for b in bosses):                      # the FRONT edges, 2 inside the cavity's outline (the corner picks read the walls, 11-21 tall, builds 86-87)
            pts.append((q.x, q.y))
    pts.sort()
    return [pts[len(pts) // 4], pts[len(pts) // 2], pts[3 * len(pts) // 4]]


def pit_crossing(plan, hull, W, k, bosses):
    """Across the pit's front edge at the middle probe: columns 0.5 .. 2.5 inside the pit (0.25 steps to 1), stopping short of a switch's clip-lip
    pocket -- the plate's solid thickness at each (its top segment); the least of them is the junction's weakest point (build 87: 0.0 -- a slot 1 mm wide)."""
    x, y = pit_front_probes(plan, bosses)[1]
    e = plan["pit"].exterior
    q0 = e.interpolate(e.project(Point(x, y)))                                               # the edge point the probe sits 0.5 inside of
    nx, ny = (x - q0.x) / 0.5, (y - q0.y) / 0.5
    pts = [(q0.x + d * nx, q0.y + d * ny) for d in (0.5, 0.75, 1.0, 1.5, 2.0, 2.5)]
    pts = [q for q in pts if plan["wells"].distance(Point(q)) > (POCKET_W - HOLE) / 2 + 0.3]        # not into a switch's clip-lip pocket (1.2 left there by design; L read 1.6 at 1.5 in, build 89)
    return [round(s[0][1] - s[0][0], 2) if s else 0.0 for s in column_tops(hull, W, k, pts)]


def sheets(hull, W, k, p, badge, strip=None):
    """Where the hull must be EMPTY: over the rim in front of y 15 (u frame; the tilted plate rises over the rim only behind y 23), in the pit
    over the tilted plate (3 inside the tilted outline, less the keywell: the deck's face leans over the pit's edge), and the flat zone / palm side seam through the
    plate (z -2.5..-1, within 0.3 of the seam: the two prisms of builds 59-88 left a hairline crack there). Returns dict(above=[(mm3, bbox)],
    pit=[(mm3, bbox)], crack=mm3). Build 88 had 14 / 2 sheets of 0.001-0.25 mm3 up to 5.5 over the rim (the fins round the knob) and a 0.08 crack."""
    hu = W.inverse() * hull
    if k == "L":
        hu = mirror(hu, about=Plane.YZ)
    T = Pos(0, 0, -TILT_DROP) * Pos(0, p["yp"], RIM_Z) * Rot(KEY_TILT, 0, 0) * Pos(0, -p["yp"], -RIM_Z)
    def pieces(q):
        return [] if q is None else [(round(s.volume, 3), tuple(round(v, 1) for v in (s.bounding_box().min.X, s.bounding_box().min.Y, s.bounding_box().max.X, s.bounding_box().max.Y, s.bounding_box().max.Z))) for s in q.solids()]
    over = hu & Pos(0, -200.0, RIM_Z + 0.05) * Box(400.0, 215.0, 30.0, align=(Align.CENTER, Align.MIN, Align.MIN))
    if p["tp_zone"] is not None and over is not None:                                       # R's console stands CONSOLE_UP over the rim by design (its pitched plane from the hinge line)
        Tt = Pos(0, Y_P0, RIM_Z + CONSOLE_UP) * Rot(p["tp_pitch"], 0, 0) * Pos(0, -Y_P0, -RIM_Z)
        over = over - (prism(p["tp_zone"].buffer(0.5, join_style=2), RIM_Z - 1.0, RIM_Z + CONSOLE_UP + 2.0) & (Tt * Pos(0, 0, RIM_Z + 0.05) * Box(400.0, 400.0, 400.0, align=(Align.CENTER, Align.CENTER, Align.MAX))))
    if over is not None and not p["swell"].is_empty:                                           # THE SWELL stands over the rim at the P / Q corner by design (build 132; off in 139)
        over = over - prism(p["swell"].buffer(0.5, join_style=2), RIM_Z - 1.0, RIM_Z + 12.0)
    if over is not None and not badge.is_empty:                                             # THE BADGE stood BADGE_T over the flat zone / the console by design (builds 136-152; empty since 153: THE CROWN is its own part)
        over = over - prism(badge.buffer(0.5, join_style=2), RIM_Z - 1.0, RIM_Z + CONSOLE_UP + BADGE_T + 2.0)
    if over is not None and strip is not None:                                              # THE STRIP beside the brim's east flange stands CONSOLE_UP over the rim by design (build 143)
        over = over - prism(strip.buffer(0.6, join_style=2), RIM_Z - 1.0, RIM_Z + CONSOLE_UP + 2.0)
    above = pieces(over)
    pit = pieces(hu & (T * prism(p["tilted"].buffer(-3.0, join_style=2), 0.3, 25.0) - prism(p["keywell"].buffer(0.3, join_style=2), ZB, RIM_Z + 5.0)))   # (not the keywell: filled by design)
    lip = p["pit"].buffer(0.3, join_style=2).intersection(p["palm_side"].buffer(1.0, join_style=2)).difference(p["flat"].buffer(0.3, join_style=2)).difference(p["ring"].buffer(0.3, join_style=2))
    if p["tp_zone"] is not None:
        lip = lip.difference(p["tp_zone"].buffer(0.8, join_style=2))
    edge = pieces(hu & prism(lip, ZB, RIM_Z + 5.0) & rest_plane(p, BORDER_UP + 0.3))      # THE DECK'S FACE AT THE PIT: nothing over the border's level (builds 103-104:
    #                                                                                        sheets of the plateau's wall standing to 7.9 at five spots, the 'blades')
    seam = p["flat"].boundary.intersection(p["palm_side"].buffer(0.3)).buffer(0.3)                                                           # the band's ends poke into the ring (empty under
    seam = seam.intersection(unary_union([p["flat"], p["palm_side"]]).buffer(0.02)).intersection(p["plan"].buffer(-0.05, join_style=2))   # the tilted plate) and out of the walls: clipped
    crack = Pos(0, 0, -1.75) * prism(seam, -0.75, 0.75) - hu
    return dict(above=above, pit=pit, edge=edge, crack=round(sum(s.volume for s in crack.solids()), 3) if crack is not None else 0.0)


def checks(A, mode="fast"):
    """mode: fast = the static clashes (every 4th switch); full = every switch + the stand's motion tests."""
    Hs = A["halves"]
    print(f"halves: SPLIT + FLOW (closing {FLOW}, rounding {ROUND}), ONE {H:.1f} slab each (hatch {PLATE_T:.0f} + cavity {-FLOOR - TOP:.1f} + deck {TOP:.1f} + rim {RIM_Z:.2f}), caps {SUNK} under the rim; "
          f"inner column from the inner edge R {X_IN['R']} / L {X_IN['L']}; placement = sheet B 'tuck': the cell along the top row behind the inner columns, the Pico along y at the outer-rear, "
          f"R: display inner-rear + V713 under the HAT, TPS43 trackpad inner-front; L: knob housing inner-front + V713 behind it")
    for k in "RL":
        h = Hs[k]
        hull = h["hull"]
        bb = hull.bounding_box()
        ks = h["keys"] if mode == "full" else h["keys"][::4]                    # fast: every 4th switch (17 s a half for all)
        sw = [_hit(q, hull) for _, q, _ in ks]
        caps = [c for _, _, c in h["keys"]]
        cap_total = _hit(caps[0].fuse(*caps[1:]), hull)                           # one fused check (3 s); per key only when it fails
        extra = ""
        if k == "R":
            pcb = max(h['pi'].solids(), key=lambda q: q.bounding_box().size.X * q.bounding_box().size.Y).bounding_box()
            stack = Compound(children=[h["hat"], h["pi"], h["header"]])   # (161: no socket)
            hpi = _clip(hull, h["pi"])                                                  # the Pi STEP is 30 s a boolean against the whole hull; 1 s against its neighbourhood
            extra = f"; STACK FROM THE TOP: Pi to hull {h['pi'].distance_to(hpi):.2f} / HAT to hull {h['hat'].distance_to(hull):.2f} (0 = on its columns), stack to hatch {stack.distance_to(h['hatch']):.1f} (free); BEZEL valid {h['bezel'].is_valid} vs hull {_hit(h['bezel'], h['hull']):.2f} (its {BZ_RIBS} crush ribs' press fit = {BZ_RIBS * RIB_W * (RIB_H - BZ_CLR) * GLASS_SINK:.2f}) / HAT {_hit(h['bezel'], h['hat']):.2f} at {_where(h['bezel'], h['hat'])} / Pi {_hit(h['bezel'], h['pi']):.2f}, legs to the HAT {h['bezel'].distance_to(h['hat']):.2f} (0 = standing on it), top {h['bezel'].bounding_box().max.Z - h['W'].position.Z:.2f} vs rim {RIM_Z}; TRACKPAD vs hull {_hit(h['pad'], hull):.2f} / hatch {_hit(h['pad'], h['hatch']):.2f}, its face to the skin {h['pad'].distance_to(hull):.2f} (0 = stuck on), skin {TP_SKIN}, no dish; HAT vs hull {_hit(h['hat'], hull):.2f} (HAT top z {h['hat'].bounding_box().max.Z - h['W'].position.Z:.2f}, rim {RIM_Z}), Pi vs hull {_hit(h['pi'], hpi):.2f} at {_where(h['pi'], hpi)} (Pi PCB x {pcb.min.X - h['W'].position.X:.2f}..{pcb.max.X - h['W'].position.X:.2f}; pocket {U_PK0:.2f}..{U_PK1:.2f}) / hatch {_hit(h['pi'], h['hatch']):.2f}, the header's pins to the HAT {h['header'].distance_to(h['hat']):.2f} (161: soldered into it, no socket); V713 box vs stack {_hit(h['v713'], stack):.2f} (under the HAT beside the Pi); OTG COVER (162) vs hull {_hit(h['picover'], hull):.2f} (four ribs' crush {4 * RIB_W * (WALL - 0.5) * (RIB_H - COVER_CLR):.2f}) / Pi {_hit(h['picover'], h['pi']):.2f}; BUTTON CAPS (162, plungers {BTN_TOP} over the PCB GUESS) vs bezel {max(_hit(c, h['bezel']) for c in h['caps']):.2f} / HAT {max(_hit(c, h['hat']) for c in h['caps']):.2f}, on the plungers {min(c.distance_to(h['hat']) for c in h['caps']):.2f} (0)"
        else:
            extra = f"; EC11 vs hull {_hit(h['ec11'], hull):.2f} / hatch {_hit(h['ec11'], h['hatch']):.2f}; EC11 to the deck {h['ec11'].distance_to(hull):.2f} (0 = clamped), vs the V713 box {_hit(h['ec11'], h['v713']):.2f}; knob vs hull {_hit(h['knob'], hull):.2f}, gap {h['knob'].distance_to(hull):.2f} / vs EC11 {_hit(h['knob'], h['ec11']):.2f} (the bore round the shaft)"
        print(f"{k}: hull valid {hull.is_valid} {len(hull.solids())} solid(s) {[round(q.volume / 1000, 2) for q in hull.solids()]} cm3 {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.1f}; hatch valid {h['hatch'].is_valid} {len(h['hatch'].solids())} {[round(q.volume / 1000, 2) for q in h['hatch'].solids()]} cm3 (vs hull {_hit(h['hatch'], hull):.2f} at {_where(h['hatch'], hull)}); "
              f"bosses {len(h['local']['bosses'])} at {[(round(b.x), round(b.y)) for b in h['local']['bosses']]}")
        capbad = [(kk, round(v, 2)) for kk, _, c in h['keys'] if (v := _hit(c, hull)) > 0.01] if cap_total > 0.01 else []
        swbad = [(kk, round(v, 2), _where(q, hull)) for (kk, q, _), v in zip(ks, sw) if v > 0.9]
        print(f"   caps vs hull by key: {capbad[:8]}{' ...' if len(capbad) > 8 else ''}; switches over the tab's 0.85: {swbad[:4]}")
        if k == "R":
            pc = (h["W"].inverse() * h["pi"]).bounding_box().center()
            print(f"   Pi PCB centre u {pc.X:.2f} y {pc.Y:.2f} vs the pocket's {(U_PK0 + U_PK1) / 2:.2f} / posts about {h['local']['F2H'].position.Y + (7.15 + 30.15) / 2:.2f} (GUESS: posts at the Zero's 58 x 23 about its PCB centre); the SD slot opens through the inner wall")
        print(f"   mm3: switch vs deck {min(sw):.2f}..{max(sw):.2f} (clip tabs in the lip: equal = ok); caps vs hull {cap_total:.2f}; ({len(ks)} switches checked); pico vs hull {_hit(h['pico'], hull):.2f} at {_where(h['pico'], hull)} / hatch {_hit(h['pico'], h['hatch']):.2f}; "
              f"pane vs hull {_hit(h['pane'], hull):.2f} / pico {_hit(h['pane'], h['pico']):.2f}; TPU pad vs hull {_hit(h['tpu'], hull):.2f}, flush {h['tpu'].bounding_box().max.Z - h['W'].position.Z - rest_z(A['plans'][k], h['tpu'].bounding_box().max.Y):.2f} (0 = its top on the rest plane); THE PANE UNDER THE PAD (159): pane vs TPU {_hit(h['pane'], h['tpu']):.2f}, the pad's underside on the pane {h['tpu'].distance_to(h['pane']):.2f} (0 = touching), the window {h['local']['pane_vis'].area:.0f} mm2 of the pane's {h['local']['pane_rec'].area:.0f}; cell vs hull {_hit(h['cell'], hull):.2f} at {_where(h['cell'], hull)} / hatch {_hit(h['cell'], h['hatch']):.2f}, CRADLE {h['local']['cradle_h']:.1f} tall, cell to it {h['cell'].distance_to(h['hatch']):.2f} (clearance 0.5 = seated); V713 box vs hull {_hit(h['v713'], hull):.2f} / hatch {_hit(h['v713'], h['hatch']):.2f}" + extra)
        wb = h["local"]["win"].bounds
        print(f"   PICO face up in the palm deck along K x, plug out the outer wall: PCB top (own z) {h['local']['pcb_top']:.2f}, the board's top {PICO_TOP_CLR} under the bay's ceiling {BAY_DOWN:.1f} below the deck's surface; pane {h['pane'].bounding_box().size.X:.1f} x {h['pane'].bounding_box().size.Y:.1f} x {PANE_T} acrylic, "
              f"{PANE_SINK} under the rim; window {wb[2] - wb[0]:.1f} x {wb[3] - wb[1]:.1f} (bbox); 4 corner pads, M2 x2 from below at the inward end (pilot {PICO_PILOT}, {PICO_SCREW} deep)")
        ux, uy, uz = h["local"]["pico_plug"]                                                  # the wall over the plug window vs the rest's chamfer foot (PAD_EDGE + 0.1 down the face)
        q = nearest_points(A["plans"][k]["plan"].buffer(-0.6, join_style=2).boundary, Point(ux, uy))[0]
        seg = column_tops(hull, h["W"], k, [(q.x, q.y)], w=0.2)[0]
        ps = A["plans"][k]["palm_side"]                                                       # NO LIP: at the rest's outline the hull's top is the recess floor and the
        run = max(getattr(e_ := A["plans"][k]["plan"].exterior.intersection(ps.buffer(-3.0, join_style=2).buffer(3.5, join_style=2)), "geoms", [e_]), key=lambda g: g.length)
        pp = A["plans"][k]
        at = lambda part, q, w=0.2: (c[0][1] - rest_z(pp, q.y)) if (c := column_tops(part, h["W"], k, [(q.x, q.y)], w=w)[0]) else None
        mid = run.interpolate(0.5, normalized=True)
        ins = lambda d: nearest_points(pp["plan"].buffer(-d, join_style=1).boundary, mid)[0]
        rnd = lambda d: round_profile(d, math.tan(math.radians(REST_SLOPE)) * inward(pp["plan"], mid)[1])   # the border's round: the top d in from the face
        print(f"   BORDER {BORDER_W} x {BORDER_UP}, round r {BORDER_R}: 0.5 / 1 in from the case's face the hull's top is {at(hull, ins(0.5), 0.05):.2f} / {at(hull, ins(1.0), 0.05):.2f} over the rest plane (the round: {rnd(0.5):.2f} / {rnd(1.0):.2f}), "
              f"{BORDER_W + 0.6} in {at(hull, ins(BORDER_W + 0.6)):.2f} (the recess floor: -{PAD_DEPTH}), the TPU's top there {at(h['tpu'], ins(BORDER_W + 0.6)):.2f} (flush: {PAD_PROUD})")
        pit, ps_ = pp["pit"], pp["palm_side"]                                                  # THE KEYWELL: along the pit's run in the palm side, 0.5 inside the pit the
        prun = max(getattr(e_ := pit.exterior.intersection(ps_.buffer(0.3, join_style=2)), "geoms", [e_]), key=lambda g: g.length)   # hull's top is the rest plane
        well, bord = [], []                                                                     # (the slot filled), 1 outside it the border
        for f in (0.3, 0.5, 0.7):
            q = prun.interpolate(f, normalized=True)
            well.append(at(hull, nearest_points(pit.buffer(-0.5, join_style=1).exterior, q)[0]))
            bord.append(at(hull, nearest_points(pit.buffer(1.0, join_style=1).exterior, q)[0]))
        print(f"   KEYWELL: 0.5 inside the pit the hull's top is {[None if v is None else round(v, 2) for v in well]} over the rest plane (the slot filled to the border's level: {BORDER_UP}), 1 outside it {[None if v is None else round(v, 2) for v in bord]} (the border: {BORDER_UP})")
        hl = h["local"]                                                                      # THE PICO'S PLUG TUNNEL: a 12 x 8 overmold from the connector's face out
        x0, x1 = hl["pico_face"], hl["pico_bore"][1]                                       # past the case's face, in the hull's own frame (u)
        Lp = x1 + 15.0 - x0
        path = hl["bay_Lf"] * Pos(x0 + Lp / 2, 0, hl["usb_z"]) * Box(Lp, PLUG_W - 1.0, PLUG_H - 1.0, align=MID)
        A_, B_ = hl["pico_doors"]; Wf = hl["pico_Wf"]; Wi = Wf.inverse()                   # THE PLUG DOOR (build 155): two leaves, A outer, B inner; open = A back PICO_TRAVEL, B 9
        oA, oB = Wf * Pos(-PICO_TRAVEL, 0, 0) * (Wi * A_), Wf * Pos(-(PICO_TRAVEL - (PICO_GROOVE[1] - PICO_LUG[1])), 0, 0) * (Wi * B_)
        print(f"   PICO PLUG TUNNEL {PLUG_W} x {PLUG_H} (Willy 21:06): a 12 x 8 x {Lp:.0f} plug path from the connector's face out through the hole vs hull {_hit(path, hl['hull']):.2f} / hatch {_hit(path, hl['hatch']):.2f} / TPU {_hit(path, hl['tpu']):.2f} / pane {_hit(path, hl['pane']):.2f}; "
              f"PLUG DOOR (build 155): hole {PICO_HOLE[0]} x {PICO_HOLE[1]} r {PICO_HOLE[2]}; leaves A {A_.volume:.0f} / B {B_.volume:.0f} mm3 valid {A_.is_valid}/{B_.is_valid}, "
              f"vs hull closed {_hit(A_, hl['hull']):.2f}/{_hit(B_, hl['hull']):.2f} open {_hit(oA, hl['hull']):.2f}/{_hit(oB, hl['hull']):.2f}, A vs B {_hit(A_, B_):.2f}; open leaves vs the plug path {_hit(oA, path):.2f}/{_hit(oB, path):.2f} (all 0)")
        cA, cB = hl["chg_doors"]; Wc = hl["chg_Wf"]; Wci = Wc.inverse(); sp = CHG_SPEC                   # THE CHARGING DOOR (162)
        ocA, ocB = Wc * Pos(-sp["travel"], 0, 0) * (Wci * cA), Wc * Pos(-(sp["travel"] - (sp["groove"][1] - sp["lug"][1])), 0, 0) * (Wci * cB)
        print(f"   CHARGE DOOR (162): hole {sp['hole'][0]} x {sp['hole'][1]} r {sp['hole'][2]} centred {V713_USB_Z} over the charger's underside; leaves A {cA.volume:.0f} / B {cB.volume:.0f} mm3 valid {cA.is_valid}/{cB.is_valid}, "
              f"vs hull closed {_hit(cA, hl['hull']):.2f}/{_hit(cB, hl['hull']):.2f} open {_hit(ocA, hl['hull']):.2f}/{_hit(ocB, hl['hull']):.2f}, A vs B {_hit(cA, cB):.2f}, vs hatch {_hit(cA, hl['hatch']):.2f}/{_hit(cB, hl['hatch']):.2f} (all 0)")
        print(f"   PLUG WINDOW at ({ux:.1f},{uy:.1f}) (top {uz + PLUG_H / 2:.2f}) in the outer wall: 0.6 in from the face the wall over it is solid {seg[0][0]:.2f}..{seg[0][1]:.2f} ({seg[0][1] - seg[0][0]:.2f} tall; the recess floor there {rest_z(A["plans"][k], uy) - PAD_DEPTH:.2f})" if seg else "   PLUG WINDOW: NO solid over it")
        bp = back_profile(A["plans"][k], hull, h["W"], k)
        segs = column_tops(hull, h["W"], k, pit_front_probes(A["plans"][k], h["local"]["bosses"]))
        cross = pit_crossing(A["plans"][k], hull, h["W"], k, h["local"]["bosses"])
        print(f"   BACK ROUND R {BACK_R}: the hull's top 1 / 3 / 7 in from the back run's outline {[None if v is None else round(v, 2) for v in bp]} (expected {[round(back_round_z(d), 2) for d in (1.0, 3.0, 7.0)]}); "
              f"PLATE-DECK FUSE: the plate's edge 0.5 inside the pit's front, its top solid segment {[round(s[0][1] - s[0][0], 2) if s else None for s in segs]} tall (the plate {TOP} alone 3.1; at the thumb block's front its wall runs on under it, 3.8); "
              f"across the pit's edge 0.5..2.5 in, the plate's thickness {cross} (0 = a slot; build 87 had one 1 mm wide)")
        sh = sheets(hull, h["W"], k, A["plans"][k], h["local"]["badge"])
        print(f"   SHEETS (must be none): over the rim in front of y 15: {len(sh['above'])} ({sum(v for v, _ in sh['above']):.3f} mm3) {sh['above'][:4]}; in the pit over the plate: {len(sh['pit'])} {sh['pit'][:4]}; over the border at the pit's edge: {len(sh['edge'])} {sh['edge'][:4]}; "
              f"the zone / palm seam empty through the plate: {sh['crack']:.3f} mm3 (build 88: 14 / 2 fins up to 5.5 over the rim, 0.08 crack)")
        if k == "R":
            ux, uy, uz = h["local"]["pi_usb"]
            wx, wy, wz = h["local"]["usb_win"]
            L = wy + WALL / 2 + 6.0 - uy
            path = h["W"] * Pos(ux, uy + L / 2, uz) * Box(11.0, L, 7.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))   # an 11 x 7 plug overmold from the port's face out past the wall
            print(f"   PI USB (Willy 12:39): the OTG port at u {ux:.1f} z {uz:.1f} faces the back wall; OTG OVAL {PI_OVAL[0]} x {PI_OVAL[1]} at u {wx:.1f} centred on the port z {wz:.1f} "
                  f"(155's lift door dropped in 161); an 11 x 7 x {L:.0f} plug path vs hull {_hit(path, hull):.2f} / HAT {_hit(path, h['hat']):.2f} / hatch {_hit(path, h['hatch']):.2f} / V713 {_hit(path, h['v713']):.2f} / bezel {_hit(path, h['bezel']):.2f} (all 0)")
            sr = h["local"]["sd_relief"]
            card = h["W"] * Pos(sr[0] + (sr[1] - sr[0] - 0.5) / 2, (sr[2] + sr[3]) / 2, (sr[4] + sr[5]) / 2) * Box(sr[1] - sr[0] - 0.5, sr[3] - sr[2] - 2 * SD_RELIEF[1] + 0.2, 1.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
            print(f"   microSD: the slot is at the pocket's OUTER wall (builds 1-84 relieved the inner wall for what is the camera FPC); relief {SD_RELIEF[0]} deep: a card {SD_RELIEF[0] - 0.5:.1f} proud vs hull {_hit(card, hull):.2f}")
        vb = A["plans"][k]["v713"].bounds
        print(f"   CELL: blind half-pipe r {CELL_R + CELL_CLR} along the top row, centre z {h['local']['cell_at'].position.Z:.1f}, cell top {h['local']['cell_top']:.1f} under the bore top {h['local']['bore_top']:.1f} (rim {RIM_Z}); "
              f"V713 bay u {vb[0]:.1f}..{vb[2]:.1f} y {vb[1]:.1f}..{vb[3]:.1f} on a {V713_PAD} pad, fence {V713_FENCE}, USB-C window {USB_W} x {USB_H} ({'back wall' if k == 'R' else 'inner face'}, GUESS)")
    if mode != "full":
        print("TENT: skipped (VELVET_CHECKS=full runs the stand: seated / drop-in / swing / sweep)")
        return
    for k in "RL":
        h = Hs[k]
        o, d = h["tent_axis"]
        fo, fd = h["flap_axis"]
        ax, fax = Axis(o, d), Axis(fo, fd)
        print(f"TENT {k}: base valid {h['base'].is_valid} {len(h['base'].solids())} solid(s) {h['base'].volume / 1000:.1f} cm3; {len(A['plans'][k]['slots'])} knuckles ({len(A['plans'][k]['hull_kn'])} hull / {len(A['plans'][k]['base_kn'])} base), "
              f"pin {tent_pin_len(A['plans'][k]):.0f} long; flat: base vs hull {_hit(h['base'], h['hull']):.2f} / hatch {_hit(h['base'], h['hatch']):.2f} / flap {_hit(h['base'], h['flap']):.2f}, "
              f"flap vs hull {_hit(h['flap'], h['hull']):.2f} / hatch {_hit(h['flap'], h['hatch']):.2f}, flap pin vs hatch {_hit(h['fpin'], h['hatch']):.2f} / flap {_hit(h['fpin'], h['flap']):.2f} / base {_hit(h['fpin'], h['base']):.2f}, hinge pin vs hull {_hit(h['tpin'], h['hull']):.2f} at {_where(h['tpin'], h['hull'])} / base {_hit(h['tpin'], h['base']):.2f}, pin valid {h['tpin'].is_valid}; REST HINGE: rest pin vs hull {_hit(h['rpin'], h['hull']):.2f} / base {_hit(h['rpin'], h['base']):.2f} / hatch {_hit(h['rpin'], h['hatch']):.2f}")
        for x, th, ph, t, _c in tent_angles():
            hull_t, hatch_t = h["hull"].rotate(ax, th), h["hatch"].rotate(ax, th)
            flap_t = h["flap"].rotate(fax, ph).rotate(ax, th)                      # its own swing, then the half's (the viewer's order with root parents)
            lift = (hull_t.bounding_box().max.Z - h["hull"].bounding_box().max.Z)
            drop = max(_hit(h["flap"].rotate(fax, ph).rotate(ax, th + d), h["base"]) for d in (4.0, 3.0, 2.0, 1.0, 0.5))
            print(f"   seat {x:.0f}: tent {th:.1f} deg (flap {ph:.1f}, plate lean {t:.1f}): SEATED hull vs base {_hit(hull_t, h['base']):.2f}, hatch vs base {_hit(hatch_t, h['base']):.2f}, "
                  f"flap vs base {_hit(flap_t, h['base']):.2f} at {_where(flap_t, h['base'])} / hull {_hit(flap_t, hull_t):.2f} / hatch {_hit(flap_t, hatch_t):.2f}, nose to the seat {flap_t.distance_to(h['base']):.2f} (0 = bearing); "
                  f"DROP-IN (lowered from +4 deg, flap held) worst clash {drop:.2f}; top rises {lift:.0f}")
        top = 35.0                                              # lift the half this far, swing the stand, lower it (30 hits the base mid-swing)
        hull_30, hatch_30 = h["hull"].rotate(ax, top), h["hatch"].rotate(ax, top)
        swing = [(a, _hit(h["flap"].rotate(fax, a).rotate(ax, top), h["base"]) + _hit(h["flap"].rotate(fax, a).rotate(ax, top), hull_30) + _hit(h["flap"].rotate(fax, a).rotate(ax, top), hatch_30))
                 for a in range(0, 141, 10)]
        sweep = [(a, _hit(h["hull"].rotate(ax, a), h["base"]) + _hit(h["hatch"].rotate(ax, a), h["base"]) + _hit(h["flap"].rotate(ax, a), h["base"])) for a in range(0, 46, 5)]
        print(f"   SWING (half at {top:.0f} deg, flap 0..140): clashes {[a for a, v in swing if v > 0.05] or 'none'}; TENT SWEEP (flap folded, 0..45 deg): clashes {[a for a, v in sweep if v > 0.05] or 'none'}")
    kn = knob()
    print(f"KNOB: printed, SUNK, {KNOB_D} x {KNOB_H:.1f}, top {KNOB_TOP - RIM_Z:.1f} vs the rim, well clearance {KNOB_CLR}, dimple {DIMPLE_D} x {DIMPLE_DEPTH} at {DIMPLE_OFF} off-centre; bore {KNOB_BORE_D} x {KNOB_BORE:.1f}, "
          f"knurl gripped {EC11_SHAFT_TOP - (EC11_PAD_Z + EC11_KNURL0):.1f}; shaft cut {EC11_PAD_Z + 19.44 - EC11_SHAFT_TOP:.1f} (0 = none), roof over the bore {KNOB_TOP - KNOB_Z0 - KNOB_BORE:.2f}, EC11 legs to the hatch {EC11_PAD_Z - 0.8 - FLOOR:.1f}; valid {kn.is_valid}, {kn.volume / 1000:.2f} cm3")
    print("GAPS: the TPS43 is datasheet geometry (corner r GUESS 2.0; vendor STEP on request); the V713 charger/boost has NO STEP (bay from the listing's 25 x 20 + 4 high, port centred: GUESS -- measure one); "
          "the cell is a tabbed 18650, leads soldered (the bore's blind ends keep LEAD of room each; no spring contacts); power switches; the Pi's USB/HDMI face the pocket interior (powered through the header, by design); Pico USB end assumed the plug (GUESS); "
          "the knob's bore on the knurled shaft is a press-fit GUESS; the pane is glued; BLE between the Pico 2 Ws is firmware's (alice-kb), unverified")
