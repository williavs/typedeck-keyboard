"""THE BACK CHAIN as lines and true arcs (build 118; Willy 18:10 'leftover ribs on the back' = the layered R6 round's 24 steps). The plan's
back run is a flow-processed polyline (r 4 corners from the feet's WALL buffer, 15-degree chords on the S-curves, 0.02-0.3 of chording
noise), so it is re-drawn as a G1 chain: the long runs stay exact lines, each corner is one arc fitted within CORNER_TOL and shifted onto
its line's tangent, and every S-curve between those anchors becomes biarcs (tangent to both anchors, split until within BIARC_TOL of the
old outline) -- tangent at every junction by construction. OCC then fillets the whole chain in one pass with a
radius law: BACK_R along the runs, r - CORNER_GAP along an arc tighter than that (an r 4 corner under the r 6 ball refused a constant
fillet, item 34; opening the corners to r 7 thins the wall over the cell and the display feet to 1.3, plan.py's assert). The plan's outline
in the zone is the chain sampled at 0.5, so the hull's wall is the chord polyline of the same arcs (0.004 off) and hull.key_deck's band
swap is seamless."""
import math

from shapely.geometry import LineString, Point, Polygon, box as sbox
from shapely.ops import unary_union
from build123d import Line, Pos, Solid, ThreePointArc, Vector, Wire, extrude, make_face
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet

from velvet.params import BACK_R

BACK_X0, BACK_Y0 = 8.0, 4.0        # the chain's zone: x past the seam wall's strip, y from under the back run's outer end (the clip box's corners stay sharp)
FIT_TOL, CORNER_TOL, LONG, BIARC_TOL, MIN_RUN, R_MIN, CORNER_GAP = 0.005, 0.2, 5.0, 0.4, 6.0, 3.0, 0.5
OPEN_R = BACK_R + CORNER_GAP      # corners in the round's path are OPENED to this (build 120: the round then runs at BACK_R with no dip anywhere)
TIGHT_R = OPEN_R - 0.1            # an arc under this dips the radius law (the opened corners and their lane arcs sit within 0.05 of OPEN_R)
STUB = 2.0                        # a wall left shorter than this by a lane change is taken into the lane change whole
EASE = 10.0                        # the round's radius eases from a corner's r - CORNER_GAP back to BACK_R over this run beside the corner arc
LAW_STEP = 1.0                     # the radius law is handed to OCC sampled this often along each edge (Law_Interpol through the samples)
SAMPLE = 1.1                       # the chain's chord length in the plan's outline (the old outline's on an r 4 corner; at 0.5 the plan's r 2 fillet solid is "valid" but every boolean on it comes back EMPTY)


class El:
    """A chain element: a line p0 -> p1, or an arc p0 -> p1 about c (ccw or not); tangents at both ends."""

    def __init__(self, kind, p0, p1, c=None, box=False):
        self.kind, self.p0, self.p1, self.c, self.box = kind, tuple(p0), tuple(p1), c, box
        if kind == "arc":
            self.r = math.dist(p0, c)
            mx, my = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2
            d = math.hypot(mx - c[0], my - c[1])
            self.mid = (c[0] + (mx - c[0]) / d * self.r, c[1] + (my - c[1]) / d * self.r)   # on the circle, on the chord's bisector (arcs < 180)
            self.ccw = (self.mid[0] - p0[0]) * (p1[1] - p0[1]) - (self.mid[1] - p0[1]) * (p1[0] - p0[0]) > 0

    def t(self, p):
        if self.kind == "line":
            return _unit(self.p0, self.p1)
        rx, ry = (p[0] - self.c[0]) / self.r, (p[1] - self.c[1]) / self.r
        return (-ry, rx) if self.ccw else (ry, -rx)

    def t0(self):
        return self.t(self.p0)

    def t1(self):
        return self.t(self.p1)

    def length(self):
        if self.kind == "line":
            return math.dist(self.p0, self.p1)
        return 2 * self.r * math.asin(min(1.0, math.dist(self.p0, self.p1) / (2 * self.r)))

    def degrees(self):
        return 0.0 if self.kind == "line" else math.degrees(self.length() / self.r)

    def dist(self, q):
        if self.kind == "line":
            return LineString([self.p0, self.p1]).distance(Point(q))
        return abs(math.dist(q, self.c) - self.r)

    def edge(self):
        if self.kind == "line":
            return Line(Vector(*self.p0), Vector(*self.p1))
        return ThreePointArc(Vector(*self.p0), Vector(*self.mid), Vector(*self.p1))

    def samples(self, step=0.5):
        """points along the element from p0, excluding p1"""
        if self.kind == "line":
            return [self.p0]
        n = max(2, int(self.length() / step) + 1)
        a0 = math.atan2(self.p0[1] - self.c[1], self.p0[0] - self.c[0])
        sweep = self.length() / self.r * (1 if self.ccw else -1)
        return [(self.c[0] + self.r * math.cos(a0 + sweep * k / n), self.c[1] + self.r * math.sin(a0 + sweep * k / n)) for k in range(n)]

    def __repr__(self):
        return "box" if self.box else (f"L{self.length():.1f}" if self.kind == "line" else f"A{self.r:.1f}/{self.degrees():.0f}")


def _unit(a, b):
    d = math.dist(a, b)
    return ((b[0] - a[0]) / d, (b[1] - a[1]) / d)


def _from(p0, t0, p1):
    """the line or circle through p0 with tangent t0 and through p1"""
    n0 = (-t0[1], t0[0])
    v = (p1[0] - p0[0], p1[1] - p0[1])
    vn = v[0] * n0[0] + v[1] * n0[1]
    if abs(vn) < 1e-6 * math.hypot(*v):
        return El("line", p0, p1)
    r = (v[0] ** 2 + v[1] ** 2) / (2 * vn)
    return El("arc", p0, p1, (p0[0] + r * n0[0], p0[1] + r * n0[1]))


def _circle3(a, b, c):
    (x1, y1), (x2, y2), (x3, y3) = a, b, c
    d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    if abs(d) < 1e-12:
        return None
    ux = ((x1 ** 2 + y1 ** 2) * (y2 - y3) + (x2 ** 2 + y2 ** 2) * (y3 - y1) + (x3 ** 2 + y3 ** 2) * (y1 - y2)) / d
    uy = ((x1 ** 2 + y1 ** 2) * (x3 - x2) + (x2 ** 2 + y2 ** 2) * (x1 - x3) + (x3 ** 2 + y3 ** 2) * (x2 - x1)) / d
    return (ux, uy), math.hypot(x1 - ux, y1 - uy)


def _densify(pts, step=0.5):
    out = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(math.dist(a, b) / step))
        out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]
    return out


def _fit(pts, tol):
    """greedy exact fit of a chord polyline (sampled every 0.5, so a long chord cannot pass for an arc through its ends): the longest run
    from i that is one line or one circle within tol"""
    pts = _densify(pts)
    out, i, n = [], 0, len(pts)
    while i < n - 1:
        best = ("line", i + 1)
        for j in range(i + 2, n):
            seg = pts[i:j + 1]
            if max(LineString([pts[i], pts[j]]).distance(Point(q)) for q in seg) <= tol:
                best = ("line", j)
                continue
            fc = _circle3(pts[i], pts[(i + j) // 2], pts[j])
            if fc and 1.0 < fc[1] < 400.0 and max(abs(math.dist(q, fc[0]) - fc[1]) for q in seg) <= tol:
                best = ("arc", j)
                continue
            break
        kind, j = best
        seg = pts[i:j + 1]
        out.append(El("line", seg[0], seg[-1]) if kind == "line" or len(seg) < 3 else El("arc", seg[0], seg[-1], _circle3(seg[0], seg[len(seg) // 2], seg[-1])[0]))
        i = j
    return out


def _turn(ta, tb):
    """degrees from tangent ta to tb"""
    return math.degrees(math.atan2(ta[0] * tb[1] - ta[1] * tb[0], ta[0] * tb[0] + ta[1] * tb[1]))


def _merge_corners(elems):
    """consecutive tight fragments (the flow's chording of an r 4 corner: A5.4/9 A4.2/11 A4.0/57) -> one arc within CORNER_TOL"""
    out, i = [], 0
    while i < len(elems):
        j = i
        while j < len(elems) and not elems[j].box and (elems[j].kind == "arc" and elems[j].r < 10.0 or elems[j].kind == "line" and elems[j].length() < 1.5):
            j += 1
        run = elems[i:j]
        if run and abs(_turn(run[0].t0(), run[-1].t1())) >= 40.0:
            pts = [run[0].p0] + [q for e in run for q in e.samples(0.5)[1:]] + [e.p1 for e in run]
            c, r = _circle3(pts[0], pts[len(pts) // 2], pts[-1])
            dev = max(abs(math.dist(q, c) - r) for q in pts)
            assert dev <= CORNER_TOL, f"a back corner at {run[0].p0} is no circle: {dev:.3f} off r {r:.2f}"
            out.append(El("arc", run[0].p0, run[-1].p1, c))
            i = j
        else:
            out.append(elems[i])
            i += 1
    return out


def _tangent_to(arc, line, at_start):
    """shift the arc's circle onto the line's tangent (|c - line| = r), its end on the line's side at the tangent foot"""
    d = _unit(line.p0, line.p1)
    n = (-d[1], d[0])
    off = (arc.c[0] - line.p0[0]) * n[0] + (arc.c[1] - line.p0[1]) * n[1]        # signed distance of c from the line
    sgn = 1.0 if off > 0 else -1.0
    c = (arc.c[0] + n[0] * (sgn * arc.r - off), arc.c[1] + n[1] * (sgn * arc.r - off))
    foot = (c[0] - n[0] * sgn * arc.r, c[1] - n[1] * sgn * arc.r)
    other = arc.p1 if at_start else arc.p0
    other = (c[0] + (other[0] - c[0]) / math.dist(other, c) * arc.r, c[1] + (other[1] - c[1]) / math.dist(other, c) * arc.r)
    if at_start:
        arc.__init__("arc", foot, other, c)
        line.__init__("line", line.p0, foot)
    else:
        arc.__init__("arc", other, foot, c)
        line.__init__("line", foot, line.p1)


def _biarc(p0, t0, p1, t1):
    """two arcs p0 -> j -> p1 tangent to t0 at p0, t1 at p1 and to each other at j (equal tangent lengths)"""
    v = (p1[0] - p0[0], p1[1] - p0[1])
    s = (t0[0] + t1[0], t0[1] + t1[1])
    A, B, C = (s[0] ** 2 + s[1] ** 2) - 4.0, -2.0 * (v[0] * s[0] + v[1] * s[1]), v[0] ** 2 + v[1] ** 2
    if abs(A) < 1e-9:
        L = -C / B
    else:
        disc = max(0.0, B * B - 4 * A * C)
        L = min(r for r in ((-B + sg * math.sqrt(disc)) / (2 * A) for sg in (1, -1)) if r > 0)
    q0 = (p0[0] + L * t0[0], p0[1] + L * t0[1])
    q1 = (p1[0] - L * t1[0], p1[1] - L * t1[1])
    j = ((q0[0] + q1[0]) / 2, (q0[1] + q1[1]) / 2)
    return [_from(p0, t0, j), _from(j, _unit(q0, q1), p1)]


def _smooth_run(run, p0, t0, p1, t1, tol):
    """the run (small elements: an S-curve's chords and micro-joints) as biarcs p0 -> p1 within tol of it, split at its middle while not
    and while the piece is longer than MIN_RUN (shorter pieces take the biarc as is: the joints' r 1 micro-arcs are the chording, not the shape)"""
    old = LineString([run[0].p0] + [e.p1 for e in run])
    ba = _biarc(p0, t0, p1, t1)
    total = sum(e.length() for e in run)
    dev = max(max(old.distance(Point(q)) for e in ba for q in e.samples(0.25) + [e.p1]),                      # the biarc off the old polyline ...
              max(min(e.dist(q) for e in ba) for r in run for q in r.samples(0.25) + [r.p1]))                    # ... and the old polyline off the biarc
    if len(run) < 2 or total < MIN_RUN or dev <= tol:
        return ba
    acc, k = 0.0, 0
    while k < len(run) - 2 and acc + run[k].length() < total / 2:
        acc += run[k].length()
        k += 1
    return _smooth_run(run[:k + 1], p0, t0, run[k].p1, run[k].t1(), tol) + _smooth_run(run[k + 1:], run[k].p1, run[k].t1(), p1, t1, tol)


def _chain(pts):
    """the clip ring's polyline (between the box's lines) as a G1 chain of lines and arcs"""
    elems = _merge_corners(_fit(pts, FIT_TOL))
    anchor = lambda e: (e.kind == "line" and e.length() >= LONG) or (e.kind == "arc" and e.degrees() >= 40.0)
    for a, b in zip(elems, elems[1:]):                                     # each corner tangent to its anchor line
        if a.kind == "arc" and anchor(a) and b.kind == "line" and anchor(b):
            _tangent_to(a, b, at_start=False)
        elif a.kind == "line" and anchor(a) and b.kind == "arc" and anchor(b):
            _tangent_to(b, a, at_start=True)
    out, i = [], 0
    while i < len(elems):
        if anchor(elems[i]):
            out.append(elems[i])
            i += 1
            continue
        j = i
        while j < len(elems) and not anchor(elems[j]):
            j += 1
        run = elems[i:j]
        p0, t0 = (out[-1].p1, out[-1].t1()) if out else (run[0].p0, run[0].t0())
        p1, t1 = (elems[j].p0, elems[j].t0()) if j < len(elems) else (run[-1].p1, run[-1].t1())
        out += _smooth_run(run, p0, t0, p1, t1, BIARC_TOL)
        i = j
    return out


def _walk(e, L, from_end):
    """(point, tangent) L along element e back from its end (from_end) or on from its start"""
    if e.length() < L + STUB:                                                   # the whole wall goes into the lane change: no stub
        return (e.p0, e.t0()) if from_end else (e.p1, e.t1())
    if e.kind == "line":
        t = _unit(e.p0, e.p1)
        q = (e.p1[0] - t[0] * L, e.p1[1] - t[1] * L) if from_end else (e.p0[0] + t[0] * L, e.p0[1] + t[1] * L)
        return q, t
    rot = L / e.r * (1.0 if e.ccw else -1.0) * (-1.0 if from_end else 1.0)
    base = e.p1 if from_end else e.p0
    vx, vy = base[0] - e.c[0], base[1] - e.c[1]
    q = (e.c[0] + vx * math.cos(rot) - vy * math.sin(rot), e.c[1] + vx * math.sin(rot) + vy * math.cos(rot))
    return q, e.t(q)


def _open_corners(chain, avoid):
    """Every corner arc under OPEN_R that the round runs through (not touching `avoid`: the seam corner, whose wall must not move) re-drawn
    at OPEN_R: its two walls shifted OUT by delta -- the least that keeps the new arc outside the old one, so the hull only gains material
    and no wall thins -- and each wall rejoined by a biarc lane-change T long whose arcs are no tighter than OPEN_R; the chain stays G1
    (build 120; Willy 22:16 'does not have a smooth consistent rounded edge': the law's dip to r - 0.5 through an r 4 corner read as a
    pinch). Returns the new chain."""
    out, i = list(chain), 1
    while i < len(out) - 1:
        e, a, b = out[i], out[i - 1], out[i + 1]
        if e.kind != "arc" or e.r >= OPEN_R or a.box or b.box or avoid.intersects(LineString([e.p0, e.mid, e.p1])):
            i += 1
            continue
        t0, t1 = e.t0(), e.t1()
        tau = math.radians(e.degrees())
        s = math.cos(tau / 2)
        delta = (OPEN_R - e.r) * (1.0 - s) + 0.1
        n0, n1 = ((e.p0[0] - e.c[0]) / e.r, (e.p0[1] - e.c[1]) / e.r), ((e.p1[0] - e.c[0]) / e.r, (e.p1[1] - e.c[1]) / e.r)   # outward normals at the feet
        bis = _unit((0.0, 0.0), (n0[0] + n1[0], n0[1] + n1[1]))
        X = (e.c[0] + bis[0] * e.r / s, e.c[1] + bis[1] * e.r / s)                                     # the walls' apex
        c = (X[0] - bis[0] * (OPEN_R - delta) / s, X[1] - bis[1] * (OPEN_R - delta) / s)
        p0, p1 = (c[0] + n0[0] * OPEN_R, c[1] + n0[1] * OPEN_R), (c[0] + n1[0] * OPEN_R, c[1] + n1[1] * OPEN_R)
        f_off = (OPEN_R - delta - e.r) * math.tan(tau / 2)                                             # the new feet lie this much further from the apex
        T = math.sqrt(4.0 * delta * OPEN_R - delta * delta) + 0.5                                      # two lane-change arcs of (T^2 + delta^2) / 4 delta >= OPEN_R
        qa, ta = _walk(a, f_off + T, from_end=True)
        qb, tb = _walk(b, f_off + T, from_end=False)
        a2 = [] if qa == a.p0 else [El("line", a.p0, qa) if a.kind == "line" else El("arc", a.p0, qa, a.c)]
        b2 = [] if qb == b.p1 else [El("line", qb, b.p1) if b.kind == "line" else El("arc", qb, b.p1, b.c)]
        lane_a, lane_b = _biarc(qa, ta, p0, t0), _biarc(p1, t1, qb, tb)
        tight = [round(x.r, 2) for x in lane_a + lane_b if x.kind == "arc" and x.r < TIGHT_R]
        assert not tight, f"open corner at {e.p0}: the lane change needs arcs of {tight} (under {TIGHT_R})"
        out[i - 1:i + 2] = a2 + lane_a + [El("arc", p0, p1, c)] + lane_b + b2
        i += 1 + len(a2) + len(lane_a) + len(lane_b)
    return out


def back_chain(plan):
    """(the plan with its back run re-drawn, the chain's elements): the zone x > BACK_X0, y > BACK_Y0; the chain runs the zone's clip
    of the outline, the clip box's two lines included (box=True), so hull can prism + fillet the zone alone."""
    zone = sbox(BACK_X0, BACK_Y0, 1e4, 1e4)
    clip = plan.intersection(zone)
    ring = list(clip.exterior.coords)[:-1]
    ring = [q for k, q in enumerate(ring) if math.dist(q, ring[k - 1]) > 1e-6]          # no zero-length chords (the clip leaves doubled vertices)
    boxseg = lambda a, b: (abs(a[0] - BACK_X0) < 1e-4 and abs(b[0] - BACK_X0) < 1e-4) or (abs(a[1] - BACK_Y0) < 1e-4 and abs(b[1] - BACK_Y0) < 1e-4)
    k0 = next(k for k, q in enumerate(ring) if boxseg(q, ring[(k + 1) % len(ring)]))
    ring = ring[k0:] + ring[:k0]
    out, chain = [], []
    for a, b in zip(ring, ring[1:] + ring[:1]):
        if boxseg(a, b):
            if chain:
                out += _chain(chain)
                chain = []
            out.append(El("line", a, b, box=True))
        else:
            chain = chain or [a]
            chain.append(b)
    if chain:
        out += _chain(chain)
    out = _open_corners(out, sbox(plan.bounds[2] - BACK_R - 2.0, -1e4, 1e4, 1e4))        # not the seam corner: the seam wall (the plan's max x) stays
    tight = [(round(e.r, 2), (round(e.p0[0], 1), round(e.p0[1], 1))) for e in out if e.kind == "arc" and e.r < R_MIN]
    assert not tight, f"the back chain keeps arcs tighter than r {R_MIN}: {tight[:4]} in {out}"
    pts = [q for e in out for q in e.samples(SAMPLE)]
    plan = unary_union([plan.difference(zone), Polygon(pts)]).buffer(0)
    assert plan.geom_type == "Polygon", f"the back chain left the plan a {plan.geom_type}"
    return plan, out


def kinks(chain):
    """(degrees, point) at every non-tangent junction of the chain, the box's corners excluded"""
    out = []
    for a, b in zip(chain, chain[1:]):
        if a.box or b.box:
            continue
        ta, tb = a.t1(), b.t0()
        d = math.degrees(math.atan2(ta[0] * tb[1] - ta[1] * tb[0], ta[0] * tb[0] + ta[1] * tb[1]))
        if abs(d) > 0.02:
            out.append((round(d, 3), (round(b.p0[0], 1), round(b.p0[1], 1))))
    return out


def radius_law(chain):
    """The round's radius along the chain's abscissa: BACK_R everywhere, r - CORNER_GAP over a corner arc (r < BACK_R + CORNER_GAP), a smoothstep
    EASE long on either side of it -- ONE continuous law, so neighbouring edges agree at their shared vertex. Returns (r(s), [(s0, s1) per element])."""
    spans, dips, s = [], [], 0.0
    for e in chain:
        spans.append((s, s + e.length()))
        if e.kind == "arc" and e.r < TIGHT_R:
            dips.append((s, s + e.length(), e.r - CORNER_GAP))
        s += e.length()

    def r(s):
        out = BACK_R
        for a, b, rc in dips:
            t = max(a - s, s - b, 0.0) / EASE
            if t < 1.0:
                out = min(out, rc + (BACK_R - rc) * t * t * (3.0 - 2.0 * t))
        return out
    return r, spans


def back_solid(chain, z0, z1, law=True):
    """the zone's prism z0..z1 with its top edge along the chain rounded by OCC's evolutive fillet under radius_law: BACK_R, dipping to
    r - CORNER_GAP over a corner arc with an EASE-long smoothstep each side (build 119; build 118 set one radius per vertex, and OCC's law
    then ran linearly over the WHOLE neighbouring element: L's 69 mm back line sat at r 3.5 end to end, R's 58 mm one swelled 3.5 -> 6).
    Each edge gets the law sampled every LAW_STEP as (u, r) pairs; law=False rounds the whole chain at the tightest radius instead.
    Returns (solid, the radii at the edges' ends)"""
    from OCP.TopExp import TopExp
    from OCP.BRep import BRep_Tool
    from OCP.TColgp import TColgp_Array1OfPnt2d
    from OCP.gp import gp_Pnt2d
    wire = Wire.combine([e.edge() for e in chain], tol=1e-4)[0]                      # the junctions agree to 1e-6, OCC's vertices want 1e-7
    assert wire.is_closed, "back_solid: the chain's wire is not closed"
    sol = extrude(make_face(wire), amount=z1 - z0).moved(Pos(0, 0, z0))
    top = max(sol.faces(), key=lambda f: f.center().Z)
    onbox = lambda e: e.geom_type.name == "LINE" and (abs(e.center().X - BACK_X0) < 1e-4 or abs(e.center().Y - BACK_Y0) < 1e-4)
    r_of, spans = radius_law(chain)
    r_min = min(r_of(s) for s0, s1 in spans for s in (s0, s1))

    def span(a, b):
        """the chain abscissa at the OCC edge's first and last vertex"""
        for e, (s0, s1) in zip(chain, spans):
            if math.dist(a, e.p0) < 1e-3 and math.dist(b, e.p1) < 1e-3:
                return s0, s1
            if math.dist(a, e.p1) < 1e-3 and math.dist(b, e.p0) < 1e-3:
                return s1, s0
        raise AssertionError(f"back_solid: no chain element runs {a} -> {b}")

    mk, radii = BRepFilletAPI_MakeFillet(sol.wrapped), set()
    for e in top.edges():
        if not onbox(e):
            mk.Add(e.wrapped)
    for ic in range(1, mk.NbContours() + 1):
        for k in range(1, mk.NbEdges(ic) + 1):
            oe = mk.Edge(ic, k)
            a, b = (BRep_Tool.Pnt_s(v) for v in (TopExp.FirstVertex_s(oe), TopExp.LastVertex_s(oe)))
            sa, sb = span((a.X(), a.Y()), (b.X(), b.Y()))
            n = max(2, round(abs(sb - sa) / LAW_STEP)) + 1
            rs = [r_of(sa + (sb - sa) * i / (n - 1)) for i in range(n)] if law else [r_min] * n
            if max(rs) - min(rs) < 1e-9:
                mk.SetRadius(rs[0], ic, k)
            else:
                arr = TColgp_Array1OfPnt2d(1, n)
                for i, rv in enumerate(rs):
                    arr.SetValue(i + 1, gp_Pnt2d(i / (n - 1), rv))
                mk.SetRadius(arr, ic, k)
            radii |= {round(rs[0], 2), round(rs[-1], 2)}
    mk.Build()
    assert mk.IsDone(), f"back_solid: OCC refuses the back round with radii {sorted(radii)} (law {law})"
    out = Solid(mk.Shape())
    assert out.is_valid and len(out.solids()) == 1, "back_solid: the back round's fillet left an invalid solid"
    return out, sorted(radii)

def chain_prism(plan, chain, z0, z1):
    """The plan's prism z0..z1 whose outline wire is the chain's TRUE ARCS in the zone and the plan's polyline elsewhere (rounded_prism_chain's
    prism before its fillet; hull.swell_round cuts its top to the swell's planes first, build 132)."""
    run = [e for e in chain if not e.box]
    start, end = run[0].p0, run[-1].p1
    ring = list(plan.exterior.coords)[:-1]
    # the polyline from the chain's end round the front to its start: walk the ring in the direction that leaves the zone
    zone = sbox(BACK_X0, BACK_Y0, 1e4, 1e4)
    i_end = min(range(len(ring)), key=lambda k: math.dist(ring[k], end))
    i_start = min(range(len(ring)), key=lambda k: math.dist(ring[k], start))
    fwd = [ring[k % len(ring)] for k in range(i_end, i_end + len(ring))]
    fwd = fwd[:next(k for k, q in enumerate(fwd) if math.dist(q, start) < 1e-6) + 1]
    if any(zone.contains(Point(q)) for q in fwd[1:-1]):                           # the wrong way round: take the other direction
        fwd = [ring[k % len(ring)] for k in range(i_end, i_end - len(ring), -1)]
        fwd = fwd[:next(k for k, q in enumerate(fwd) if math.dist(q, start) < 1e-6) + 1]
    assert not any(zone.contains(Point(q)) for q in fwd[1:-1]), "rounded_prism_chain: the polyline part crosses the zone"
    edges = [e.edge() for e in run] + [Line(Vector(*a), Vector(*b)) for a, b in zip(fwd, fwd[1:]) if math.dist(a, b) > 1e-6]
    wire = Wire.combine(edges, tol=1e-4)[0]
    assert wire.is_closed, "rounded_prism_chain: the outline wire is not closed"
    return extrude(make_face(wire), amount=z1 - z0).moved(Pos(0, 0, z0))


def rounded_prism_chain(plan, chain, z0, z1, r):
    """geo.rounded_prism for a plan whose back run is the chain: the outline wire is the chain's true arcs in the zone and the plan's
    polyline elsewhere, so OCC's r fillet on the top loop rides tangent arcs there (on the chain's 1.1 chords it left a 1.2 mm vertex
    tolerance in the swap, build 118c)"""
    sol = chain_prism(plan, chain, z0, z1)
    top = max(sol.faces(), key=lambda f: f.center().Z)
    out = sol.fillet(r, top.edges())
    assert out.is_valid, "rounded_prism_chain: invalid after the fillet"
    return out
