"""2D/3D helpers: shapely outlines -> build123d solids, overlap probes, the flow filter, the rest's slope plane."""
import math

from shapely.geometry import Polygon, box as sbox
from shapely.ops import unary_union
from shapely import affinity
from shapely.geometry.polygon import orient
from build123d import Align, Box, Compound, Polyline, Pos, Rot, extrude, import_step, loft, make_face

from velvet.params import *  # noqa: F401,F403


def prism(poly, z0, z1):
    poly = poly.buffer(0.01, join_style=2, mitre_limit=20.0).buffer(-0.01, join_style=2, mitre_limit=20.0)   # DISSOLVED: a shapely union of pieces sharing a colinear
    #                                                                                      edge with different endpoints can keep both traversals (a zero-width slit); OCC
    #                                                                                      gives the prism two faces 0.002 apart there and a cut leaves the hull's material
    #                                                                                      between them as a sheet (R's thumb pit diagonal, build 117). A 0.01 mitre
    #                                                                                      closing fills the slit and moves nothing else (plan polygons: symdiff 0.0002 mm2)
    if poly.geom_type in ("MultiPolygon", "GeometryCollection"):
        parts = [prism(g, z0, z1) for g in poly.geoms if g.geom_type == "Polygon" and g.area > 1.0]
        out = parts[0]
        for q in parts[1:]:
            out += q
        return out
    assert poly.geom_type == "Polygon", f"prism wants one polygon, got {poly.geom_type}"
    pts = []
    for q in list(poly.simplify(0.02).exterior.coords)[:-1]:
        if not pts or math.dist(q, pts[-1]) > 0.05:
            pts.append(q)
    if math.dist(pts[0], pts[-1]) <= 0.05:
        pts.pop()
    return Pos(0, 0, z0) * extrude(make_face(Polyline(*pts, close=True)), amount=z1 - z0)


def fuzzy_cut(a, b, fuzzy=0.01):
    """a - b with OCC's fuzzy value: entities within `fuzzy` of each other are taken as coincident. For the ONE boolean that is on a knife edge, the
    console's step chamfer (build 116, R: with the box's floor at RIM_Z - c or + 0.15 the plain cut handed back an empty 5-solid hull, at - 0.15 the
    right slope; every 14 mm window of the tool cut cleanly; rebuilt topology, clean and fix changed nothing; fuzzy 0.001 half-cut, 0.01 cut right,
    0.05 went invalid -- scratchpad chamfer_fix*). Not a default: the plain cut is loud where it fails, the fuzzy one hides what it merges."""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.TopTools import TopTools_ListOfShape
    op = BRepAlgoAPI_Cut()
    args, tools = TopTools_ListOfShape(), TopTools_ListOfShape()
    args.Append(a.wrapped)
    tools.Append(b.wrapped)
    op.SetArguments(args)
    op.SetTools(tools)
    op.SetFuzzyValue(fuzzy)
    op.Build()
    out = Compound(op.Shape())
    sols = out.solids()
    return sols[0] if len(sols) == 1 else out


def rect(f, w, h):
    """The frame's w x h rectangle, corners in ring order (a valid Polygon)."""
    return [(f * Pos(x, y, 0)).position.to_tuple()[:2] for x, y in ((-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2))]


def placed(step, x, y, z, rz):
    """(Location, solid): the STEP's (or shape's) plan centre at (x, y), its lowest point at z, turned rz about z."""
    s = step if hasattr(step, "bounding_box") else import_step(str(step))
    b = s.bounding_box()
    loc = Pos(x, y, z) * Rot(0, 0, rz) * Pos(-b.center().X, -b.center().Y, -b.min.Z)
    return loc, loc * s


def prism_holed(poly, z0, z1):
    """A prism of a polygon WITH its holes (prism() takes the exterior only): the exterior's prism less each hole's; a MultiPolygon piece by piece."""
    if poly.geom_type == "MultiPolygon":
        parts = [prism_holed(g, z0, z1) for g in poly.geoms]
        out = parts[0]
        for q in parts[1:]:
            out += q
        return out
    out = prism(Polygon(poly.exterior.coords), z0, z1)
    for ring in poly.interiors:
        out -= prism(Polygon(ring.coords), z0 - 1.0, z1 + 1.0)
    return out


def _where(a, b):
    """Where a and b overlap: the overlap solids' joint bbox, as text."""
    bbs = [i.bounding_box() for sa in a.solids() for sb in b.solids() if (i := sa & sb) and i.volume > 0.01]
    if not bbs:
        return "-"
    return f"x {min(b.min.X for b in bbs):.1f}..{max(b.max.X for b in bbs):.1f} y {min(b.min.Y for b in bbs):.1f}..{max(b.max.Y for b in bbs):.1f} z {min(b.min.Z for b in bbs):.1f}..{max(b.max.Z for b in bbs):.1f}"


def _clip(big, part, m=3.0):
    """big cut to part's bbox + m: checks against a heavy STEP only see the neighbourhood."""
    bb = part.bounding_box()
    return big & Pos(bb.center()) * Box(bb.size.X + 2 * m, bb.size.Y + 2 * m, bb.size.Z + 2 * m, align=MID)


def fuzzy_fuse(a, b, fuzzy=0.01):
    """a + b with OCC's fuzzy value (see fuzzy_cut): the last try of hull.swap_band when the plain fuse hands back a solid missing part of the piece."""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
    from OCP.TopTools import TopTools_ListOfShape
    op = BRepAlgoAPI_Fuse()
    args, tools = TopTools_ListOfShape(), TopTools_ListOfShape()
    args.Append(a.wrapped)
    tools.Append(b.wrapped)
    op.SetArguments(args)
    op.SetTools(tools)
    op.SetFuzzyValue(fuzzy)
    op.Build()
    out = Compound(op.Shape())
    sols = out.solids()
    return sols[0] if len(sols) == 1 else out


def _hit(a, b):
    """Overlap volume, solid by solid on BOTH sides (build123d's & on a Compound drops the placement)."""
    return sum((i.volume if (i := sa & sb) else 0.0) for sa in a.solids() for sb in b.solids())


def flow(shape):
    """Closing (fills creases narrower than 2*FLOW) then opening (rounds convex corners to ROUND)."""
    s = shape.buffer(FLOW, join_style=1, quad_segs=6).buffer(-FLOW, join_style=1, quad_segs=6)
    s = s.buffer(-ROUND, join_style=1, quad_segs=6).buffer(ROUND, join_style=1, quad_segs=6)
    if s.geom_type != "Polygon":
        s = max(s.geoms, key=lambda g: g.area)
    return s.simplify(0.02)   # 0.15 chorded the hinge face's collinear vertices into a line 0.14 off at one end (the rail poked through it, build 63)


def clipped(x0, y0, x1, y1, big, small=CLIP_S):
    """A rectangle with its corners clipped at 45 deg: bottom-left + top-right by big, the other two by small (8 points, CCW)."""
    bl, br, tr, tl = big, small, big, small
    return [(x0 + bl, y0), (x1 - br, y0), (x1, y0 + br), (x1, y1 - tr), (x1 - tr, y1), (x0 + tl, y1), (x0, y1 - tl), (x0, y0 + bl)]


def bevel_cut(bot, z0, top, z1, xy=lambda x, y: (x, y)):
    """A window through a plate: a ruled loft from the bottom outline (z0) to the top one (z1), both 8 points, plus 1 mm past each end."""
    f = lambda pts, z: make_face(Polyline(*[(*xy(x, y), z) for x, y in pts], close=True))
    return loft([f(bot, z0), f(top, z1)]) + extrude(f(bot, z0), amount=-1.0) + extrude(f(top, z1), amount=1.0)


def rest_plane(p, dz=0.0, up=True):
    """The half-space over (up) / under the rest's sloped top plane, offset dz along z: through the rest's back edge at the rim, falling REST_SLOPE toward the front."""
    yb = p["foot"]["rest"].bounds[3]
    return Pos(0, yb, RIM_Z - TILT_DROP + dz) * Rot(REST_SLOPE, 0, 0) * Box(4000, 4000, 400, align=(Align.CENTER, Align.CENTER, Align.MIN if up else Align.MAX))


def rest_z(p, y):
    """The rest's sloped top plane's z (local frame) at y: rest_plane's face."""
    return RIM_Z - TILT_DROP + (y - p["foot"]["rest"].bounds[3]) * math.tan(math.radians(REST_SLOPE))


def plate_z(p, y):
    """The TILTED plate's top (local frame) at plan y: the tilt about the pivot line (yp, RIM_Z) less TILT_DROP (hull.tilt's T on z' = 0)."""
    a = math.radians(KEY_TILT)
    return (y - p["yp"] - RIM_Z * math.sin(a)) * math.tan(a) + RIM_Z * (1.0 - math.cos(a)) - TILT_DROP


def well_profile(p):
    """THE KEYWELL'S TOP in the y-z plane (build 148): the caps' ceiling line (CAP_TOP_T - KEY_PROUD over the tilted plate) in front, the border's
    level (the rest plane + BORDER_UP) behind, their 5 deg crease at y_c replaced by a CREASE_R arc tangent to the ceiling line and meeting the
    border's level CREASE_END deg short of tangent -- the arc only PLACES the junctions (y_f, y_b) clear of the run's vertices; the top built
    between them is the arc's CHORD, a plane (well_top, hull.well_cap: a cylinder face there broke every fuse). Returns (y_c, z_c, the arc's
    centre (y, z), (y_f, y_b), (the lines' slopes), the arc's mid point (y, z))."""
    a, s = math.tan(math.radians(KEY_TILT)), math.tan(math.radians(REST_SLOPE))
    cap = lambda y: plate_z(p, y) + (CAP_TOP_T - KEY_PROUD) / math.cos(math.radians(KEY_TILT))
    lvl = lambda y: rest_z(p, y) + BORDER_UP
    yc = -60.0 + (lvl(-60.0) - cap(-60.0)) / ((cap(-40.0) - cap(-60.0)) - (lvl(-40.0) - lvl(-60.0))) * 20.0
    zc = cap(yc)
    th = math.atan(a) - math.atan(s) - math.radians(CREASE_END)                                   # the arc's turn (the lines' 5 deg less the end angle)
    n1 = (math.sin(math.atan(a)), -math.cos(math.atan(a)))                                      # the ceiling line's normal into the material (down)
    def end(d):                                                                                 # the arc from the tangent point d in front of the corner
        tf = (yc - d * math.cos(math.atan(a)), zc - d * math.sin(math.atan(a)))
        o = (tf[0] + n1[0] * CREASE_R, tf[1] + n1[1] * CREASE_R)
        r0 = (-n1[0], -n1[1])                                                                   # the radius vector to the tangent point, turned back by th
        rb = (r0[0] * math.cos(th) + r0[1] * math.sin(th), -r0[0] * math.sin(th) + r0[1] * math.cos(th))
        pb = (o[0] + rb[0] * CREASE_R, o[1] + rb[1] * CREASE_R)
        rm = (r0[0] * math.cos(th / 2) + r0[1] * math.sin(th / 2), -r0[0] * math.sin(th / 2) + r0[1] * math.cos(th / 2))
        return tf, o, pb, (o[0] + rm[0] * CREASE_R, o[1] + rm[1] * CREASE_R)
    lo, hi = 0.0, 4.0 * CREASE_R
    for _ in range(80):                                                                        # d such that the arc's end lies ON the border line
        mid = (lo + hi) / 2.0
        tf, o, pb, pm = end(mid)
        lo, hi = (mid, hi) if pb[1] > lvl(pb[0]) else (lo, mid)
    tf, o, pb, pm = end((lo + hi) / 2.0)
    assert abs(pb[1] - lvl(pb[0])) < 1e-6 and tf[0] < yc < pb[0], (tf, pb, yc)
    return yc, zc, o, (tf[0], pb[0]), (a, s), pm


def well_top(p, y):
    """THE KEYWELL's top (local frame) at plan y -- the lowest of its ceilings: KEYWELL_MAX, and the smooth profile of the border's level (the rest
    plane + BORDER_UP) behind / the caps' top plane less KEY_PROUD over the tilted plate in front, the CHORD of the CREASE_R arc between (well_profile, build 148;
    a normal distance in hull.deck_edge's under_t, vertical here: / cos KEY_TILT)."""
    yc, zc, (oy, oz), (yf, yb), _, _ = well_profile(p)
    if y < yf:
        z = plate_z(p, y) + (CAP_TOP_T - KEY_PROUD) / math.cos(math.radians(KEY_TILT))
    elif y > yb:
        z = rest_z(p, y) + BORDER_UP
    else:                                                                                      # THE CHORD between the arc's ends: a plane (a cylinder
        zf = plate_z(p, yf) + (CAP_TOP_T - KEY_PROUD) / math.cos(math.radians(KEY_TILT))        # face of R >= 150 on the fill's top made every fuse of
        zb = rest_z(p, yb) + BORDER_UP                                                          # the fill into the hull hand back the fill alone --
        z = zf + (zb - zf) * (y - yf) / (yb - yf)                                               # fillcheck148c-f; three planes fuse)
    return min(z, KEYWELL_MAX)


def swell_z(p, x, y):
    """THE SWELL's top (build 132) at plan (x, y): the lower of plane A (LIP_UP over the tilted plate) and B, THE ROLL (build 139): the cylinder tangent to the rim at x_b (plan.swell_R), rising to plane A at the crease crossing."""
    return min(plate_z(p, y) + LIP_UP, roll_z(p, x))


def roll_z(p, x):
    """THE ROLL's top (build 139) at plan x: the cylinder of plan.swell_R tangent to the rim at x_b, rising east of it (the rim west of it)."""
    R = p["swell_R"]
    return RIM_Z + (R - math.sqrt(max(0.0, R * R - (x - p["x_b"]) ** 2)) if x > p["x_b"] else 0.0)


def lip_cross_y(p):
    """The lip's crossover (build 132): the plan y where plane A (LIP_UP over the tilted plate) rises through the border's level (the rest plane + BORDER_UP) --
    south of it the border's level is the lip's top, north of it the swell's."""
    return (rest_z(p, 0.0) + BORDER_UP - plate_z(p, 0.0) - LIP_UP) / ((plate_z(p, 1.0) - plate_z(p, 0.0)) - (rest_z(p, 1.0) - rest_z(p, 0.0)))


def lip_z(p, x, y):
    """The lip's top at plan (x, y): the border's level (the rest plane + BORDER_UP, capped at the rim) or the swell, whichever is higher (SWELL off, build 139: the level alone)."""
    lvl = min(rest_z(p, y) + BORDER_UP, RIM_Z)
    return max(lvl, swell_z(p, x, y)) if SWELL else lvl


def tilt_shadow(poly, yp, z=0.0):
    """Where a tilted body's section at height z (over the switch plate) lands in the flat frame: the tilt about the pivot line (yp, RIM_Z) scales y by
    cos KEY_TILT about yp and moves everything under the pivot BACK by (RIM_Z - z) sin KEY_TILT (1.7 at the plate). The flat palm deck meets the
    tilted walls, not the flat ones."""
    a = math.radians(KEY_TILT)
    return affinity.translate(affinity.scale(poly, xfact=1.0, yfact=math.cos(a), origin=(0.0, yp)), 0.0, (RIM_Z - z) * math.sin(a))




def tp_plan(grow=0.0):
    """The TPS43's outline in the u frame (R), grown by grow."""
    return sbox(TP_U - TP_W / 2 + TP_R, TP_Y - TP_D / 2 + TP_R, TP_U + TP_W / 2 - TP_R, TP_Y + TP_D / 2 - TP_R).buffer(TP_R + grow, quad_segs=8)


def layered_prism(poly, z0, z1, r_top=0.0, r_bot=0.0, n=None):
    """A prism whose top / bottom outline edges are ROUNDED by r_top / r_bot: n inset layers each, a quarter circle in section
    (n = r / 0.25 by default: steps at the printer's layer height; 4 layers on r2 read as TERRACES, Willy 12:39). OCC's fillet fails on the
    finished hull (fillet_probe: 68/95 edges), so rounds are built from 2D offsets instead, and applied as outline BANDS late (hull.key_deck)."""
    n = n or max(4, int(round(max(r_top, r_bot) / 0.25)))
    core = prism(poly, z0 + r_bot, z1 - r_top)
    for r, top in ((r_top, True), (r_bot, False)):
        if r <= 0:
            continue
        for k in range(n):
            a0, a1 = (k / n) * math.pi / 2, ((k + 1) / n) * math.pi / 2       # up the quarter circle: 0 = the core's edge, pi/2 = the face
            inset, zlo, zhi = r * (1 - math.cos(a1)), r * math.sin(a0), r * math.sin(a1)
            core += prism(poly.buffer(-inset, join_style=1, quad_segs=4), (z1 - r_top + zlo) if top else (z0 + r_bot - zhi), (z1 - r_top + zhi) if top else (z0 + r_bot - zlo))
    return core


def rounded_prism(poly, z0, z1, r_top=0.0, r_bot=0.0):
    """A prism of the polygon with its top / bottom outline edges ROUNDED by OCC's fillet: ONE face per edge (layered_prism's inset layers read as
    terraces: 4 on r 2, Willy 12:39; 6-12 lines along every edge, 08:36 'lots to clean up'). The whole loop is filleted: the plan's and the base
    slab's loops take r 2 / 1.5 now the hinge face is one line (build 107, scratchpad fillet_all / fillet_bot / arc_keep); the plateau's loop
    never will (0.05-0.2 chords, 14 holes: plateau_clean) -- its inner edges stay layered, the back run's r 6 too (a 6 ball over 0.5 chords). Loud."""
    sol = prism(poly, z0, z1).solids()[0]
    for r, top in ((r_top, True), (r_bot, False)):
        if r <= 0:
            continue
        face = max(sol.faces(), key=lambda f: f.normal_at().Z) if top else min(sol.faces(), key=lambda f: f.normal_at().Z)
        try:
            sol = sol.fillet(r, face.edges())
        except ValueError as ex:
            raise ValueError(f"rounded_prism: OCC refuses r {r} on the {'top' if top else 'bottom'} loop ({len(face.edges())} edges): {ex}") from None
    assert sol.is_valid, "rounded_prism: invalid after the fillet"
    return sol


def band(poly, z0, z1, r):
    """The outline band of poly between z0 and z1: the outer r of it (what a round replaces)."""
    return prism(poly.buffer(1.0, join_style=1), z0, z1) - prism(poly.buffer(-r, join_style=1, quad_segs=4), z0 - 1, z1 + 1)


def hinge_straight(plan, x_face=None, tol=0.2):
    """The plan with its hinge face (the outline's min-x run) one straight line ON the face (x = its min x): the run's end vertices moved onto the
    line, the ones between dropped. The flow's buffers leave the face a 0.08 zigzag (6-7 vertices on L), and OCC's fillet refuses the whole outline's
    top edge chain with it in the solid (scratchpad fillet_try5-7, build 105) -- straightened, the chain fillets. The result COVERS the plan (the
    bumps are inside the line, the moved ends swing their edges outward): a keep built on it clips to prism(plan) with no step."""
    pts = list(plan.exterior.coords)[:-1]
    x0 = min(x for x, _ in pts) if x_face is None else x_face
    on = [x < x0 + tol for x, _ in pts]
    n = len(pts)
    kept = [((x0, y) if on[i] else (x, y)) for i, (x, y) in enumerate(pts) if not (on[i] and on[i - 1] and on[(i + 1) % n])]
    out = Polygon(kept)
    assert out.is_valid and out.buffer(1e-6).covers(plan) and out.area - plan.area < 20.0, f"hinge_straight: {len(pts) - len(kept)} vertices dropped, area +{out.area - plan.area:.2f}, covers {out.buffer(1e-6).covers(plan)}"
    return out





def shape_tolerance(s):
    """The shape's largest vertex / edge tolerance (ShapeAnalysis): 0.2 is the ceiling -- a 1.2 vertex tolerance silently killed a later cut (build 109)."""
    from OCP.ShapeAnalysis import ShapeAnalysis_ShapeTolerance
    from OCP.TopAbs import TopAbs_EDGE, TopAbs_VERTEX
    sa = ShapeAnalysis_ShapeTolerance()
    return max(sa.Tolerance(s.wrapped, 1, TopAbs_VERTEX), sa.Tolerance(s.wrapped, 1, TopAbs_EDGE))


def tessellate(s, tol=0.02, ang=0.1):
    """The shape's triangles, every face present (build 148). build123d's tessellate / export_stl mesh the whole shape in OCC's RELATIVE mode
    (deflection x the edge's size) and BRepMesh then leaves the key wall's fillet patches at the fans bare -- 0.03-0.06 trimming edges where two
    chord fillets meet over the sloped ceiling -- (a crash in deghost's mesh_volume, a 2.6 mm2 hole in the STL; velvet/STL's hull of Oct 14
    already had them); an ABSOLUTE pass meshes those but leaves a 0.1 mm2 plane sliver at the shoulder's ogee end bare instead. So: the relative
    pass first at 0.02 (the same mesh as before everywhere it works), then each bare face alone, finer first (absolute 0.02, 0.005, relative
    0.01: the 0.01-thick crescent between the shoulder's arc and its chords wants a deflection under its thickness) then coarser (0.05, 0.1, 0.2:
    the fan patches); one still bare raises with its place."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopAbs import TopAbs_Orientation
    from OCP.TopLoc import TopLoc_Location
    from build123d import Vector
    BRepMesh_IncrementalMesh(s.wrapped, tol, True, ang, True)
    verts, tris, off, fell = [], [], 0, []
    for f in s.faces():
        loc = TopLoc_Location()
        poly = BRep_Tool.Triangulation_s(f.wrapped, loc)
        if poly is None:                                                                   # alone: FINER first (a 0.01-thick crescent between the shoulder's
            for t_, rel, a_ in ((0.02, False, 0.1), (0.005, False, 0.1), (0.01, True, 0.1), (0.05, False, 0.1), (0.1, False, 0.2), (0.2, False, 0.3)):   # arc and its chords needs
                BRepMesh_IncrementalMesh(f.wrapped, t_, rel, a_, False)                      # absolute 0.02 or finer; the fan fillet patches take 0.02-0.05)
                loc = TopLoc_Location()
                poly = BRep_Tool.Triangulation_s(f.wrapped, loc)
                if poly is not None:
                    fell.append((f.area, t_))
                    break
        c = f.center()
        assert poly is not None, f"a face has no triangulation, relative {tol} / {ang} nor alone at absolute 0.005-0.2: {f.geom_type} {f.area:.4f} mm2 at ({c.X:.2f}, {c.Y:.2f}, {c.Z:.2f})"
        trsf = loc.Transformation(); rev = f.wrapped.Orientation() == TopAbs_Orientation.TopAbs_REVERSED
        verts += [Vector(v.X(), v.Y(), v.Z()) for v in (poly.Node(i).Transformed(trsf) for i in range(1, poly.NbNodes() + 1))]
        tris += [((t.Value(1) + off - 1, t.Value(3) + off - 1, t.Value(2) + off - 1) if rev else (t.Value(1) + off - 1, t.Value(2) + off - 1, t.Value(3) + off - 1)) for t in poly.Triangles()]
        off += poly.NbNodes()
    if fell:
        print(f"   MESH: {len(fell)} face(s) the relative pass left bare, meshed alone (areas {', '.join(f'{a:.2f}' for a, _ in fell[:6])}{', ...' if len(fell) > 6 else ''} mm2)", flush=True)
    return verts, tris


def write_stl(s, path, tol=0.02, ang=0.1):
    """The shape to a binary STL off tessellate's absolute mesh (every face present, or it raises)."""
    from OCP.StlAPI import StlAPI_Writer
    tessellate(s, tol, ang)
    w = StlAPI_Writer(); w.ASCIIMode = False
    assert w.Write(s.wrapped, str(path)), f"STL write failed: {path}"


def mesh_volume(s, tol=0.02):
    """The solid's volume off its own tessellation (a signed tetra sum): exact where OCC's GProp drifts -- 0.1 % (100 mm3) on velvet's hull with
    0.18 edge tolerances, 5 mm3 on a 550 mm3 band cut from it (build 122). For asserts on small swapped solids; tessellating the whole hull is slow."""
    verts, tris = tessellate(s, tol, 0.2)
    v = 0.0
    for a, b, c in tris:
        A, B, C = verts[a], verts[b], verts[c]
        v += (A.X * (B.Y * C.Z - B.Z * C.Y) - A.Y * (B.X * C.Z - B.Z * C.X) + A.Z * (B.X * C.Y - B.Y * C.X)) / 6.0
    return abs(v)
