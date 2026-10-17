"""One half's hull + hatch in its u frame: the shell, key deck, bays, pockets, rest, tilt, bosses, hinges."""
import copy
import copyreg
import hashlib
import inspect
import math
import pickle
import sys
import time
from pathlib import Path

import numpy as np

from shapely.geometry import LineString, Point, Polygon, box as sbox
from shapely import affinity
from shapely.ops import substring, unary_union
from build123d import Align, Axis, Box, Compound, Cone, Cylinder, Face, Polyline, Pos, Rot, Solid, ThreePointArc, Wire, chamfer, extrude, fillet, import_step, make_face
from OCP.gp import gp_Dir, gp_Pnt, gp_Vec
from OCP.Quantity import Quantity_Color, Quantity_ColorRGBA, Quantity_TOC_RGB

from velvet.params import *  # noqa: F401,F403
from velvet.geo import fuzzy_cut, fuzzy_fuse, _hit, band, bevel_cut, clipped, layered_prism, lip_cross_y, placed, prism, prism_holed, rect, rest_plane, rest_z, well_profile, well_top, rounded_prism, roll_z, plate_z, swell_z, tilt_shadow, tp_plan, shape_tolerance, mesh_volume
from velvet.parts import oval_cover, button_cap
from velvet.doors import door_bypass, ellipse, wprism
from velvet.lights import PINS_K, STRAP_L, STRAP_W, arm_slots, arms, clean_outline, collar_plan, collar_room, crown_part, crown_pins, cutters, diffuser_part, filaments, filament_part, foot_plan, guide_part, lead_hole, leads, ledge_plan, nets, overlay_part, ridge_plan, slot_plan, tp_window, vias
from velvet.backline import back_solid, chain_prism, rounded_prism_chain
from velvet.stand import back_at_axis, head_env, ring_relief




BIGB = 3000.0   # a half-space's box


def flat_body(Z, palm_side, plan):
    """The flat zone + the palm side as ONE polygon: their own vertices (the prisms' faces must lie exactly on the hull's walls) plus the
    hairline between them (closing the union 0.1 finds it; the closed polygon itself re-approximates the outline's arcs by up to 0.02) and
    the hairline's MOUTHS filled out to the wall: where the two meet the outline 0.8 apart the closing's arc dips 0.085 inside the wall, and
    hull & prism(keep) shaved the wall's r 2 round there into a 0.011 mm2 sliver the mesher drops (R's housing corner, builds 109-110:
    scratchpad keep_slice / mouths). The wall band 0.2 deep within 0.2 of a hairline piece: 2-3 pieces, 0.3-0.4 mm2 a side."""
    u = unary_union([Z, palm_side])
    closed = u.buffer(0.1, join_style=1).buffer(-0.1, join_style=1)
    gap = closed.difference(u)
    mouth = plan.difference(plan.buffer(-0.2, join_style=2)).intersection(gap.buffer(0.2, join_style=2))
    return unary_union([Z, palm_side, gap, mouth])


CACHE = Path(__file__).resolve().parent.parent / ".cache"   # Half.build's per-step state, ON when the dir exists (mkdir .cache; gitignored)
FRESH = False                                                   # tests and golden set True: always from scratch


def _rgba(r, g, b, a):
    return Quantity_ColorRGBA(Quantity_Color(r, g, b, Quantity_TOC_RGB), a)


def _rgb(r, g, b):
    return Quantity_Color(r, g, b, Quantity_TOC_RGB)


# imported STEPs carry OCP colours, which do not pickle (build 94: 'cannot pickle Quantity_ColorRGBA' at inner_bay's Pi stack): reduce them to floats
copyreg.pickle(Quantity_ColorRGBA, lambda c: (_rgba, (c.GetRGB().Red(), c.GetRGB().Green(), c.GetRGB().Blue(), c.Alpha())))
copyreg.pickle(Quantity_Color, lambda c: (_rgb, (c.Red(), c.Green(), c.Blue())))
for _t in (gp_Vec, gp_Pnt, gp_Dir):                            # build123d Vectors (inner_bay's pi_posts / hat_posts) wrap gp_Vec: the same
    copyreg.pickle(_t, lambda v, _t=_t: (_t, (v.X(), v.Y(), v.Z())))


def deghost(hull, side, tag):
    """A boolean can leave zero-volume CLOSED SHELLS inside the solid -- ghosts: the solid stays valid and one piece, until a later boolean splits them off
    as solids of their own or returns None (build 132: three at the P corner's outer wall, (174.5, 7.4-8.4, z -9.5..7.8), came out of the console's step round
    as 4 solids; build 131's final hull carried the same ones). The solid rebuilt from its outer shell alone (the one with the faces); the volume must hold."""
    shells = hull.shells()
    if len(shells) <= 1:
        return hull
    main = max(shells, key=lambda s: len(s.faces()))
    out = Solid(main)
    ghost = sum(s.volume for s in shells if s is not main)
    drift = out.volume - hull.volume - ghost                                               # the ghosts are VOIDS: the outer shell alone is the hull + their volume. GProp on
    #   the SAME faces (build 148: two mesh passes of the same faces differed by 34 mm3 once a bare fan patch was meshed alone in one pass and not
    #   the other -- the mesh drift was a mesher artefact, not geometry; 139's +0.084 for 0.084 of ghosts holds by GProp within 5)
    assert out.is_valid and abs(drift) < 5.0, f"{side} {tag}: dropping {len(shells) - 1} ghost shell(s) ({ghost:.3f} mm3) left the hull {'invalid' if not out.is_valid else f'{drift:+.2f} mm3 off beyond them'}"
    print(f"   {side} {tag}: {len(shells) - 1} ghost shell(s) dropped ({ghost:.3f} mm3; {[(round(s.bounding_box().center().X, 1), round(s.bounding_box().center().Y, 1)) for s in shells if s is not main]})", flush=True)
    return out


def swap_band(hull, band, pieces, side, tag):
    """The hull with the band's content replaced by the pieces, CHECKED: the band's content after the swap must be the pieces' (mesh volumes, 2 mm3)
    -- a common ON the round's tangent line came back EMPTY (builds 108-123: L's bottom edge a 1.5 x 2 rebate, see hinges) and a fuse order flips
    on a pickled operand (memory occ-fuse-order); retried with the piece first, then fuzzy 0.01. Build 145's hunt: L's hull volume 610 mm3 apart
    between builds of the same plan after the bottom round's fuse -- the B-reps identical, GProp's drift; the check stays, the scare was not real."""
    def vol(s):                                                      # the mesh's volume (exact) -- GProp's where a face of the piece will not mesh
        if s is None or not s.solids():                              # (the plateau's keep_out & fil: tessellate hands back a None triangulation)
            return 0.0
        try:
            return mesh_volume(s)
        except AttributeError:
            return s.volume
    want = sum(vol(q) for q in pieces)
    cut = hull - band
    left = vol(cut & band)
    assert left < 2.0, f"{side} {tag}: the band cut left {left:.0f} mm3 in the band (the pieces {want:.0f})"
    for attempt in range(3):
        out = cut
        for piece in pieces:
            out = out + piece if attempt == 0 else piece + out if attempt == 1 else fuzzy_fuse(out, piece)
        got = vol(out & band)                                            # the band's content (mesh volumes). NOT the hull's GProp growth: between two builds of
        if abs(got - want) < 2.0 and len(out.solids()) == len(hull.solids()):   # the same plan L's hull read 610 mm3 apart after this fuse with the B-reps IDENTICAL
            if attempt:                                                           # (A - B and B - A empty, build 145): GProp drifts 0.5 % on the hull. Not is_valid:
                print(f"   {side} {tag}: the swap fused on attempt {attempt} ({'the piece first' if attempt == 1 else 'fuzzy'})", flush=True)   # the plateau's swap
            return out                                                            # in key_deck has always come out invalid here, healed downstream
        print(f"   {side} {tag}: the swap left {got:.0f} of the pieces' {want:.0f} mm3 in the band (attempt {attempt}: {len(out.solids())} solids), retrying", flush=True)
    raise AssertionError(f"{side} {tag}: the band swap holds {got:.0f} of the pieces' {want:.0f} mm3 after 3 attempts")


def fillet_law(sol, edges, law):
    """OCC's fillet on `edges`, each under the radius law(x, y) sampled every mm along it as (u, r) pairs (a constant where the law is flat), like
    backline.back_solid (build 143: the key wall's round fading as the deck rises to the console; 144: the knob badge's round growing into the seam
    wall's). ValueError when OCC refuses, like Shape.fillet."""
    from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet
    from OCP.TopExp import TopExp
    from OCP.BRep import BRep_Tool
    from OCP.TColgp import TColgp_Array1OfPnt2d
    from OCP.gp import gp_Pnt2d
    mk = BRepFilletAPI_MakeFillet(sol.wrapped)
    for e in edges:
        mk.Add(e.wrapped)
    for ic in range(1, mk.NbContours() + 1):
        for k in range(1, mk.NbEdges(ic) + 1):
            oe = mk.Edge(ic, k)
            a, b = (BRep_Tool.Pnt_s(v) for v in (TopExp.FirstVertex_s(oe), TopExp.LastVertex_s(oe)))
            n = max(2, round(math.dist((a.X(), a.Y()), (b.X(), b.Y())))) + 1
            rs = [law(a.X() + (b.X() - a.X()) * i / (n - 1), a.Y() + (b.Y() - a.Y()) * i / (n - 1)) for i in range(n)]
            if max(rs) - min(rs) < 1e-6:
                mk.SetRadius(rs[0], ic, k)
            else:
                arr = TColgp_Array1OfPnt2d(1, n)
                for i, rv in enumerate(rs):
                    arr.SetValue(i + 1, gp_Pnt2d(i / (n - 1), rv))
                mk.SetRadius(arr, ic, k)
    mk.Build()
    if not mk.IsDone():
        raise ValueError("Failed creating a fillet (law)")
    out = Solid(mk.Shape())
    if not out.is_valid:
        raise ValueError("the law fillet left an invalid solid")
    return out


def led_tray(side):
    """THE LED TRAY's plan (build 134): the LEDs LED_AT[side] in a row along y in a sunk parallelogram, LED_TRAY = (the width across, the ends' skew along y,
    the margin past the end LEDs). Returns (the polygon, w, skew, y0, y1)."""
    xs, ys = [x for x, _ in LED_AT[side]], [y for _, y in LED_AT[side]]
    w, skew, m = LED_TRAY
    cx, y0, y1 = sum(xs) / len(xs), min(ys) - m, max(ys) + m
    return Polygon([(cx - w / 2, y0 - skew / 2), (cx + w / 2, y0 + skew / 2), (cx + w / 2, y1 + skew / 2), (cx - w / 2, y1 - skew / 2)]), w, skew, y0, y1



def stack_in_F():
    """THE STACK AS BUILT (161): DP.stack_in_F's Pi and 2x20 header placed with velvet's Z_PI / Z_PI_TOP (PI_GAP under the HAT, Willy 20:50
    '6mm of space between them'), the header's pins trimmed at the HAT's underside (they are soldered into the PiTFT: no socket). Returns (pi, header, None)."""
    pi = Pos(0, DP.Y_PI0 + DP.PI_D / 2, Z_PI + DP.PI_PCB / 2) * Rot(0, 0, 180) * Rot(90, 0, 0) * import_step(str(DP.PI_STEP))
    hdr = Pos(-24.13, DP.Y_ROW1, Z_PI_TOP) * Rot(0, 0, 90) * import_step(str(DP.HEADER_STEP))
    hdr = hdr & Pos(0, 0, Z_PI - 1.0) * Box(100, 100, DP.Z_HAT - 0.01 - (Z_PI - 1.0), align=BIG)   # tails trimmed 1.0 under the Pi, pins at the HAT's underside
    return Compound(children=list(pi.solids())), hdr, None


class Half:
    """One half's hull + hatch in its u frame, built step by step (STEPS, in order). Each step reads and writes the shared state on self;
    the 2D plan p is read-only. build() returns the dict assemble() places from."""

    STEPS = ("shell", "key_deck", "rest_skin", "cell_bore", "pico_bay", "charger_bay", "stand_keepouts", "inner_bay", "wrist_rest", "pick_bosses", "tilt", "deck_edge", "wall_round", "lip_round", "raise_bosses", "hinges", "plug_tunnel", "step_chamfer", "hatch_plate", "badge", "fittings")

    def __init__(self, side, p):
        self.side, self.p = side, p
        self.plan, self.wells, self.fu, self.xin = p["plan"], p["wells"], p["fu"], p["xin"]
        self.Kf = Pos(self.xin, 0, 0) * Rot(0, 0, YAW)
        self.ax_x, self.z_ax = p["ax_x"], ZB + AX_UP
        self.fy = FLAP_Y[side]
        self.inner, self.hatch_plan, self.plateau, self.ears, self.ear2, self.Z = p["inner"], p["hatch_plan"], p["plateau"], p["ears"], p["ear2"], p["flat"]
        self.out, self.pk = {}, None
        self.yp = min(f.position.Y for f in self.fu.values()) - 13.0 - 6.0                       # the tilt line: 6 in front of the front-most cap
        self.t0 = time.perf_counter()

    def lap(self, what):
        print(f"   {self.side} {what}: {time.perf_counter() - self.t0:.0f} s", flush=True)

    def build(self):
        """Every step in order. With the .cache dir present (mkdir it; tests and golden set FRESH) the state after each step is pickled there under a
        key -- a hash of the upstream modules' sources and of hull.py with the LATER steps' sources blanked -- so an edit to step n invalidates n..
        only and the next build resumes after the last step whose key still matches (a look change at the end: ~2 min, not 10). Willy 20:24 'how
        come every little change is taking so long'."""
        start, use = 0, CACHE.is_dir() and not FRESH
        if use:
            for i in range(len(self.STEPS), 0, -1):
                f = CACHE / f"{self.side}_{i:02d}_{self.STEPS[i - 1]}.pkl"
                if f.exists():
                    try:
                        key, state = pickle.load(open(f, "rb"))
                    except (EOFError, pickle.UnpicklingError):                    # a dump that died mid-write (a crashed build)
                        print(f"   {self.side} CACHE: {f.name} is truncated, ignored", flush=True)
                        continue
                    if key == self._key(i):
                        self.__dict__.update(state)
                        self.t0, start = time.perf_counter(), i
                        print(f"   {self.side} CACHE: resumed after {self.STEPS[i - 1]} (step {i} of {len(self.STEPS)})", flush=True)
                        break
        for i in range(start, len(self.STEPS)):
            getattr(self, self.STEPS[i])()
            if use:
                state = {k: v for k, v in self.__dict__.items() if k != "p"}            # the plan holds lambdas; plan_of rebuilds it and its source is in the key
                pickle.dump((self._key(i + 1), state), open(CACHE / f"{self.side}_{i + 1:02d}_{self.STEPS[i]}.pkl", "wb"))
        return self.finish()

    def _key(self, n):
        """What the first n steps depend on: the upstream modules' sources, hull.py with steps n.. blanked, the side."""
        from velvet import backline, caps, frames, geo, params, parts, plan, stand
        src = inspect.getsource(sys.modules[__name__])
        for name in self.STEPS[n:]:
            src = src.replace(inspect.getsource(getattr(type(self), name)), "")
        h = hashlib.sha1(src.encode())
        for m in (params, geo, frames, plan, parts, stand, caps, backline):
            h.update(inspect.getsource(m).encode())
        h.update(self.side.encode())
        return h.hexdigest()

    def shell(self):
        """The outline prism with its top bevel, the hatch bed, the one wiring cavity under the deck."""
        hatch_plan, inner, lap, p, plan = self.hatch_plan, self.inner, self.lap, self.p, self.plan
        hull = prism(plan, ZB, RIM_Z)
        inner = p["inner"]
        hatch_plan = p["hatch_plan"]
        cavity = inner
        hull -= prism(cavity, FLOOR - 0.1, -TOP)                     # the cavity under the deck: one wiring space (over the rail too: the pinky switches' pins)
        hull -= prism(hatch_plan, ZB - 1, FLOOR)                     # the hatch bed (0.1 over the cavity's floor: a tool whose top lies ON the floor face is skipped by OCC --
        #                                                              build 76 left the skirt's slab in the bed, 14 cm3 under the hatch)
        lap("cavity")
        self.cavity = cavity
        self.hatch_plan = hatch_plan
        self.hull = hull
        self.inner = inner

    def key_deck(self):
        """Key wells through the top; the plateau drop (no key lip: outside the parts' plateau the top comes down to the switch plate); switch cut-outs + clip-lip pockets."""
        fu, hull, lap, p, plan, plateau, wells = self.fu, self.hull, self.lap, self.p, self.plan, self.plateau, self.wells
        hull -= prism(wells, 0.0, RIM_Z + 1)                         # the wells   # ponytail: no chamfer round them -- a tapered cut of this outline corrupts the boolean
        lap("wells + switch cuts")
        # NO KEY LIP (Willy 21:44): outside the PLATEAU (every non-key part + its wall, flowed) the top comes down to the switch plate,
        # the outer walls with it, so the caps stand proud; the plateau keeps the rim for the display, trackpad, knob, cell, Pico, charger
        plateau = p["plateau"]
        hull -= prism(plan.buffer(5.0), 0.0, RIM_Z + 1) - prism(plateau, -1.0, RIM_Z + 2)
        for f in fu.values():                                       # switch cut-outs + clip-lip pockets
            hull -= f * Pos(0, 0, -TOP - 1) * Box(HOLE, HOLE, TOP + 2, align=BIG)
            hull -= f * Pos(0, 0, -TOP - 0.01) * Box(POCKET_W, POCKET_W, TOP - POCKET + 0.01, align=BIG)
        # ROUNDED TOP EDGES: the deck's outline edge (outside the plateau) and the plateau's edge (the outline + the step down to the deck),
        # each band swapped for inset layers; the bands are wall (every well and pocket keeps >= WALL from these edges)
        r = EDGE_R_TOP
        deck_band = band(plan, -r - 0.5, 0.5, r) - prism(plateau, -r - 1.0, 1.0)
        deck_band -= prism(self.outer_zone(r + 1.5), -r - 2.0, 2.0)   # not along the tilted zone's OUTER WALL: that edge is rounded AFTER the tilt (wall_round)
        piece, plain = rounded_prism(plan, -r - 1.0, 0.0, r_top=r) & deck_band, (prism(plan, -r - 1.0, 0.0) & deck_band).volume   # plain: the SQUARE edge's content of the band (the band's 0.5 of air above the top left out)
        assert plain < 5.0 or (piece is not None and 0.75 * plain < piece.volume < 0.98 * plain), f"{self.side}: the deck's round swap holds {0 if piece is None else piece.volume:.0f} of the band's {plain:.0f} mm3"
        if plain >= 5.0:                                                # (the band is 1 mm3 now: the plateau and the tilted zone's outer wall take the whole outline, build 123)
            hull = swap_band(hull, deck_band, [piece], self.side, "DECK ROUND")   # ONE fillet face per edge (build 108; the layers read as terraces). The band's inner wall sits ON the
        #                                                                tangent line: a grazing common came back EMPTY on the base plates, no error (stand.build_base, build 123)
        plat_band = band(plateau, RIM_Z - r - 0.5, RIM_Z + 0.5, r) - prism(p["ring_up"].buffer(r + 1.0, join_style=2).difference(plan.exterior.buffer(r + 0.5)), RIM_Z - r - 1.0, RIM_Z + 1.0)   # not where the seam wall goes UP (the round carved a 2 mm groove at its foot, build 64)
        #                                                                       -- except within r + 0.5 of the OUTLINE: round the outer-back corner's arc the plateau is only 2.2-3.3 wide between the
        #                                                                       outline and the ring_up, and the exclusion took the outline's band with the ring's: 11 mm of square edge on both halves,
        #                                                                       builds 121-129 (Willy 07:50). keep_out is clipped to the plateau, whose edge IS the wall at this step (the pit is cut above)
        plat_band -= prism(p["step_low"].buffer(STEP_C + 1.0, join_style=2), RIM_Z - r - 1.0, RIM_Z + 1.0)                                                           # nor where it is chamfered (below)
        rim = prism_holed(plan.exterior.buffer(r + 1.0), RIM_Z - r - 2.0, RIM_Z + 1.0)            # the plateau's OUTLINE edge: OCC's fillet on the plan's prism, clipped to the
        keep_out = rounded_prism_chain(plan, p["back_chain"], RIM_Z - r - 1.0, RIM_Z, r) & prism(plateau, RIM_Z - r - 2.0, RIM_Z + 1.0)   # the chain's TRUE ARCS along the back (on its 1.1 chords the swap left a 1.2 mm vertex tolerance, 118c)   # plateau (nothing added past its edge); its INNER
        fil = plat_band & rim                                                                  # edges (the v713 block's wall at the alphas; the tilt line, cut away later)
        if p["tp_zone"] is not None:                                                           # stay layered: OCC never fillets the plateau's loop. R's console stretch too:
            fil = fil - prism(p["tp_zone"].buffer(0.55, join_style=2), RIM_Z - r - 3.0, RIM_Z + 2.0)   # inner_bay rounds it on the pitched top (edge_round), 0.05 over this edge
        rest = plat_band - fil
        pieces = [keep_out & fil] + ([layered_prism(plateau, RIM_Z - r - 1.0, RIM_Z, r_top=r) & rest] if rest is not None and rest.solids() else [])
        hull = swap_band(hull, plat_band, [q for q in pieces if q is not None and q.solids()], self.side, "PLATEAU ROUND")
        # (The BACK ROUND band swap of builds 116-120 -- band2d ∩ back_zone swapped for back_solid's keep -- is gone: dead behind `if BACK_R > r` since
        #  build 121, and enabled in 128-129 it laid a second r 1.8 round over this one's: coincident blend faces, the hull invalid from here on, the
        #  cell bore skipped twice. The outline's round along the back run and round its corners is THIS swap's; the corner arc is in it since 130.)
        lap("rounded top edges")
        self.hull = hull
        self.plateau = plateau

    def plug_tunnel(self):
        """THE PICO'S PLUG TUNNEL + COVER (Willy 21:06 'the same treatment you gave to the pi hole ... make sure reliable tunnel for all sized usb
        works + cover for bay'): ONE PLUG_W x PLUG_H bore along the bay's x from 1 outside the outer face to 0.5 off the connector's face -- through
        the wall (the window), the rail's top (RAIL_H) and the cavity's ceiling under the pad -- cut AFTER deck_edge (whose fill re-solidified the
        old window: a bar -7.4..-2.5 across it in build 93, scratchpad tunnel_bisect.py) and hinges (the rail). The Pico hangs PICO_DROP lower, so
        the bore leaves 2.1 of skin under the pad's recess floor. The COVER is the Pi window's (parts.usb_cover) at PLUG_W x PLUG_H along the bore,
        its outer face flush with the case's face (clipped by the outline: the wall is yawed 5 deg to the bay)."""
        hull, out, p = self.hull, self.out, self.p
        Lf, wall_x, usb_z, face_x = out["bay_Lf"], out["wall_x"], out["usb_z"], out["pico_face"]
        x0, x1 = face_x - 1.0, wall_x + WALL + 1.0                                 # from 1 INSIDE the connector's face (build 96: 0.35 of ceiling stood between the bay's end and the
        #                                                                            bore; 97: the USB-end ceiling pads overhang the board's end 2 and sat in a path started 1.1 early)
        bore = Lf * Pos((x0 + x1) / 2, 0, usb_z) * Box(x1 - x0, PLUG_W, PLUG_H, align=MID)
        v0, n0 = hull.volume, len(hull.solids())
        r = hull - bore
        if v0 - r.volume < 0.8 * PLUG_W * PLUG_H * WALL:                           # the plain cut skipped the wall (build 120, R: 141 of 582 mm3) -- retry fuzzy, like cell_bore
            print(f"   {self.side} PLUG TUNNEL: the plain cut bored only {v0 - r.volume:.0f} mm3, retrying fuzzy", flush=True)
            r = fuzzy_cut(hull, bore)
        hull = r
        assert v0 - hull.volume > 0.8 * PLUG_W * PLUG_H * WALL, f"{self.side}: the plug tunnel bored only {v0 - hull.volume:.0f} mm3 (the window alone is {PLUG_W * PLUG_H * WALL:.0f})"
        assert len(hull.solids()) == n0, f"{self.side}: the plug tunnel left {len(hull.solids())} solids (was {n0})"
        Ki = self.Kf.inverse()                                                      # THE DOOR (build 155) in the wall's own frame (K: the outer wall is x = const there)
        qk = (Ki * Pos(*out["pico_plug"])).position                                # the window's centre on the wall's outer face
        ext_k = LineString([((Ki * Pos(x, y, 0)).position.X, (Ki * Pos(x, y, 0)).position.Y) for x, y in p["plan"].exterior.coords])   # the outline in K
        xs = [max(g.x for g in getattr(c, "geoms", [c])) for yk in (qk.Y + PICO_POCKET[0] - 1.0, qk.Y, qk.Y + PICO_POCKET[1] + 1.0)
              if not (c := ext_k.intersection(LineString([(-1000.0, yk), (1000.0, yk)]))).is_empty]   # the outer wall's face x in K along the pocket's run
        assert len(xs) == 3 and max(xs) - min(xs) < 0.05 and abs(qk.X - xs[1]) < 0.1, f"{self.side}: the outer wall is not one plane along the plug door's run in K: {[round(x, 2) for x in xs]}, window at {qk.X:.2f}"
        Wf = self.Kf * Pos(xs[1], qk.Y, qk.Z) * Rot(0, 0, 90) * Rot(0, -REST_SLOPE, 0)   # W: x along the wall (+ = toward the back) ROLLED with the bay (its bore's 13 x 9 section
        #                                                                            rises REST_SLOPE toward the back with the palm: a level door left its corners showing, build 155), y INWARD; origin the hole's centre on the face
        hull, doors, _ = door_bypass(hull, Wf, bore, self.side)
        out.update(pico_doors=doors, pico_bore=(x0, x1), pico_Wf=Wf)
        print(f"   {self.side} PLUG TUNNEL {PLUG_W} x {PLUG_H}: {v0 - hull.volume:.0f} mm3 bored, {x1 - x0:.1f} long from the face to the connector")
        self.hull = hull

    def step_chamfer(self):
        """THE CONSOLE'S STEP DOWN to the keys (Willy 13:22 'smooth coming from front also'; 15:23 'simplicity and fluidity'): ONE EDGE_R_TOP round
        along the flat zone's edge where it steps down -- to the keys (plan.ring_low) and to the palm deck (plan.step_low) -- like every other top edge
        (build 122; builds 90-121 cut a 45 deg STEP_C chamfer there, whose end at the console's front corner stood as a post: Willy 23:36 'the connection
        of the wrist rest to the trackpad plate'; builds 85-89 a layered 4 mm round: terraces). The BAND SWAP of edge_round / build_base: the outline
        band (r + 0.5 in, 1 out, r + 1 under the top) swapped for a KEEP -- the zone's prism, its outline simplified 0.1-0.3 (the finest OCC fillets:
        the flow's sub-0.3 fragments along the straight sides, 0.06 off the wall at 0.1; the 3 x 6 piece behind the display's hinge takes 0.3) with
        OCC's fillet on its top edges along the step. A standoff keep (0.3 out, 0.2 up: the chamfer's) subtracted as a tool rounds NOTHING -- an r 1.8
        fillet on it bit 6 mm3 of 70 (build 122a): a round needs the keep's faces ON the hull's. R's console pieces in front of the hinge are rounded in
        the console's frame tp_T (the top at RIM_Z there); the seam ring's raised part keeps its own edge. THEN THE TRAY (R): cut after the round, its
        mouth (2.3 in from the console's edge) would sit on the band's inner wall."""
        c, r, p, hull = STEP_C, EDGE_R_TOP, self.p, self.hull
        low = p["step_low"]                                            # the seam ring's step down + the console's front toward the palm deck
        jobs = []                                                      # (piece, frame): split at the display's hinge line, the front part in the console's frame
        for piece in getattr(low, "geoms", [low]):
            if p["tp_zone"] is not None and p["tp_zone"].buffer(0.5).intersects(piece):
                xe, y_block, y_fade0, y_fade1, strip = self.out["strip"]                       # THE KEY WALL'S RUN (build 143): the console's step piece runs on along the key
                piece = unary_union([piece, p["ring"].intersection(sbox(xe - 0.5, Y_P0 - 1.0, xe + 3.5, y_block))]).buffer(0.05, join_style=2)   # wall to the block's NW corner,
                piece = max(getattr(piece, "geoms", [piece]), key=lambda g: g.area)              # ONE piece in the console's frame (builds 117-142: split at Y_P0, the north part at
                jobs.append((piece, self.out["tp_T"]))                                           # the rim, its round ending 3 short of ring_up as a D-shaped end face beside the brim)
            else:
                jobs.append((piece, Pos(0, 0, 0)))
        for piece, Tf in jobs:
            b = piece.bounds
            kw = p["tp_zone"] is not None and Tf.position.Z > 0.5 and "strip" in self.out      # the key wall's run: its east edge split at y_fade1, rounded under the fade law
            if kw:
                xe, y_block, y_fade0, y_fade1, strip = self.out["strip"]
            loc = p["flat"].intersection(sbox(b[0] - c - 6.0, b[1] - c - 6.0, b[2] + c + 6.0, b[3] + c + 6.0))   # the FLAT zone's edge (the palm side is plateau too: its step comes later)
            zone_, on_edge = piece.buffer(c + 1.0, join_style=2), self.plan.exterior.buffer(0.3)
            keep, outline, n, off = None, None, 0.0, 0.0
            for g in getattr(loc, "geoms", [loc]):
                if g.area < 1.0 or g.distance(piece) > 0.5:
                    continue
                for tol in (0.1, 0.2, 0.3):
                    gs = g.simplify(tol)
                    s0 = Tf * prism(gs, RIM_Z - r - 3.0, RIM_Z)
                    top = max(s0.faces(), key=lambda f: f.normal_at().Z)
                    seg = lambda e: LineString([(e.start_point().X, e.start_point().Y), (e.end_point().X, e.end_point().Y)])
                    es = [e for e in top.edges() if piece.distance(Point(e.center().X, e.center().Y)) < c + 1.0
                          or (seg(e).intersection(zone_).length > 0.3 and on_edge.contains(seg(e)) and (e.center().X > 1.0 or (self.side == "L" and min(e.start_point().Y, e.end_point().Y) < KNOB_Y + 2.0)))]   # + the OUTLINE's edges through the band's zone (not the
                    #                                                                            seam face -- except L's beside the web's 45 deg end, build 154: the seam edge's last 3 mm run into the step's round there, one chain
                    if kw:                                                                          # the key wall's run: the edges to y_fade1 only (the block's north wall and the back
                        es = [e for e in es if e.center().Y < y_fade1 + 0.1]                        # band lie beyond); its east edge Y_P0..y_block under the law: r 0.1 past y_fade1
                        #                                                                            (a split vertex at y_fade1 is collinear: geo.prism's closing drops it -- dry run 143)
                    #                                                                                                            seam face): one blended fillet round the console's front-west corner.
                    #                                                                                                            The swap had put the keep's SHARP knee edge back over the console round's
                    #                                                                                                            last 3.2 mm (the round's end face a notch at y -37.7, the two rounds' ends
                    #                                                                                                            cut by each other's wall, build 142; Willy 08:17 'bottom edge of trackpad
                    #                                                                                                            does not meet the corner of the curved side wall correctly')
                    try:
                        s_ = fillet_law(s0, es, lambda x, y: r if x < xe - 0.1 else max(0.1, min(r, RIM_Z + CONSOLE_UP - self.deck_z(y) - 0.2))) if kw else s0.fillet(r, es)
                        break
                    except ValueError:
                        s_ = None
                assert s_ is not None, f"{self.side}: OCC rounds no simplification of the console's outline at {[round(v) for v in b]}"
                dev = max(gs.exterior.distance(Point(q)) for q in g.exterior.coords if piece.distance(Point(q)) < c + 1.0)
                n += sum(e.length for e in es)
                off += s0.volume - s_.volume
                print(f"   {self.side} STEP ROUND r {r}: outline simplified {tol} ({dev:.2f} off the wall), {len(es)} edges" + (f", the key wall's (x {xe:.2f}) under the fade law (1.8 to y {y_fade0:.1f}, 0.1 at {y_fade1:.1f})" if kw else ""))
                if "seat" in self.out and Tf.position.Z > 0.5:          # the raised console: the brim's SEAT cut back to the rim in the keep too, as the hull has it --
                    s_ -= prism(self.out["seat"], RIM_Z, RIM_Z + 30.0)    # the swap had filled the seat's key-side corner to the console's top and the brim sat in the hull
                    #                                                      by 1.3 mm3 (build 122c); trimming the BAND at the seat instead sliced the keep's fillet blend at the
                    #                                                      (70, 2) vertex: the hull invalid, 2 solids, tolerance 1.36 (scratchpad, 122d)
                keep = s_ if keep is None else keep + s_
                outline = g if outline is None else outline.union(g)
            full = n * r * r * (1 - math.pi / 4)
            assert off > 0.8 * full, f"{self.side}: the keep's round took {off:.0f} mm3 of a full round's {full:.0f} at {[round(v) for v in b]}"
            if kw:                                                                                  # THE MOAT at the key block's west foot (the tilt's CUT_OFF between the tilted block
                y_rim = self.deck_y(RIM_Z)                                                          # and the plateau, x xe..xe + CUT_OFF) is capped at the rim where the deck rises past
                #                                                                                   it (y 20.4-31.2): a 0.25 slit 0.9-1.9 deep beside the brim (builds 69-142); filled to
                fill = prism(sbox(xe - 0.05, y_rim - 0.5, xe + CUT_OFF + 0.02, y_block + 0.1), RIM_Z - 0.5, RIM_Z + 12.0) & self.under_t(0.0)   # the deck: the block's face meets the strip
                void = mesh_volume(fill - hull)
                hull += fill
                left = mesh_volume(fill - hull)                                                     # (not the hull's GProp growth: 1.3 for a 3.1 void, the drift)
                assert hull.is_valid and len(hull.solids()) == 1 and 0.5 < void < 12.0 and left < 0.05, f"R: the moat fill: the void {void:.2f} mm3, {left:.2f} left, valid {hull.is_valid}, {len(hull.solids())} solid(s)"
                print(f"   R MOAT: {void:.1f} mm3 filled at the key block's west foot (x {xe:.2f}..{xe + CUT_OFF:.2f}, y {y_rim:.1f}..{y_block:.1f}), to the deck")
            zone = piece.buffer(c + 1.0, join_style=2).intersection(p["flat"].buffer(0.5, join_style=2))
            if kw:                                                                                  # along the key wall only: not the block's north wall nor the back band
                zone = zone.intersection(unary_union([sbox(-1000.0, -1000.0, 1000.0, Y_P0), sbox(-1000.0, Y_P0 - 0.01, xe + 1.0, y_block + 0.3)]))
            band = Tf * (prism(outline.buffer(1.0, join_style=1), RIM_Z - r - 1.0, RIM_Z + 5.0) - prism(outline.buffer(-(r + 0.5), join_style=1, quad_segs=6), RIM_Z - r - 2.0, RIM_Z + 6.0))
            band = band & prism(zone, ZB, RIM_Z + 30.0)
            if kw:                                                                                  # R's console: THE TRACKPAD POCKET + its collar room stay open -- the band's inner wall
                band -= prism(collar_room(), ZB, RIM_Z + 30.0)                                      # ran 0.3 into the room's front (its re-fill 25 mm3 at z 8.3-9.9 under the collar, dry 153)
            if kw:                                                                                  # outside the outline only ABOVE the deck's top (the band's 1 out trenched the wall's
                band = band & (prism(outline, ZB, RIM_Z + 30.0) + self.over_t(0.0))               # foot 0.1-0.5 where the deck rose past its bottom, builds 117-142)
            else:
                band -= prism(p["ring_up"].buffer(c + 1.0, join_style=2), ZB, RIM_Z + 30.0)
            sols = sorted(band.solids(), key=lambda q: -q.volume)     # ONE solid: OCC's common with a two-solid tool hands back ONE piece's overlap, no error (9 of
            band = sols[0]                                             # 567 mm3 when the seat's strip cut a crumb off the band's far end, build 122d)
            assert sum(q.volume for q in sols[1:]) < 0.05 * band.volume, f"{self.side}: the step round's band is {len(sols)} solids at {[round(v) for v in b]}: {[round(q.volume) for q in sols]}"
            kb, hb, mt0 = keep & band, (hull & band).volume, shape_tolerance(hull)
            hull = (hull - band) + kb
            assert hull.is_valid and len(hull.solids()) == 1, f"{self.side}: the step round at {[round(v) for v in b]} left the hull {'invalid' if not hull.is_valid else str(len(hull.solids())) + ' solids'}"
            got, want = mesh_volume(hull & band), mesh_volume(kb)      # the band after, by a fresh boolean and MESH volumes: the hull's GProp volume drifts ~0.1 % (100 mm3)
            assert abs(got - want) < 1.0, f"{self.side}: the band holds {got:.1f} mm3 after the swap, the rounded keep {want:.1f}"   # on this topology (0.18 edge tolerances) and 5 mm3 on
            mt = shape_tolerance(hull)                                 # a band cut from it; build 122b's +96 'cut nothing' was the drift (scratchpad)
            assert mt <= max(0.2, mt0 + 0.005), f"{self.side}: the step round raised the hull's tolerance {mt0:.3f} -> {mt:.3f}"   # (L carries 0.216 since inner_bay)
            print(f"   {self.side} STEP ROUND r {r}: the keep's round {off:.0f} mm3 along {n:.0f} mm (a full round {full:.0f}); the band {hb:.0f} -> {got:.0f} mm3 (the lift's 0.25 inset and the front wall's lean filled to the edge); tolerance {mt:.3f}; at x {b[0]:.0f}..{b[2]:.0f} y {b[1]:.0f}..{b[3]:.0f}")
        # (R: THE TRACKPAD TRAY of builds 118-152 -- TP_IND into the console's top round the pad, its mouth flared -- is gone: the crown's window frames the pad
        #  and the overlay sits sunk in it, build 153)
        self.hull = hull
        # (the rest's outline edge is the TPU's own round: no hull edge there -- Willy 20:57 'i dont want lip on the rests')

    def rest_skin(self):
        """The wrist rest's sloped skin over the cavity, re-added after the plateau drop."""
        hull, p, plan = self.hull, self.p, self.plan
        hull += prism(p["palm_side"], FLOOR, RIM_Z) & rest_plane(p, -TOP - PAD_DEPTH)   # the palm side's 5 mm skin (the deck over the TPU recess). The old region, palm + WALL, ran 3 past the tilt line: a strip at RIM_Z the tilt stood up as a rim
        self.hull = hull

    def cell_bore(self):
        """The 18650's blind half-pipe along the top row, bored from below into the flat band; the band's outside stays a box (Willy 11:10 'battery shape only on the inside')."""
        Kf, hull, out, p = self.Kf, self.hull, self.out, self.p
        # THE CELL: a blind half-pipe along the top row, bored into the solid above the cavity; it drops in from below, the hatch's cradle holds it up;
        # a tabbed cell, its leads soldered and dropped into the cavity at the bore's ends (LEAD of room each), no spring contacts
        ck = p["cell_k"].bounds
        Lc, r = ck[2] - ck[0], (ck[3] - ck[1]) / 2
        zc = FLOOR + 0.3 + CELL_R
        Fc = Kf * Pos((ck[0] + ck[2]) / 2, (ck[1] + ck[3]) / 2, zc)
        for attempt in range(2):                                    # OCC can hand the hull back UNCHANGED from its first boolean after rest_skin's fuse (build 83 L: 0.00 removed, the cell
            v0 = hull.volume                                        # 8.6 cm3 in the hull; bore_repeat.py: the same cut on the same hull removes 0 on try 0 and 10.49 on tries 1-3 -- the
            tool = Fc * Rot(0, 90, 0) * Cylinder(r, Lc, align=MID)
            hull = hull - tool if attempt == 0 else fuzzy_cut(hull, tool, 0.01)   # the retry FUZZY (build 118: the plain cut returns the hull unchanged twice over the back round's
            #                                                                    evolutive-fillet blend patches (B-spline faces) that the bore passes under; fuzzy 0.01 takes the full 10.5 cm3)
            #                                                          (118-: failed attempt leaves the hull's tolerances enlarged, the next goes through); a clipped tool instead gave a corrupt
            if v0 - hull.volume > 8000.0:                           # hull (solid to -infinity under the band, bore_probe84.py). So: retry once, then ASSERT -- a skipped cut is a build error
                break
            print(f"   {self.side} cell bore: OCC returned the hull unchanged (attempt {attempt}), retrying", flush=True)
        assert v0 - hull.volume > 8000.0, f"{self.side}: the cell bore cut was skipped twice ({(v0 - hull.volume):.0f} mm3 removed)"
        left = _hit(Fc * Rot(0, 90, 0) * Cylinder(CELL_R, CELL_L, align=MID), hull)
        assert left < 1.0 and hull.bounding_box().min.Z > ZB - 0.5, f"{self.side}: the cell bore is wrong ({left:.0f} mm3 of cell in the hull, hull z from {hull.bounding_box().min.Z:.1f})"
        out.update(cell_at=Fc, cell_top=zc + CELL_R, bore_top=zc + r)
        self.hull = hull

    def pico_bay(self):
        """The Pico's bay IN THE PALM DECK: a pocket under the sloped deck's surface, ceiling pads + pilots, the pane's window and recess, the plug
        window through the outer wall, the printed frame. Built in the bay's own frame (origin on the deck's surface at the bay's centre, z normal to
        the REST_SLOPE plane, x along K x toward the outer wall) and placed by `Lf`."""
        hull, lap, out, p = self.hull, self.lap, self.out, self.p
        # THE PICO: along K x in front of the alphas, face up, hanging from the bay's ceiling under a clear pane, plug out the outer wall; four corner
        # pads from the ceiling hold the PCB (M2 from below into the inward pair), the solder side faces the cavity / the open hatch
        c = p["K"](p["pico_k"]).centroid
        zs = RIM_Z - TILT_DROP + (c.y - p["foot"]["rest"].bounds[3]) * math.tan(math.radians(REST_SLOPE))   # the deck's surface at the bay's centre
        wall_x = p["xo"] - (p["pico_k"].bounds[0] + p["pico_k"].bounds[2]) / 2.0                 # the outer wall's inner face from the bay's centre, along the bay's x
        Lf = Pos(c.x, c.y, zs) * Rot(REST_SLOPE, 0, 0) * Rot(0, 0, YAW)
        hull -= Lf * Pos(0, 0, -BAY_DOWN) * Box(BAY_L, BAY_W, 30.0, align=(Align.CENTER, Align.CENTER, Align.MAX))   # the pocket, through the skin into the cavity
        pico_s = import_step(str(SL.PICO_STEP))
        b = pico_s.bounding_box()
        z_pico = -BAY_DOWN - PICO_TOP_CLR - PICO_DROP - b.size.Z                                    # the board's lowest point under the ceiling (PICO_DROP: the plug tunnel)
        Pl = Pos(0.675, 0, z_pico) * Rot(0, 0, -90) * Pos(-b.center().X, -b.center().Y, -b.min.Z)   # the board in the bay's frame: its +y (the plug) along +x, the PCB
        pico_loc = Lf * Pl                                                                          # centred (the USB shell overhangs it by 1.35 toward the wall)
        pcb_top = max(pico_s.solids(), key=lambda q: q.bounding_box().size.X * q.bounding_box().size.Y).bounding_box().max.Z   # in the board's own z
        dz = pcb_top + PAD_CLR                                                                      # the pads' bottom, in the board's own z
        pad_h = (-TOP - PAD_DEPTH + 0.5) - (z_pico + dz - b.min.Z)                                  # up to 0.5 INTO the deck's 5 mm skin: the bay's ceiling (BAY_DOWN 5.6
        for xc, w in (((PAD_W - 2.0) / 2, PAD_W + 2.0), (22.0 - PAD_W / 2, PAD_W + 2.0)):           # under the surface) is 0.6 below the skin, in the cavity's air -- pads
            for yc, l in (((PAD_L - 2.0) / 2, PAD_L + 2.0), (52.0 - PAD_L_USB / 2, PAD_L_USB + 2.0)):   # stopping there floated (build 79: 2 solids dropped)
                hull += pico_loc * Pos(xc, yc, dz) * Box(w, l, pad_h, align=BIG)
        for hx, hy in PICO_HOLES:
            if hy < 10:                                             # the inward pair
                hull -= pico_loc * Pos(hx, hy, dz - 0.01) * Cylinder(PICO_PILOT / 2, PICO_SCREW, align=BIG)

        def to_l(x, y):                                             # the board's own x / y -> the bay's frame
            q = (Pl * Pos(x, y, 0)).position
            return (q.X, q.Y)
        lx0, ly0, lx1, ly1 = -0.3, PAD_L + 0.3, 21.3, 51.0 - PAD_L_USB - 0.3                       # the window: the board between its corner pads, full width
        win = Polygon([to_l(x, y) for x, y in ((lx0, ly0), (lx1, ly0), (lx1, ly1), (lx0, ly1))])
        # THE PANE UNDER THE PAD (builds 159-161; params PANE_WIN / LENS_IN / PANE_R): no frame; 159's lip and 160's tongue gone in 161.
        # (159's text:) The pane (the same size) PANE_SINK = PAD_DEPTH under the deck, its top on the pad
        # recess's floor; THE LIP -- the deck + border over the pane's top edge, LIP_IN wide -- from the keys' side, the TPU pad over the rest of it up to the arc
        # (pockets: the pad's hole is the window, a D over the board's top edge). The pane slides in under the lip from the pad's side.
        rec = win.buffer(LEDGE + PANE_CLR, join_style=2)                                           # the pane's seat: the pane + PANE_CLR, open to the top (the pad + the ring lie on it)
        xl, yb_, xr, yt = win.bounds                                                                # the lens's extents: the light window; its top edge yt
        yt -= LENS_IN                                                                               # (160: LENS_IN down the board -- the pin row stays under the ring)
        hs, hm = PANE_WIN
        w, d = (xr - xl) / 2.0, hm - hs
        Rc = (w * w + d * d) / (2.0 * d)                                                            # the pad's edge: the arc through (xl, yt - hs) and (xr, yt - hs), yt - hm at the middle
        circle = Point((xl + xr) / 2.0, yt - hm + Rc).buffer(Rc, quad_segs=256)
        vis = win.intersection(circle).intersection(sbox(-1000.0, -1000.0, 1000.0, yt)).buffer(-1.0, join_style=1).buffer(1.0, join_style=1)   # THE WINDOW: the lens, corners r 1.0
        # THE PANE UNDER THE PAD (159-161): no frame, no lip, no ring (Willy 21:25 'im open to you just throwing a curved edge on the d shaped hole in the
        # tpu to simplify things'; 161's hull ring was a floating island, a printed ring one part too many): the pane sits on its ledge in the open seat, the
        # TPU pad covers it up to the LENS -- the pad's own D hole, its top edge rounded PANE_R (wrist_rest) -- and holds it down. Pane in, pad on.
        pov = rec.difference(vis)                                                                   # the pad's part of the pane
        hull -= Lf * prism(win, -BAY_DOWN - 0.1, -PANE_SINK - PANE_T + 0.1)                        # the light path under the pane
        hull -= Lf * prism(rec, -PANE_SINK - PANE_T, 5.0)                                           # the seat, open to the top
        pane = Lf * prism(win.buffer(LEDGE, join_style=2), -PANE_SINK - PANE_T, -PANE_SINK)
        to_plan = lambda g: Polygon([(Lf * Pos(x, y, 0.0)).position.to_tuple()[:2] for x, y in g.exterior.coords])   # the bay's plane onto the plan
        out.update(pane_vis=to_plan(vis), pane_rec=to_plan(rec), pane_pov=to_plan(pov), pane_hole=to_plan(vis), pane_ring=None)
        print(f"   {self.side} PANE: the window {vis.area:.0f} mm2 of the pane's {win.buffer(LEDGE).area:.0f} ({hs} tall at its sides, {hm} at the middle, r {Rc:.0f}): the pad's own hole; "
              f"the pad over {pov.area:.0f} mm2 of the pane; the pane {PANE_SINK} under the deck, {PANE_T} thick on a {LEDGE_T} ledge")
        usb_z = z_pico + 0.05 + 0.96 + 0.95
        # (the plug's window and tunnel are bored by plug_tunnel, AFTER deck_edge and hinges: builds 1-95 cut the window here and deck_edge's fill
        #  put a bar back across it -- scratchpad tunnel_bisect.py)
        # (THE FRAME with its brim, builds 59-158: gone in 159 -- the chin and the pad lap the pane)
        out.update(pico_loc=pico_loc, pane=pane, frame=None, frame_local=None, z_pico=z_pico, pcb_top=pcb_top, win=win, bay_Lf=Lf,
                   wall_x=wall_x, usb_z=usb_z, pico_face=(Pl * pico_s).bounding_box().max.X,   # the bay frame: the wall's inner face, the connector's centre z and face x
                   pico_plug=(Lf * Pos(wall_x + WALL, 0, usb_z)).position.to_tuple())   # the plug window's centre on the wall's outer face (u); NOT usb_win: that is R's Pi OTG window (build 93 probed the back wall): checks probe the wall over it
        lap("cell + Pico")
        self.hull = hull

    def charger_bay(self):
        """Where the V713 sits on the hatch (its box is a clearance GUESS)."""
        out, p = self.out, self.p
        # THE V713 BAY (no STEP: a clearance box, GUESS): a pad + fence on the hatch, its USB-C window through the nearest wall
        vb = p["v713"].bounds
        out["v713_at"] = ((vb[0] + vb[2]) / 2, (vb[1] + vb[3]) / 2, FLOOR + V713_PAD)
        self.vb = vb

    def stand_keepouts(self):
        """The boss keep-out rects: the Pico bay (in the deck), cell, charger, the flap's ears."""
        ear2, ears, fy, hatch_plan, p, side = self.ear2, self.ears, self.fy, self.hatch_plan, self.p, self.side
        fy = FLAP_Y[side]
        ears = p["ears"]
        keep = [(p["foot"]["pico"].bounds, 0.5), (p["foot"]["cell"].bounds, 1.0), (p["v713"].buffer(V713_FENCE).bounds, 0.5), (ears.bounds, 0.5)]   # (rect, clearance beyond the boss radius)
        if LED_AT[side]:                                                                           # (L has none since build 153: the filament runs)
            xs, ys = [x for x, _ in LED_AT[side]], [y for _, y in LED_AT[side]]                      # + THE LED TRAY (build 134): no boss under it
            keep.append(((min(xs) - LED_TRAY[0] / 2 - 1.0, min(ys) - LED_TRAY[2] - LED_TRAY[1] - 1.0, max(xs) + LED_TRAY[0] / 2 + 1.0, max(ys) + LED_TRAY[2] + LED_TRAY[1] + 1.0), 0.5))
        ear2 = p["ear2"]
        print(f"   ears inside the hatch bed: {hatch_plan.contains(ear2)} ({hatch_plan.exterior.distance(ear2):.1f} to its edge)")
        self.ear2 = ear2
        self.ears = ears
        self.fy = fy
        self.keep = keep

    def inner_bay(self):
        """R: the display pocket, the stack's wall columns, the bezel, the trackpad dish. L: the sunk knob well + cup, the charger bay roofed."""
        hull, keep, out, p, pk, side = self.hull, self.keep, self.out, self.p, self.pk, self.side
        if side == "R":
            # THE DISPLAY POCKET: open from the rim to the hatch; the HAT's glass 2.4 under the rim
            pk = sbox(U_PK0, Y_P0, U_PK1, Y_P0 + PK_D)
            hull -= prism(pk, FLOOR - 0.1, RIM_Z + 1)
            hull -= prism(sbox(U_PK0, pk.bounds[3] - 0.01, U_PK1, pk.bounds[3] + WIRE), FLOOR - 0.1, RIM_Z - ROOF_T)   # the wire bay behind it, roofed
            # THE STACK HANGS FROM THE TOP (Willy 2026-10-11 20:33 "screwed into the top and not the bottom"): columns built out of the
            # pocket's side walls between the Pi's top and the HAT's underside; M2.5 up through the Pi from below, down through the HAT
            # from above; the hatch carries nothing (it comes off with the stack in place, the Pi's underside open for wiring)
            z_pi = (F2H * Pos(0, 0, Z_PI)).position.Z                   # (161: velvet's Z_PI, PI_GAP under the HAT -- not DP's socketed stack)
            pi_top, z_hat = z_pi + DP.PI_PCB, (F2H * Pos(0, 0, DP.Z_HAT)).position.Z
            pi_q = [(F2H * Pos(x, y, 0)).position for x, y in PI_HOLES_F]
            hat_q = [(F2H * Pos(x, y, 0)).position for x, y in HAT_HOLES_F]
            def col(q, into=0.5):                                   # a post + its bar out of the nearer side wall (into it by `into`)
                wall = U_PK0 - into if q.X < (U_PK0 + U_PK1) / 2 else U_PK1 + into
                return Point(q.X, q.Y).buffer(POST_D / 2, quad_segs=8).union(sbox(min(q.X, wall), q.Y - POST_D / 2, max(q.X, wall), q.Y + POST_D / 2))
            # SCREWED FROM THE TOP (164; Willy 00:14): the soldered stack drops in from above as one; four M2.5 down through the HAT's corner holes. The BACK two
            # (past the Pi's end) into columns under the HAT; the FRONT two sit over the Pi's front holes: through the HAT, a printed PI_SPACER (PI_GAP long) and the
            # Pi into posts UNDER the Pi (161-163's columns BETWEEN the boards would stop a soldered stack going in; the pegs and the bezel's legs are gone)
            front, back = hat_q[:2], hat_q[2:]
            assert all(abs(a.X - b.X) < 0.1 and abs(a.Y - b.Y) < 0.1 for a, b in zip(front, pi_q[:2])), "the HAT's front holes are not over the Pi's"
            hull += prism(unary_union([col(q) for q in back]), pi_top, z_hat)
            bb0 = hull.bounding_box()
            posts = prism(unary_union([col(q, 0.5 if q.X < (U_PK0 + U_PK1) / 2 else 2.4) for q in front]), z_pi - FRONT_POST_H, z_pi)   # (164: R's outer side has no wall
            for q in front:                                         # under the pocket's 6.4: the outer bar runs 6.5 past the pocket's edge and turns UP beside the Pi's end into the wall -- an L)
                if q.X > (U_PK0 + U_PK1) / 2:
                    posts += prism(sbox(U_PK1, q.Y - POST_D / 2, U_PK1 + 2.4, q.Y + POST_D / 2), z_pi - FRONT_POST_H, z_hat - 0.2)   # (2.4: past ~3.4 the tilt step lifts the key block's zone -- 5.8 / 6.5 put 1.5 mm3 over the seam wall and 0.1 into the pit, tests 164)   # up to where 161's column met the wall
            hull += posts
            bb1 = hull.bounding_box()
            assert len(hull.solids()) == 1 and abs(bb1.min.X - bb0.min.X) < 0.01 and abs(bb1.max.X - bb0.max.X) < 0.01, f"R: the front posts: {len(hull.solids())} solid(s) {[[round(v, 1) for v in (b.min.X, b.max.X, b.min.Y, b.max.Y, b.min.Z, b.max.Z)] for b in (q.bounding_box() for q in sorted(hull.solids(), key=lambda q: q.volume)[:-1])]}, x {bb0.min.X:.2f}..{bb0.max.X:.2f} -> {bb1.min.X:.2f}..{bb1.max.X:.2f}"
            assert z_pi - FRONT_POST_H > FLOOR + 0.3, f"R: the front posts reach the hatch ({z_pi - FRONT_POST_H:.2f} vs {FLOOR})"
            for q, z in [(q, z_hat) for q in back] + [(q, z_pi) for q in front]:
                hull -= Pos(q.X, q.Y, z - STACK_SCREW[1]) * Cylinder(STACK_SCREW[0] / 2, STACK_SCREW[1] + 0.5, align=BIG)
            spacer = Cylinder(PI_SPACER[0] / 2, PI_GAP, align=BIG) - Cylinder(PI_SPACER[1] / 2, PI_GAP + 1.0, align=BIG)
            spacers = [Pos(q.X, q.Y, pi_top) * spacer for q in front]
            pry = Pos((U_PK0 + U_PK1) / 2, Y_P0 - 0.01, RIM_Z - PRY_D) * Box(PRY_W, LIP_W + 0.2, PRY_D + 1.0, align=(Align.CENTER, Align.MAX, Align.MIN))   # the pry notch in the pocket's front edge: UNDER the
            hull -= pry                                                                        # bezel's brim flange now (Willy 12:25 'whats with the notch at the top of the trackpad?'), reached once the bezel is off
            out["pry"] = sbox((U_PK0 + U_PK1) / 2 - PRY_W / 2, Y_P0 - 0.21 - LIP_W, (U_PK0 + U_PK1) / 2 + PRY_W / 2, Y_P0)
            tft, _ = DP.hat_in_F()
            pi, hdr, sock = stack_in_F()
            assert _hit(posts, F2H * pi) < 0.01, f"R: the front posts sit in the Pi by {_hit(posts, F2H * pi):.2f} mm3"
            for s_ in spacers:                                      # the spacers clear of the boards' parts and the hull (F == hull frame here: F2H places the STEPs)
                for n_, q_ in (("Pi", F2H * pi), ("header", F2H * hdr), ("HAT", F2H * tft), ("hull", hull)):
                    assert _hit(s_, q_) < 0.01, f"R: a stack spacer sits in the {n_} by {_hit(s_, q_):.2f} mm3"
            print(f"   R STACK SCREWS: M2.5 x4 from the top, pilots {STACK_SCREW[0]} x {STACK_SCREW[1]}; back two into the columns (to z {z_hat:.2f}), front two through spacers {PI_SPACER[0]}/{PI_SPACER[1]} x {PI_GAP} into posts under the Pi (z {z_pi - FRONT_POST_H:.2f}..{z_pi:.2f}, hatch {FLOOR}); screws: back M2.5 x 6, front M2.5 x 12-14", flush=True)
            pcb = max(pi.solids(), key=lambda q: q.bounding_box().size.X * q.bounding_box().size.Y).bounding_box()
            for q in pi.solids():                                   # the CAMERA FPC connector sticks 0.8 past the Pi's inner edge: a relief through the inner wall, from the STEP
                b = q.bounding_box()                                #   (builds 1-84 called this the SD slot; the slot is at the OUTER end, pi_ports.py)
                if b.min.X < pcb.min.X - 0.1 and b.max.X < pcb.min.X + 10:
                    lo, hi = (F2H * Pos(b.min.X, b.min.Y, b.min.Z)).position, (F2H * Pos(b.max.X, b.max.Y, b.max.Z)).position
                    hull -= Pos((lo.X - 3.0 + U_PK0 + 0.1) / 2, (lo.Y + hi.Y) / 2, (lo.Z + hi.Z) / 2) * Box(U_PK0 + 0.1 - (lo.X - 3.0), hi.Y - lo.Y + 3.0, hi.Z - lo.Z + 3.0, align=MID)
            sd = [q for q in pi.solids() if (b := q.bounding_box()).max.X > pcb.max.X - 1.5 and 9.0 < b.size.X < 13.0 and 10.0 < b.size.Y < 14.0]   # the microSD slot: at the outer end, by the header
            assert len(sd) == 1, f"microSD slot: {len(sd)} candidates on the Pi STEP"
            b = sd[0].bounding_box()
            lo, hi = (F2H * Pos(pcb.max.X, b.min.Y, b.min.Z)).position, (F2H * Pos(pcb.max.X, b.max.Y, b.max.Z)).position
            d, m = SD_RELIEF
            sdb = Box(d + 0.5, hi.Y - lo.Y + 2 * m, hi.Z - lo.Z + 2 * m, align=(Align.MIN, Align.CENTER, Align.CENTER))
            sdb = sdb.fillet(min(SD_R, (hi.Z - lo.Z) / 2 + m - 0.1), [e for e in sdb.edges() if abs(e.length - (d + 0.5)) < 0.01])   # (162) the relief's long edges rounded: a stadium, not a box
            hull -= Pos(lo.X - 0.5, (lo.Y + hi.Y) / 2, (lo.Z + hi.Z) / 2) * sdb                 # the CARD's relief into the outer wall (it stands ~2 proud)
            out["sd_relief"] = (lo.X, lo.X + d, lo.Y - m, hi.Y + m, lo.Z - m, hi.Z + m)
            usbs = [q for q in pi.solids() if (b := q.bounding_box()).max.Y > pcb.max.Y + 0.3 and 7.0 < b.size.X < 9.0 and 2.0 < b.size.Z < 4.0]   # the two micro-B: they stick past the back edge
            assert len(usbs) == 2, f"micro-USB ports: {len(usbs)} candidates on the Pi STEP"
            otg = max(usbs, key=lambda q: q.bounding_box().center().X).bounding_box()           # the OTG (data) port is 41.4 from the SD end, PWR 54: the inner of the two
            oc = (F2H * Pos(otg.center().X, otg.max.Y, otg.center().Z)).position
            assert abs(oc.X - (F2H.position.X + PI_USB_F)) < 0.1, f"PI_USB_F is off: the STEP's OTG port is at u {oc.X:.2f}, params say {F2H.position.X + PI_USB_F:.2f}"
            out["pi_usb"] = (oc.X, oc.Y, oc.Z)                       # (centre x, the port's face y, centre z) in u
            # THE BEZEL (Willy 21:15): a plate in the pocket from the glass up to the rim, the screen + buttons showing; it stands on the
            # HAT's PCB on four legs over the pegs and PRESS-FITS in the pocket on crush ribs (no screws); a pry notch in the front wall lifts it.
            # Window + slots from the STEP's faces.
            big = max(tft.solids(), key=lambda q: q.volume)
            gz = max(f.bounding_box().max.Z for f in big.faces())
            g = max((f for f in big.faces() if f.normal_at().Z > 0.99 and abs(f.bounding_box().max.Z - gz) < 0.01), key=lambda f: f.area).bounding_box()
            bar = min(tft.solids(), key=lambda q: q.volume)
            btz = bar.bounding_box().max.Z
            base = max((f for f in bar.faces() if f.normal_at().Z > 0.99 and f.bounding_box().max.Z < btz - 0.5), key=lambda f: f.area).bounding_box()
            hw, y0p, y1p = (U_PK1 - U_PK0) / 2 - BZ_CLR, DP.LID_Y0 - POCKET_CLR + BZ_CLR, DP.LID_Y0 - POCKET_CLR + PK_D - BZ_CLR
            z0, z1 = gz, gz + GLASS_SINK
            bez = Pos(0, 0, z0) * Box(2 * hw, y1p - y0p, z1 - z0, align=(Align.CENTER, Align.MIN, Align.MIN)).moved(Pos(0, y0p, 0))
            holes_f = list(HAT_HOLES_F)
            pcb_z = DP.Z_HAT + DP.LID_PCB
            # (164: the legs on the HAT's pegs are gone -- the HAT is screwed down; the bezel rides its crush ribs over the glass, the screw heads under it)
            bez -= Pos(base.min.X - 0.3, base.min.Y - 0.3, z0 - 0.01) * Box(base.max.X - base.min.X + 0.6, base.max.Y - base.min.Y + 0.6, base.max.Z - z0 + 0.31, align=(Align.MIN, Align.MIN, Align.MIN))
            bot = clipped(g.min.X - 0.2, g.min.Y - 0.2, g.max.X + 0.2, g.max.Y + 0.2, CLIP_S)
            top = clipped(g.min.X - 0.2 - BZ_RUN, g.min.Y - 0.2 - BZ_RUN, g.max.X + 0.2 + BZ_RUN, g.max.Y + 0.2 + min(BZ_RUN, 0.5 * (base.min.Y - g.max.Y)), CLIP_B)
            bez -= bevel_cut(bot, z0, top, z1)
            posts = (bar & Pos(0, 0, base.max.Z + 0.3) * Box(200, 200, 0.05, align=MID)).solids()   # each button's post where it passes the bezel
            slots = [Polygon(clipped(b.min.X - 0.4, b.min.Y - 0.4, b.max.X + 0.4, b.max.Y + 0.4, CLIP_S)) for b in (q.bounding_box() for q in posts)]   # (clipped corners: the keyholes' mouths are bevelled in the brim, build 138)
            for sl in slots:
                bez -= prism(sl, z0 - 1, z1 + 1)
            # THE BUTTON CAPS (162; Willy 22:09): the real buttons are short (BTN_TOP over the PCB, GUESS), the STEP's posts tall; a cap per slot rides on the
            # plunger, its stem through the slot CAP[0] proud of the bezel, its foot in the bezel's button-bar relief (base.max.Z + 0.3 ceiling) -- the bezel keeps it
            z_pl = DP.Z_HAT + DP.LID_PCB + BTN_TOP
            caps = [button_cap(sl, z_pl, z1 + CAP[0]) for sl in slots]
            assert z_pl + CAP[2] + 0.1 <= base.max.Z + 0.3 or z_pl + CAP[2] <= z0, f"R: the caps' feet ({z_pl:.2f}..{z_pl + CAP[2]:.2f}) do not fit under the bezel (relief ceiling {base.max.Z + 0.3:.2f}, plate {z0:.2f})"
            trim = prism(unary_union([sl.buffer(CAP[1] + 0.1) for sl in slots]), z_pl, z_pl + 50.0)   # (164) the STEP's buttons stand taller than Willy's (3.5 over the PCB, 163):
            hat_show = Compound(children=[r_ for r_ in (q - trim for q in tft.solids()) if r_ is not None and r_.volume > 0.01])   # every HAT solid cut back at the plungers' top under the caps
            for sx in (-1, 1):                                      # CRUSH RIBS on both x faces: the press fit
                for i in range(RIB_N):
                    yr = y0p + (y1p - y0p) * (i + 1) / (RIB_N + 1)
                    bez += Pos(sx * (hw + RIB_H / 2 - 0.01), yr, z0) * Box(RIB_H + 0.02, RIB_W, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))
            for sy in (-1, 1):                                      # and on both y faces (Willy 12:39 'really tight and secure'): the pocket's front wall, the wire bay roof's front face
                for i in range(RIB_N):
                    xr = -hw + 2 * hw * (i + 1) / (RIB_N + 1)
                    if sy < 0 and abs(xr) < PRY_W / 2 + RIB_W:          # not into the pry notch
                        continue
                    bez += Pos(xr, (y0p if sy < 0 else y1p) + sy * (RIB_H / 2 - 0.01), z0) * Box(RIB_W, RIB_H + 0.02, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))
            ring = LineString(list(Polygon(top).buffer(2.2, join_style=2).exterior.coords))
            keep_out = unary_union([sl.buffer(BZ_SLOT[0] + 0.5) for sl in slots] + [Point(x, y).buffer(BZ_LEG_D / 2 + 0.3) for x, y in holes_f])
            band = sbox(-hw + 1.2, y0p + 1.2, hw - 1.2, y1p - 1.2)
            traces = unary_union([substring(ring, a * ring.length, b * ring.length).buffer(BZ_GROOVE[0] / 2, cap_style=2) for a, b in ((0.03, 0.40), (0.52, 0.88))]
                                 + [Point(*substring(ring, t * ring.length, t * ring.length).coords[0]).buffer(0.75) for t in (0.03, 0.40, 0.52, 0.88)])
            traces = traces.intersection(band).difference(keep_out)
            out["bz_cuts"] = (Polygon(top), slots, traces)                  # for the brim (finish: bezel_brim), in the bezel's frame (F2H: a translation)
            print(f"   BEZEL: glass {g.max.X - g.min.X:.1f} x {g.max.Y - g.min.Y:.1f} (STEP face), {len(slots)} button slots (posts {posts[0].bounding_box().size.X:.1f} x {posts[0].bounding_box().size.Y:.1f}), window bevel {BZ_RUN} run, clipped {CLIP_B}/{CLIP_S}, traces {traces.area:.0f} mm2 x {BZ_GROOVE[1]} deep")
            for c in caps:
                assert c.is_valid and _hit(c, bez) < 0.01, f"R: a button cap sits in the bezel by {_hit(c, bez):.2f} mm3"
            print(f"   R BUTTON CAPS: 4 caps on plungers {BTN_TOP} over the PCB (Willy's measure), stems {slots[0].bounds[2] - slots[0].bounds[0] - 2 * CAP[3]:.1f} x {slots[0].bounds[3] - slots[0].bounds[1] - 2 * CAP[3]:.1f} from z {z_pl:.2f} to {z1 + CAP[0]:.2f} (bezel top {z1:.2f}), feet +{CAP[1]} x {CAP[2]} under a {base.max.Z + 0.3:.2f} ceiling; {caps[0].volume:.0f} mm3 each")
            out.update(F2H=F2H, hat=F2H * hat_show, pi=F2H * pi, header=F2H * hdr, socket=None, bezel=F2H * bez, caps=[F2H * c for c in caps], caps_local=caps, pi_spacers=[F2H * s_ for s_ in spacers], pi_spacer_local=spacer,
                       pi_posts=[(F2H * Pos(x, y, 0)).position for x, y in PI_HOLES_F],
                       hat_posts=[(F2H * Pos(x, y, 0)).position for x, y in HAT_HOLES_F[2:]],
                       z_pi=(F2H * Pos(0, 0, Z_PI)).position.Z, z_hat=(F2H * Pos(0, 0, DP.Z_HAT)).position.Z)
            # THE TRACKPAD CONSOLE, FLAT at RIM_Z + CONSOLE_UP (Willy 15:34 'flush with pitft bezel'; builds 97-116 pitched it, Willy 21:17, plan.tp_pitch
            # now 0): the wedge above the plane cut over plan.tp_zone (the housing, the corner fill, the seam wall; CUT_OFF past every wall: no cut
            # face on a hull face). Under it a pocket from the cavity's ceiling up to a TP_SKIN
            # skin; the TPS43 sticks to the skin face-up on its own 3M adhesive, its pads (SDA SCL 3V3 GND RST RDY) face down into the cavity for
            # the wires; no finger dish (build 118)
            # THE CONSOLE RAISED CONSOLE_UP (Willy 12:25 'the area around the trackpad can be slightly raised so when i place the pi tft bezel it appears to
            # integrate'): its plane starts RIM_Z + CONSOLE_UP at the hinge line -- the bezel's brim top -- and falls to the border's level at the front
            # (plan.tp_pitch); the brim's front flange sits in a SEAT cut back to the rim, COVER_CLR round it, so the bezel still drops in
            tb = out["bezel"].bounding_box()
            out["bz_outline"] = sbox(tb.min.X, tb.min.Y, tb.max.X, tb.max.Y)                        # the bezel's plan (its ribs included)
            fo = out["bz_outline"].buffer(LIP_W, join_style=2)                                     # THE SEAT (build 143; Willy 08:41 'the little keywall corner that is close to
            fo = unary_union([fo.buffer(-BZ_CORNER, join_style=2).buffer(BZ_CORNER, join_style=1, quad_segs=16),   # the pitft bezel needs to line up'): the brim's foot with its
                              fo.intersection(sbox(-1000.0, -1000.0, (fo.bounds[0] + fo.bounds[2]) / 2, 1000.0))])   # key-side corners r BZ_CORNER (bezel_brim) + COVER_CLR, the WHOLE
            bz_seat = fo.buffer(COVER_CLR, join_style=1, quad_segs=16)                              # the brim's old seat (builds 122-152): the LED tray keeps 0.5 off it (badge)
            # THE CROWN (build 153; Willy 10:44-11:42; lights.py): ONE plate over the whole console -- the bezel's body + flange, the LED wing, the trackpad
            # border, the plateau lid -- CROWN_T on the rim. FLUSH: the console is NOT lifted any more (the crown IS the raised console), its step edges are
            # cut square under the crown's edge (step_chamfer: the seat), the crown's own r CROWN_R round is the console's edge round; no console round.
            # Its plan: the console's top 0.1 inside its step edges, to the seam face (the lip's hook, bezel_brim's since build 141), the flange with its r
            # BZ_CORNER key-side corners, the strip to 0.3 off the key wall's face, the lid band (the K box from the flange's east edge to its east end, 0.3
            # off the step chamfer's foot, BADGE_IN inside the back round); closed r 1.5 -- one intentional outline (Willy 11:42 'clean intentional lines')
            console = p["tp_zone"].intersection(p["flat"]).buffer(-0.1, join_style=2)
            xe = p["flat"].intersection(sbox(fo.bounds[2], Y_P0, 1000.0, Y_P0 + 1.0)).bounds[2]     # THE STRIP (build 143): the key wall's face beside the brim (x 71.6)
            ext_ = list(p["flat"].exterior.coords)
            y_block = max(max(ya, yb) for (xa, ya), (xb, yb) in zip(ext_, ext_[1:]) if abs(xa - xe) < 0.05 and abs(xb - xe) < 0.05 and min(ya, yb) < Y_P0 + 0.1 < max(ya, yb))
            y_fade0, y_fade1 = self.deck_y(RIM_Z - EDGE_R_TOP - 0.2), self.deck_y(RIM_Z - 0.3)     # (the console at the rim now: the key wall's run fades where the deck meets the rim)
            strip = p["flat"].buffer(-0.25, join_style=2).intersection(sbox(bz_seat.bounds[2], Y_P0 - 1.0, xe + 0.5, y_block))
            out["strip"] = (xe, y_block, y_fade0, y_fade1, strip)
            lipz = sbox(0.0, fo.bounds[1], out["bz_outline"].bounds[0], fo.bounds[3])               # the lip to the seam face along the bezel ...
            seam = console.buffer(0.1, join_style=2).intersection(sbox(-1.0, -1000.0, 0.5, fo.bounds[3] - 1.5))   # ... and along the console (the hook over the wall's round): the
            #                                                                                        console's own edge there (a box left a 0.5 tab beside its front corner's arc: refused)
            diag = console.buffer(0.45, join_style=2).intersection(p["plan"].difference(p["plan"].buffer(-0.5, join_style=2))).intersection(sbox(-1.0, console.bounds[1] + 0.5, 1000.0, fo.bounds[3] - 1.5))
            #                                                                                        THE FRONT DIAGONAL (build 156): the console's plan ran 0.35 inside the outline from the seam face
            #                                                                                        down to its front corner (the tp zone's edge) and the crown hung over the wall's round -- 0.73 under
            #                                                                                        its edge, 61 mm2 (test_crowns); its edge sits ON the outline there now, like the seam face, the hook under it
            cstrip = p["flat"].intersection(sbox(fo.bounds[2] - 0.5, Y_P0 - 1.0, xe - 0.3, 45.0))
            lidbox = p["K"](sbox(LIGHT_LID[0] - 6.0, LIGHT_LID[1], LIGHT_LID[2], LIGHT_LID[3])).intersection(p["plan"].buffer(-BADGE_IN, join_style=2)).intersection(p["flat"].buffer(-0.2, join_style=2))
            crown = unary_union([console, seam, diag, fo, lipz, cstrip, lidbox]).buffer(1.5, join_style=1).buffer(-1.5, join_style=1).intersection(sbox(0.0, -1000.0, 1000.0, 1000.0))
            crown = clean_outline(max(getattr(crown, "geoms", [crown]), key=lambda g: g.area), CROWN_R, tag="R CROWN PLAN")
            assert crown.geom_type == "Polygon" and not crown.interiors, f"R: the crown's plan is a {crown.geom_type} with {len(crown.interiors)} hole(s)"
            out["crown_plan"], out["bz_seat"] = crown, bz_seat
            seat = unary_union([crown.buffer(COVER_CLR, join_style=1), p["tp_zone"].intersection(p["flat"]).buffer(0.5, join_style=2)])   # THE SEAT: step_chamfer cuts the
            out["seat"] = seat                                                                     # console's step piece back to the rim under it (build 122's brim-seat mechanism, the whole console now)
            Tt = Pos(0, Y_P0, RIM_Z + CONSOLE_UP) * Rot(p["tp_pitch"], 0, 0) * Pos(0, -Y_P0, -RIM_Z)   # the console's frame (builds 97-152's lift: z RIM_Z here = the crown's top now; the pad's place)
            assert abs(p["tp_pitch"]) < 1e-6, "R: the crown's trackpad border wants the console flat (plan.tp_pitch 0)"
            v0 = hull.volume
            hull -= prism(tp_window(), -TOP - 3.0, RIM_Z + 5.0)                                     # THE TRACKPAD POCKET through the top (Willy 10:56 'place it from top'): the pad drops in
            hull -= prism(collar_room(), TP_TOP - TP_T - TP_LEDGE - 0.5, RIM_Z + 5.0)              # onto the crown's ledge (its top TP_TOP, as it sat under the skin); the pocket widened for
            #                                                                                        the collar on three sides (the display side keeps its 1.8 wall)
            print(f"   CONSOLE FLAT at the rim under THE CROWN ({crown.area:.0f} mm2 {[round(v, 1) for v in crown.bounds]}, CROWN_T {CROWN_T} proud: the raised console IS the crown; no lift, no console round); the trackpad pocket open to the top, {v0 - hull.volume:.0f} mm3 (the collar room {collar_room().area - tp_window().area:.0f} mm2)", flush=True)
            out.update(tp_T=Tt)
            print(f"   TRACKPAD: pocket to the nearest key well {tp_plan(TP_CLR).distance(p['foot']['wells']):.1f}, to the display pocket {tp_plan(TP_CLR).distance(pk):.1f}, to the outline {p['plan'].exterior.distance(tp_plan(TP_CLR)):.1f}")
            keep += [((pk.bounds[0], pk.bounds[1], pk.bounds[2], pk.bounds[3] + WIRE), 0.5), (tp_plan(TP_CLR).bounds, 0.5)]   # no boss under the trackpad's pocket
        else:
            # THE KNOB, SUNK: a well to WELL_FLOOR, a PANEL_T deck under it (a boss into the cavity), the EC11's bushing up through it
            # and its nut on top: the encoder hangs from the top, centred, so the knob runs 0.3 off the well's wall
            hull += Pos(KNOB_U, KNOB_Y, WELL_FLOOR - PANEL_T) * Cylinder(KNOB_R + KNOB_CLR + CUP_W, -TOP - WELL_FLOOR + PANEL_T + 0.5, align=BIG)   # the cup under the deck
            hull -= Pos(KNOB_U, KNOB_Y, WELL_FLOOR) * Cylinder(KNOB_R + KNOB_CLR, RIM_Z - WELL_FLOOR + 1, align=BIG)
            hull -= Pos(KNOB_U, KNOB_Y, WELL_FLOOR - PANEL_T - 1) * Cylinder(7.2 / 2, PANEL_T + 2, align=BIG)    # the M7 bushing
            print(f"   KNOB WELL: centre u {KNOB_U:.1f}, to the nearest key well {Point(KNOB_U, KNOB_Y).buffer(KNOB_R + KNOB_CLR).distance(p['foot']['wells']):.1f}, to the outline {p['plan'].exterior.distance(Point(KNOB_U, KNOB_Y).buffer(KNOB_R + KNOB_CLR)):.1f}")
            hull -= prism(p["v713"].buffer(V713_FENCE + 0.2, join_style=2), FLOOR - 0.1, RIM_Z - ROOF_T)   # the charger's bay (fence + 0.2 inside), roofed
            keep += [((KNOB_U - (KNOB_R + KNOB_CLR + CUP_W), KNOB_Y - (KNOB_R + KNOB_CLR + CUP_W), KNOB_U + (KNOB_R + KNOB_CLR + CUP_W), KNOB_Y + (KNOB_R + KNOB_CLR + CUP_W)), 0.5)]   # no boss under the knob's cup
        self.hull = hull
        self.keep = keep
        self.pk = pk

    def wrist_rest(self):
        """The rest's slope, the TPU pad's recess, its raised border, the pad itself."""
        hull, lap, out, p, plan = self.hull, self.lap, self.out, self.p, self.plan
                # THE PALM DECK'S TOP: the rest plane (5 deg toward you) over everything in front of the tilt line; its outline edge is ONE STEP_C chamfer
        # (step_chamfer; the 4-layer round here read as a stair all round the rest, Willy 16:20 'the weird indent you put all around the wrist rests')
        top = prism(plan, ZB, RIM_Z + 5) & rest_plane(p, BORDER_UP, up=False) & Pos(0, 0, KEYWELL_MAX) * Box(4000.0, 4000.0, 400.0, align=(Align.CENTER, Align.CENTER, Align.MAX))   # THE BORDER'S LEVEL: the rest plane + BORDER_UP, capped at KEYWELL_MAX
        lip = p["palm_side"].buffer(CUT_OFF, join_style=1).intersection(p["pit"])         # CUT_OFF past the deck's face into the pit (the tilt's moat): a tool face ON
        front = prism(unary_union([p["palm_side"], lip]), ZB - 1.0, RIM_Z + 5)             # the face left zero-thickness sheets of the plateau's wall standing to
        #                                                                                   RIM_Z - EDGE_R_TOP at five spots along the thumb clusters' pit edge (the
        #                                                                                   blades of builds 103-104, scratchpad blade_scan / blade_shape)
        hull -= front - top                                                                 # everything on the palm side (the deck in front of the tilt line + the
        #                                                                                   pocket inside the thumb arc) down to the border's level
        hull, self.rest_keep = self.edge_round(hull, BORDER_R, p["palm_side"], lambda dz, up: rest_plane(p, BORDER_UP + dz, up), avoid=(p["flat"], p["ring"]), tag="REST ROUND")
        #                                                                                   the border's outer edge rounded -- BEFORE the recess (the band's inner face
        #                                                                                   is in the recess's footprint: cut first, a tool face ON a face)
        recess = rest_plane(p, -PAD_DEPTH)
        pad = p["pad"].difference(out["pane_hole"])                                                 # THE PAD OVER THE PANE (159-161): plan.py's pad (it covers the pane's seat) less the LENS,
        pad = max(getattr(pad, "geoms", [pad]), key=lambda g: g.area)                        # the pad's own hole; the TPU 0.2 off the tray's walls but to the lens's edge
        pad_tpu = unary_union([pad.buffer(-0.2, join_style=1, quad_segs=6), pad.intersection(out["pane_hole"].buffer(0.3, join_style=2))])
        assert pad.geom_type == "Polygon" and pad_tpu.geom_type == "Polygon", f"{self.side}: the pad is a {pad.geom_type}, the TPU a {pad_tpu.geom_type}"
        out["pad_plan"], out["pad_tpu_plan"] = pad, pad_tpu
        hull -= prism_holed(pad, ZB, RIM_Z + 5) & recess                                    # the recess (holed where the pad has a hole), PAD_DEPTH under the plane
        #                                                                                   (Willy 22:46 'like 2mm of border a little raised around the rests')
        tpu = prism_holed(pad_tpu, ZB, RIM_Z + 5) & recess & rest_plane(p, PAD_PROUD, up=False)   # the TPU, 0.2 off the tray's walls, its top FLUSH with
        #                                                                                   the border (PAD_PROUD = BORDER_UP: Willy 09:03 'raise the tpu and get rid of that lip')
        # THE WINDOW'S ROUND (161; Willy 20:26 'curve the edges like all the other edges in the build', 21:25 'a curved edge on the d shaped hole in the tpu'):
        # PANE_R on the top edge of the pad's own lens hole -- OCC's fillet on the TPU part (the hole's wire on its top face; a small part, its volume is honest)
        top = max((f for f in tpu.faces() if f.normal_at().Z > 0.9), key=lambda f: f.center().Z)
        lens = out["pane_vis"].centroid
        wire = min(top.inner_wires(), key=lambda w: math.hypot(w.center().X - lens.x, w.center().Y - lens.y))
        v0 = tpu.volume
        tpu = tpu.fillet(PANE_R, wire.edges())
        full = wire.length * PANE_R * PANE_R * (1 - math.pi / 4)
        assert tpu.is_valid and len(tpu.solids()) == 1 and 0.6 * full < v0 - tpu.volume < 1.4 * full, f"{self.side}: the window's round on the TPU took {v0 - tpu.volume:.0f} mm3 (a full round {full:.0f}); valid {tpu.is_valid}, {len(tpu.solids())} solid(s)"
        out["tpu"] = tpu
        print(f"   {self.side} WINDOW ROUND r {PANE_R} on the TPU's lens edge: {len(wire.edges())} edges, {wire.length:.0f} mm, {v0 - tpu.volume:.0f} mm3 off (a full round {full:.0f})")
        lap("pockets")
        self.hull = hull

    def edge_round(self, hull, r, run_zone, plane, cap=None, avoid=(), tag="EDGE ROUND"):
        """ONE r round on the outline's top edge along run_zone's outline run (the palm side's border, R's pitched console): key_deck's band pattern,
        the outline band (r + 0.5 in, r + 1 under the top) swapped for a KEEP -- the plan's prism under the top's plane (plane(dz, up): the half-space
        over / under it offset dz; capped at z `cap`) with OCC's fillet on its top edges along the run: one face per edge (the layered round of builds
        85-89 read as terraces). OCC refuses the chain while the hinge face carries the flow's 0.08 zigzag (plan.hinge_straight), and the keep must
        be the plan's own prism (a keep of its own polygon simplified a corner differently: one unorientable fillet face, build 105). The band's inner
        face is r + 0.5 in -- AT r it sliced the fillet's corner blends (r / cos(half the turn) past a vertex) into a -0.04 mm2 inverted sliver (build
        106); it ends 0.3 inside `avoid`'s walls (an end face ON a wall would be a sheet) and runs CUT_OFF past run_zone (into the pit: the tilt's moat).
        Returns the hull and the rounded keep (deck_edge rounds the keywell's outline strip with the rest's)."""
        p, plan = self.p, self.plan
        under = lambda z: Pos(0, 0, z) * Box(BIGB, BIGB, BIGB, align=(Align.CENTER, Align.CENTER, Align.MAX))
        over = lambda z: Pos(0, 0, z) * Box(BIGB, BIGB, BIGB, align=(Align.CENTER, Align.CENTER, Align.MIN))
        keep = prism(plan, ZB - 1.0, RIM_Z + 30.0) & plane(0.0, False)
        if cap is not None:
            keep = keep & under(cap)
        sols = keep.solids()
        assert len(sols) == 1, f"{self.side} {tag}: the keep is {len(sols)} solids"
        sol = sols[0]
        run = plan.exterior.intersection(run_zone.buffer(0.1, join_style=2))
        run = max(getattr(run, "geoms", [run]), key=lambda g: g.length)
        near = run.buffer(0.15)
        seg = lambda e: LineString([(e.start_point().X, e.start_point().Y), (e.end_point().X, e.end_point().Y)])
        chain = [e for f in sol.faces() if f.normal_at().Z > 0.9 for e in f.edges()
                 if seg(e).intersection(near).length > min(0.3, e.length / 2) and plan.exterior.distance(Point(e.center().X, e.center().Y)) < 0.5]   # (not a crease across the top)
        n = sum(e.length for e in chain)
        k = None
        for rr in (r, r - 0.5):
            try:
                k = sol.fillet(rr, chain)
                break
            except ValueError:
                print(f"   {self.side} {tag}: OCC refuses r {rr} on the run's {len(chain)} edges", flush=True)
        assert k is not None and k.is_valid, f"{self.side}: {tag} fails on the run's {len(chain)} edges"
        print(f"   {self.side} {tag} r {rr}: {len(chain)} edges, {n:.0f} mm; {sol.volume - k.volume:.0f} mm3 off the keep (a full round {n * rr * rr * (1 - math.pi / 4):.0f})")
        keep_r = k & prism(plan, ZB - 2.0, RIM_Z + 31.0)
        zone = run.buffer(r + 1.5, join_style=1).intersection(run_zone.buffer(CUT_OFF, join_style=2))
        for g in avoid:
            zone = zone.difference(g.buffer(0.3, join_style=2))
        band = prism(plan.buffer(1.0, join_style=1), ZB, RIM_Z + 5.0) - prism(plan.buffer(-(r + 0.5), join_style=1, quad_segs=6), ZB - 1.0, RIM_Z + 6.0)
        bottom = plane(-r - 1.0, True)
        if cap is not None:
            bottom = bottom + over(cap - r - 1.0)
        band = band & prism(zone, ZB - 1.0, RIM_Z + 6.0) & bottom
        v0 = hull.volume
        hull = (hull - band) + (keep_r & band)
        print(f"   {self.side} {tag}: {v0 - hull.volume:.0f} mm3 off the edge along {run.length:.0f} mm of outline ({len(hull.solids())} solid(s))")
        return hull, keep_r

    def pick_bosses(self):
        """Boss spots for the hatch screws: on the wall ring or inside, clear of everything, 22 apart."""
        fu, hatch_plan, keep, side = self.fu, self.hatch_plan, self.keep, self.side
        # bosses for the hatch: on the hull's walls, clear of switches, the cup, the stack, the cell, the charger, the Pico
        sw = [f.position.to_tuple()[:2] for f in fu.values()]

        def ok(c, rects):
            return all(max(abs(c.x - x), abs(c.y - y)) > 7.8 + BOSS_D / 2 + 1 for x, y in sw) \
                and not any(r[0] - BOSS_D / 2 - m < c.x < r[2] + BOSS_D / 2 + m and r[1] - BOSS_D / 2 - m < c.y < r[3] + BOSS_D / 2 + m for r, m in rects)

        def pick(poly, n, rects, spacing=22.0):
            """Up to n bosses: on the wall ring or anywhere inside, farthest-first, never closer than `spacing` to each other."""
            shr = poly.buffer(-BOSS_D / 2 + 0.5, join_style=1, quad_segs=4)
            rings = [g.exterior for g in getattr(shr, "geoms", [shr]) if g.area > 50]
            cand = [c for ring in rings for c in (ring.interpolate(t, normalized=True) for t in np.linspace(0, 1, 480, endpoint=False)) if ok(c, rects)]
            b = shr.bounds
            cand += [c for x in np.arange(b[0], b[2], 4.0) for y in np.arange(b[1], b[3], 4.0) if shr.contains(c := Point(x, y)) and ok(c, rects)]
            out_ = []
            while cand and len(out_) < n:
                c = max(cand, key=lambda c: min((c.distance(b) for b in out_), default=1e9) if out_ else -c.y + c.x)
                if out_ and min(c.distance(b) for b in out_) < spacing:
                    break
                out_.append(c)
            assert len(out_) >= 3, f"{side}: only {len(out_)} boss spots {spacing} apart"
            return out_
        bosses = pick(hatch_plan, 8, keep)
        self.bosses = bosses

    def tilt(self):
        """The keys turn KEY_TILT about the top-front edge and drop TILT_DROP; the back band + inner bay stay flat (low profile); a skirt and a seam wall close the cavity."""
        Z, fu, hatch_plan, hull, inner, lap, out, p, plan, plateau, side, wells = self.Z, self.fu, self.hatch_plan, self.hull, self.inner, self.lap, self.out, self.p, self.plan, self.plateau, self.side, self.wells
        # THE TILT: the body behind yp turns KEY_TILT about its top-front edge, the rest stays flat, a hollow skirt fills down to ZB
        yp = min(f.position.Y for f in fu.values()) - 13.0 - 6.0                                     # 6 in front of the front-most cap
        T = Pos(0, 0, -TILT_DROP) * Pos(0, yp, RIM_Z) * Rot(KEY_TILT, 0, 0) * Pos(0, -yp, -RIM_Z)
        BIGB = 3000.0
        ahead = lambda y: Pos(0, y, 0) * Box(BIGB, BIGB, BIGB, align=(Align.CENTER, Align.MAX, Align.CENTER))   # y < y
        behind = lambda y: Pos(0, y, 0) * Box(BIGB, BIGB, BIGB, align=(Align.CENTER, Align.MIN, Align.CENTER))  # y > y
        under_t = lambda z: T * Pos(0, 0, z) * Box(BIGB, BIGB, BIGB, align=(Align.CENTER, Align.CENTER, Align.MAX))   # under the tilted plane z' = z
        skirt = (rounded_prism(plan, ZB, ZB + 80.0, r_bot=EDGE_R_BOT) & behind(yp) & under_t(ZB + 0.01)) - prism(inner, FLOOR - 0.1, ZB + 81.0)
        above_zb = Pos(0, 0, ZB) * Box(BIGB, BIGB, BIGB, align=(Align.CENTER, Align.CENTER, Align.MIN))
        # THE FLAT BODY -- the flat zone + the palm side -- is kept as ONE column and cut from the body to be tilted with a tool CUT_OFF
        # OUTSIDE it. Two tools (builds 59-88: the zone's prism, the palm side's) left the hairline between their arcs in neither (a 0.001-0.02
        # crack along the housing from the hatch to the top, scratchpad sheets88.py), and every cut face ON a hull face -- the housing's wall, the
        # inner wall, the deck's face -- came out of OCC as a zero-thickness sheet in the body, which the tilt stood over the rim as a fin
        # (Willy 13:22 'you can see defects': the picket fence round the knob, fin_probe87 / sheets88: 14 sheets up to 5.5 over the rim).
        # The ring fill below has the 0.25 the tilted body loses along the zone; along the palm side the kept column has it.
        # The STRIP of the key block's front between its pre-tilt edge and the pit (the block's shadow at the plate) is spared BELOW the plate's
        # top: its plate rides with the tilt and lands AT the pit, under the deck's face. Cut with the rest (builds 82-87) the plate's front edge
        # landed 1.2 behind the pit: a 1 mm slot through the plate along the whole key block (Willy 13:57 'still a gap between the key plate and
        # the wrist rest'; scratchpad pit_profile87.py: y -52.9..-51.9 empty, the deck face at -53.4). The spared column stops 0.01 under the
        # plate's top (a tool face ON the plate's top would be another sheet).
        strip = p["palm_side"].intersection(p["tilted"])
        keep = flat_body(Z, p["palm_side"], plan)
        cut = prism(keep.buffer(CUT_OFF, join_style=1), ZB - 1.0, 200.0)
        spare = strip.buffer(CUT_OFF, join_style=1)
        for g in getattr(spare, "geoms", [spare]):
            if g.area > 0.1:
                cut -= prism(g, ZB - 1.5, -0.01)
        # LOW PROFILE (Willy 23:41 'lower the battery ... and the plastic for the charger lower than the keys'): the back band (cell, Pico)
        # and the inner bay (R: Pi stack, L: V713) stay FLAT on the hatch, the keys alone tilt; a wall along the seam (over a 6 mm wire
        # gap) closes the tilted cavity and props the top row
        Z = p["flat"]
        Zp = prism(Z, ZB - 1.0, 200.0)
        sw = unary_union([Polygon(rect(f, HOLE + 2.0, HOLE + 2.0)) for f in fu.values()])
        ring = p["ring"]
        moat = keep.buffer(CUT_OFF + 0.05, join_style=1).difference(keep).intersection(ring.buffer(1.0))   # the CUT_OFF moat where the ring meets the console and the palm
        ring = unary_union([ring, moat])                                                                  # side: the ring (opened 0.1) left a 0.2 pinhole there (build 90, 0.017 mm3)
        fill = (prism(ring, FLOOR + 6.0, 200.0) & under_t(-0.01) & behind(yp) & Pos(0, 0, RIM_Z) * Box(BIGB, BIGB, BIGB, align=(Align.CENTER, Align.CENTER, Align.MAX))) - T * prism(sw, -40.0, RIM_Z + 1)   # capped 0.01 UNDER the tilted DECK plane (over it, build 94-128: a 0.01 skin
        #                                                                                                                 -- and never over the RIM: the plane passes RIM_Z behind the key block, the strip stood 0.25 proud of the plateau at the corner (build 130)
        #                                                                                                                 on the plate's outer 0.3 along the pit's walls, drawn as 'spike' lines at the
        #                                                                                                                 back corners, Willy 08:56). (Capped by the tilted hull itself, the
        #                                                                                                        plateau's step at the zone's edge -- vertical along y, leaning over the ring along x -- was kept by
        #                                                                                                        OCC as a zero-thickness sheet from the flat top up to the tilted plateau: the 'hairline slivers' since v4)
        hull += prism(self.outer_zone(1.0, out=0.6), ZB, 0.0)   # THE OUTER WALL GROWN 0.6 OUT along the tilted zone, to the plate's top: turned, the wall below the pivot swings
        #                                                       back and on the 5 deg yawed wall that read 0.1-0.3 INSIDE the outline (scratchpad wall_depth, build 113); the clip below
        #                                                       lands the grown wall exactly ON it (Willy 10:58 'the key plate section seems to be adding some weirdness to the wall')
        keep_out = unary_union([keep, keep.buffer(0.3, join_style=2).difference(plan)])              # the flat body's tool 0.3 OUTSIDE the outline: with its walls ON the hull's, OCC's
        flat_part = hull & prism(keep_out, ZB - 1.0, 200.0)                                           # common came back EMPTY, no error (build 120, R: the back chain's opened corners;
        v_hull = hull.volume                                                                           # the empty flat body surfaced 5 steps later as a plug tunnel that bored no wall)
        assert flat_part.volume > 0.3 * v_hull, f"{side}: the flat body's common came back with {flat_part.volume:.0f} of {v_hull:.0f} mm3"
        hull = (T * ((hull & behind(yp)) - cut) & above_zb) + flat_part + (skirt - Zp) + fill   # the zone is cut from the body BEFORE the tilt: the rotation
        #                                        pulls every point toward the pivot by (1 - cos 10) d = 2 mm at the back (and a yawed wall 0.2-0.4 sideways), so the tilted plateau inside the
        #                                        zone lands past the zone's footprint, where a vertical cut after the tilt missed it -- 2 mm wedges of tilted plateau 10 mm over the seam wall
        #                                        along every edge of the zone, builds 59-67 (proved term by term in scratchpad tilt_terms.py). The ring fills the 2 mm the body moved away.
        hull &= prism(plan, ZB - 1.0, 200.0)   # the tilted body LEANS OUT of the outline: its walls below the pivot plane swing back (0.17/mm, 4 at ZB) and the yawed outer wall
        #                                        drifts out 0.2 at the back; the flat zone's vertical cut stops at the outline, so lips stood on the flat walls' outer faces (0.35 x 5 along
        #                                        the display bay's back, 128 mm3) and a 0.2 skin 13 tall along the Pico bay's wall (47.7 tall hulls, builds 59-61; in 58 it was the
        #                                        detached 43.9 mm3 solid finish() dropped every build). Nothing of the hull belongs outside the outline before the hinges.
        out["flat"] = Z
        hull -= prism(hatch_plan, ZB - 1, FLOOR)                                                    # the hatch bed, flat
        debris = [q for q in hull.solids() if q.volume < 0.5]                                       # BOOLEAN DEBRIS: zero-volume sheets OCC leaves in the CUT_OFF moat (build 120, R: a
        if debris:                                                                                  # 0.2 x 1.5 x 8 sheet at the key block's back edge, 0.03 mm3, from an outline change 20 mm
            print(f"   {side} TILT: {len(debris)} boolean slivers dropped {[(round(q.volume, 3), [round(v, 1) for v in (q.bounding_box().min.X, q.bounding_box().min.Y, q.bounding_box().min.Z)]) for q in debris]}", flush=True)
            hull = Compound([q for q in hull.solids() if q.volume >= 0.5])                          # away; with it in the compound the plug tunnel's bore skipped the wall)
        out["T"], out["yp"] = T, yp
        lap("tilt")
        self.T = T
        self.Z = Z
        self.hull = hull
        self.yp = yp

    def deck_z(self, y):
        """The tilted deck's top at y (plan.py's dy_up formula; build 143): (y - yp) sin a + RIM_Z (1 - cos a) - TILT_DROP."""
        a = math.radians(KEY_TILT)
        return (y - self.yp) * math.sin(a) + RIM_Z * (1.0 - math.cos(a)) - TILT_DROP

    def deck_y(self, z):
        """The y where the tilted deck's top reaches z."""
        a = math.radians(KEY_TILT)
        return self.yp + (z - RIM_Z * (1.0 - math.cos(a)) + TILT_DROP) / math.sin(a)

    def well_cap(self, p):
        """THE KEYWELL'S TOP as one sheet of three planes (build 148): under the border's level (the rest plane + BORDER_UP) behind, the caps' ceiling
        (CAP_TOP_T - KEY_PROUD over the tilted plate: KEY_PROUD under every cap top, Willy 21:50) in front, their 5 deg crease at y_c replaced by a
        CREASE_R tangent arc -- a plane-min's crease split the run's fan chords under 0.4 and the key wall's chain refused (dry 148); an OCC fillet
        of the crease needs the fill wider than its band (0.7 at t1), this y-z profile does not. Extruded across the half; KEYWELL_MAX stays a plane."""
        yc, zc, _, (yf, yb), (a, sl), (ym, zm) = well_profile(p)
        y0, y1 = -300.0, 300.0
        zf, zb = zc - (yc - yf) * a, zc + (yb - yc) * sl
        face = make_face(Polyline((0.0, y0, -200.0), (0.0, y0, zc - (yc - y0) * a), (0.0, yf, zf), (0.0, yb, zb), (0.0, y1, zc + (y1 - yc) * sl), (0.0, y1, -200.0), close=True)).faces()[0]
        #                                                                 (the CHORD: a cylinder of R >= 150 on the fill's top made every fuse hand back the fill alone)
        print(f"   {self.side} WELL CAP: the caps' ceiling meets the border's level at y {yc:.2f} (z {zc:.2f}); the R {CREASE_R} arc's chord from y {yf:.1f} to {yb:.1f} (bends {math.degrees(math.atan(a) - math.atan((zb - zf) / (yb - yf))):.1f} / {math.degrees(math.atan((zb - zf) / (yb - yf)) - math.atan(sl)):.1f} deg)", flush=True)
        return extrude(face, amount=300.0, both=True)

    def under_t(self, z):
        """The half-space under the tilted deck plane z' = z (a method, not tilt's local lambda: the step cache pickles the state)."""
        return self.T * Pos(0, 0, z) * Box(BIGB, BIGB, BIGB, align=(Align.CENTER, Align.CENTER, Align.MAX))

    def over_t(self, z):
        """The half-space over the tilted deck plane z' = z."""
        return self.T * Pos(0, 0, z) * Box(BIGB, BIGB, BIGB, align=(Align.CENTER, Align.CENTER, Align.MIN))

    def swell_b(self, dz=0.0, up=False):
        """B of THE SWELL, THE ROLL (build 139; builds 132-138: a plane rising SWELL_FALL_LEGACY): a cylinder of plan.swell_R tangent to the rim dz over RIM_Z at x_b
        (where builds 132-138's plane had its foot), rising toward the outer wall -- the half-space under it (the box less the cylinder) or over it (the cylinder,
        400 long: with a 3000 one the hull came out with a face the MESHER refused -- top_map's tessellate None, scratchpad swell138 C)."""
        cyl = Pos(self.p["x_b"], 0, RIM_Z + dz + self.p["swell_R"]) * Rot(90, 0, 0) * Cylinder(self.p["swell_R"], 400.0, align=MID)
        return cyl if up else Box(BIGB, BIGB, BIGB, align=MID) - cyl

    def outer_zone(self, w, out=0.0):
        """The outline band w deep (and `out` past the outline) along the tilted zone's OUTER WALL: the only outline the tilted zone touches (the back
        band is flat zone, the front the palm side, the inner side the seam ring / the console)."""
        plan = self.plan
        band = plan.buffer(out, join_style=2).difference(plan.buffer(-w, join_style=2))
        z = band.intersection(self.p["tilted"].buffer(0.5, join_style=2))
        return max(getattr(z, "geoms", [z]), key=lambda g: g.area)   # the run itself: the zone's corners graze the outline elsewhere (L: 4 + 7 mm2 at the pit's corners)

    def lip_round(self):
        """THE LIP'S ROUND (Willy 14:16 'the lip coming off the wrist rests going to the rest of the board offers nice opp for smoothness'): the
        border's r on the lip's outer top edge -- plan.lip, the rest's rim run on along the key block's outer wall, filled by deck_edge to the
        border's level (capped RIM_Z: flush into the back band) -- so the rim's round runs from the rest along the keys in one line, between the rest's round and
        the back's plateau (avoid). The tilted plate's outer edge is under the lip now: the WALL ROUND of builds 114-116 (r 2 on that edge; the
        wall itself flat: key_deck leaves the outer run unrounded, tilt grows the wall 0.6 out before turning it) would have cut a groove under it."""
        p = self.p
        zone = unary_union([p["lip"], p["wall_run"].buffer(3.0, cap_style=2)])
        if SWELL:                                                                                  # (SWELL off, build 139: the whole outer run, flush into the plateau's back band)
            zone = zone.intersection(sbox(-1000.0, -1000.0, 1000.0, lip_cross_y(p)))               # SOUTH of the crossover: the
        #                                                                 border's level is the lip's top (build 132: north of it the swell's, swell_round below)
        self.hull, _ = self.edge_round(self.hull, BORDER_R, zone, lambda dz, up: rest_plane(p, BORDER_UP + dz, up), cap=RIM_Z,
                                       avoid=(p["palm_side"], p["ring_up"]), tag="LIP ROUND")   # (build 127: ring_UP, not the whole ring -- it overlaps the lip y 2..9 at the corner)
        self.hull = deghost(self.hull, self.side, "LIP ROUND")                                  # (the whole outer run, build 139: a 5-face zero-volume shell at the P corner's outer wall, (174.6, 7.4): dropped)
        if SWELL:
            self.swell_round()

    def swell_round(self):
        """THE SWELL'S ROUND (build 132): BORDER_R on the outline's top edge from the lip's crossover (where plane A rises through the border's level) up
        the lip, round the corner arc and along the back run toward the cell bay. The keep is the back chain's TRUE-ARC prism (backline.chain_prism)
        under planes A and B: OCC refused r 1.8 on every chain over the plan's 1 mm chords with a sloped top, and the union keep (the border's level
        OR the swell) broke the fillet on even a single edge far away. Its run edges are filleted as the LONGEST chain OCC accepts from the lip's
        end northward (a chain ending on a tangent junction it cannot close fails: R takes the whole run to the cell bay's corner, L stops short),
        and the band swap covers that stretch alone; beyond it the plateau's own round stays (plane B is within 0.2 of the rim there)."""
        p, plan, r = self.p, self.plan, BORDER_R
        yx = lip_cross_y(p)
        ybox = Pos(0, yx - 6.0, 0) * Box(BIGB, BIGB, BIGB, align=(Align.CENTER, Align.MIN, Align.CENTER))
        def keep_of(dx):                                                                           # the keep clipped to the run's neighbourhood: x >= the cell bay's back-outer
            xbox = Pos(p["foot"]["cell"].bounds[2] - dx, 0, 0) * Box(BIGB, BIGB, BIGB, align=(Align.MIN, Align.CENTER, Align.CENTER))   # corner - dx. With THE SHOULDER's ogee
            sol = chain_prism(plan, p["back_chain"], ZB - 1.0, RIM_Z + 30.0) & self.under_t(LIP_UP) & self.swell_b() & ybox & xbox   # 70 mm west in the keep's outline OCC refused
            return max(sol.solids(), key=lambda s: s.volume)                                       # r 1.8 on EVERY chain, both orders (L, build 134); clipped, 13 of 13 (scratchpad diag134_L)
        x0 = p["foot"]["cell"].bounds[2] + 1.2                                                     # the run reaches the cell bay's back-outer corner: OCC closes the chain there (R), not mid-wall
        zone = unary_union([p["lip"], p["wall_run"].buffer(3.0, cap_style=2), plan.exterior.buffer(4.0).intersection(sbox(x0, -1000.0, 1000.0, 1000.0))]).intersection(sbox(-1000.0, yx, 1000.0, 1000.0))
        run = plan.exterior.intersection(zone.buffer(0.1, join_style=2))
        run = max(getattr(run, "geoms", [run]), key=lambda g: g.length)
        if run.coords[0][1] > run.coords[-1][1]:
            run = LineString(run.coords[::-1])                                                   # south (the lip) to north (the cell bay)
        near = run.buffer(0.3)
        pts = lambda e: [e.position_at(t) for t in (0.1, 0.3, 0.5, 0.7, 0.9)]
        for dx in (10.0, 8.0, 12.0, 6.0, 14.0):                                                   # THE CREASE (build 135): R's chain crosses the swell's ridge (plane A to plane B on the
            sol = keep_of(dx)                                                                      # outline at (168.6, 19.1)) and OCC's fillet stopped THERE -- 3 of 9 edges, the rest of the
            #                                                                                      run square under a 2.8 trench (build 134, tests: lip 6.92 vs 9.79). The same keep clipped
            #                                                                                      2 mm either way fillets 9 of 9 (scratchpad det135c: -8 / -12 / -14 / -6 pass on R, -14
            #                                                                                      refuses on L, as built passes on L): a knife edge, so the clip is retried until the chain
            #                                                                                      covers the run. Builds 132-133 passed as built; the ogee 70 mm west flipped it.
            chain = [e for f in sol.faces() if f.normal_at().Z > 0.9 for e in f.edges()
                     if sum(near.contains(Point(q.X, q.Y)) for q in pts(e)) >= 3 and plan.exterior.distance(Point(e.position_at(0.5).X, e.position_at(0.5).Y)) < 0.5]
            chain.sort(key=lambda e: run.project(Point(e.position_at(0.5).X, e.position_at(0.5).Y)))
            k, n, order = None, len(chain), "north to south"
            while n >= 3 and k is None:                                                                # the longest chain OCC accepts, in either order: MakeFillet's result
                for order, edges in (("north to south", chain[:n][::-1]), ("south to north", chain[:n])):   # depends on the order the edges are added (L: the same 9 edges
                    try:                                                                               # fail south-first, pass north-first) and on where the chain ends
                        k = sol.fillet(r, edges)
                        if k.is_valid:
                            break
                    except ValueError:
                        pass
                    k = None
                if k is None:
                    n -= 1
            s_end = max(run.project(Point(e.position_at(t).X, e.position_at(t).Y)) for e in chain[:n] for t in (0.0, 1.0)) if k is not None else 0.0
            if k is not None and s_end > 0.8 * run.length:
                break
            print(f"   {self.side} SWELL ROUND: the keep clipped at the cell bay - {dx:.0f} fillets {n if k is not None else 0} of {len(chain)} run edges ({s_end:.0f} of {run.length:.0f} mm): another clip", flush=True)
        assert k is not None, f"{self.side}: SWELL ROUND: OCC refuses r {r} on every chain from the lip's end ({len(chain)} edges on the run)"
        s_end = max(run.project(Point(e.position_at(t).X, e.position_at(t).Y)) for e in chain[:n] for t in (0.0, 1.0))
        q_end = run.interpolate(s_end)
        keep_r = k & prism(plan, ZB - 2.0, RIM_Z + 31.0)
        strip = substring(run, 0.0, s_end).buffer(r + 1.5, join_style=1).intersection(zone.buffer(CUT_OFF, join_style=2)).intersection(sbox(p["x_b"] - 1.0, -1000.0, 1000.0, 1000.0))   # from 0.5 before the swell's FILL (plan.swell: x_b - 0.5, its outside strip 0.3 further; build 138 at x_b - 0.5 left L a 1.1
        #                                                                 notch and R a 0.6 sliver; 137 at x_b - 0.3 a 1.9 shard: Willy 21:34); the roll is within 0.01 of the rim there (build 139)
        band = prism(plan.buffer(1.0, join_style=1), ZB, RIM_Z + 5.0) - prism(plan.buffer(-(r + 0.5), join_style=1, quad_segs=6), ZB - 1.0, RIM_Z + 6.0)
        band = band & prism_holed(strip, ZB - 1.0, RIM_Z + 6.0) & (self.over_t(LIP_UP - r - 1.0) + Pos(0, 0, RIM_Z - r - 1.0) * Box(BIGB, BIGB, BIGB, align=(Align.CENTER, Align.CENTER, Align.MIN)))
        #                                                               (the band's floor under the roll: a FLAT plane r + 1 under the rim, not the roll lowered r + 1 -- the swap left five
        #                                                                0.003 mm2 two-edged slivers of that floor on the wall, CYLINDRICAL ones the mesher refuses (planar ones mesh), build 139)
        v0 = self.hull.volume
        self.hull = deghost((self.hull - band) + (keep_r & band), self.side, "SWELL ROUND")
        assert self.hull.is_valid and len(self.hull.solids()) == 1, f"{self.side}: SWELL ROUND left the hull {'invalid' if not self.hull.is_valid else str(len(self.hull.solids())) + ' solids'} ({[(round(f.center().X, 1), round(f.center().Y, 1), round(f.center().Z, 2)) for f in self.hull.faces() if not f.is_valid][:6]})"
        full = 0.2146 * r * r * run.length                                                    # what a full r round takes off a square edge over the run
        assert s_end > 0.8 * run.length, f"{self.side}: SWELL ROUND: OCC accepted only {s_end:.0f} of {run.length:.0f} mm of the run ({n} of {len(chain)} edges): the rest would stay square (build 134 R: 27 of 70 with a mis-clipped keep)"
        assert v0 - self.hull.volume < 2.5 * full, f"{self.side}: SWELL ROUND took {v0 - self.hull.volume:.0f} mm3 off the edge (a full round is {full:.0f}): the swap left a trench (build 134 R: 186 vs 39)"
        b_end = roll_z(p, q_end.x) - RIM_Z                 # the roll over the rim at the chain's end (plane A far below at y -1000: the roll alone)
        print(f"   {self.side} SWELL ROUND r {r}: {n} of {len(chain)} run edges ({order}), {s_end:.0f} of {run.length:.0f} mm from the crossover y {yx:.1f} to ({q_end.x:.1f}, {q_end.y:.1f}) (plane B {b_end:+.2f} over the rim there); {v0 - self.hull.volume:.0f} mm3 off the edge ({len(self.hull.solids())} solid(s))")

    def deck_edge(self):
        """The palm deck's edge at the PIT: solid from the tilted plate up into the skin DECK_FILL in from the edge, so the deck's face there is one
        plain vertical wall flush with the plate's edge (no ramp: Willy 10:13; the 50 deg planes of builds 73-78 and the staircase of 71-72 are
        gone). Not under the Pico bay's pocket."""
        p, hull, T = self.p, self.hull, self.T
        over_t = lambda z: T * Pos(0, 0, z) * Box(BIGB, BIGB, BIGB, align=(Align.CENTER, Align.CENTER, Align.MIN))   # over the tilted plane z' = z
        fill = p["pit"].buffer(DECK_FILL, join_style=2).intersection(p["palm_side"]).difference(p["foot"]["pico"].buffer(0.3, join_style=2)).simplify(0.2)
        hull += prism(fill, FLOOR, RIM_Z) & over_t(-0.01) & rest_plane(p, -TOP - PAD_DEPTH + 0.5, up=False)   # solid from the tilted plate up into the skin
        # THE BORDER'S FOOT RECEDES (build 148): in front of the crease the fill stands under the border, and the round's footprint on the fill's top
        # reaches r tan(dihedral / 2) <= 2.14 from the wall's edge while the fill is 1.0-1.6 wide at the fans (width148) -- the round ran into the
        # border's step and refused; rolled under its lip instead, the lip left 0.03 slivers no mesher takes. So wherever the border stood nearer than
        # EDGE_R_TOP + 0.5 to the open pit's edge along the run, its top within that reach comes down to the fill's profile: a wedge under the border's
        # level that dies out at the chord's end (y_b, 3 deg), cut while the pit is still empty (the tool's bottom and inner wall stand in air, 0.25 in).
        o = p["open_pit"]
        foot = o.exterior.buffer(EDGE_R_TOP + 0.5, join_style=1).intersection(unary_union([p["palm_side"], p["pit"]])).difference(p["pit"].buffer(-0.25, join_style=2))
        foot = foot.difference(p["lip"].buffer(0.5, join_style=2)).difference(p["flat"]).difference(p["housing"])
        if p["tp_zone"] is not None:
            foot = foot.difference(p["tp_zone"].buffer(WALL + 2.0, join_style=2))
        tool = (prism(foot, FLOOR, RIM_Z + 5.0) - self.well_cap(p)) & rest_plane(p, BORDER_UP + 0.3, up=False)
        bz = foot.difference(p["pit"]); b = bz.bounds
        want = sum(max(0.0, rest_z(p, y) + BORDER_UP - well_top(p, y)) * 0.0625 for x in [b[0] + 0.125 + 0.25 * i for i in range(int((b[2] - b[0]) / 0.25) + 1)]
                   for y in [b[1] + 0.125 + 0.25 * j for j in range(int((b[3] - b[1]) / 0.25) + 1)] if bz.contains(Point(x, y))) if not bz.is_empty else 0.0
        v0 = hull.volume
        hull = hull - tool
        got = v0 - hull.volume
        assert hull.is_valid and len(hull.solids()) == 1 and abs(got - want) < 0.35 * want + 3.0, f"{self.side} BORDER FOOT: the cut took {got:.1f} mm3, the wedge is {want:.1f}; valid {hull.is_valid}, {len(hull.solids())} solid(s)"
        print(f"   {self.side} BORDER FOOT: the border's top within {EDGE_R_TOP + 0.5} of the open pit's edge down to the fill's profile where it stood over it: {got:.0f} mm3 (the wedge {want:.0f}) over {bz.area:.0f} mm2", flush=True)
        well = p["keywell"]                                            # THE KEYWELL (Willy 22:46 'this deep channel'): the block's wall top between the deck
        hull += prism_holed(well, FLOOR, RIM_Z) & over_t(-0.01) & self.well_cap(p) & Pos(0, 0, KEYWELL_MAX) * Box(4000.0, 4000.0, 400.0, align=(Align.CENTER, Align.CENTER, Align.MAX)) & self.rest_keep   # face and the caps, from the tilted
        #                                                                                   (& rest_keep: the border's round runs on over the strip at the outline) plate up to the border's level (capped at KEYWELL_MAX)
        # THE LIP RUNS THROUGH TO THE BACK (Willy 16:03 'does the lip not continue straight through to the back side? fix it'): capped at KEYWELL_MAX
        # like the rest of the keywell it stopped where the tilted plate rose through 7.7, 4 mm short of the back band, and the plate's square edge
        # ran the last stretch (top_map R x 170-176 y 0-14, build 117). The lip's own fill is capped at RIM_Z instead: the border's plane reaches
        # the rim at y 4 and the lip meets the back band flush -- no step, no chamfer there (the outer back corner never had one)
        hull += prism_holed(p["lip"], FLOOR, RIM_Z) & over_t(-0.01) & rest_plane(p, BORDER_UP, up=False) & Pos(0, 0, RIM_Z) * Box(4000.0, 4000.0, 400.0, align=(Align.CENTER, Align.CENTER, Align.MAX)) & self.rest_keep
        print(f"   KEYWELL: {well.area:.0f} mm2 of the pit's margin filled to the border's level (the rest plane + {BORDER_UP}), {CAP_CLR} off the caps; THE LIP {p['lip'].area:.0f} mm2 of it along the outer wall, to the rim")
        # THE SWELL (build 132; Willy 12:37 'it looks like the p key and q are not sitting in secure slots', 13:22 '2. lip follows the plate'): the lip's
        # top stopped at the rim while the plate climbs 10 deg to the back -- beside P / Q it was 4.3 .. 2.6 over the plate, 1.2 behind them, and the
        # pinky stagger left 10 mm of bare plate behind the key: the caps stood 6-8.6 proud of the case's edge (scratchpad pq_wall / corner_map). Now
        # the lip, the stagger's notch and the plateau's corner (plan.swell) are filled from the tilted plate (the plateau's part: from 2 under the rim)
        # up to the LOWER of two planes: A, LIP_UP over the plate (the lip follows the plate), and B, THE ROLL: a cylinder tangent to the rim at x_b rising to A (build 139; a plane
        # rising 11.2 deg in 132-138: a crease at the foot) -- one crease between them, the corner ~3 over the rim; lip_round re-rounds the whole run
        if not SWELL:                                                                              # THE SWELL OFF (build 139; Willy 22:36): the corner's rim stays at RIM_Z
            print(f"   NO SWELL: the outer-back corner's rim at {RIM_Z} (the lip the border's level capped at the rim along the whole outer run)")
            # THE RIM STRIP (build 140; Willy 23:14 'this ugly block was exposed with that change fix it - i love the curve now'): the plateau's corner
            # behind P / Q and the lip's tab in the stagger's notch (plan.corner: the flat + the lip beyond the STRIP_W strip inside the outline, north of
            # P) stood 1 .. 2 over the tilted plate once the swell's fill was gone (the tab 1.6 x 3.3 x 2; the corner's wall 1.7 at the lip's end tapering to
            # 0 at the cell bay: top_map / pblock140). Down to the plate, 0.05 over it (no cut face ON the deck's): the rim runs round the corner as the
            # strip, its inner wall the lip's running on and dying out into the back band where the plate rises through the rim. The tab is not filled
            # (plan.lip / keywell leave `corner` out); the flat's part is what the cut takes, measured on the piece's MESH (the hull's GProp drifts)
            cz = p["corner"].intersection(p["flat"]); b = cz.bounds
            want = sum(max(0.0, RIM_Z - plate_z(p, y) - 0.05) * 0.25 for x in [b[0] + 0.25 + 0.5 * i for i in range(int((b[2] - b[0]) / 0.5) + 1)]
                       for y in [b[1] + 0.25 + 0.5 * j for j in range(int((b[3] - b[1]) / 0.5) + 1)] if cz.contains(Point(x, y)))
            tool = prism(p["corner"].buffer(0.05, join_style=2), FLOOR, RIM_Z + 2.0) & over_t(0.05)
            piece = hull & tool
            got = mesh_volume(piece) if piece and piece.solids() else 0.0
            hull -= tool
            assert 0.7 * want < got < 1.3 * want + 1.0, f"{self.side}: the rim strip's cut takes {got:.1f} mm3 of the corner, the plate's plane says {want:.1f}"
            print(f"   RIM STRIP {STRIP_W} wide round the outer-back corner: the plateau's corner ({cz.area:.0f} mm2) + the lip's tab ({p['corner'].area - cz.area:.0f}) down to the tilted plate, {got:.0f} mm3 off (the plane's {want:.0f})", flush=True)
            self.hull = hull
            return
        a, b = self.under_t(LIP_UP), self.swell_b()
        v0 = hull.volume
        hull += prism_holed(p["swell_pit"], FLOOR, RIM_Z + 20.0) & over_t(-0.01) & a & b
        clip = p["swell"].buffer(0.3, join_style=2)                                                 # the plateau's part: its wall CUT_OFF OUTSIDE the outline, not ON the hull's wall --
        ext = self.plan.buffer(CUT_OFF, join_style=2).difference(self.plan).intersection(clip)     # there the plateau's round met the piece's wall at its tangent line z RIM_Z - BORDER_R
        hull += prism_holed(unary_union([p["swell"], ext]), FLOOR, RIM_Z + 20.0) & Pos(0, 0, RIM_Z - 2.0) * Box(BIGB, BIGB, BIGB, align=(Align.CENTER, Align.CENTER, Align.MIN)) & a & b
        v1 = hull.volume                                                                           # and L's fuse left a 1 x 0.002 sliver face on one chord (135.3, 19.5, 8.10), the hull
        hull -= prism(self.plan.buffer(1.0, join_style=2).difference(self.plan).intersection(clip), RIM_Z - 2.0, RIM_Z + 25.0)   # invalid (build 132); the strip outside trimmed back by planes
        assert hull.is_valid and 1.0 < v1 - hull.volume < 10.0 * ext.area, f"{self.side}: THE SWELL's outside strip: {v1 - hull.volume:.1f} mm3 trimmed over {ext.area:.1f} mm2, valid {hull.is_valid}"
        print(f"   THE SWELL: +{hull.volume - v0:.0f} mm3 over {p['swell'].area:.0f} mm2 (on the plate {p['swell_pit'].area:.0f}): LIP_UP {LIP_UP} over the plate under plane B, THE ROLL R {p['swell_R']:.0f} from the rim at x_b {p['x_b']:.1f}, {roll_z(p, self.plan.bounds[2]) - RIM_Z:.1f} over it at the outer wall (plane A caps it)")
        # FUSED TO THE PLATE (Willy 12:39 'leaves the key plate floating - they should be connected'): the plate's front face leans 10 deg away from the
        # deck's vertical face below the plate's top (0.52 at its underside): that wedge, the strip between the plate's shadow at its top and at its
        # underside, is filled from the underside plane up to the top -- the deck's face runs straight down onto the plate's underside; the wiring
        # gap under the plate's edge (2.0 over the hatch) is untouched
        under_t = lambda z: T * Pos(0, 0, z) * Box(BIGB, BIGB, BIGB, align=(Align.CENTER, Align.CENTER, Align.MAX))
        slit = tilt_shadow(p["tilted"], p["yp"], 0.0).difference(tilt_shadow(p["tilted"], p["yp"], -TOP)).buffer(0.02, join_style=2)
        slit = unary_union([g for g in getattr(slit, "geoms", [slit]) if g.area > 1.0 and g.buffer(-0.15, join_style=2).area > 0.3]).intersection(p["pit"].buffer(1.0))   # (> 0.3 wide: the
        #                                                                   yawed side edges' shadows shift too -- 0.05 x 8 mm sheets lying on the plate at its back corners, build 126)
        hull += prism(slit, FLOOR, RIM_Z) & over_t(-TOP - 0.01) & under_t(0.01)
        # THE JUNCTION PLUG (Willy 15:34 'where the tilted key plate piece fits into the rests on the side walls, seems to be a hole'): inside the OUTER
        # wall at the pit's outer-front corner the tilted body's front face leans away from the rest's wall end below the plate -- a slit 0.5 wide at
        # z -3 opening to 1.5 at -9, 6.5 deep, on both halves (scratchpad slit_map / wall_probe, build 116); the fuse above covers the pit's edge
        # only from the plate's underside up. The wall band (WALL - 0.3 in from the outline: the inner face inside the wall, not on the cavity's)
        # over yc - 2 .. yc + 5 is filled from over the base's bottom round up to the plate's top, in the flat frame; the hinge slots are cut later.
        xc, yc = p["pit_corner"]
        plug = self.plan.exterior.buffer(WALL - 0.3, join_style=2).intersection(self.plan).intersection(sbox(0.55 * self.plan.bounds[2], yc - 2.0, 1000.0, 25.0))   # ... and ON up the whole outer
        #                                                                 wall to the back corner (build 132): a slit void 0.3 x 0.7 x 17 sat inside the wall at the P corner (174.4, 8.1) between the rail's
        #                                                                 top and the plate (in build 131's hull too, 0.064 mm3) and the swell's booleans tripped over it -- the strip is solid wall now
        v0 = hull.volume
        hull += prism(plug, ZB + 2.0, RIM_Z) & under_t(-0.01)
        print(f"   DECK EDGE: {fill.area:.0f} mm2 solid under the deck's edge at the pit, {DECK_FILL} in; the plate's leaning front face fused over {slit.area:.0f} mm2; the junction plug +{hull.volume - v0:.0f} mm3 over {plug.area:.0f} mm2")
        self.hull = deghost(hull, self.side, "DECK EDGE")

    def wall_round(self):
        """THE KEY WALL'S ROUND (build 147; Willy 13:22 'use the same smooth round edges we have around the outside on the key wall also', 18:57
        'B!!'): EDGE_R_TOP on the keywell's top edge along THE RUN -- the open pit's line (plan.smooth_open: arcs of WALL_R at every concave corner,
        S bends over the staggers, no fill between caps, every near-straight bend dropped) where the wall faces the deck, the thumbs' bowl and
        L's inner side (plan.wall_zone), from the lip's corner to where the tilted plate rises within r + 1 of the keywell's ceiling -- ONE chain
        per part over 15 mm, its ends on edges over 3 (an end on a fan's 1 mm chord refuses). The fillet runs on THE HULL'S OWN BAND: the piece of
        the hull within r + 4 of the run, from r + 2 under the keywell's top (the lowest of its three ceilings: the rest plane + BORDER_UP, KEYWELL_MAX,
        the caps' top plane CAP_TOP_T - KEY_PROUD, build 148) down to 0.05 over the plate, cut out, rounded and put back -- no keep.
        A keep (the keywell as deck_edge fills it, widened over the palm deck's DECK_FILL band -- the bare keywell refused every chain -- cut to the
        run's neighbourhood) took the same chain, but its common with the hull's band came back EMPTY, plain or fuzzy, both orders (the wall and
        top faces coincide), and clipping it to the keywell's footprint left a 1 x 2.8 trench along the deck where the deck is flush with the
        keywell's top (178 mm3, the first dry build). A 5 deg bend on the strip's y-running stretch refused the chain under the sloped top at every
        radius while the same loop under a flat top took it: plan.drop_short drops such bends (scratchpad wallcheck147 a-n, swap147)."""
        p, hull, T, r = self.p, self.hull, self.T, EDGE_R_TOP
        over_t = lambda z: T * Pos(0, 0, z) * Box(BIGB, BIGB, BIGB, align=(Align.CENTER, Align.CENTER, Align.MIN))
        over = lambda z: Pos(0, 0, z) * Box(BIGB, BIGB, BIGB, align=(Align.CENTER, Align.CENTER, Align.MIN))
        o = p["open_pit"]
        h = lambda y: well_top(p, y) - plate_z(p, y)                                                  # the wall's height over the plate at y (its three ceilings)
        lo, hi = -100.0, 100.0
        for _ in range(40):
            mid = (lo + hi) / 2.0
            lo, hi = (mid, hi) if h(mid) > r + 1.0 else (lo, mid)
        run = o.boundary.intersection(p["wall_zone"]).difference(self.plan.exterior.buffer(BORDER_R + 1.5)).intersection(sbox(-1000.0, -1000.0, 1000.0, lo))
        parts = [g for g in getattr(run, "geoms", [run]) if g.length >= 15.0]
        seg = lambda e: LineString([(e.start_point().X, e.start_point().Y), (e.end_point().X, e.end_point().Y)])
        v0 = hull.volume
        for part in parts:
            zone = part.buffer(r + 4.0, join_style=1).intersection(self.plan.buffer(-(BORDER_R + 1.0), join_style=2))   # (never the outline's own round)
            #                                                 (& well_cap: the band stops at the fill's top profile -- in front of the crease the border stands over it, and
            #                                                 the fill beside the run is 1.0-1.6 wide at 122 of 217 samples (width148): the round's footprint ran into the
            #                                                 border's step and refused; so the piece's top is the one plane, the round rolls under the border's lip (<= 0.18
            #                                                 undercut at 1.0 wide) and the border's proud part stays in the hull, build 148)
            #                                                 (r + 2 under the LOWEST of the wall's three ceilings: over min(a, b, c) - d = over a - d or b - d or c - d)
            def pick(zone):                                                                        # the band, the hull's piece in it, THE WALL'S TOP EDGE along the run
                band = prism(zone, ZB, RIM_Z + 5.0) & (rest_plane(p, BORDER_UP - r - 2.0, up=True) + over(KEYWELL_MAX - r - 2.0) + over_t(CAP_TOP_T - KEY_PROUD - r - 2.0)) & over_t(0.05) & self.well_cap(p)
                piece = band & hull                                                             # THE BAND FIRST (build 149): the same piece as hull & band, but on a
                #                                                                                 FRESH hull the fillet refused r 1.5-1.8 on L's chain (every end cut,
                #                                                                                 every radius law, the chain cut short) and took it from band & hull or
                #                                                                                 from the piece re-read through BRepTools (the cache: the dry builds
                #                                                                                 passed, the golden refused): wallcheck149 d-f, the velvet-142 lesson
                assert piece is not None and len(piece.solids()) == 1, f"{self.side} WALL ROUND: the hull's band is {len(piece.solids()) if piece else None} solids"
                sol = max(piece.solids(), key=lambda s: s.volume)
                near = part.buffer(0.2)
                ek = lambda e: (round(e.center().X, 2), round(e.center().Y, 2), round(e.center().Z, 2), round(e.length, 2))
                tops, walls = {}, set()                                                             # shared by a top face and a vertical one
                for f in sol.faces():                                                             # (build 148: the crease arc's tangent lines lie on the fill's top
                    nz = f.normal_at().Z                                                            # 0.26 beside a near-parallel stretch of the run -- a top-faces-only
                    for e in f.edges():                                                             # pick took one twice and the chain refused)
                        if nz > 0.8:
                            tops.setdefault(ek(e), e)
                        elif abs(nz) < 0.3:
                            walls.add(ek(e))
                chain = [e for kk, e in tops.items() if kk in walls
                         and o.exterior.distance(Point(e.center().X, e.center().Y)) < 0.3 and seg(e).intersection(near).length > min(0.3, e.length / 2) and part.distance(Point(e.center().X, e.center().Y)) < 0.3]
                chain.sort(key=lambda e: o.exterior.project(Point(e.center().X, e.center().Y)))
                on_wall = lambda e: any(p[k].buffer(0.5).contains(Point(q.X, q.Y)) for k in ("housing", "flat") for q in (e.start_point(), e.end_point()))   # (build 150: a chain
                while chain and chain[0].length < 3.0 and not on_wall(chain[0]):                 #  end on a block's wall may be short -- the round dies on the wall's face; L's end
                    chain.pop(0)                                                                #  line runs 2.2 from the turn to the silo. The 3 mm rule is for ends on fan chords)
                while chain and chain[-1].length < 3.0 and not on_wall(chain[-1]):
                    chain.pop()
                return band, piece, sol, chain
            band, piece, sol, chain = pick(zone)
            try:
                k = sol.fillet(r, chain)
            except Exception:
                # THE CHAIN'S FREE END (build 149): where the run ends away from the lip -- L's ledge out of the silo ends 1.0 wide at the palm band's
                # chamfered corner -- the fillet's end found no face to die on (r 1.8 refused, 1.3 took: wallcheck149 a-f, fresh builds); the band's
                # zone is cut flush 0.05 before the end vertex, square to the end edge, so the round runs to a plane and ends on it (as it does
                # at the band's own boundary). Tried first without: the lip's end and R's console end take r 1.8 as they are (build 148).
                print(f"   {self.side} WALL ROUND: OCC refuses r {r} on the run's {len(chain)} edges -- the band cut flush at the chain's free end(s)", flush=True)
                lipside = self.plan.exterior.buffer(BORDER_R + 3.0)
                for e, nb in ((chain[0], chain[1]), (chain[-1], chain[-2])):
                    a, b, oc = e.start_point(), e.end_point(), nb.center()
                    E, F = (a, b) if (a - oc).length > (b - oc).length else (b, a)             # E: the end edge's outer vertex
                    if lipside.contains(Point(E.X, E.Y)):
                        continue
                    ux, uy = E.X - F.X, E.Y - F.Y; l = math.hypot(ux, uy); ux, uy = ux / l, uy / l
                    Ex, Ey = E.X - 0.05 * ux, E.Y - 0.05 * uy
                    zone = zone.difference(Polygon([(Ex - 500.0 * uy, Ey + 500.0 * ux), (Ex + 500.0 * uy, Ey - 500.0 * ux),
                                                    (Ex + 500.0 * uy + 1000.0 * ux, Ey - 500.0 * ux + 1000.0 * uy), (Ex - 500.0 * uy + 1000.0 * ux, Ey + 500.0 * ux + 1000.0 * uy)]))
                    print(f"   {self.side} WALL ROUND: the band ends square at ({Ex:.1f}, {Ey:.1f})", flush=True)
                band, piece, sol, chain = pick(zone)
                k = sol.fillet(r, chain)
            rr = r
            n = sum(e.length for e in chain)
            off = sol.volume - k.volume
            print(f"   {self.side} WALL ROUND r {rr}: {len(chain)} edges, {n:.0f} mm from ({part.coords[0][0]:.0f}, {part.coords[0][1]:.0f}) to ({part.coords[-1][0]:.0f}, {part.coords[-1][1]:.0f}); "
                  f"{off:.0f} mm3 off the band's piece (a full round {n * rr * rr * (1 - math.pi / 4):.0f}; the piece {sol.volume:.0f} of the band's {piece.volume:.0f})", flush=True)
            v1 = hull.volume
            hull = (hull - band) + k
            assert hull.is_valid and abs((v1 - hull.volume) - off) < 0.2 * off + 2.0, f"{self.side} WALL ROUND: the band swap took {v1 - hull.volume:.0f} mm3, the round {off:.0f}; valid {hull.is_valid}"
            print(f"   {self.side} WALL ROUND: {v1 - hull.volume:.0f} mm3 off the wall's edge ({len(hull.solids())} solid(s))", flush=True)
        self.hull = deghost(hull, self.side, "WALL ROUND")
        print(f"   WALL ROUND: {len(parts)} chain(s), {v0 - self.hull.volume:.0f} mm3 off; the run ends where the wall is {r + 1.0} tall, y {lo:.1f}", flush=True)

    def raise_bosses(self):
        """Each boss from the flat hatch up to the tilted deck / the rest's skin / the flat top; the insert holes."""
        Z, bosses, hull, p, yp, under_t = self.Z, self.bosses, self.hull, self.p, self.yp, self.under_t
        for b in bosses:                                            # from the flat hatch up to the (tilted) deck, or the rest's skin
            if p["foot"]["palm"].buffer(BOSS_D / 2).contains(b):
                boss = Pos(b.x, b.y, FLOOR) * Cylinder(BOSS_D / 2, RIM_Z - FLOOR, align=BIG) & rest_plane(p, -TOP - PAD_DEPTH + 0.01, up=False)
                hull += boss & (prism(p["palm_side"], FLOOR - 1.0, RIM_Z + 1.0) + under_t(-TOP + 0.5))   # past the deck's face (in the pit) 0.5 INTO the tilted plate from below: a boss 0.5
                #                                                                                   from the pit's edge stood 2.5 proud of the plate there, 7 wide, a hump at the deck face's foot (builds 82-88, sheets88)
            elif b.y < yp or Z.contains(b):
                hull += Pos(b.x, b.y, FLOOR) * Cylinder(BOSS_D / 2, -TOP - FLOOR + 0.01, align=BIG)
            else:
                hull += Pos(b.x, b.y, FLOOR) * Cylinder(BOSS_D / 2, 120.0, align=BIG) & under_t(-TOP + 0.5)   # 0.5 into the plate (a 0.01 overlap under the keywell's fill left
                #                                                                                   L's two pit bosses with a top the mesher dropped: pink rings in build 102)
            hull -= Pos(b.x, b.y, FLOOR - 0.02) * Cylinder(INSERT_D / 2, INSERT_L, align=BIG)
        self.hull = hull

    def hinges(self):
        """The tent hinge (rail, knuckles, bore, head room), the rest hinge, the charging windows, the charger bay re-cut flat, the rounded bottom edge."""
        Kf, ax_x, hull, inner, lap, out, p, pk, plan, side, vb, z_ax = self.Kf, self.ax_x, self.hull, self.inner, self.lap, self.out, self.p, self.pk, self.plan, self.side, self.vb, self.z_ax
        hull += prism(p["rail"], ZB, ZB + RAIL_H)
        for y in p["base_kn"]:
            hull -= Kf * Pos(ax_x, y + KNUCKLE / 2, z_ax) * Rot(90, 0, 0) * Cylinder(TENT_R + TENT_CLR, KNUCKLE + KN_CLR, align=MID)
        for y in p["hull_kn"]:
            hull += Kf * Pos(ax_x, y + KNUCKLE / 2, z_ax) * Rot(90, 0, 0) * Cylinder(TENT_R, KNUCKLE, align=MID)
        rs = p["rslots"]
        r0, r1 = rs[0], rs[-1] + KNUCKLE
        hull += prism(p["rest_rail"], ZB, ZB + RAIL_H)                                            # the rest hinge's rail (the hatch stops short of it, as at the tent hinge)
        for y in (rs[0], rs[2]):
            hull += Kf * Pos(ax_x, y + KNUCKLE / 2, z_ax) * Rot(90, 0, 0) * Cylinder(TENT_R, KNUCKLE, align=MID)
        hull -= Kf * Pos(ax_x, rs[1] + KNUCKLE / 2, z_ax) * Rot(90, 0, 0) * Cylinder(TENT_R + TENT_CLR, KNUCKLE + KN_CLR, align=MID)
        seat_t, seat_r = back_at_axis(p) - PIN_HEAD[1], REST_FRONT + PIN_HEAD[1]                    # THE HEADS (build 131, Willy 08:39 'the caps are capping before they reach the end of the
        caps = {"tpin": hull & (Kf * head_env(seat_t, 1, ax_x, z_ax)),                                 # hinge'): each pin's head is the case's OWN material inside a D-cut Ø PIN_HEAD[0] envelope from the
                "rpin": hull & (Kf * head_env(seat_r, -1, ax_x, z_ax))}                               # seat (PIN_HEAD[1] inside the end face) out to the surface -- the hull's part here (taken BEFORE
        self.out["pin_caps"] = caps                                                                    # the bores: a bore through it would leave a tube), the base's in build_base -- so its end IS the
        #                                                                                                corner's form (the r 4 arc, the bottom round), flush. The rooms below: the envelope + 0.2; the D
        #                                                                                                keys the head, the snap ring holds it. (Before: the tent pin's head 4.5 inside the back face, the
        #                                                                                                rest pin's 2.5 inside the front, each a stub in a hole through the corner's round)
        # THE PIN BORES, after every knuckle is on (build 125 cut the through bore BEFORE the rest's knuckles went on: 28 mm of solid knuckle on the axis, the rest pin unfit)
        ya, yb = p["y0"] - 0.5, p["y1"] + ROUND + 1.0                                                   # THE TENT PIN'S BORE: from outside the back face to 0.5 past its nose
        hull -= Kf * Pos(ax_x, (ya + yb) / 2, z_ax) * Rot(90, 0, 0) * Cylinder(PIN_BORE / 2, yb - ya, align=MID)
        ya, yb = REST_FRONT - 1.0, r1                                                                    # THE REST PIN'S: from outside the front face to 0.5 past its nose (r1 - 0.5)
        hull -= Kf * Pos(ax_x, (ya + yb) / 2, z_ax) * Rot(90, 0, 0) * Cylinder(PIN_BORE / 2, yb - ya, align=MID)
        #   (build 125-126 bored the whole axis so a rod from either end pushes the other pin out -- through the rail between the hinges, 5 tall under
        #    the 9 tall plug tunnel: the Ø 5.35 channel broke into the tunnel's floor, a trench with a 2.7 lip (Willy 08:38); the pins come out by their heads)
        hull -= Kf * head_env(seat_t, 1, ax_x, z_ax, clr=0.2)                                        # the tent pin's head room, from the BACK face: the envelope + 0.2, D-keyed
        hull -= Kf * ring_relief(p["y0"] + 0.5, -1, ax_x, z_ax)                                                 # the tent pin's snap ring clicks into the first hull knuckle (its nose ends 0.5 inside it)
        hull -= Kf * ring_relief(r1 - 0.5, 1, ax_x, z_ax)                                                         # the rest pin's snap ring clicks into the last rest knuckle
        hull -= Kf * head_env(seat_r, -1, ax_x, z_ax, clr=0.2)                                       # the rest pin's room, from the FRONT face
        for y_face, sense in ((back_at_axis(p), 1), (REST_FRONT, -1)):                                   # THE PULL NOTCHES (build 132, Willy 12:37 'your rec is fine'): a Ø PIN_NOTCH[0]
            hull -= Kf * Pos(ax_x, y_face - sense * (PIN_NOTCH[1] - 4.0) / 2, z_ax + PIN_HEAD[0] / 2 + 0.2) * Rot(90, 0, 0) * Cylinder(PIN_NOTCH[0] / 2, PIN_NOTCH[1] + 4.0, align=MID)   # half-moon on the
        #                                                                                                       room's ceiling above each head's rim, PIN_NOTCH[1] into the end face, 4 clear outside it
        if side == "R":
            ux, uy, uz = out["pi_usb"]                               # THE PI'S OTG OVAL (build 155's door dropped in 161, Willy 21:02 'drop this door - forget about it -
            wy = pk.bounds[3] + WIRE + WALL / 2                      # its not needed'): a PI_OVAL hole centred on the port through the back wall (the pocket + the wire bay
            Wf = Pos(ux, wy + WALL / 2, uz) * Rot(0, 0, 180)         # are the plug's way in); a full oval now the port sits 9 over the hatch (155's arch was clipped at it)
            v0 = hull.volume
            hull = fuzzy_cut(hull, Wf * wprism(ellipse(*PI_OVAL), -1.0, WALL + 1.0))
            assert v0 - hull.volume > 0.7 * math.pi * PI_OVAL[0] * PI_OVAL[1] / 4 * WALL, f"R: the OTG oval bored only {v0 - hull.volume:.0f} mm3"
            cover = oval_cover(*PI_OVAL)                                # THE OTG COVER (162): a friction oval plug + a low brim (parts.oval_cover), +y out on the back face
            placed_cover = Pos(ux, wy, uz) * cover
            crush = 4 * RIB_W * (WALL - 0.5) * (RIB_H - COVER_CLR)
            assert 0.5 * crush < _hit(placed_cover, hull) < 1.5 * crush, f"R: the OTG cover's ribs press {_hit(placed_cover, hull):.2f} mm3 into the wall (four ribs: {crush:.2f})"
            out.update(usb_win=(ux, wy, uz), pi_door=None, pi_Wf=Wf, pi_cover=placed_cover, pi_cover_local=cover)
            print(f"   R OTG COVER: oval {PI_OVAL[0]} x {PI_OVAL[1]} plug {WALL} deep, brim {PI_COVER[0]} x {PI_COVER[1]}, ribs press {_hit(placed_cover, hull):.2f} mm3 (four: {crush:.2f}); {placed_cover.volume:.0f} mm3")
        else:
            pass
        hull -= prism(p["v713"].buffer(V713_FENCE + 0.2, join_style=2), FLOOR - 0.1, FLOOR + V713_PAD + V713[2] + V713_PCB + 2.0)   # the charger's bay again, flat (the tilt carried the old cut up)
        # THE CHARGING DOORS (162; Willy 22:04 'raise the charging holes a little ... and tambour those doors'): the window is a PICO_HOLE with the Pico's two-leaf
        # slide door behind it (doors.door_bypass, CHG_SPEC), centred V713_USB_Z over the charger's underside; R on the back face (W x = -u: the leaves slide
        # east, clear of the OTG oval 20 mm west), L on the inner face (W x = -y: the leaves slide toward the back). After the bay's re-cut: the pad
        # behind the wall stands 2.1 into the bay (the hatch's fence is relieved for it, below).
        zc = FLOOR + V713_PAD + V713_USB_Z
        if side == "R":
            Wc = Pos(out["v713_at"][0], pk.bounds[3] + WIRE + WALL, zc) * Rot(0, 0, 180)
        else:
            Wc = Pos(vb[0] - V713_FENCE - 0.2 - WALL, out["v713_at"][1], zc) * Rot(0, 0, -90) * Rot(0, 180, 0)   # W x = +y, y = +x (inward), z DOWN: the leaves slide toward
            #                                                                                   the front (behind the window the inner wall is the hinge's: air 19 back, 162's first
            #                                                                                   cut), the tab and its tail at the hole's bottom edge (the frame is upside down)
        hull, cdoors, cclear = door_bypass(hull, Wc, None, side, CHG_SPEC, "CHARGE DOOR")
        out.update(chg_doors=cdoors, chg_Wf=Wc, chg_clear=cclear, chg_plug=Wc * Pos(0, 5.0, 0) * Box(11.6, 14.4, 7.4, align=MID))   # (164) the plug's path: the hatch's fence gives way
        # ROUNDED BOTTOM EDGE, all round except the hinge side (the rail, knuckles and the rest hinge's leaf stay)
        r = EDGE_R_BOT
        bot_band = band(plan, ZB - 0.5, ZB + r + 0.5, r + 0.5) - Kf * Pos(ax_x - 20.0, 0, ZB - 1.0) * Box(100.0, 1000.0, 100.0, align=(Align.MIN, Align.CENTER, Align.MIN))   # r + 0.5: the band's inner wall PAST the
        #                                                                round's tangent line -- ON it, OCC's common with the rounded prism came back EMPTY on L (0 of 1212 mm3, no error): L's
        #                                                                bottom edge was a 1.5 x 2 rebate from build 108 to 123 (the base plates' lesson, stand.build_base)
        piece, plain = rounded_prism(plan, ZB, ZB + r + 1.0, r_bot=r) & bot_band, (prism(plan, ZB, ZB + r + 1.0) & bot_band).volume   # plain: the square edge's content of the band (with the band's 0.5 of air under ZB counted the round read 67 %, build 123b)
        assert piece is not None and 0.75 * plain < piece.volume < 0.98 * plain, f"{self.side}: the bottom round swap holds {0 if piece is None else piece.volume:.0f} of the band's {plain:.0f} mm3"   # (see key_deck)
        hull = swap_band(hull, bot_band, [piece], self.side, "BOTTOM ROUND")   # ONE fillet face per edge (build 108); the band's content checked (145)
        lap("hinge + rounded bottom")
        self.hull = deghost(hull, self.side, "HINGES")

    def hatch_plate(self):
        """The hatch: the cell cradle, countersunk screw holes, the flap's ears + pin bore, the charger's pad + fence."""
        Kf, ax_x, bosses, fy, hatch_plan, lap, out, p = self.Kf, self.ax_x, self.bosses, self.fy, self.hatch_plan, self.lap, self.out, self.p
        # the hatch: 0.2 off its walls, countersunk M3 from below; the boards' posts, the EC11 pedestal and the charger's pad stand on it
        hatch = prism(hatch_plan.buffer(-0.2, join_style=1, quad_segs=4), ZB, FLOOR)
        Fct = out["cell_at"]                                        # THE CELL CRADLE (flat now: a low saddle): the tilt lifted the 18650 off the hatch; a saddle rises under it (the hatch holds it in)
        ccz = Fct.position.Z - (CELL_R + CELL_CLR)
        hatch += Pos(Fct.position.X, Fct.position.Y, FLOOR - 0.01) * Rot(0, 0, YAW) * Box(CELL_L - 6.0, 12.0, ccz - FLOOR + 3.0, align=BIG)
        hatch -= Fct * Rot(0, 90, 0) * Cylinder(CELL_R + CELL_CLR, CELL_L + 20.0, align=MID)
        out["cradle_h"] = ccz - FLOOR
        for b in bosses:
            hatch -= Pos(b.x, b.y, ZB - 1) * Cylinder(SCREW_D / 2, PLATE_T + 2, align=BIG)
            hatch -= Pos(b.x, b.y, ZB - 0.01) * Cone(HEAD_D / 2, SCREW_D / 2, (HEAD_D - SCREW_D) / 2 + 0.01, align=BIG)
        for sy in (-1, 1):                                          # the flap's ears under the hatch, a 3 mm pin through them and the flap's root
            hatch += Kf * Pos(ax_x - FLAP_P, fy + sy * (FLAP_W / 2 + EAR_GAP + EAR[1] / 2), ZB - EAR[2]) * Box(EAR[0], EAR[1], EAR[2] + 0.01, align=BIG)
        hatch -= Kf * Pos(ax_x - FLAP_P, fy, ZB - PIV_DROP) * Rot(90, 0, 0) * Cylinder((FLAP_PIN_D + FLAP_PIN_CLR) / 2, FLAP_W + 2 * EAR_GAP + 2 * EAR[1] + 2, align=MID)
        for sy in (-1, 1):                                          # (164) the stubs' heads sunk flush in the ears' outer faces
            yo = sy * (FLAP_W / 2 + EAR_GAP + EAR[1])
            hatch -= Kf * Pos(ax_x - FLAP_P, fy + yo, ZB - PIV_DROP) * Rot(90, 0, 0) * Cylinder(FLAP_STUB[0][0] / 2 + 0.2, 2 * (FLAP_STUB[0][1] + 0.1), align=MID)
        hatch += prism(p["v713"].buffer(V713_FENCE, join_style=2), FLOOR - 0.01, FLOOR + V713_PAD + V713_FENCE)   # the charger's pad + fence
        hatch -= prism(p["v713"], FLOOR + V713_PAD, FLOOR + V713_PAD + V713_FENCE + 1)
        hatch -= out["chg_clear"]
        hatch -= out["chg_plug"]                                                                # (164) the fence rail in front of the port stood 0.9 into the plug (test_charge_doors R: 14.9 mm3)
                                                                     # (162) the charging door's pad stands 2.1 into the bay: the fence gives way to it
        if False:   # ponytail: the EC11 hangs from the deck now; the old pedestal stays out
            hatch += Pos(KNOB_U, KNOB_Y, FLOOR - 0.01) * Box(PEDESTAL, PEDESTAL, EC11_PAD_Z - FLOOR + 0.01, align=BIG)
            hatch -= Pos(KNOB_U, KNOB_Y, EC11_PAD_Z - 1.2) * Box(13.0, 13.0, 1.21, align=BIG)                           # its body's underside sits in a shallow pocket (pads reachable from the sides)
        lap("bosses + hatch")
        self.Fct = Fct
        self.hatch = hatch

    def badge(self):
        """THE BADGE -> THE CROWN'S FLOOR (build 153; builds 136-152: L's knob housing + plate BADGE_T proud with a BADGE_R round, R's island). The raised
        plate is a separate printed part now (THE CROWN: lights.py, crown()) and the hull under it is FLAT at the rim. L: the band swap stays, with a FLAT
        keep (t 0, no round): the housing's own round, the step edges' rounds under the crown's edge (+ 1 outside) and the seam wall's round under the
        crown's west edge (the `over` band through the face, build 145) go; the knob well stays through the keep. The crown's plan (L): the flat zone north
        of the knob 0.3 inside its STEP edges, BADGE_IN inside the OUTLINE, to the seam face up to BADGE_IN before the shoulder's arc, the housing's top
        in, the lid band (the K box from 26 west of the 152 lid, 0.3 off the step chamfer's foot) to its east end; closed r 1.5: one intentional outline
        (Willy 11:42). R: nothing raised; the LED tray's plan (the 144 wedge, BADGE_M inside the old island's plan) for fittings."""
        hull, p, out, side = self.hull, self.p, self.out, self.side
        if side == "R":
            mouth = tp_plan(TP_CLR).buffer(TRAY_M + TRAY_FLARE, join_style=1)
            inner = p["flat"].buffer(-BADGE_IN, join_style=2).intersection(p["tp_zone"]).difference(mouth.buffer(0.5, join_style=1)).difference(out["bz_seat"].buffer(0.5, join_style=2))
            plate = inner.intersection(sbox(-1000.0, -1000.0, mouth.bounds[0], 1000.0))        # THE LED WEDGE (build 144): the console's free region west of the trackpad
            plate = max(getattr(plate, "geoms", [plate]), key=lambda g: g.area)
            plate = plate.buffer(-BADGE_W, join_style=1).buffer(BADGE_W, join_style=1).simplify(0.1)
            tray = plate.buffer(-BADGE_M, join_style=1).simplify(0.1)
            assert tray.geom_type == "Polygon" and out["crown_plan"].buffer(-1.0).contains(tray), "R: the LED tray is not under the crown (1 inside its edge)"
            out["led_tray"], out["badge"] = tray, Polygon()
            print(f"   R BADGE: none (THE CROWN, build 153); the LED tray {tray.area:.0f} mm2 {[round(v, 1) for v in tray.bounds]} in the crown's seat floor")
            return
        flat, plan, housing = p["flat"], p["plan"], p["housing"]
        kc = Point(KNOB_U, KNOB_Y)
        y_arc0 = max(y for x, y in flat.exterior.coords if x < 0.05)                               # the seam face's top end on the flat zone: the shoulder's arc starts here
        strap = LineString(STRAP_L).buffer(STRAP_W / 2, cap_style=2).intersection(plan.buffer(-BADGE_IN, join_style=2)).intersection(flat.buffer(-0.2, join_style=2))   # THE STRAP (154)
        lidbox = p["K"](sbox(LIGHT_LID[0] - 26.0, LIGHT_LID[1], LIGHT_LID[2], LIGHT_LID[3])).intersection(plan.buffer(-BADGE_IN, join_style=2)).intersection(flat.buffer(-0.2, join_style=2))
        bez = Point(kc.x, kc.y).buffer(KNOB_BEZ_R, quad_segs=32)                                 # THE BEZEL RING's disc (the well is the plate's hole)
        crown = unary_union([strap, lidbox, bez]).buffer(4.0, join_style=1).buffer(-4.0, join_style=1)   # closed r 4: the band / strap and strap / ring junctions flow
        crown = clean_outline(max(getattr(crown, "geoms", [crown]), key=lambda g: g.area), CROWN_R, tag="L CROWN PLAN")
        assert crown.geom_type == "Polygon" and not crown.interiors, f"L: the crown's plan is a {crown.geom_type} with {len(crown.interiors)} hole(s)"
        plate = crown.intersection(p["K"](sbox(-1000.0, -1000.0, LIGHT_LID[0] - 0.3, 1000.0)))      # the swap under the plate part only: the lid part sits on the untouched plateau (152)
        plate = max(getattr(plate, "geoms", [plate]), key=lambda g: g.area)
        well = Pos(kc.x, kc.y, RIM_Z - EDGE_R_TOP - 2.0) * Cylinder(KNOB_R + KNOB_CLR, EDGE_R_TOP + 4.0, align=BIG)
        #                                                                                          (154: the crown stays BADGE_IN off the seam face: no band through it
        zb = RIM_Z - EDGE_R_TOP - 1.0                                                              # under it; the band's wall is never ON the face: OCC skins, build 144)
        margin = plate.buffer(1.0, join_style=1).intersection(plan).difference(flat).buffer(0.01, join_style=2)   # 1 outside the plate off the flat zone's edge (over the pit / the deck)
        band = prism(unary_union([plate, margin]), zb, RIM_Z + 5.0)
        keep = prism(plate, zb, RIM_Z) - well
        v0, mt0 = hull.volume, shape_tolerance(hull)
        hull = deghost((hull - band) + keep, side, "BADGE")
        assert hull.is_valid and len(hull.solids()) == 1, f"L: the crown's floor left the hull {'invalid' if not hull.is_valid else str(len(hull.solids())) + ' solids'}"
        got, want = mesh_volume(hull & band), mesh_volume(keep)
        assert abs(got - want) < 1.0, f"L: the crown floor's band holds {got:.1f} mm3 after the swap, the flat keep {want:.1f}"
        mt = shape_tolerance(hull)
        assert mt <= max(0.2, mt0 + 0.005), f"L: the crown's floor raised the hull's tolerance {mt0:.3f} -> {mt:.3f}"
        tool = prism(bez, RIM_Z - SILO_SINK, RIM_Z + 5.0)                                        # THE STEP AT THE WELL'S MOUTH, SILO_SINK deep (154): the bezel ring's skirt keys in it
        ring = bez.difference(Point(kc.x, kc.y).buffer(KNOB_R + KNOB_CLR, quad_segs=32)).area
        took = (hull & tool).volume                                                                # (the piece's own volume: the whole hull's GProp drifts ~30 mm3 per boolean)
        hull -= tool
        left = hull & tool
        assert hull.is_valid and len(hull.solids()) == 1 and abs(took - ring * SILO_SINK) < 0.05 * ring * SILO_SINK + 1.0 and (left is None or left.volume < 0.05), f"L: the step at the well's mouth took {took:.1f} mm3 of the ring's {ring * SILO_SINK:.1f}, {left.volume if left else 0:.2f} left"
        print(f"   L SILO: the step at the well's mouth ({ring:.0f} mm2, out to R {KNOB_BEZ_R}) sunk {SILO_SINK} under the rim for the bezel ring's skirt ({took:.0f} mm3); the silo's own round stays beyond it")
        print(f"   L BADGE: THE CROWN'S FLOOR {plate.area:.0f} mm2 flat at the rim (the housing's, the step edges' and the seam wall's rounds under it gone, the well through); the crown {crown.area:.0f} mm2 {[round(v, 1) for v in crown.bounds]}; hull {v0:.0f} -> {hull.volume:.0f} mm3; tolerance {mt:.3f}", flush=True)
        out["crown_plan"], out["badge"] = crown, Polygon()
        self.hull = hull

    def fittings(self):
        """THE LEDS AND SLIDE SWITCHES (build 133; Willy 12:37 'good to go on all approved'): R's three LEDs stand in THE LED TRAY (build 134; build 153: in the crown's
        seat floor beside the touchpad -- L's two are GONE, Willy 10:49: the filament runs round the knob), normal to the top -- a LED_SEAT bore LED_SINK deep for
        its flange, LED_BORE on through to the cavity, the dome tip under the diffuser's pocket (LIT FROM UNDERNEATH, build 152); each 805 lies in a SW805_POCKET
        in the back wall's inner face (1.2 of wall left) with its knob through a slot the knob + its travel long, 0.5 proud of the face. Real STEPs placed
        (LED_STEP, SW805_STEP); each must clear the hull. Then THE CROWN'S LIGHTS (build 153; lights.py), all in the PLAN frame (z 0 = RIM_Z): the nets (the 152
        K nets through p["K"]; L's run on with the encoder runs), their grooves, vias and the crown's pin holes cut, the guides and filaments placed, R's
        diffuser with its arms in the tray; the crown itself is built on the finished hull (crown())."""
        hull, out, p, plan, side = self.hull, self.out, self.p, self.plan, self.side
        leds, exits = [], []
        tray = out.get("led_tray", Polygon())
        if LED_AT[side]:
            led0 = import_step(str(LED_STEP))
            led0 = Pos(-1.27, 0, -3.0) * led0                                                      # on the leads' axis (the STEP's x 1.27), the flange's underside (the STEP's z 3.0) at z 0
            zt = RIM_Z                                                                             # THE TRAY's top: the crown's seat floor (152: the island's top, RIM_Z + BADGE_T + 1.2)
            v0 = hull.volume
            hull -= prism(tray, zt - TP_IND, zt + 2.0)                                             # sunk TP_IND like the trackpad's tray was ...
            flare = extrude(make_face(Polyline(*[(x, y, 0.0) for x, y in tray.exterior.coords[:-1]], close=True)), amount=TRAY_FLARE + 0.5, taper=-45)
            hull -= Pos(0, 0, zt - TRAY_FLARE) * flare                                             # ... with its 45 deg flared mouth
            assert v0 - hull.volume > 0.9 * tray.area * TP_IND, f"{side}: the LED tray cut only {v0 - hull.volume:.0f} mm3 (its box is {tray.area * TP_IND:.0f})"
            zf = zt - TP_IND - LED_SINK                                                            # the flange's underside under the tray's floor
            for x, y in LED_AT[side]:
                hull -= Pos(x, y, zf) * Cylinder(LED_SEAT / 2, LED_SINK + 2.0, align=BIG)
                hull -= Pos(x, y, FLOOR + 1.0) * Cylinder(LED_BORE / 2, zf - FLOOR, align=BIG)
                leds.append(Pos(x, y, zf) * led0)
                col = hull & (Pos(x, y, FLOOR) * Cylinder(LED_BORE / 2 + 1.5, zf - FLOOR + 1.0, align=BIG))   # HELD WITHOUT GLUE (build 137): the deck round the bore, from the hatch
                zs = sorted((s.bounding_box().min.Z, s.bounding_box().max.Z) for s in col.solids())    # floor to the seat; its topmost piece's underside = where the leads exit
                ceil = zs[-1][0]
                room = hull & (Pos(x, y, ceil - 2.5) * Cylinder(3.5, 2.5, align=BIG))              # a 7 x 2.5 disc under the exit: the bent leads' room
                free = 1.0 - (room.volume if room else 0.0) / (math.pi * 3.5 ** 2 * 2.5)
                assert 2.0 < zf - ceil < 20.0 and free >= 0.5, f"{side}: the LED at ({x}, {y}): the leads exit {zf - ceil:.1f} under the seat, {free * 100:.0f} % of the disc under the exit free -- no room to bend them"
                exits.append((round(zf - ceil, 1), round(free * 100)))
        out["led_tray"] = tray
        sw0 = import_step(str(SW805_STEP))                                                         # x 0..11.7 along the body, y -4..0 its height, z -6.3..1.7: pins down, the knob up
        sw0 = Pos(-11.7 / 2, 2.0, 0) * sw0                                                         # centred on its body; the body's top face (the knob's root) at z 0
        sws, sockets = [], []
        pl, ph, pd = SW805_POCKET
        kx = (3.65 + 5.65) / 2 - 11.7 / 2 + SW805_TRAVEL / 2                                        # the knob's centre + half its travel along the body (the slot runs on from the knob's rest)
        sl = SW805_SLOT[0] + SW805_TRAVEL + 0.4
        ext = plan.exterior
        for name, x0, y0, z in SW805_AT[side]:
            s = ext.project(Point(x0, y0)); q = ext.interpolate(s)                                  # the wall's face at the nearest outline point
            a, b = ext.interpolate(s - 0.5), ext.interpolate(s + 0.5)
            tx, ty = b.x - a.x, b.y - a.y; tl = math.hypot(tx, ty); tx, ty = tx / tl, ty / tl
            if plan.contains(Point(q.x - ty, q.y + tx)):                                           # the local frame: +x along the wall, +y OUTWARD (the left normal; flipped if it points in)
                tx, ty = -tx, -ty
            F = Pos(q.x, q.y, z) * Rot(0, 0, math.degrees(math.atan2(ty, tx)))                     # (a sloped or curved wall keeps its skin: L's outer wall runs 5 deg off y, build 133 run 2)
            hull -= F * Pos(0, -SW805_SKIN - pd / 2, 0) * Box(pl, pd, ph, align=MID)                # the pocket: from the skin inward through the wall into the cavity
            hull -= F * Pos(kx, -0.5, 0) * Box(sl, 3.0, SW805_SLOT[1], align=MID)                    # the slot through the skin
            # THE SOCKET (build 156; Willy 18:07 / 18:19, params SW805_BOSS / SW805_RIB): the boss round the pocket on the wall's inside + crush ribs on its four faces;
            # the switch presses in from the inside until its top meets the skin and stays by friction (133-155: 1.8 of the body's 5.3 in the wall, 0.2 of air a side)
            by0, by1, rz, rx = -SW805_SKIN - 0.8, -SW805_SKIN - 5.3 - 0.4, SW805_RIB[0], SW805_RIB[1]
            bw = SW805_BOSS
            sock = Pos(0, (by0 + by1) / 2, 0) * (Box(pl + 2 * bw, by0 - by1, ph + 2 * bw, align=MID) - Box(pl, by0 - by1 + 0.2, ph, align=MID))
            ry0, ry1 = -SW805_SKIN, -SW805_SKIN - 5.3 + 0.2                                        # the ribs from the skin's inside to 0.2 short of the body's back
            for xr in (-pl / 2 + 2.5, pl / 2 - 2.5):                                                 # two a face on the top and bottom faces (z), one on each end (x), 0.02 into the face
                for sz in (-1, 1):
                    sock += Pos(xr, (ry0 + ry1) / 2, sz * (ph / 2 - rz / 2 + 0.01)) * Box(RIB_W, ry0 - ry1, rz + 0.02, align=MID)
            for sx in (-1, 1):
                sock += Pos(sx * (pl / 2 - rx / 2 + 0.01), (ry0 + ry1) / 2, 0) * Box(rx + 0.02, ry0 - ry1, RIB_W, align=MID)
            sock = F * sock
            vol = lambda q: 0.0 if q is None else q.volume
            was = vol(hull & sock)
            hull += sock
            now = vol(hull & sock)                                                                  # judged by a fresh boolean of the socket's region, not the hull's volume
            assert now > 0.98 * sock.volume and len(hull.solids()) == 1, f"{side}: the 805 {name} socket fused {now:.0f} of {sock.volume:.0f} mm3 ({len(hull.solids())} solids)"
            sockets.append((name, round(sock.volume - was)))
            sws.append((name, F * Pos(0, -SW805_SKIN, 0) * Rot(-90, 0, 0) * sw0))                    # +z -> +y: the knob out through the wall, the body's top on the skin
            for lx in (-pl / 2 + 0.8, pl / 2 - 0.8):                                                 # THE SKIN at the pocket's four corners: the wall's outermost run from 1 outside
                for lz in (-ph / 2 + 0.6, ph / 2 - 0.6):                                             # the face reaches the face and is SKIN thick (a window / the shoulder's sweep
                    c = hull & (F * Pos(lx, -5.0, lz) * Box(0.3, 13.0, 0.3, align=MID))             # would leave it thin or missing: R's two misplacements of build 133)
                    rr = sorted(((F.inverse() * sb).bounding_box().min.Y, (F.inverse() * sb).bounding_box().max.Y) for sb in c.solids()) if c else []
                    assert rr and rr[-1][1] > -0.15 and rr[-1][1] - rr[-1][0] > SW805_SKIN - 0.15, f"{side}: the 805 {name} pocket breaks the wall at ({q.x + lx * tx:.1f}, {q.y + lx * ty:.1f}, z {z + lz:.1f}): runs {[(round(u, 2), round(v, 2)) for u, v in rr]} (skin {SW805_SKIN})"
        crush = 6 * RIB_W * 0.2 * (5.3 - 0.2)                                                    # the 805's six ribs, 0.2 into its body over 5.1 (7.3 mm3): the press fit
        for q, n in [(q, f"LED {i}") for i, q in enumerate(leds)] + [(q, f"805 {n}") for n, q in sws]:
            lo, hi = (0.7 * crush, 1.15 * crush) if n.startswith("805") else (-1.0, 0.01)
            assert lo < _hit(q, hull) < hi, f"{side}: the {n} sits in the hull by {_hit(q, hull):.2f} mm3 (wants {lo:.1f}..{hi:.1f})"
        print(f"   805 SOCKETS: {sockets} mm3 fused (a {SW805_BOSS} boss round the pocket to {SW805_SKIN + 5.3 + 0.4:.1f} behind the face, six ribs {RIB_W} wide {SW805_RIB} proud: {crush:.1f} mm3 of crush a switch)")
        print(f"   FITTINGS: {len(leds)} LED(s) at {[(x, y) for x, y in LED_AT[side]]}" + (f" in the LED tray {tray.bounds[2] - tray.bounds[0]:.1f} x {tray.bounds[3] - tray.bounds[1]:.1f} (the wedge), {TP_IND} deep in the crown's seat floor ({LED_SEAT} seat {LED_SINK} deep, {LED_BORE} through; the leads exit (mm under the seat, % of the disc under free) {exits})" if leds else " (the filament runs instead)") + f"; {len(sws)} slide switch(es) {[n for n, _ in sws]} in the walls (pocket {SW805_POCKET} from {SW805_SKIN} inside the face, slot {SW805_SLOT[0]} x {SW805_SLOT[1]}); the LEDs clear of the hull, each 805 pressed in by its ribs", flush=True)
        # THE CROWN'S LIGHTS (build 153): the nets, filaments and vias in the plan frame; the grooves, via holes and pin holes cut; the guides and filaments
        # placed and checked; R's diffuser + arms. The slots, ridges and pins go to crown().
        K = p["K"]
        nets_, fils_, vias_, leads_ = nets(side, K), filaments(side, K), vias(side, K), leads(side, K)
        crown = out["crown_plan"]
        slots = [slot_plan(l) for l in nets_.values()]
        avoid = slots + [foot_plan(l) for l in nets_.values()]
        avoid += [s.buffer(LEAD_W / 2 + 0.2, cap_style=2) for _, s, _, _ in leads_] + [LineString([(sx, sy), (sx + dx * LEAD_VIA / math.sqrt(2.0), sy + dy * LEAD_VIA / math.sqrt(2.0))]).buffer(HOLE_D / 2 + 0.2) for _, _, (sx, sy), (dx, dy) in leads_]   # (163) the lead ways
        tight = []
        if side == "L":
            avoid += [Point(KNOB_U, KNOB_Y).buffer(KNOB_R + KNOB_CLR), p["housing"].buffer(0.5)]   # (154: none in the silo ring -- 3.0 of wall, the cap's boss on it)
        else:
            avoid += arm_slots(LED_AT[side]) + [out["bz_outline"].buffer(1.0)]
            tight = [tray, collar_room()]
        pins_ = crown_pins(side, K, crown, avoid, tight=tight)
        assert len(pins_) >= len(PINS_K) + 3, f"{side}: only {len(pins_)} pins found for the crown (the lid's {len(PINS_K)} + 3 at least)"   # (156: the lid's 7 -- the pin between the east vias gone)
        Fl = Pos(0, 0, RIM_Z)
        v0 = hull.volume
        for c in cutters(side, K, nets_, vias_, pins_, fils_, leads_):
            hull -= Fl * c
        cut = v0 - hull.volume
        assert cut > 700.0, f"{side}: the crown's lights cut only {cut:.0f} mm3"
        guides_u = {n: guide_part(l, [f for fn, f in fils_.items() if fn.split("_")[0] == n], notches=[s for _, s, _, _ in leads_]) for n, l in nets_.items()}
        fils_u = {n: filament_part(f) for n, f in fils_.items()}
        guides, fils = {n: Fl * g for n, g in guides_u.items()}, {n: Fl * f for n, f in fils_u.items()}
        for n in nets_:
            assert _hit(guides[n], hull) < 0.01, f"{side}: the {n} guide sits in the hull by {_hit(guides[n], hull):.2f} mm3"
        for n, f in fils.items():
            host = n.split("_")[0]
            assert _hit(f, guides[host]) < 0.01 and _hit(f, hull) < 0.01, f"{side}: the {n} filament sits in its guide by {_hit(f, guides[host]):.2f} / the hull by {_hit(f, hull):.2f} mm3"
        for x, y, deep in vias_:
            if deep:                                                                               # the hole opens into the cavity: no hull between its end and the hatch plane
                col = hull & (Pos(x, y, FLOOR - 1.0) * Cylinder(0.3, VIA_DEEP - FLOOR + 1.1, align=BIG))
                assert col is None or col.volume < 0.01, f"{side}: the deep via at ({x:.1f}, {y:.1f}) ends in solid hull ({col.volume:.2f} mm3 between z {FLOOR - 1.0} and {VIA_DEEP + 0.1}): no cavity under it"
            else:                                                                                  # ... into the bore's lead room: nothing of the hull from the groove's floor down to 7.5 under the rim
                col = hull & (Pos(x, y, RIM_Z - 30.0) * Cylinder(0.3, 30.5, align=BIG))
                tops = sorted(s.bounding_box().max.Z for s in col.solids()) if col else []
                assert not tops or tops[-1] < RIM_Z - 7.5, f"{side}: the via at ({x:.1f}, {y:.1f}) does not open into the lead room (the hull's top under it at {tops[-1]:.1f})"
        gaps = []
        if leads_:                                                                                 # THE LEAD WAYS (163, R): each hole breaks into the key bay, its axis 1+ off the bore's wall; the channel keeps 1.5+ over the bore
            Fc = out["cell_at"]; r_b = CELL_R + CELL_CLR
            A, B = np.array((Fc * Pos(-100, 0, 0)).position.to_tuple()), np.array((Fc * Pos(100, 0, 0)).position.to_tuple())
            for n, strip, (sx, sy), (dx, dy) in leads_:
                z0 = RIM_Z - GROOVE_D + 0.3
                pts = [np.array((sx + dx * t / math.sqrt(2.0), sy + dy * t / math.sqrt(2.0), z0 - t / math.sqrt(2.0))) for t in np.arange(0.0, LEAD_VIA + 0.01, 0.5)]
                gap = min(np.linalg.norm(np.cross(q - A, B - A)) / np.linalg.norm(B - A) for q in pts) - r_b - HOLE_D / 2
                assert gap >= 1.0, f"{side}: the {n} lead hole at ({sx:.1f}, {sy:.1f}) runs {gap:.2f} off the bore's wall (wants 1.0+)"
                gaps.append(round(gap, 1))
                col = hull & lead_hole(sx, sy, dx, dy, z0, 0.3, LEAD_VIA + 1.0)                   # a 0.3 probe down the whole axis + 1 beyond: all air = the hole is through into the bay
                assert col is None or col.volume < 0.01, f"{side}: the {n} lead hole at ({sx:.1f}, {sy:.1f}) has solid hull on its axis ({0.0 if col is None else col.volume:.2f} mm3): not through"
                for q in (strip.interpolate(d) for d in np.arange(0.5 + GROOVE_W / 2 + 1.0, strip.length - 2.5, 0.5)):   # north of the hole's mouth (its 45 deg cut reaches ~1.3 north of the start under the floor)
                    col = hull & (Pos(q.x, q.y, RIM_Z - 30.0) * Cylinder(0.3, 30.5, align=BIG))
                    segs = sorted((s.bounding_box().min.Z, s.bounding_box().max.Z) for s in col.solids()) if col else []
                    assert segs and segs[-1][1] - segs[-1][0] >= 1.5 and abs(segs[-1][1] - (RIM_Z - GROOVE_D)) < 0.05, f"{side}: under the {n} lead channel at ({q.x:.1f}, {q.y:.1f}) the skin is {segs[-1] if segs else None} (wants 1.5+ under {RIM_Z - GROOVE_D:.1f})"
        for n, f in fils_.items():                                                                 # the skin under each groove's floor: GROOVE_D off the bore's 3.2 crown, more elsewhere
            q = f.interpolate(0.5, normalized=True)
            col = hull & (Pos(q.x, q.y, RIM_Z - 30.0) * Cylinder(0.3, 30.5, align=BIG))
            segs = sorted((s.bounding_box().min.Z, s.bounding_box().max.Z) for s in col.solids())
            assert segs and segs[-1][1] - segs[-1][0] >= 0.85 and abs(segs[-1][1] - (RIM_Z - GROOVE_D - FIL_SINK)) < 0.05, f"{side}: under the {n} filament's trench the skin is {segs[-1] if segs else None}"   # (0.90 over the cavity -- L's encoder runs, the deck's underside at 7.1 --, 1.35 over the bore; FIL_SINK 0.7)
        if side == "R":
            dif_u = diffuser_part(tray, LED_AT[side])
            dif = Pos(0, 0, RIM_Z - TP_IND) * dif_u
            assert _hit(dif, hull) < 0.01 and max(_hit(q, dif) for q in leds) < 0.01, f"R: the diffuser sits in the hull by {_hit(dif, hull):.2f} / the LEDs by {max(_hit(q, dif) for q in leds):.2f} mm3"
            for q in leds:
                tip = q.bounding_box().max.Z
                assert RIM_Z - DOME_POCKET[1] + 0.05 < tip < RIM_Z - 0.1, f"R: a dome's tip is at {tip:.2f}, the seat floor at {RIM_Z} (it should sit 0.1-{DOME_POCKET[1]:.1f} under it, in the diffuser's pocket)"
            out["led_bezel"], out["diffuser_local"] = dif, dif_u
            slots += arm_slots(LED_AT[side])
            print(f"   LED DIFFUSER: {dif_u.volume:.0f} mm3 in the tray, pockets {DOME_POCKET} over {len(leds)} dome tips, THE ARMS {[round(a.length, 1) for a in arms(LED_AT[side])]} mm long onto the bezel plate", flush=True)
        else:
            out["led_bezel"], out["diffuser_local"] = None, None
        print(f"   CROWN LIGHTS: {cut:.0f} mm3 cut ({len(nets_)} grooves, {len(vias_)} vias ({sum(1 for v in vias_ if v[2])} deep), {len(pins_)} pin holes); guides {[round(g.volume) for g in guides_u.values()]} mm3, "
              f"{len(fils_)} filaments {FIL_L} x {FIL_D} (GUESS: no catalog STEP); all clear; every via open"
              + (f"; LEAD WAYS {len(leads_)}: {LEAD_W} x {GROOVE_D} channels to K y {LEAD_Y}, {HOLE_D} holes 45 deg down {LEAD_VIA} long into the key bay, {min(gaps)} off the bore's wall, no lead-room vias" if leads_ else ""), flush=True)
        out.update(guides=guides, filaments=fils, guides_local=guides_u, lid_plan=crown, crown_slots=slots, crown_ridges=ridge_plan(side, K), crown_pins=pins_, nets=nets_, fil_runs=fils_)
        out["leds"], out["switches"] = leds, sws
        self.hull = hull

    def finish(self):
        """Drop boolean slivers, heal a tolerance flaw (ShapeFix, volume unchanged), hand everything out."""
        side, hull = self.side, self.hull
        # THE KEYWALL'S BACK EDGE (164; Willy 00:04 'this same one on both sides - the section just looks unfinished - finish it however just make it clean',
        # his red line along the key block's back): the tilted deck's top stands 0.6-1.8 over the flat plateau (RIM_Z) along K y 29.7-30.8, its west end a
        # 1.8 step at K x -15.3, its east end 0.4-0.7 (scratchpad creasemap / ends). Three 45 deg planes in K, each from a straight foot line on the rim:
        # inside the strip the top is cut down to RIM_Z + the distance from the line, outside it (the jogs past the line) down to the rim.
        Kf = self.Kf
        def bevel(p0, p1, s_out, H=4.0):                            # inside = the line's left (p0 -> p1)
            L = math.dist(p0, p1); ux, uy = (p1[0] - p0[0]) / L, (p1[1] - p0[1]) / L; nx, ny = -uy, ux
            pts = [(p0[0] + nx * a, p0[1] + ny * a, RIM_Z + b) for a, b in ((-s_out, 0.0), (0.0, 0.0), (H, H), (-s_out, H))]
            return Kf * extrude(make_face(Polyline(*pts, close=True)), amount=L, dir=(ux, uy, 0.0))
        tools = [bevel((63.5, 29.7), (-16.0, 29.7), 1.8), bevel((-15.6, 31.5), (-15.6, 18.0), 0.6), bevel((63.4, 26.5), (57.4, 30.0), 3.0)]
        v0 = hull.volume
        for t in tools:
            hull -= t
        cut = v0 - hull.volume
        assert hull.is_valid and len(hull.solids()) == 1 and 15.0 < cut < 250.0, f"{side}: the keywall bevel cut {cut:.0f} mm3, valid {hull.is_valid}, {len(hull.solids())} solid(s)"
        print(f"   {side} KEYWALL BEVEL: 45 deg from the rim along K y 29.7 (x -16..63.5), x -15.6 (west end) and the east end's diagonal; {cut:.0f} mm3", flush=True)
        v0 = hull.volume                                            # REBUILT by one boolean: the fused hull's B-rep can carry a face the mesher drops (build 102:
        hull = hull & Box(3000.0, 3000.0, 3000.0, align=MID)       # the hatch showed through the deck over L's two bosses under the keywell; ShapeFix left it, this heals it)
        assert abs(hull.volume - v0) < 1.0, f"{side}: the rebuild changed the hull's volume {v0:.0f} -> {hull.volume:.0f}"
        if len(hull.solids()) > 1:                                  # drop boolean slivers
            print(f"   {side}: hull had {len(hull.solids())} solids {[(round(q.volume, 1), [round(v) for v in (q.bounding_box().center().X, q.bounding_box().center().Y, q.bounding_box().min.Z, q.bounding_box().max.Z)]) for q in hull.solids()]}; keeping the biggest")
            dropped = sum(q.volume for q in hull.solids()) - max(q.volume for q in hull.solids())
            hull = max(hull.solids(), key=lambda q: q.volume)
            assert dropped < 1.0, f"{side}: {dropped:.1f} mm3 of hull dropped as slivers -- a cut left a floating piece (R's console fin, builds 97-99)"
        if not hull.is_valid:                                       # the tilt + slope booleans leave a tolerance flaw; ShapeFix heals it, volume unchanged
            v0 = hull.volume
            hull = hull.fix()
            print(f"   {side}: hull repaired: valid {hull.is_valid}, volume {v0:.0f} -> {hull.volume:.0f}")
        self.crown(hull)                                            # THE CROWN (build 153), on the finished hull; R's fused with the bezel
        self.out.update(hull=hull, hatch=self.hatch, bosses=self.bosses, Kf=self.Kf, z_ax=self.z_ax, fy=self.fy)
        return self.out

    def crown(self, hull):
        """THE CROWN (build 153; lights.crown_part), on the FINISHED hull in the plan frame: the plate CROWN_T on the rim over out["crown_plan"], the slots
        (the guides' stems; R's arms), the holes (L: the knob well; R: the display window's top outline -- its bevel is in the bezel's body --, the keyholes
        with their bevelled mouths, the trackpad window), the silkscreen ridges (R: the bezel's own traces raised too), the pins. R: + the lip's hook over
        the seam wall's round and the seam edge's LIP_R_SEAM round (builds 114-152's brim), the trackpad COLLAR + LEDGE under the window, fused with the
        bezel's body (the fuse order retried: OCC flakes on a cache-resumed bezel, build 142); THE OVERLAY placed on the pad."""
        from shapely import affinity
        out, side = self.out, self.side
        foot, t, ridges = out["crown_plan"], CROWN_T, (out["crown_ridges"] if RIDGES else Polygon())   # (157: RIDGES off -- a flat top)
        if side == "L":
            holes, before = [Point(KNOB_U, KNOB_Y).buffer(KNOB_R + KNOB_CLR, quad_segs=32)], None
        else:
            fx, fy, _fz = out["F2H"].position.to_tuple()
            T2 = lambda g: affinity.translate(g, fx, fy)
            T2p = lambda q: (q[0] + fx, q[1] + fy)
            top, kslots, traces = out["bz_cuts"]
            holes = [T2(top)] + [T2(sl) for sl in kslots] + [tp_window()]
            ridges = unary_union([ridges, T2(traces)]) if RIDGES else ridges
            run, big = BZ_SLOT

            def before(plate):                                                                     # THE KEYHOLES' mouths bevelled 45 deg `run` into the top (build 138)
                for sl in kslots:
                    x0, y0, x1, y1 = sl.bounds
                    plate -= bevel_cut([T2p(q) for q in clipped(x0, y0, x1, y1, CLIP_S)], t - run, [T2p(q) for q in clipped(x0 - run, y0 - run, x1 + run, y1 + run, big + run, CLIP_S + run)], t + 0.01)
                return plate
        plate, ne = crown_part(foot, holes, out["crown_slots"], ridges, out["crown_pins"], t, before)
        crown = Pos(0, 0, RIM_Z) * plate
        if side == "R":
            ys = [y for x, y in foot.exterior.coords if x < 0.05]
            y0h, y1h = min(ys), max(ys)                                                            # the crown's run along the seam face
            hook_plan = foot.intersection(self.p["plan"].buffer(-0.02, join_style=2)).difference(self.p["plan"].buffer(-BZ_HOOK, join_style=2))   # THE LIP'S HOOK: the band BZ_HOOK in from the outline
            #                                                                                        wherever the crown's plan reaches it -- the seam face (since 141) AND the console's front diagonal (build 156) --
            hook = prism(hook_plan, RIM_Z - 2.0, RIM_Z + 0.1) - hull                                   # 0.02 off the wall's face, over the wall's round: its underside the hull's own surface
            out["crown_hook"] = hook_plan
            crown += hook
            band = prism(sbox(-1.0, y0h - 1.0, LIP_R_SEAM, y1h + 1.0), RIM_Z + t - LIP_R_SEAM, RIM_Z + t + 1.0)   # THE SEAM EDGE's r LIP_R_SEAM round (the console's profile)
            band -= Pos(LIP_R_SEAM, (y0h + y1h) / 2, RIM_Z + t - LIP_R_SEAM) * Rot(90, 0, 0) * Cylinder(LIP_R_SEAM, y1h - y0h + 4.0, align=MID)
            crown -= band
            zl = TP_TOP - TP_T - TP_LEDGE
            crown += prism(collar_plan(), zl, RIM_Z + 0.01) + prism(ledge_plan(), zl, zl + TP_LEDGE)   # THE COLLAR + THE LEDGE under the trackpad window (the ledge's top IS the pad's underside: no 0.01 over it, dry 153)
            assert crown.is_valid and len(crown.solids()) == 1, f"R: the crown with its hook, collar and ledge is {len(crown.solids())} solids, valid {crown.is_valid}"
            lip_out = crown - prism(out["bz_outline"], ZB, RIM_Z + 5.0)
            if _hit(lip_out, hull) >= 0.05:                                                        # (diagnostic: where)
                ov = lip_out & hull
                print(f"   CROWN R: IN THE HULL -- {[(round(q.volume, 2), [round(v, 1) for v in (q.bounding_box().min.X, q.bounding_box().min.Y, q.bounding_box().min.Z, q.bounding_box().max.X, q.bounding_box().max.Y, q.bounding_box().max.Z)]) for q in sorted(ov.solids(), key=lambda q: -q.volume)[:8]]}", flush=True)
            assert _hit(lip_out, hull) < 0.05 and _hit(lip_out, out["hat"]) < 0.01, f"R: the crown sits in the hull by {_hit(lip_out, hull):.2f} / the HAT by {_hit(lip_out, out['hat']):.2f} mm3"
            bez = out["bezel"]
            fusers = (("bez + crown", lambda: bez + crown), ("crown + bez", lambda: crown + bez), ("fuzzy bez, crown", lambda: fuzzy_fuse(bez, crown)),
                      ("fuzzy crown, bez", lambda: fuzzy_fuse(crown, bez)), ("a copied bezel + crown", lambda: copy.deepcopy(bez) + crown))
            for name, fuse in fusers:
                out["bezel"] = fuse()
                if out["bezel"] is not None and out["bezel"].is_valid and len(out["bezel"].solids()) == 1:
                    break
                print(f"   CROWN R: the fuse {name} gave {len(out['bezel'].solids()) if out['bezel'] is not None else None} solid(s) on the resumed bezel", flush=True)
            assert out["bezel"].is_valid and len(out["bezel"].solids()) == 1, f"R: the crowned bezel is {len(out['bezel'].solids())} solids"
            crown = out["bezel"]
            overlay = Pos(0, 0, TP_TOP) * overlay_part()
            pad = prism(tp_plan(), TP_TOP - TP_T, TP_TOP)
            for q, n in ((overlay, "overlay"), (pad, "pad")):
                assert _hit(q, crown) < 0.01 and _hit(q, hull) < 0.01, f"R: the {n} sits in the crown by {_hit(q, crown):.2f} / the hull by {_hit(q, hull):.2f} mm3"
            out["overlay"], out["overlay_local"] = overlay, overlay_part()
            print(f"   CROWN R: the hook {BZ_HOOK} wide over the outline's round under the crown's edge ({hook_plan.area:.0f} mm2: the seam face y {y0h:.1f}..{y1h:.1f} + the front diagonal; {hook.volume:.0f} mm3), the seam edge r {LIP_R_SEAM}; the collar {TP_COLLAR} + the ledge {TP_LEDGE} under the trackpad window from z {zl:.2f}; the overlay {overlay.volume:.0f} mm3 on the pad at {TP_TOP}, {OVERLAY_SINK} under the top; the bezel fused: {crown.volume:.0f} mm3", flush=True)
        else:
            wellp = Point(KNOB_U, KNOB_Y).buffer(KNOB_R + KNOB_CLR, quad_segs=32)                 # (the plate's hole polygon: the boss's hole the same)
            boss = prism(Point(KNOB_U, KNOB_Y).buffer(KNOB_BEZ_R - COVER_CLR, quad_segs=32), -SILO_SINK, 0.01) - prism(wellp, -SILO_SINK - 1.0, 1.0)   # THE BEZEL RING's SKIRT (154) in the step at the well's mouth
            crown += Pos(0, 0, RIM_Z) * boss
            assert crown.is_valid and len(crown.solids()) == 1, f"L: the crown with its silo boss is {len(crown.solids())} solids, valid {crown.is_valid}"
            assert _hit(crown, hull) < 0.05, f"L: the crown sits in the hull by {_hit(crown, hull):.2f} mm3"
            print(f"   CROWN L: the bezel ring {KNOB_R + KNOB_CLR:.1f}..{KNOB_BEZ_R:.1f} round the knob, its skirt {boss.volume:.0f} mm3 {SILO_SINK} into the step, {COVER_CLR} off its wall")
            out["overlay"], out["overlay_local"] = None, None
        for n, g in out["guides"].items():
            assert _hit(g, crown) < 0.01, f"{side}: the {n} guide sits in the crown by {_hit(g, crown):.2f} mm3"
        if out.get("led_bezel") is not None:
            assert _hit(out["led_bezel"], crown) < 0.01, f"R: the diffuser sits in the crown by {_hit(out['led_bezel'], crown):.2f} mm3"
        out["crown"], out["crown_local"] = crown, Pos(0, 0, -RIM_Z) * crown
        print(f"   CROWN {side}: {foot.area:.0f} mm2 {[round(v, 1) for v in foot.bounds]}, {t} on the rim, r {CROWN_R} on {ne} top edges, {len(out['crown_slots'])} slots, {len(out['crown_pins'])} pins, ridges {ridges.area:.0f} mm2; {crown.volume:.0f} mm3", flush=True)


def build_half(side, p):
    """One half in its u frame: hull, hatch, placements (Half, step by step)."""
    return Half(side, p).build()
