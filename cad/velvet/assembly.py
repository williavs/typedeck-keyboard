"""Both halves on their bases in world: assemble() places every part, parts() labels them for the STEP, kinematics() declares the joints."""
from build123d import Box, Color, Compound, Cylinder, Plane, Pos, Rot, import_step, mirror

import cadgen

from velvet.params import *  # noqa: F401,F403
from velvet.geo import placed, prism, tp_plan
from velvet.frames import keys, legends
import os
from velvet.plan import plan_of
from velvet.stand import back_at_axis, rest_pin_len, tent_pin_len, build_base, flap, shank, tent_angles, tent_axes, flap_stubs
from velvet.hull import build_half
from velvet.parts import knob


def assemble():
    plans = {k: plan_of(k) for k in "RL"}
    halves = {}
    cell0 = import_step(str(CELL_STEP))
    pico0 = import_step(str(SL.PICO_STEP))
    cb = cell0.bounding_box()
    cell0 = Pos(-cb.center().X, -cb.center().Y, -cb.center().Z) * cell0           # the STEP's axis is x
    for k in "RL":
        h = build_half(k, plans[k])
        s = 1 if k == "R" else -1
        W = Pos(s * SEP / 2, 0, -ZB + BASE_T)                                       # the base on the desk (z 0), the hull on the base, inner edges SEP apart
        mir = (lambda q: q) if k == "R" else (lambda q: mirror(q, about=Plane.YZ))
        T = h["T"]                                                                  # the top shell's tilt (y/z only: commutes with the x-mirror)
        pico = mir(h["pico_loc"] * (pico0 if k == "R" else mirror(pico0, about=Plane.YZ.offset(pico0.bounding_box().center().X))))   # a REAL board in L's
        #                                                                             mirrored bay: the placement mirrors, the board mirrors twice -- about its OWN centre
        #                                                                             (about x = 0 it landed 21 off, build 79)
        cc = h["cell_at"].position
        vx, vy, vz = h["v713_at"]
        vw, vd = (V713[0], V713[1]) if k == "R" else (V713[1], V713[0])
        base, bcaps = build_base(k, plans[k], h["Kf"])
        Kf, z_ax, fy = h["Kf"], h["z_ax"], h["fy"]
        seat_t = back_at_axis(plans[k]) - PIN_HEAD[1]                                                           # THE PINS (build 131): a shank from the seat to the nose + the head = the
        pin_u = Kf * Pos(plans[k]["ax_x"], seat_t, z_ax) * Rot(0, 0, 180) * shank(tent_pin_len(plans[k]))     # case's own form inside the head envelope (the hull's part + the base's)
        pin_u = pin_u + h["pin_caps"]["tpin"] + bcaps["tpin"]
        rs = plans[k]["rslots"]
        rpin_u = Kf * Pos(plans[k]["ax_x"], REST_FRONT + PIN_HEAD[1], z_ax) * shank(rest_pin_len(plans[k]))     # the rest hinge's, from the FRONT face
        rpin_u = rpin_u + h["pin_caps"]["rpin"] + bcaps["rpin"]
        for n, q in (("tent", pin_u), ("rest", rpin_u)):
            assert q.is_valid and len(q.solids()) == 1, f"{k}: the {n} pin is {'invalid' if not q.is_valid else str(len(q.solids())) + ' solids'} (the head's parts did not fuse to the shank)"
        flap_u = Kf * Pos(plans[k]["ax_x"] - FLAP_P, fy, ZB - PIV_DROP) * flap()
        fpin_u = Kf * Pos(plans[k]["ax_x"] - FLAP_P, fy, ZB - PIV_DROP) * flap_stubs()   # (164) two stub pins, one from each ear
        ta = tent_axes(k, plans[k])
        out = dict(hull=W * mir(h["hull"]), hatch=W * mir(h["hatch"]), pico=W * pico, pane=W * mir(h["pane"]), frame=None, tpu=W * mir(h["tpu"]), pdoor_a=W * mir(h["pico_doors"][0]), pdoor_b=W * mir(h["pico_doors"][1]), cdoor_a=W * mir(h["chg_doors"][0]), cdoor_b=W * mir(h["chg_doors"][1]),   # THE SLIDE DOORS (build 155), printed in place in the hull
                   base=W * mir(base), flap=W * mir(flap_u), fpin=W * mir(fpin_u), tpin=W * mir(pin_u), rpin=W * mir(rpin_u), tpin_u=pin_u, rpin_u=rpin_u, seat_t=seat_t,
                   tent_axis=ta["tent"], flap_axis=ta["flap"],                                              # (origin, direction): + lifts the inner edge / raises the flap
                   cell=W * Pos(s * cc.X, cc.Y, cc.Z) * Rot(0, 0, s * YAW) * cell0,
                   v713=W * Pos(s * vx, vy, vz) * Box(vw, vd, V713[2] + V713_PCB, align=BIG),   # the clearance box, GUESS (no STEP)
                   keys=[(kk, W * T * sw, W * T * cap) for kk, sw, cap in keys(plans[k]["fr"])], local=h, W=W,
                   legends=[(kk, W * T * g) for kk, g in legends(plans[k]["fr"])] if os.environ.get("VELVET_LEGENDS") else [],   # render only
                   ledbezel=(W * mir(h["led_bezel"])) if h["led_bezel"] is not None else None, leds=[W * mir(q) for q in h["leds"]], switches=[(n, W * mir(q)) for n, q in h["switches"]],   # THE FITTINGS (build 133)
                   crown=W * mir(h["crown"]), guide_cyan=W * mir(h["guides"]["cyan"]), guide_magenta=W * mir(h["guides"]["magenta"]),   # THE CROWN + THE LIGHTS (builds 152-153)
                   fils={n: W * mir(f) for n, f in h["filaments"].items()}, fil_cyan=W * mir(h["filaments"]["cyan"]), fil_magenta=W * mir(h["filaments"]["magenta"]),
                   overlay=(W * h["overlay"]) if h.get("overlay") is not None else None)
        if k == "R":
            out.update(pad=W * h["tp_T"] * prism(tp_plan(), RIM_Z - TP_IND - TP_SKIN - TP_T, RIM_Z - TP_IND - TP_SKIN), bezel=W * h["bezel"], picover=W * h["pi_cover"], caps=[W * c for c in h["caps"]], spacers=[W * s for s in h["pi_spacers"]], hat=W * h["hat"], pi=W * h["pi"], header=W * h["header"])
        else:
            enc = Rot(90, 0, 0) * import_step(str(EC11_STEP))
            eb = enc.bounding_box()
            enc = enc & Pos(0, 0, -5) * Box(60, 60, EC11_SHAFT_TOP - EC11_PAD_Z + 5, align=BIG)   # the shaft CUT to length
            out["ec11"] = W * Pos(-KNOB_U, KNOB_Y, EC11_PAD_Z) * Pos(-eb.center().X, -eb.center().Y, 0) * enc      # FLAT (the dial block is flat since build 85)
            out["knob"] = W * Pos(-KNOB_U, KNOB_Y, KNOB_Z0) * knob()
        halves[k] = out
    return dict(plans=plans, halves=halves)


def parts(A):
    groups = []
    for k in "RL":
        h = A["halves"][k]

        def lab(q, name, col):
            q.label, q.color = name, col
            return q
        kids = [lab(h["hull"], f"hull_{k}", Color(0.12, 0.12, 0.14)), lab(h["hatch"], f"hatch_{k}", Color(0.9, 0.2, 0.6)), lab(h["pico"], f"pico_{k}", Color(0.1, 0.5, 0.2)),
                lab(h["pane"], f"pane_acrylic_2mm_{k}", Color(0.75, 0.92, 1.0)), lab(h["tpu"], f"wrist_pad_tpu_{k}", Color(0.35, 0.1, 0.45)), lab(h["cell"], f"cell_18650_{k}", Color(0.95, 0.75, 0.1)),
                lab(h["v713"], f"v713_clearance_GUESS_{k}", Color(0.9, 0.5, 0.1)), lab(h["cdoor_a"], f"charge_door_a_{k}", Color(0.2, 0.75, 0.6)), lab(h["cdoor_b"], f"charge_door_b_{k}", Color(0.15, 0.6, 0.5)), lab(h["pdoor_a"], f"pico_door_a_{k}", Color(0.2, 0.45, 1.0)), lab(h["pdoor_b"], f"pico_door_b_{k}", Color(0.2, 0.8, 0.3))]
        if k == "R":
            kids += [lab(h["pad"], "tps43_trackpad", Color(0.75, 0.05, 0.05)), lab(h["overlay"], "tp_overlay", Color(0.9, 0.6, 0.2)), lab(h["picover"], "pi_otg_cover", Color(0.25, 0.25, 0.3)), *[lab(c, f"pitft_cap_{i}", Color(0.9, 0.3, 0.2)) for i, c in enumerate(h["caps"])], *[lab(s, f"pi_stack_spacer_{i}", Color(0.85, 0.85, 0.85)) for i, s in enumerate(h["spacers"])], lab(h["hat"], "pitft_2_2", Color(0.2, 0.3, 0.8)), lab(h["pi"], "pi_zero_2_w", Color(0.1, 0.5, 0.2)),
                     lab(h["header"], "header_2x20", Color(0.1, 0.1, 0.1))]
        else:
            kids += [lab(h["ec11"], "ec11", Color(0.3, 0.3, 0.3)), lab(h["knob"], "knob_printed", Color(0.1, 0.1, 0.1))]
        for kk, sw, cap in h["keys"]:
            kids += [lab(sw, kk, Color(0.55, 0.3, 0.75)), lab(cap, f"{kk}_cap", Color(0.15, 0.9, 0.85))]
        kids += [lab(g, f"{kk}_legend", Color(0.08, 0.08, 0.1)) for kk, g in h["legends"]]
        kids += [lab(q, f"led_3mm_{k}{i}", Color(1.0, 0.25, 0.2)) for i, q in enumerate(h["leds"])] + [lab(q, f"slide_805_{n}_{k}", Color(0.2, 0.2, 0.22)) for n, q in h["switches"]] + ([lab(h["ledbezel"], f"led_diffuser_{k}", Color(0.75, 0.92, 1.0))] if h["ledbezel"] is not None else [])
        kids += [lab(h["crown"], f"crown_{k}", Color(0.12, 0.12, 0.14)), lab(h["guide_cyan"], f"light_guide_cyan_{k}", Color(0.2, 0.88, 1.0)), lab(h["guide_magenta"], f"light_guide_magenta_{k}", Color(1.0, 0.31, 0.85))]   # THE CROWN (build 153)
        kids += [lab(f, f"cob_filament_30x2_GUESS_{n}_{k}", Color(1.0, 0.9, 0.55)) for n, f in h["fils"].items()]   # THE LIGHTS (build 152), L's encoder runs (153)
        kids += [lab(h["flap"], f"flap_{k}", Color(0.85, 0.45, 0.1)), lab(h["fpin"], f"flap_pin_{k}", Color(0.8, 0.8, 0.85))]   # the stand hangs from the half: inside its group
        g = Compound(children=kids)
        g.label = f"half_{k}"
        groups.append(g)
        groups += [lab(h["base"], f"base_{k}", Color(0.3, 0.3, 0.35)), lab(h["tpin"], f"tent_pin_{k}", Color(0.8, 0.8, 0.85)), lab(h["rpin"], f"rest_pin_{k}", Color(0.8, 0.8, 0.85))]
    return groups


def kinematics():
    """From the 2D plans only (the decorator wants the dict at import)."""
    mates, poses = [], {"flat": {}}
    for k in "RL":                                              # innermost first, every parent the root (a parent that moves + a nested child = the motion twice: cyber_split latch trap)
        ta = tent_axes(k, plan_of(k))
        (fo, fd) = ta["flap"]
        mates.append(cadgen.revolute(f"flap_{k}", parent="#velvet_case", child=f"#flap_{k}", limits=(0.0, 150.0), origin=fo, direction=fd))
    for k in "RL":
        ta = tent_axes(k, plan_of(k))
        (o, d) = ta["tent"]
        mates.append(cadgen.revolute(f"tent_{k}", parent="#velvet_case", child=f"#half_{k}", limits=(0.0, 45.0), origin=o, direction=d))
        poses["flat"].update({f"tent_{k}": 0.0, f"flap_{k}": 0.0})
    for x, th, ph, _t, _c in tent_angles():
        poses[f"tent_{th:.0f}"] = {f"{j}_{k}": v for k in "RL" for j, v in (("tent", th), ("flap", ph))}
    return {"mates": mates, "poses": poses}
