"""Print STLs, each in its print orientation."""


from build123d import Compound, Cylinder, Plane, Pos, Rot, mirror

from velvet.params import *  # noqa: F401,F403
from velvet.geo import write_stl as export_stl                                   # absolute 0.05 mesh, every face or it raises (build 148)
from velvet.stand import flap, pin_coupon, flap_stub
from velvet.parts import knob


def export(A):
    out = HERE.parent / "STL"                                                               # the project's STL/ (HERE is the velvet package: the files went to velvet/STL, build 131)
    out.mkdir(exist_ok=True)
    for k in "RL":
        h = A["halves"][k]
        doors = [h["pdoor_a"], h["pdoor_b"], h["cdoor_a"], h["cdoor_b"]]      # THE SLIDE DOORS (the Pico plug's, 155; the charging window's, 162) print in place inside the hull (build 155): one STL, the leaves as their own shells
        hd = Compound(children=[h["hull"], *doors])
        export_stl(Pos(0, 0, -hd.bounding_box().min.Z) * hd, str(out / f"velvet_hull_{k}.stl"))   # BOTTOM-DOWN (164; Willy 01:45): the top prints clean, supports inside under the deck
        export_stl(h["hatch"], str(out / f"velvet_hatch_{k}.stl"))                       # countersinks down
        export_stl(Pos(0, 0, -h["tpu"].bounding_box().min.Z) * h["tpu"], str(out / f"velvet_wrist_pad_tpu_{k}.stl"))   # TPU, as it sits
        export_stl(Pos(0, 0, -h["pane"].bounding_box().min.Z) * h["pane"], str(out / f"velvet_pico_pane_{k}.stl"))   # (164) THE PANE (159), clear, as it sits: the plates lay it flat
        mir = (lambda q: q) if k == "R" else (lambda q: mirror(q, about=Plane.YZ))        # THE LIGHTS (build 152): L's parts are mirror images (the half is built
        flat = lambda q: Pos(0, 0, -q.bounding_box().min.Z) * q                            # in the u frame and mirrored into the world, like the hull)
        loc = h["local"]
        export_stl(flat(Rot(180, 0, 0) * mir(loc["crown_local"])), str(out / f"velvet_crown_{k}.stl"))                 # THE CROWN (build 153) top-down: the slots and ridges on the bed, the pins (R: the bezel's legs, the collar) up
        for n, g in loc["guides_local"].items():
            export_stl(flat(Rot(180, 0, 0) * mir(g)), str(out / f"velvet_light_guide_{n}_{k}.stl"))                   # upside down: the stem on the bed, the channels open up (clear)
        if loc["diffuser_local"] is not None:
            export_stl(flat(mir(loc["diffuser_local"])), str(out / f"velvet_led_diffuser_{k}.stl"))                   # as it sits: the pockets down, the arms up (clear)
        if k == "R":
            export_stl(flat(loc["overlay_local"]), str(out / "velvet_tp_overlay_R.stl"))
            export_stl(flat(Rot(-90, 0, 0) * loc["pi_cover_local"]), str(out / "velvet_pi_otg_cover.stl"))                          # THE OTG COVER (162): face down
            export_stl(flat(loc["pi_spacer_local"]), str(out / "velvet_pi_stack_spacer.stl"))                                      # THE STACK SPACER (164), print 2
            export_stl(flat(Rot(180, 0, 0) * loc["caps_local"][0]), str(out / "velvet_pitft_cap.stl"))                              # A BUTTON CAP (162), print 4: top down                                # THE OVERLAY (build 153): flat, another colour
    export_stl(knob(), str(out / "velvet_knob.stl"))                                    # as it sits: the dimple up (top-down it would hang), the bore's roof bridges 6
    for k in "RL":
        h = A["halves"][k]
        export_stl(h["base"], str(out / f"velvet_base_{k}.stl"))                            # as it sits: the slab flat, knuckles up
        pl = A["plans"][k]
        Kf, z_ax = h["local"]["Kf"], h["local"]["z_ax"]                                                        # THE PINS as placed (their heads are the case's form): back in K, the axis to the origin, flat on the D
        for n, q, seat, sense in (("tent", h["tpin_u"], h["seat_t"], 1), ("rest", h["rpin_u"], REST_FRONT + PIN_HEAD[1], -1)):
            q = Rot(0, 0, 180 if sense > 0 else 0) * Pos(-pl["ax_x"], -seat, -z_ax) * (Kf.inverse() * q)   # the shank along +y from the seat, the head at -y
            export_stl(Pos(0, 0, PIN_D / 2 - PIN_FLAT) * q, str(out / f"velvet_{n}_pin_{k}.stl"))
    for rh in (0.2, 0.3, 0.4):                                                                  # THE SNAP RING COUPONS: a knuckle block + a stub pin per ring height; print, pick the click
        export_stl(pin_coupon(rh), str(out / f"velvet_pin_coupon_{rh:.1f}.stl"))
    export_stl(Pos(0, 0, FLAP_RB) * Rot(180, 0, 0) * flap(), str(out / "velvet_flap.stl"))   # upside down: the plate top, barrel top and nose top are flush at +RB -> flat on the bed, no support (118 on its side is too slender)
    export_stl(flat(flap_stub()), str(out / "velvet_flap_pin.stl"))   # (164) a stub, print 2
