"""Small printed parts of our own: the knob, the Pi USB window's cover."""
import math

from build123d import Align, Axis, Box, Cone, Cylinder, Polyline, Pos, Rot, Sphere, chamfer, extrude, make_face
from shapely.geometry import box as sbox

from velvet.params import *  # noqa: F401,F403


def knob():
    """OUR knob (Willy 2026-10-11): a scalloped cylinder, chamfered top, a round bore for the EC11's knurled 6 mm shaft. Print it top-down."""
    k = Cylinder(KNOB_R, KNOB_H, align=BIG)
    k = chamfer(k.edges().group_by(Axis.Z)[-1], KNOB_CHAMF)
    for i in range(KNURL_N):
        k -= Rot(0, 0, 360.0 * i / KNURL_N) * Pos(KNOB_R, 0, -1) * Cylinder(KNURL_R, KNOB_H + 2, align=BIG)
    k -= Pos(0, 0, -0.01) * Cylinder(KNOB_BORE_D / 2, KNOB_BORE + 0.01, align=BIG)
    k -= Pos(DIMPLE_OFF, 0, KNOB_H - DIMPLE_DEPTH + DIMPLE_R) * Sphere(DIMPLE_R)   # the finger dimple
    return k


def cover_lip(w_win, h_win, into=0.0):
    """A cover's BRIM (Willy 10:02 'add a small brim to them so we aren't attempting to set them completely flush ... opportunity to do some cool
    design'; 10:52 '2 mm chamfered lip'): a flange LIP_W past the window all round, on the wall's outer face, LIP_T proud, its outer face's four
    edges chamfered LIP_C. In the cover's frame (+y out, the wall's outer face at y = WALL / 2). `into`: the flange reaches that far INSIDE the
    wall's face too (the caller trims it to the window: a lip built in the wall's frame and a plug clipped by the outline only touch, and OCC's
    fuse left them two solids, build 114)."""
    lip = Pos(0, WALL / 2 + (LIP_T - into) / 2, 0) * Box(w_win + 2 * LIP_W, LIP_T + into, h_win + 2 * LIP_W, align=MID)
    face = max(lip.faces(), key=lambda f: f.normal_at().Y)
    lip = chamfer(face.edges(), length=LIP_C)
    assert lip.is_valid and len(lip.solids()) == 1, "cover_lip: the chamfer failed"
    return lip


def usb_window(w=CHG_W, h=CHG_H, r=CHG_R, mouth=CHG_MOUTH):
    """A CHARGING WINDOW's cutter (Willy 12:25 'make the charger holes sexier ... wide enough to accept all usb cords'): a w x h bore through the
    wall with its four long edges rounded r, and its mouth on the outer face flared 45 deg by `mouth` all round (a tapered extrude of the rounded
    rectangle: convex, so it is safe -- memory: tapered extrudes of concave outlines corrupt booleans). In the window's frame: centred on the wall,
    +y out, the wall's outer face at y = WALL / 2; the bore runs WALL + 2 (1 past each face)."""
    rr = min(r, h / 2 - 0.2)
    bore = Box(w, WALL + 2.0, h, align=MID)
    bore = bore.fillet(rr, [e for e in bore.edges() if abs(e.length - (WALL + 2.0)) < 0.01])
    sec = Pos(0, WALL / 2 - mouth, 0) * Rot(90, 0, 0) * make_face(Polyline(*[(x, y) for x, y in sbox(-w / 2 + rr, -h / 2 + rr, w / 2 - rr, h / 2 - rr).buffer(rr, quad_segs=6).exterior.coords[:-1]], close=True))
    flare = extrude(sec, amount=mouth + 0.5, taper=-45)                                           # grows 1 per 1 toward the face: the mouth w + 2 mouth wide
    out = bore + flare
    assert out.is_valid and len(out.solids()) == 1, "usb_window: the flare did not fuse"
    return out


def usb_cover(w_win=USB_W, h_win=USB_H, lip=True):
    """A USB window's COVER (the Pi's at USB_W x USB_H, the Pico's at PLUG_W x PLUG_H) (Willy 12:39 'a nice streamlined cover for it that fits tight'): a plug the window's size less COVER_CLR a side,
    WALL deep (flush inside), crush ribs RIB_W x RIB_H on its top and bottom faces (the bezel's press fit: 0.2 interference a side), and its BRIM
    (cover_lip) on the wall's face unless lip=False (the Pico's lip is placed in the wall's own frame: that wall is yawed 5 deg to the bay).
    Centred on the window, +y out. Print it outer face down. (The fingernail notch of builds 1-113 is gone: the lip is the grip.)"""
    w, h = w_win - 2 * COVER_CLR, h_win - 2 * COVER_CLR
    c = Box(w, WALL, h, align=MID)
    for sz in (-1, 1):
        for sx in (-1, 1):
            c += Pos(sx * w / 4, -0.25, sz * (h / 2 + RIB_H / 2 - 0.01)) * Box(RIB_W, WALL - 0.5, RIB_H + 0.02, align=MID)   # the ribs stop 0.5 short of the outer face
    if lip:
        c += cover_lip(w_win, h_win)
    return c



def led_bezel(tray, w, skew, at):
    """THE LED INNER BEZEL (build 140; Willy 22:58 'another dope cyberpunk inner bezel to drop in, plug the space between leds and their holes and offer
    some cool design that integrates? i can print in transparent'): a drop-in plate in the LED tray (hull.led_tray's parallelogram, the LEDs `at`),
    TP_IND thick -- flush with the badge -- its top edge bevelled 45 deg into the tray's TRAY_FLARE mouth; a spigot per LED down the LED_SEAT bore
    to 0.2 over the flange (it plugs the 0.5 gap round the 2.9 body and locates the plate); a window per LED (the body through, the dome 1.0 proud)
    countersunk 45 deg at the top; a groove between neighbours, running with the tray's skewed ends. Print it transparent, top up. Tray frame:
    z 0 = the tray's floor, xy = the plan (the caller places it: hull.fittings)."""
    clr, spig_d, win_d, sink, (gw, gd) = LED_BEZEL
    inner = tray.buffer(-clr, join_style=2)
    face = lambda z: make_face(Polyline(*[(x, y, z) for x, y in inner.exterior.coords[:-1]], close=True))
    part = extrude(face(0.0), amount=TP_IND - TRAY_FLARE) + extrude(face(TP_IND - TRAY_FLARE), amount=TRAY_FLARE, taper=-45)   # the plate + its lip, CLR inside the mouth
    spig = LED_SINK - 1.0 - 0.2                                                                    # down the seat bore to 0.2 over the flange's top (the flange is 1.0 tall)
    xs, ys = [x for x, _ in at], sorted(y for _, y in at)
    for x, y in at:
        part += Pos(x, y, -spig) * Cylinder(spig_d / 2, spig + 0.1, align=BIG)
        part -= Pos(x, y, -spig - 1.0) * Cylinder(win_d / 2, spig + TP_IND + 2.0, align=BIG)
        part -= Pos(x, y, TP_IND - sink) * Cone(win_d / 2, win_d / 2 + sink + 0.2, sink + 0.2, align=BIG)   # 45 deg: + sink at the top
    ang = math.degrees(math.atan2(skew, w))
    for ya, yb in zip(ys, ys[1:]):
        part -= Pos(sum(xs) / len(xs), (ya + yb) / 2, TP_IND - gd) * Rot(0, 0, ang) * Box(w - 2.2, gw, gd + 0.1, align=BIG)
    assert part.is_valid and len(part.solids()) == 1, f"the LED bezel is {'invalid' if not part.is_valid else str(len(part.solids())) + ' solids'}"
    return part


def oval_cover(w_win, h_win, lip_w=PI_COVER[0], lip_t=PI_COVER[1]):
    """THE OTG COVER (162; Willy 22:04 'a nice cover on that pi usb cable access, a friction fitting one low profile'): an elliptic plug the oval
    less COVER_CLR a side, WALL deep (flush inside), four crush ribs RIB_W x RIB_H at the axes' ends (0.2 interference: the bezel's press fit), and
    a low brim lip_w past the oval all round, lip_t proud on the wall's face, its outer edge rounded. Centred on the oval, +y out, the wall's outer
    face at y = WALL / 2 (usb_cover's frame). Print it face down."""
    from velvet.doors import ellipse
    from velvet.geo import prism
    w, h = w_win - 2 * COVER_CLR, h_win - 2 * COVER_CLR
    body = Rot(-90, 0, 0) * prism(ellipse(w, h), -WALL / 2, WALL / 2 + 0.2)                              # 0.2 out past the face, into the brim: one solid
    for s_ in (-1, 1):
        body += Pos(0, -0.25, s_ * (h / 2 + RIB_H / 2 - 0.01)) * Box(RIB_W, WALL - 0.5, RIB_H + 0.02, align=MID)          # the ribs stop 0.5 short of the outer face
        body += Pos(s_ * (w / 2 + RIB_H / 2 - 0.01), -0.25, 0) * Box(RIB_H + 0.02, WALL - 0.5, RIB_W, align=MID)
    lip = Rot(-90, 0, 0) * prism(ellipse(w_win + 2 * lip_w, h_win + 2 * lip_w), WALL / 2, WALL / 2 + lip_t)            # on the wall's face (not into it: 162's first cut pressed 14 mm3 of brim into the wall)
    face = max(lip.faces(), key=lambda f: f.center().Y)
    lip = lip.fillet(min(lip_t - 0.2, 0.5), face.edges())
    out = body + lip
    assert out.is_valid and len(out.solids()) == 1, "oval_cover: the brim did not fuse"
    return out


def button_cap(slot, z_bot, z_top, lip_w=CAP[1], lip_t=CAP[2], clr=CAP[3]):
    """A PiTFT BUTTON CAP (162; Willy 22:09 'pi tft has these type of button that are going to require you to build some custom button covers,
    they are standard tactile buttons 1mm tall'): a stem the bezel's slot less `clr` a side from the plunger's top z_bot to z_top (CAP[0] proud of
    the bezel), its top edges chamfered 0.5; a foot lip_w past the slot all round, lip_t thick, at the stem's bottom -- it rides in the bezel's
    button-bar relief and the bezel keeps it in. In the bezel's frame (F); `slot` the slot's polygon (clipped corners)."""
    from velvet.geo import prism
    stem = prism(slot.buffer(-clr, join_style=2), z_bot, z_top)
    top = max(stem.faces(), key=lambda f: f.center().Z)
    stem = chamfer(top.edges(), length=0.5)
    cap = stem + prism(slot.buffer(lip_w, join_style=2), z_bot, z_bot + lip_t)
    assert cap.is_valid and len(cap.solids()) == 1, "button_cap: the foot did not fuse"
    return cap
