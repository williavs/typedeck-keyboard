"""velvet_fold parameters: every dimension, clearance and placement constant, with its reason. Nothing here touches the CAD kernel beyond frames."""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent.parent / "cyber_split"))
import velvet_layout as VL  # noqa: E402
import cyber_display as DP  # noqa: E402
from build123d import Align, Pos  # noqa: E402

SL = VL.SL


X_IN = {"R": 84.5, "L": 55.0}   # the inner column's home key from the half's inner edge: R = wall 3 + pocket 65.62 + wall 3 + 12.9
                                # (margin 1 + half a cap 9 + the yaw's 1.7 + slack); L = the knob housing's 15.3 + the shot's 39.5
YAW = VL.YAW                    # -5: the block turned tops-outward inside its half (the same sign in both u frames)
SEP = 20.0                      # the halves on the desk: inner edges this far apart (cosmetic)
TOP, POCKET = 3.0, 1.2          # deck (switch plate) thickness, EVEN everywhere under the keys (Willy 00:20 'even, thick'); the KS-33 clip lip pocket from below leaves 1.2 at the clips
HOLE, POCKET_W = 14.0, 15.0     # KS-33 plate cut-out (body 14.02); pocket under it (the barbs reach 14.6)
FLOOR = -12.5                   # the hatch top: the 18650 (19.4 bore) and the Pi stack's posts
PLATE_T = 2.0                   # the hatch
ZB = FLOOR - PLATE_T            # the underside, -14.5, everywhere   # ponytail: ONE thickness; a 20.2 key zone saves 4.2 under the hands but needs a stepped hatch
WALL, MARGIN = 3.0, 1.0         # walls; cap footprint to the well wall
FLOW, ROUND = 18.0, 4.0         # the outline's closing / opening radii (18: the creases between the column tops and round the thumb row fill)
EDGE_R_TOP, EDGE_R_BOT = 1.8, 1.5   # ROUNDED outline edges -- the top one = BORDER_R, ONE radius all round (Willy 23:14 'the wrist rest curve should be applied all the way around'); (Willy 00:08 'clean up all edges by rounding'): every top edge (deck, plateau, palm) / every bottom edge, except the hinge side, as inset layers (geo.layered_prism)
SUNK = 0.8                      # cap tops under the rim plane
RIM_Z = SL.CAP_TOP + SUNK       # 9.9
H = RIM_Z - ZB                  # the slab, 24.4
ROOF_T = 2.4                    # R's wire bay's roof, L's charger bay's roof
INSERT_D, INSERT_L, BOSS_D = 4.0, 5.0, 7.5   # M3 heat-set insert, boss
SCREW_D, HEAD_D = 3.4, 6.5      # M3 clearance, countersunk head
POCKET_CLR = DP.LID_POCKET_CLR  # 0.3 round the HAT in its pocket
GLASS_SINK = 2.4                # the glass this far under the rim (the HAT's button bar, 4.3 over the glass in the STEP, then stands 1.9 proud)
PK_W = DP.HAT_W + 2 * POCKET_CLR                 # 65.62
PK_D = DP.HAT_D + 2 * POCKET_CLR                 # 57.6
U_PK0, U_PK1 = WALL, WALL + PK_W                 # the display pocket, u
WIRE = 8.0                      # the wire bay behind the pocket (the HAT's header tails, the Pi's wires)
BALL_D = VL.BALL_D
BALL_U, BALL_Y = X_IN["R"] - 39.5, -15.5         # the ball's centre: the shot's 39.5 inward of the inner column
CUP_CLR, CUP_T = 0.5, 1.5       # round the ball; the cup floor (= the underside)
BALL_Z = ZB + CUP_T + BALL_D / 2                 # 4.0: "drop it down into the frame as much as we can" (the bowl's floor 1.0 over the underside)
CUP_R = BALL_D / 2 + CUP_CLR                     # 17.5
CUP_WALL = 2.5                  # the cup's pedestal wall through the cavity
TP_W, TP_D, TP_T = 43.3, 40.3, 2.03   # Azoteq TPS43 (datasheet v1.06 fig 2.1, stack-up A: PCB 1.0 + 3M 468 0.13 + parts), +-0.2 / +-10%
TP_R = 2.0                      # its rounded corners: GUESS (the datasheet draws them, no number)
TP_CLR, TP_SKIN = 0.3, 0.6      # round it in its pocket; the printed skin over it = the overlay (datasheet: 0.2 mylar .. 3 mm; thinner = more signal)
TP_IND, TRAY_M, TRAY_FLARE = 0.8, 0.4, 0.3   # THE TRAY (Willy 22:16 'i wanted the sunken down area for the trackpad'): the pad's footprint + TRAY_M all round sunk TP_IND into the
#                                             console's top (the skin over the pad stays TP_SKIN: the pad sits TP_IND lower), its mouth flared 45 deg by TRAY_FLARE; the console's
#                                             wall beside the pocket is WALL, its edge round EDGE_R_TOP: TRAY_M + TRAY_FLARE <= WALL - EDGE_R_TOP - 0.5 (build 121 at 1.5 + 0.5 ran into the round)
KNOB_D, KNOB_CHAMF = 32.0, 0.5  # OUR knob (32: Willy 21:31 "bigger diameter ... centre closer to the keys"), SUNK FLUSH (Willy 21:26 "totally sink in the encoder ... finger sized dimple ... really tight slick")
KNURL_N, KNURL_R = 0, 0.8       # plain sides: sunk, nothing to grip there
KNOB_BORE_D = 6.0               # round bore on the EC11's knurled 6 mm shaft (r 2.96), press fit GUESS (+ a drop of glue)
KNOB_CLR = 0.3                  # the well round the knob: tight -- the EC11's nut centres it in the deck hole
DIMPLE_D, DIMPLE_DEPTH = 9.0, 1.5   # the finger dimple: a sphere cap off-centre for spinning it (its centre 1.5 in from the rim of the knob)
PANEL_T, CUP_W = 2.4, 2.0      # the panel the EC11's bushing goes through (its nut on top, M7 x 2 GUESS); the cup's wall under the deck
EC11_BODY_TOP, EC11_BUSH_TOP, EC11_KNURL0 = 4.5, 9.5, 12.5   # the STEP over its pads: the bracket's top face, the bushing's, the knurl's start
KNOB_R = KNOB_D / 2
DIMPLE_OFF = KNOB_R - DIMPLE_D / 2 - 1.5
HOUSING_R = KNOB_R + KNOB_CLR + WALL             # 15.3: the round knob housing at L's inner-front corner
KNOB_U, KNOB_Y = HOUSING_R, BALL_Y               # the knob's centre on L: its housing's nose is the inner edge
EC11_STEP = VL.EC11_STEP
EC11_PAD_Z = FLOOR + 0.8 + 0.4   # SUNK (Willy 21:39 'sink the module down so im not cutting the shaft'): its legs (0.8 under the pads) 0.4 off the hatch
WELL_FLOOR = EC11_PAD_Z + EC11_BODY_TOP + PANEL_T  # the knob well's floor = the panel's top: the EC11 hangs from it, bracket against its underside
PEDESTAL = 15.0                 # the panel's boss under the deck (no boss over the hatch there)
Y_P0 = 5.0                                       # the display pocket's front (R; was set by the ball cup)
TP_U, TP_Y = BALL_U - 4.0, Y_P0 - 2.0 - TP_CLR - TP_D / 2   # the pad's centre: 4 inside the ball's u (the console block is FLAT: its wall + the seam ring need 2 WALL between the
#                                 pocket and the tilted thumb well, it had 2.3), 2.0 of wall in front of the display pocket
CELL_STEP = SL.REFS / "modules/18650_cell.step"
CELL_L, CELL_R, CELL_CLR, LEAD = 65.0, 9.2, 0.5, 3.0   # LEAD: room at each blind end for a tabbed cell's folded tab + its soldered wire down into the cavity (no spring contacts, Willy 11:10)
CELL_X0 = -13.0                 # the cell's bore starts here in K (behind the inner column), runs outward along the top row
V713 = (25.0, 20.0, 4.0)        # Willy's charger/boost (references/modules/CHARGER_V713_SOURCE.md): board from the listing, height the caretaker's; NO STEP
V713_CLR, V713_PAD, V713_FENCE = 0.5, 2.0, 1.5   # its bay: clearance round it (0.5: R's bay sits between the Pi's USB plug path and the HAT's rear-outer column, 27.6 of room for a 25 board + fence), a pad under it on the hatch, a fence round it
V713_PCB, V713_USB_Z = 1.0, 4.1                  # GUESS: PCB thickness; the USB-C receptacle's centre over the board's underside (162: 2.6 -> 4.1, Willy 22:04 'raise the
#                                 charging holes a little (the usb hole is sitting on top of the board)': +1.5, his board's receptacle sits higher than the guess)
V713_R = (34.3, 44.0)           # R: the bay's inner-front corner (u, y): under the HAT's rear, 25 along u, the USB-C edge to the back; OUT OF THE PI'S USB PLUG PATH (Willy 12:39
#                                 'a reliable cutout to reach the pi usb'): the OTG port (u 26.9) faces the back wall through the pocket's rear, where the bay sat (u 12..39, build 84);
#                                 its bay cut (+ fence + 0.2) now ends 0.3 inside the HAT's rear-outer column (x 62.3), the fence 0.4 off an 11 mm plug
PI_USB_F = -8.90                # the Pi Zero 2 W's USB (OTG) micro-B centre x in F, MEASURED off its STEP (scratchpad pi_ports.py: x 22.92..30.90 in u, F2H.x 35.81; PWR 12.6 inward,
#                                 the SD slot at the OUTER end, the camera FPC at the inner); hull.inner_bay asserts it against the STEP's connector
COVER_CLR = 0.15   # the USB window's printed COVER (Willy 12:39 'a nice streamlined cover for it that fits tight'): this much a side in the window, crush ribs
LIP_W, LIP_T, LIP_C = 2.0, 1.2, 1.0
BZ_CORNER, BZ_HOOK, LIP_R, LIP_R_SEAM = 4.0, 1.5, 1.0, 1.8   # THE BEZEL'S FIT (build 141; Willy 00:21 "the pitft bezel should have rounded corners and fitt smoother with
#                                 the key wall and outwr rounded edge of the sidewall"): the brim's key-side plan corners' r; its seam-side LIP runs out to the seam face
#                                 as a hook this wide wrapping the wall's r 1.8 round (its underside the hull's own surface), from the console's rounded ridge to the
#                                 flange's north end; the brim's top edges rounded LIP_R (the 45 deg LIP_C chamfer of builds 114-140; the covers keep theirs) and the
#                                 lip's seam edge LIP_R_SEAM: the console's seam-edge profile (11.1 down to 9.3 at the face) carried through the bezel in one line
CONSOLE_UP = LIP_T              # THE CONSOLE RAISED (Willy 12:25 'the area around the trackpad can be slightly raised so when i place the pi tft bezel it appears to integrate'): R's pitched console in front of the display pocket stands this much over the rim at its hinge line, so the bezel's brim top (RIM_Z + LIP_T) and the console are one surface; the brim's front flange sits in a seat cut into it    # every cover's BRIM (Willy 10:02 'add a small brim to them so we aren't attempting to set them completely flush'): a flange LIP_W
                                        # past the window on the wall's face, LIP_T proud, its outer edges chamfered LIP_C (Willy 10:52: '2 mm chamfered lip')
#                                 RIB_W x RIB_H on its top and bottom faces (the bezel's press fit), a fingernail notch (width, depth) along its outer face's bottom edge
SD_RELIEF = (2.5, 0.6)          # the microSD card's relief into the pocket's outer wall: this deep past the PCB edge, this much round the slot (the card stands ~2 proud; build 84 gave it 0.3)
SD_R = 1.2                      # (162; Willy 22:04 'make the sd card slot less of a rectangular box and make it sexy like the other holes'): the relief's four long edges rounded this r
V713_L = (WALL + V713_FENCE, 7.0 + V713_FENCE)   # L: behind the knob housing, its fence against the INNER wall (the 32 knob had pushed it into the thumb wells), 25 along y, the USB-C out the inner face
USB_W, USB_H = 12.0, 8.0        # USB plug overmold through a wall   GUESS (the Pi's OTG window on R's back wall, the V713's)
CHG_W, CHG_H, CHG_R, CHG_MOUTH = 13.0, 9.0, 2.5, 1.0   # THE CHARGING WINDOWS (the V713's USB-C: R's back face, L's inner face; Willy 12:25 'make the charger holes sexier? no covers. wide enough to accept all usb cords'): the Pico tunnel's 13 x 9 (a 12 x 8 overmold + 0.5 a side), corners rounded, the mouth flared 45 deg by CHG_MOUTH on the outer face; no cover
PLUG_W, PLUG_H = 13.0, 9.0      # the PICO'S PLUG TUNNEL (Willy 21:06 'reliable tunnel for all sized usb'): a bore this wide and tall from the outer face to the
#                                 connector -- a 12 x 8 overmold (the biggest common micro-B) with 0.5 a side; checks run a 12 x 8 plug path through it
PICO_DROP = 1.0                 # the Pico hangs this much lower under the bay's ceiling than the pane needs: the bore then leaves 2.1 of skin under the
#                                 pad's recess floor (the connector's centre was 2.6 under the tunnel's ceiling: no overmold over 5 tall reached it, builds 1-95)
PICO_HOLES = ((4.8, 2.0), (4.8, 49.0), (16.2, 2.0), (16.2, 49.0))
PICO_PILOT, POST_D = 1.6, 5.0
FRAME_T = 1.6                   # the printed frame over each Pico's pane, flush with the rim (Willy 21:16 'matching bezel ... hold nicely in the plexi')
PANE_T, PANE_SINK, PANE_CLR, LEDGE, LEDGE_T = 2.0, 2.0, 0.2, 1.5, 1.6   # 2 mm acrylic in a recess on a 1.6 ledge, sunk PANE_SINK = PAD_DEPTH: its top on the pad recess's floor (159; was FRAME_T under
#                                 its frame on a 2.0 ledge -- BAY_DOWN stays 5.6)
PANE_WIN = (1.0, 7.5)           # THE LENS (159-161; the lip, the ring and the bezel D of 159-160 are gone: the lens is the TPU pad's own hole, its edge rounded PANE_R) (build 159; Willy 19:35, red lines on 158's render: 'make this the approximate new wrist rest cover for the pi picos - keep the same
#                                 size of the plexi, but lets lose the sharp bezel and do something to your liking more idiomatic ... extend the wrist pad and case over the plexi'):
#                                 NO FRAME; 19:59 'theres only supposed to be a small piece of the pi pico exposed now and its replaced with more wrist rest'): the TPU pad
#                                 rises over the pane and the case's LIP (the deck + border, LIP_IN wide) laps its top edge; THE WINDOW = a D over the board's top edge, PANE_WIN
#                                 tall at its ends and at its middle (the pad's edge is the circular arc between, dipping at the middle: a lens closing at the pane's top corners --
#                                 20:01 'fill in this whole area colored with red': the pad up to the curve), corners r 1.0; 20:14 'use bezel d shape of the pi with the case not the
#                                 rest tpu': THE BEZEL RING -- the case keeps a PANE_BEZ wide D round the lens (the deck + border over the pane, 2.7 thick), the pad outside it.
#                                 The pane slides in under
#                                 the lip from the pad's side (its cavity runs LIP_IN further that way), the pad goes on after and holds it. FRAME_T / FR_* / LIP_W / LIP_T / LIP_C: unused since 159
LENS_IN = 4.0                   # THE LENS DOWN THE BOARD (160; Willy 20:45 'scoot the picos up a little bit so we dont see the pins and instead see some of the cool
#                                 resistors and stuff'): the lens's top edge this far inside the board's edge (the pin row + its silk 3.5); the bay itself sits as
#                                 close to the keys as plan.py allows (3.5 to the pit), so the window moves down the board, not the board up
PANE_R = 1.0                    # THE WINDOW'S ROUND (builds 160-161; Willy 20:26 'curve the edges like all the other edges in the build', 21:25 'a curved edge on the d
#                                 shaped hole in the tpu to simplify things'): one r on the top edge of the TPU pad's lens hole (OCC's fillet on the part, hull.wrist_rest)
BAY_DOWN = PANE_SINK + PANE_T + LEDGE_T        # the Pico bay's ceiling, this far under the deck's surface (the bay sits in the sloped palm deck)
FR_IN, FR_RUN, FR_FIT = 1.0, 1.2, 0.1   # the frame laps 1.0 over the board's edge; its bevel's run; its press fit in the pane's recess
BZ_CLR, BZ_LEG_D = 0.15, 5.0    # the Pi bezel: fit in the pocket, its legs (on the HAT's PCB over the pegs)
BZ_RUN, BZ_GROOVE = 2.0, (0.8, 0.4)   # its window bevel's run; the engraved traces (width, depth)
BZ_SLOT = (0.8, 1.5)            # THE KEYHOLES (build 138; Willy 21:36 'some bezely love to the four keyholes on the pitft bezel'): each button slot's mouth bevelled
#                                 45 deg this run into the brim, the mouth's corners clipped like the window's (two big by this, two small by CLIP_S); the slot's own
#                                 corners clipped CLIP_S (0.8 would touch the post's corner at 0.4 clearance); the traces keep run + 0.5 off the slots
BZ_DIP = 0.3                    # the bezel's BRIM (hull.bezel_brim) lies on the console's rim only where its top is within this of RIM_Z
CLIP_S, CLIP_B = 0.6, 6.0       # the cyberpunk corners: two small, two big (diagonal)
PICO_TOP_CLR, PAD_CLR, PAD_W, PAD_L = 0.3, 0.15, 6.3, 4.5   # the Pico's top under the ceiling; the corner pads over the PCB (clear of the USB shell, x 6.51..14.49 in the STEP)
PAD_L_USB = 3.5                 # the USB-end pads stop short of the small parts beside the connector (the STEP: parts to y 46.6, z 0.67 over the PCB)
PICO_SCREW = 5.5                # M2 pilot up into the inward pads + ledge
BAY_W, BAY_L = 24.0, 54.0       # the Pico bay: the board + 1.5 round. IN THE PALM DECK between the rest and the alphas, along K x with the plug end at the
#                                 outer wall (Willy 08:47 'move the picos to that blank space between the wrist rest and kb', 10:13 'hug the keys ... hug the
#                                 current wrist rests', 10:23 'parallel with wrist rest and keys'); the outer-rear bay of v3-v5 is gone (the hull is shorter there)
PI_HOLES_F = ((-29.0, 7.15), (29.0, 7.15), (-29.0, 30.15), (29.0, 30.15))   # the Zero's 58 x 23 holes in F: the STEP's circles r 1.35 sit at (+-29.0, 7.2 / 30.2), 0.05 off
HAT_HOLES_F = ((-29.0, 7.1), (29.0, 7.1), (-29.0, 56.0), (29.0, 56.0))        # the PiTFT's 4 corner holes in F, MEASURED off its STEP (r 1.3); DP.HAT_HOLES had them 1.35 too far in (the bezel's outer legs sat half in the pocket wall: Willy 'outer screw holes are cut off')
STACK_SCREW, FRONT_POST_H, PI_SPACER = (2.2, 4.5), 5.0, (5.0, 2.9)   # SCREWED FROM THE TOP (164; Willy 00:14 'screw it in from the top and the soldered pi will stay in place - make those four pegs have pi sized screw holes and eliminate any other pi tft harness'): M2.5 self-tapping pilots (dia, deep); the two front posts under the Pi; the printed spacers between the boards at the front holes (OD, ID; PI_GAP long)
# (163-: PEG_D, PEG_L = 2.4, 1.3 -- NO SCREWS (Willy: 'hold pi in place ... with pretty simple friction'): pegs from the columns into the Pi's 2.75 / the HAT's 2.7 holes locate the boards; the header's 40 pins hold the Pi, the hatch backs it up 0.3 below
RIB_N, RIB_W, RIB_H = 3, 1.2, 0.35  # the bezel's crush ribs a side, on all FOUR faces (Willy 12:39 'really tight and secure'; press-fit in its pocket: 0.15 clearance, ribs 0.35 proud -> 0.2 interference)
BZ_RIBS = 4 * RIB_N - 1         # the front face loses its middle rib to the pry notch
PRY_W, PRY_D = 10.0, 2.0        # the fingernail notch in the pocket's front wall top to lift the bezel
BIG = Align.CENTER, Align.CENTER, Align.MIN
MID = Align.CENTER, Align.CENTER, Align.CENTER
PI_GAP = 6.0                    # THE STACK AS BUILT (build 161; Willy 20:14 'this middle header does not exist on my pi stack - its soldered with the plain headers
#                                 directly into the pitft', 20:50 'the pcbs have 6mm of space between them'): the Pi's PCB top this far under the HAT's underside, no
#                                 socket; the shared cyber_display stack (HEADER_BODY + SOCKET_H = 11.0) is cyber_split's -- velvet places its own Pi (hull.stack_in_F)
Z_PI_TOP = DP.Z_HAT - PI_GAP    # the Pi PCB's top in F (-8.5; DP: -13.54)
Z_PI = Z_PI_TOP - DP.PI_PCB     # its underside
F2H = Pos((U_PK0 + U_PK1) / 2, Y_P0 + POCKET_CLR - DP.LID_Y0, RIM_Z - GLASS_SINK - DP.Z_GLASS)   # the stack's frame F -> R's u frame
KNOB_Z0 = EC11_PAD_Z + EC11_BUSH_TOP + 0.4     # 2.0: the knob's bottom just over the bushing (the nut under it on the panel)
BADGE_SEAM_Y, BADGE_W = 9.5, 2.5   # (build 145) L's badge reaches the seam face this far either way of the knob (the housing's circle is 2.5 off the face there; it kisses it at kc.y); R's wedge corners (r 1.5 refused the r 1.8 round)
BADGE_T, BADGE_R, BADGE_M, BADGE_IN = 2.0, 1.8, 2.5, EDGE_R_TOP + 0.5   # THE BADGE (build 136; Willy 17:37 'make the encoder casing more rounded now and appear to integrate with
#                                 the lights', 20:38 'encoder casing a'): L's knob housing, a tongue and the LED tray's plate raised BADGE_T over the flat zone as ONE plate
#                                 (hull.badge), its top edge rounded BADGE_R all round (the housing's step edge keeps the EDGE_R_TOP round it had; OCC refused 1.8 on the
#                                 step edges with 1.0 elsewhere, and a 1.2 plate with r 1.0 read flatter than today's housing), the tray BADGE_M inside the plate's edge,
#                                 the plate BADGE_IN inside the flat zone's other edges and the outline (their rounds run on unbroken, a 0.5 flat, then the badge).
#                                 R (build 137; Willy 21:11 'a similar thing with the lights on the right side'): an ISLAND, the tray + BADGE_M, on the raised console beside
#                                 the touchpad, 0.55 off its tray's mouth, 0.6 off the bezel brim's seat, 2.4 off the console's edge -- the row at x 11.8, y -4.2 / -9.7 / -15.2
#                                 and the tray's end margin 2.5 (was 3.0) are what fits there clip-free (scratchpad rbadge_fit: m 3.0 fits nowhere; a clipped plate's slivers
#                                 broke OCC's fillet)
KNOB_TOP = RIM_Z + BADGE_T - 0.2                # flush IN THE BADGE: 0.2 under its top (was 0.2 under the rim)
KNOB_H = KNOB_TOP - KNOB_Z0
DIMPLE_R = (DIMPLE_D ** 2 / 4 + DIMPLE_DEPTH ** 2) / (2 * DIMPLE_DEPTH)   # the dimple's sphere
_d = DIMPLE_OFF - KNOB_BORE_D / 2                                         # the dimple's centre to the bore's edge
_dip = max(0.0, DIMPLE_DEPTH - (DIMPLE_R - (DIMPLE_R ** 2 - min(_d, DIMPLE_R) ** 2) ** 0.5))   # the dimple's depth over the bore's edge
EC11_SHAFT_TOP = min(EC11_PAD_Z + 19.44, KNOB_TOP - max(_dip + 0.6, 1.2) - 0.3)   # the shaft's tip, UNCUT if it fits under a 1.2 roof
assert EC11_SHAFT_TOP == EC11_PAD_Z + 19.44, 'the shaft would need cutting'
KNOB_BORE = EC11_SHAFT_TOP + 0.3 - KNOB_Z0
TOPS = [VL.U + VL.COL_Y[c] + 9.0 + MARGIN for c in range(VL.COLS)]   # each column's well top in K
TILT_DROP = 9.0                 # the tilted body sits this much lower (Willy 23:13 'thinner ... tilt does not demand thickness'): the front-most thumb switch 0.7 off the hatch (scratchpad slack.py)
KEY_TILT = 10.0                 # deg: the WHOLE top shell behind the rest rises toward the back (Willy 21:48 '10 degree tilt'; 22:28 'lippage ... nasty opening': tilting only the key zone made fins), hinged across its top-front edge; a hollow skirt fills down to the flat hatch; the rest stays flat
CUT_OFF = 0.25                  # the tilt's cut tools stand this far OUTSIDE the flat body: a cut face ON a hull face (the housing's wall, the deck's face, the outer walls) leaves
#                                 a zero-thickness sheet of it that the tilt stands over the rim as a fin (builds 85-88: the 'picket fence' round the knob, up to 5.5 over the rim);
#                                 the ring fill below has the 0.25 the tilted body loses. Off every round's layer face (0.17 / 0.20 / 0.30 in)
REST_ANGLE, REST_FRONT = {"R": 30.0, "L": 22.0}, -152.0   # the WEDGE (Willy 07:37 'a straight line from the peripherals to the end of the wrists', 'rectangular with
#                                 splayed sides', 'sort of a wedge'; 08:15 'a line tighter from the edge of peripherals to wrist rest inside edge'; 08:40 'on the right side
#                                 you can bring the inner edge in closer'): each half's front is one trapezoid -- the outer edge straight on the hinge line, the front edge
#                                 (K y REST_FRONT) square to it, the inner edge the seam down to the KNEE then REST_ANGLE deg outward to the front corner. The knee is
#                                 computed: the diagonal is TANGENT to the tightest of the parts' walls (plan.py: the trackpad housing on R, the knob's / the stand's on L).
#                                 L's 22 deg is the steepest that keeps the STAND (164.5 wide, its barrel 47 from L's seam) + its ears inside the walls (23 deg: -0.4);
#                                 R has 29.5 more seam room (X_IN) and takes 30. The seam-side thumb key went for this line.
BORDER_W, BORDER_UP = 2.0, 0.8   # THE BORDER (Willy 22:46 'i wanted like 2mm of border a little raised around the rests not this deep channel'): a hull band
#                                 this wide round the whole pad -- inside the case's outline, round the pane's recess, between the pad and the keys -- standing
#                                 BORDER_UP over the TPU's top: the TPU sits IN its tray. History: 6 in and 1 proud (16:52 'ugly still'), to the face with no
#                                 lip (20:57), a 1.5 rim flush beside the key line's slot (22:46 'a deep channel with huge thin lip')
#                                 EVERYTHING on the palm side that is not pad or the pane's recess stands at that level (the landings where the pad is far
#                                 from the pit too): one surface from the TPU's edge to the caps, no groove. The pane's frame sits BORDER_UP below it like the TPU
BORDER_R = 1.8                  # THE BORDER'S OUTER EDGE: one round of this radius along the palm side's outline run (Willy 09:03 'smooth to lay hand on - i imagine
#                                 case needs to be rounded over there'): OCC's fillet on the outline's prism (hull.rest_round). 0.2 under BORDER_W: a 0.2 flat
#                                 between the round's tangent line and the recess wall (at BORDER_R = BORDER_W the recess cut's wall ran along the tangent line,
#                                 0.008 off it on the sloped top: a hairline face). The hinge side's top edge is in the run (its bottom edge is not)
PAD_DEPTH, PAD_PROUD = 2.0, BORDER_UP   # the TPU wrist pad (Willy 22:13 'i have tpu'): in a recess PAD_DEPTH under the rest plane, its top PAD_PROUD over the plane:
#                                 FLUSH with the border (09:03 'raise the tpu and get rid of that lip so the plastic border and tpu pad are same height'): 2.8 thick
CAP_CLR = 0.6                   # THE KEYWELL: the key block's wall between the border and the caps (2 wide, 6-8 deep at the thumbs: the deep channel) is
#                                 filled up to the border's level, this far off the caps (DSA caps travel straight down; MARGIN 1 stays inside the wells);
#                                 the lowest thumb cap (t3) stands 1.1 proud of it, the others 2-3
KEEPOUT_CLOSE = 3.0             # the caps' keep-outs are closed this far both ways (mitre): gaps under twice this between neighbouring caps are keep-out
WALL_R = EDGE_R_TOP + 1.0       # THE OPEN PIT's corner arcs (plan.smooth_open): no tighter than the key wall's round, with margin (a round cannot roll a tighter
#                                 corner; the chain rule's chords at 22.5 deg need R >= r / cos 11 + 0.26, the closing's re-chording of an S arc gave 13 deg pieces)
BRIDGE_KW = 1.0                 # where a keep-out bridge between two caps (plan: THE RED LINE) runs along the deck's face, this much keywell stays between them
TP_LIP = BORDER_UP              # the trackpad console's front wall meets the border's level (its pitched top flows into the border; was 0.5 over the TPU)
REST_SLOPE = 5.0                # deg: the rest's top falls toward you (the heel of the palm lower than the knuckles)
STEP_C = 2.0                    # the flat blocks' top edge where it steps DOWN to the tilted deck: one 45 deg chamfer (Willy 15:23 'simplicity and fluidity';
#                                 the layered 4 mm round of builds 85-89 rendered as terraces). Also where the seam wall goes up (ring_up) vs down (ring_low)
DSA_H = 7.67                    # the KeyV2 DSA cap's height over its skirt (dsa_row(3) 1u mesh zmax 7.673, measured 2026-10-15; caps.dsa_cap asserts it). NOT
#                                 SL.CAP_TOP's 6.70 (sophie's flat_1u STEP): the caps' top is CAP_SKIRT + DSA_H = 10.07 over the plate, 0.17 OVER RIM_Z (SUNK
#                                 0.8 was computed on 6.70) -- RIM_Z left as is (build 148; flag).
CAP_TOP_T = SL.CAP_SKIRT + DSA_H   # 10.07: the caps' top plane in the tilted frame (every key frame at z 0 on the one plate; all caps 1u)
KEY_PROUD = 2.0                 # THE KEY WALL'S CEILING (build 148; Willy 21:50 'a rule for all keys yeah - keep it consistent and simple'): the keywell's
#                                 fill is never nearer than this to the caps' top plane -- the plane CAP_TOP_T - KEY_PROUD in the tilted frame (hull.deck_edge
#                                 / wall_round). It rules where the tilted plate has fallen far under the flat border level (the thumb row); the rest plane
#                                 + BORDER_UP capped at KEYWELL_MAX rules behind. Not the border, not the lip (the rim running through): a step up onto the
#                                 border where the lowered wall meets it at the deck face.
CREASE_R = 240.0                # the keywell's top where the caps' ceiling meets the border's level (y -55.7, a 5 deg crease): the CHORD of an arc of
#                                 this radius in the y-z profile (geo.well_profile / hull.well_cap), y -65.6 .. -49.0, bends 2 / 3 deg (the arc itself, a
#                                 cylinder face of R >= 150 on the fill's top, made every fuse of the fill hand back the fill alone: fillcheck148c-f). A plane-min's crease split
#                                 the run's fan chords under 0.4 and OCC refused the key wall's chain; R 30 put a junction 1.5 from a 90 deg corner (the
#                                 rule wants 1.8) and 0.26 beside a near-parallel stretch (the chain picked the tangent line up twice); an OCC fillet of
#                                 the crease needs the fill wider than its band (0.7 at t1). 240 / 250 keep both arc ends 3.7 / 3.2 from every run vertex
#                                 on both halves (210, 230, 260 land under 0.1 off one) -- build 148.
CREASE_END = 1.0                # deg: the crease arc leaves the ceiling line tangent and meets the border's level at THIS angle, not tangent -- a tangent
#                                 arc's top ran into the border's top edge along the pit's exterior wall as a cusp (thickness s^2 / 2R: under 0.002 for
#                                 0.9 mm at R 210) and EVERY fuse of the fill into the hull handed back the fill alone (fillcheck148b; R 30's 0.35 mm cusp
#                                 had fused by luck); a 1 deg junction splits the run with a bend the chain rule prices at 0.016 (build 148)
BAY_HEAD = 100.0                # L's ENCODER BAY (build 151; Willy 07:46 'remove the incline thats unnecessary ... smooth out that second line so there
#                                 isnt a lip coming out - just smooth straightish'; 05:04 'turn earlier so the encoder silo is exposed and this same wall
#                                 continues on the other side'): THE LINE's heading, deg from +x -- from the first thumb cap's keep-out corner (where the
#                                 thumb wall's face turns north) straight to the silo's wall. 100: 26 deg off the face's last stretch (74), meeting the
#                                 silo's wall at 55 deg (y -29); the fill wedge between them is the wall's end, its top the border's level, the round
#                                 dying on the silo's wall. Everything east of the line is the key bay's floor: the silo's whole east flank stands on
#                                 it, and the fill north of the silo between column 0 and the flat zone (150's stub, 'the incline': a 7.7 top over a
#                                 plate rising from 4.5 at y -11 to 7.7 at y 8) is gone -- the plate runs through and the flat zone's wall stands on it.
#                                 (150 turned the wall into the silo with an R 2.8 arc + an end line 4 mm north: 'a lip coming out'. 149 had cut the
#                                 fill NORTH of the silo and kept the ledge along it.)
KEYWELL_MAX = RIM_Z - STEP_C - 0.2   # the border's / the keywell's ceiling: 0.2 under the flat zone's chamfer foot. The rest plane + BORDER_UP climbs 5 deg
#                                 toward the keys and would pass the chamfer behind y -21 (L's thumb strip, the thumb well's ring side); capped, it meets the
#                                 chamfered step with a 0.2 lip instead of standing over it
BACK_R = EDGE_R_TOP             # the BACK RUN's top edge round: the same as every other edge (Willy 23:14 'that back curve is too intense'; builds 85-120 had 6, items 58-59)
#                                 keeps EDGE_R_TOP): the band's slab is solid 12.9 deep under it; at the wire bay's / the charger bay's back wall the ROOF_T 2.4 roof keeps 1.6 at the wall line
DECK_FILL = 6.0                 # the palm deck is solid this far in from its edge at the pit (from the tilted plate up into the skin); its face there is a
#                                 plain vertical wall, flush with the tilted plate's edge (Willy 10:13 'i never asked for that slope': the 50 deg ramp of
#                                 builds 70-78 ate the deck the Picos now sit in). The thumb caps' tops stand 2..3 over the deck beside them, the alphas' front row 3.5..4.
# (The rugged-case rib flutes of builds 84-115 are gone: Willy 12:33 'i dont think we need the ribs right? just added complexity with no real gain'.)
BASE_RIM, BASE_PAD = 14.0, 10.0   # the base is only a rim frame + a pad round the stand (Willy 22:13 'only ... support around the stands')  # the WRIST REST (Willy 21:43 'a continuation of the base and case into wrist rests that tent along'): this deep in front of the thumbs, its back edge (K y) before the wells bite it
BASE_T = 7.6                    # the tenting base slab: the flap's barrel folds into it (barrel bottom 7.0 under the hull)
TENT_R, TENT_CLR, KNUCKLE, KN_CLR = 4.5, 0.3, 14.0, 0.6   # the hinge barrel; cove clearance; knuckle length; between knuckles
AX_UP = 1.0                     # the axis this far over the hull's underside (the barrel's bottom 0.5 off the desk through the base)
RAIL_W, RAIL_H = 9.5, 5.0       # a solid rail along the hull's bottom-outer edge carrying the knuckles and coves (5.0: the pinky switches' pins clear it)
PIN_D, PIN_CLR = 5.0, 0.35      # the printed hinge pin; diametral clearance GUESS (cyber_split's coupon said 0.35)
PIN_BORE = PIN_D + PIN_CLR
PIN_HEAD, PIN_SEAT, PIN_PROUD = (6.5, 2.0), 0.5, 1.5
PIN_FLAT, PIN_NOSE = 0.4, 2.0
PIN_RING, PIN_RELIEF = (0.4, 1.6), 2.0    # (Willy 2026-10-17: the 0.4 coupon fits, 0.2/0.3 too loose) THE SNAP RING 0.5 behind the nose (radial x long, 45 deg both sides) clicks into a relief (PIN_BORE + 2 x ring wide, this long) in the
# THE FITTINGS (build 133; Willy 12:37 'good to go on all approved', 13:22 'I dont see my leds, or switches installed'): real catalog STEPs, the holes GUESS
LED_STEP = SL.REFS / "kicad-packages3D/LED_THT.3dshapes/LED_D3.0mm_Clear.step"   # a 3 mm LED (KiCad packages3D): body Ø 2.9 on the leads' axis (the STEP's x 1.27: the bbox centre is 0.2 off it), flange Ø 3.8 x 1.0 (a flat at 1.7) from z 3.0, the tip at 8.3 (scratchpad led_profile, build 133)
LED_BORE, LED_SEAT, LED_SINK = 3.3, 3.9, 3.5   # GUESS: the leads' bore through the deck into the cavity; the flange (3.8, a flat at 1.7) pressed down a LED_SEAT bore to its
#                                 seat LED_SINK under the tray's floor (the dome 1.8 proud). HELD WITHOUT GLUE (build 137; Willy 21:13 'make sure that all lights stay secure
#                                 without glue'): the seat stops it going in (a finger on the dome), the snug seat (0.05 a side on the flange: printed holes run small) and the
#                                 LEADS BENT OUTWARD under the deck where the bore exits into the cavity hold it against lifting; fittings asserts every bore exits into open
#                                 cavity with room for the bend (half a 7 x 2.5 disc under the exit free). Was LED_SEAT 4.0 (builds 133-136: dropped in loose)
LED_BEZEL = (0.1, 3.7, 3.1, 0.4, (0.6, 0.3))   # THE LED INNER BEZEL (build 140; Willy 22:58 "another dope cyberpunk inner bezel to drop in, plug the space between leds
#                                 and their holes ... i can print in transparent"): GUESS -- the plate's clearance in the tray, the spigot's Ø in the LED_SEAT bore (printed
#                                 pins run big; the LED's flange is 3.8 in the 3.9 bore), the window's Ø round the 2.9 body, the countersink at the top, the grooves (w, d)
LED_PITCH, LED_TRAY = 5.5, (7.0, 2.5, 2.5)   # THE LED TRAY (build 134; Willy 17:04 'led placement terrible - on left on flat area near encoder with nice bezel and cyberpunk
#                                 design - on right side to the left of the touchpad same matching designs'): the LEDs in a row LED_PITCH apart along y in a sunk parallelogram
#                                 tray (width across, the ends' skew along y, the margin past the end LEDs), TP_IND deep with the trackpad tray's 45 deg flared mouth; the domes 1.0 proud
LED_AT = {"R": ((11.8, -4.2), (11.8, -9.7), (11.8, -15.2)), "L": ((32.0, 13.25), (32.0, 18.75))}   # plan frame: R on the console beside the touchpad (x 3.5..17.8 free there; the boss
#                                 at (6.2, 0.5) and the knee's wall below y -14 bound it: scratchpad led_free), L on the flat zone beside the knob, east of the charger bay; R 3 / L 2 (Willy 12:37)
SW805_STEP = SL.REFS / "Adafruit_CAD_Parts/805 slide switch/805 slide switch.step"   # Adafruit 805 (EG1218 SPDT slide, 0.3 A): body 11.7 x 4.0 x 5.3 + 1.0 of pins, the knob 2 x 2 x 1.7 at x 3.65..5.65
SW805_SKIN, SW805_POCKET, SW805_SLOT, SW805_TRAVEL = 1.2, (12.2, 4.4, 8.0), (2.4, 2.4), 2.0   # GUESS: the wall left outside the body; its pocket (along the wall x tall x deep from that skin
#                                 inward, through the wall's mass into the cavity); the knob's slot (w x h) + its travel along the body (the knob 0.5 proud of the face)
SW805_AT = {"R": (("pi", 11.0, 74.0, 2.0), ("main", 63.5, 74.0, 2.0)), "L": (("main", 146.0, -12.0, -5.0),)}   # (name, x, y, z): a point by the wall's face (plan frame); the   # (162: R's main z -8 -> 2.0: the charging door's pocket (u 37..66, z -11..-2) runs under it; above the pocket, under the wire bay's roof) (163: the pi -3 -> 2.0, level: Willy 22:50 'put 805's level')
#                                 pocket sits on the outline's nearest point, along the wall (R: x 13 clears the charge window at x 22-32 z -5..-12 and the seam corner;
#                                 x 60.5 is the last straight stretch before the shoulder -- at 70 the pocket broke out of the sweep, build 133 run 2). R's Pi switch in
#                                 the wire bay behind the display pocket (void -14..7 there), R's main power switch beyond the charger (void -14..-4), L's main on the OUTER wall between P's and
#                                 the next pinky key's housings (void -7..1 six in from the face, scratchpad wall_void; L's back wall is the cell and the charger, no room)
#                                 BUILD 156: R's pi switch at x 11 (was 13): its pocket's end 2.4 from the OTG door's pocket (u 19.5; 0.4 before -- the door came in 155), the
#                                 boss fused into the seam corner's mass; R's main at 63.5 (was 60.5): 1.1+ from the charging window's east bevel (the window sits at 40..54.5 now,
#                                 not 22-32: the pocket's end was ON the bevel), the pocket's end at 69.6 before the shoulder's sweep (the face recedes 0.1 at 70.5, 0.6 at 72.5)
SW805_BOSS, SW805_RIB = 1.2, (0.4, 0.45)   # THE SWITCH SOCKET (build 156; Willy 18:07 'hold the switches secure without screws ... make sure it fits snug', 18:19 'a nice snug
#                                 pocket that holds with friction, extending out a little into the inside of the cavity'): a boss this thick round each 805's pocket on the wall's
#                                 inside, from 0.8 behind the skin (in the wall's mass) to 0.4 past the body's back (the pins stand out of its mouth for the wires); RIB_W-wide crush
#                                 ribs the body's height on the pocket's four faces, (top / bottom, the ends) proud: the 12.2 x 4.4 pocket less them = 11.3 x 3.6 for the 11.7 x 4.0
#                                 body, 0.2 interference a side (the bezel's RIB_H recipe). Before: 1.8 of the body's 5.3 in the wall with 0.2 of air a side, nothing behind
PIN_NOTCH = (2.4, 1.2)          # THE PULL NOTCH (build 132, Willy 12:37): a half-moon this wide above each pin head's rim, this deep into the end face -- a nail or a small screwdriver under the rim to draw the head
LIP_UP = 2.0                    # THE SWELL (build 132; Willy 12:37 'the p key and q are not sitting in secure slots', 13:22 '2. lip follows the plate'): along the pinky column
#                                 the lip's top stops this far over the TILTED PLATE (plane A) instead of at the rim -- the plate climbs through the rim at the back, the lip
#                                 was 4.3 .. 2.6 over it beside P / Q and the caps stood 6-8.6 proud of the case's edge, 10 mm of bare plate behind them (the pinky stagger)
SWELL = False                   # THE SWELL OFF (build 139; Willy 22:36 'im complaining about that outer corner ... on the side wall. that one shouldnt need to raise'):
#                                 the outer-back corner's rim stays at RIM_Z -- no fill over plan.swell (empty), no swell round; the lip is the border's level capped at the
#                                 rim along the whole outer run (hull.lip_round, as before 132) and the plateau's own round takes the corner. The bare plate behind P / Q sits
#                                 under the rim already (+0.09 at 3 cells, scratchpad notch139), so nothing is cut. The P / Q caps' outer sides stand open (~5 proud at the
#                                 corner), as before 132. THE ROLL (the cylinder swell, below) stays in the code for SWELL = True
STRIP_W = WALL + 0.3            # THE RIM STRIP (build 140; Willy 23:14 "this ugly block was exposed with that change fix it - i love the curve now"): with the
#                                 swell off, the rim round the outer-back corner is a strip this wide (the lip's: the pit's edge is WALL + 0.3 in) at RIM_Z; the
#                                 plateau's corner behind P / Q and the lip's tab in the stagger's notch beyond it (plan.corner) go down to the tilted plate
LIP_UP_LEGACY = 3.5             # THE ROLL (build 139; Willy 22:12 'the outer rim doesnt need to raise that much in the corner', 'sharp edges on the rounded edge on
#                                 both sides on the back'): plane B is a CYLINDER now -- tangent to the rim at x_b, the foot of builds 132-138's plane (LIP_UP_LEGACY and
#                                 SWELL_FALL_LEGACY keep it there: R 151.7 / L 124.0), its radius plan.swell_R set so it meets plane A (LIP_UP, now 2.0: the corner ~1.8 over the rim,
#                                 was 4.7) at the same crease
#                                 crossing on the outline; no crease at the foot, so the swell's round runs into the plateau's with no seam. SWELL_FALL (11.2 deg) is gone
SWELL_FALL_LEGACY = 11.2        # deg: plane B of builds 132-138 -- the rim at x_b (plan.py sets it so the A / B crease crosses the outline mid-way along the shallow arc beside the corner: at
#                                 11.2 deg x_b lands ~3 inside the cell bay's wall and B is UNDER the rim along that wall, so the swell's round never swaps the band there --
#                                 at 10 deg it did, 0.1-0.3 over the rim, and left 0.01 mm2 slivers on the band's floor that made the hull invalid, build 132)
#                                 back run's straight) rising this pitch toward the outer wall: the swell's top is the LOWER of A and B, so it dies out flush into the
#                                 cell bay's rim (no step there), the outer-back corner ~3 over the rim, P's back 3.5 over the plate, r0c3's outer side 2.7
#                                 last knuckle the nose reaches: the pins stay put (free in their 0.35 bores they walked; the D-flat keys nothing), a rod pushes them out
FLAP_P, FLAP_S, FLAP_W, FLAP_T = 76.0, 41.0, 164.5, 5.0   # W 164.5 (Willy 23:51 "stands until the rests ... flexing", 23:56 "wider so it supports the rests too"): K y -132..32.5; the front ear under the rest; the nose seats where the base reaches (y -86..33), the front of the plate hangs clear of the desk   # the flap: its pivot from the hinge axis (on the hull); pivot to the nose along the plate; width; plate thickness
FLAP_NOSE = FLAP_T / 2          # the plate's end is a full round
FLAP_RB = 3.3                   # the barrel round the pin at the flap's root (1.65 wall round the 3.3 bore)
FLAP_Y = {"R": -49.75, "L": -49.75}  # the flap's centre line in K (both under the key block's hatch; the slots land under R's display pocket / L's knob housing)
FLAP_PIN_D, FLAP_PIN_CLR = 3.0, 0.3   # its pin: 3 mm printed or a length of filament   GUESS
FLAP_STUB = ((4.6, 1.2), 10.0, 1.0, (0.4, 1.6), 0.4)   # THE FLAP'S STUB PINS (164; Willy 00:43 'the flap pins dont need to be full length ... design them like the other nice pins'):
#                                 one from each ear, the tent pins' form at 3 mm -- head (dia, thick) sunk flush in the ear, 10 into the flap's barrel, nose, snap ring into a groove
#                                 in the barrel, the D-flat (0.4) to print on; 163-: one 177 mm rod through ears and barrel
PIV_DROP = FLAP_RB + 0.4        # the pivot under the hull's underside (the barrel 0.4 off the hatch)
FLAP_E = PIV_DROP - 0.4 - FLAP_T / 2   # the plate's centre line over the pivot when folded: its top face flush with the barrel's top
EAR, EAR_GAP = (2 * FLAP_RB + 0.4, 6.0, PIV_DROP + 1.65 + 1.6), 0.3   # the ears under the hatch either side of the barrel (1.6 under the bore)
NOTCHES = (91.0, 99.0, 112.0)   # the nose's seats on the base, from the axis
NOTCH_D, NOTCH_CLR = 4.5, 0.2   # each seat: a vertical stop face (the side away from the hinge) 4.5 deep + a ramp at the flap's own slope
assert EC11_SHAFT_TOP - (EC11_PAD_Z + EC11_KNURL0) > 3.0, "the knob must grip 3 mm of knurl"
# THE LIGHTS (build 152; velvet/lights.py): the plateau light guides, their lid, the COB filaments; the badge trays' caps
LIGHT_LID = (-14.0, 35.7, 58.5, 52.0)   # the plateau lid in K: over the 18650 bore (x -13..58, y 33..52.4), 2.3 inside the outline's back round, 0.3 off the step
#                                 chamfer's foot (y 35.4), its east end before the shoulder's ogee (the inboard limit falls past x 58.5); 0.3 off R's PiTFT brim (K x -14.3)
LID_CORNER = 3.0                # its plan corners (true arcs; the top edge round is BADGE_R, the height BADGE_T: the badge's language)
TRACE_W, PAD_D, HOLE_D = 2.4, 3.0, 2.2   # a net's slot / guide stem width, its via pads, the lead hole under each pad (into the bore's lead room)
GROOVE_W, GROOVE_D, GROOVE_CLR = 3.4, 1.2, 0.1   # the guide's foot groove in the plateau (the skin over the bore 3.2 -> 2.0 left), the foot's clearance in it
LIGHT_CLR = 0.1                 # the lid's slot round the guide's stem: SNUG (Willy 10:08 'fit snuggly'; printed slots run small, sand to fit)
VIA_DEPTH = 6.5                 # the lead hole from the groove's floor (RIM_Z - GROOVE_D = 8.7) down: the lead rooms' ceilings are 6.1 / 4.3 at the pads
FIL_L, FIL_D = 30.0, 2.0        # Willy's COB filament (23:51 '30mm cob leds - 2mm in diameter'): GUESS -- no catalog STEP (step.parts, KiCad: none)
LEAD_CH = 1.0                   # the leads' channel along the net, up from the guide's underside
LEAD_W, LEAD_Y, LEAD_VIA = 1.4, 36.5, 16.0   # THE LEAD WAYS (163, R; Willy 20:22 'holes at either ends of cob leds that feed diagonally straight into the big cavity - not into the
#                                 battery cavity'): a channel LEAD_W wide, GROOVE_D deep, from each filament's tab room south (K) to y LEAD_Y under the crown, then a HOLE_D hole
#                                 45 deg down-south, LEAD_VIA long, into the key bay. The filament ends sit 1.1 over the bore (a 45 deg hole there meets it in 1.5; the lead rooms
#                                 are the battery cavity): the channel's floor keeps 2.0 over the bore, the hole's axis 2+ off its wall; R's lead-room vias are gone
FIL_SINK = 0.7                  # THE FILAMENT TRENCH (Willy 12:57 'cob leds need to be UNDER diffusers so the effect takes'; 153's filaments stood out of the 1.2 crown's
#                                 slot bare): the filament's axis sits FIL_SINK + GROOVE_D - FIL_D / 2 = 0.9 under the plateau top in a FIL_D + 0.4 wide trench; the
#                                 guide's channel is a ROUND bore FIL_D / 2 + 0.2 about it (14:46 Willy 'the channel walls dont connect to the base': a 2.4 square
#                                 channel in the 2.4 stem left the slab over it hanging from its ends) -- at the stem's base (z 0) the bore is 1.58 wide: 0.41 of wall
#                                 a side, 0.9 of clear over the filament; the skin under the trench 0.95 over the cavity (L's encoder runs), 1.35+ over the bore
FIL_TAB = 4.5                   # THE TAB ROOM (Willy 14:35 'cob filaments have small metal tabs 3-4 mm coming out each end - i can connect wires to extend them'): the
#                                 trench and the guide's channel run this much past each filament end, straight, for the tab and the wire soldered on it; the wire
#                                 goes on along the net's lead channel to the via (the encoder runs' vias sit at the tab room's end). Resistors / a MOSFET per colour
#                                 ride the wires in the cavity: no case feature
RIDGE_W, RIDGE_H = BZ_GROOVE    # the lid's silkscreen ridges = the PiTFT bezel's traces, raised (0.8 x 0.4)
RIDGES = False                  # THE RAISED TRACES OFF (build 157; Willy 18:31 'lets just remove the raised traces from both of the bezels - this way i can print flat'): no
#                                 silkscreen ridges, rings or dots on either crown and no raised traces on R's bezel plate -- the crown's top is one plane, printed top-down on it
LID_PIN = (3.0, 2.5)            # the lid's press pins (Ø, long) into LID_PIN + 0.2 holes, where the bore's crown is 5+ under the top
TCAP_T, TCAP_R, TCAP_SLOT, TCAP_PAD, TCAP_PIN, TCAP_CLR = 1.0, 0.6, 1.6, 2.0, (1.6, 1.2), 0.1   # THE TRAY CAP: thick, its edge round, its trace slots, their pads, its pins, the
#                                 slot round the diffuser's peg (Willy 10:08: the diffuser's LED points stand up as pegs that fit snugly into the cover -- always)
LED_SINK = LED_SINK + 1.2       # THE LEDS LIT FROM UNDERNEATH (build 152): the flange seat 1.2 deeper -- the dome tips 0.2 under the badge top, under the diffuser
DOME_POCKET = (3.1, 0.7)        # the diffuser's pocket over each dome tip (Ø, deep)
# THE CROWNS (build 153; Willy 2026-10-16 10:44-11:42: one cap piece per half, 'cohesive and secure', 'flush with case not raised', 'pegs on cover and
# holes in case', 'clean intentional lines that follow through'; velvet/lights.py): one printed plate per half -- R the PiTFT bezel + the LED wing +
# the trackpad border + the plateau lid, L the badge plate + the plateau lid; the hull flat at the rim under all of it
CROWN_T = LIP_T                 # 1.2: the brim's thickness -- R's crown IS the raised console (the hull's console is not lifted any more: CONSOLE_UP's
#                                 lift is gone, the crown stands CROWN_T on the rim everywhere), L's plate drops from 2.0 to it
CROWN_R = LIP_R                 # 1.0: the top edge round a 1.2 plate takes (the brim's)
BADGE_T, BADGE_R = CROWN_T, CROWN_R   # the lid / plate language IS the crown's now (hull.badge raises nothing: the hull is flattened under the crown)
KNOB_TOP = RIM_Z + CROWN_T - 0.2; KNOB_H = KNOB_TOP - KNOB_Z0   # the knob flush in the crown, 0.2 under its top
LED_AT = {"R": LED_AT["R"], "L": ()}   # L's two 3 mm LEDs REPLACED by the filament runs round the knob (Willy 10:49)
TP_COLLAR, TP_LEDGE = 1.0, 1.0  # THE TRACKPAD IN THE CROWN (Willy 10:56 'support on the bottom so i can place it from top'): a collar under the window into
#                                 the widened pocket (its two side walls + the front; the display side keeps the hull's wall), a ledge under the pad's two
#                                 side edges (its connection edges open). GUESS: the TPS43's pads are on a long (north / south) edge
OVERLAY_T, OVERLAY_SINK = TP_SKIN, TP_IND   # the overlay ('thin piece of different colored pla'): 0.6 thick, sunk 0.8 under the crown's top, snug (LIGHT_CLR)
TP_TOP = RIM_Z + CROWN_T - OVERLAY_SINK - OVERLAY_T   # 9.7: the pad's top (as it was under the skin); the ledge TP_T + TP_LEDGE under it
ARM_W = TCAP_SLOT               # 1.6: the R LEDs' clear peg ARMS (CROWN_T tall) running out of the tray onto the bezel plate (Willy 11:13 LGTM)
KNOB_BEZ_R = HOUSING_R - EDGE_R_TOP - 0.1   # 17.4: THE BEZEL RING round the knob (build 154; Willy 13:34 'following the encoder all the way around with bezel and
#                                 letting the side wall provide the side wall curve'): the L cover's ring from the well to 0.1 inside the silo round's tangent --
#                                 the silo's own r EDGE_R_TOP round stays the hull's all the way round
SILO_SINK = 1.0                 # the ring's SKIRT (Willy 12:34 'fit on securely'): the silo ring's top cut this much under the rim from the well out to KNOB_BEZ_R, the
#                                 cover's ring (less COVER_CLR outside, the well through) sits in that step: it keys itself
ENC_SE = 1.0                    # L's ENCODER RUNS (both diagonals, their filaments, vias, the strap and its ridge) this much to the south-east, off the shoulder's ogee
#                                 (build 156; Willy 18:21): the magenta slot was 4.68 from the outline there (2.4 inside the cover's edge) -> 5.68; the cyan keeps 4.4 to the flat
#                                 zone's step edge and 5.7 to the knob's bezel ring (1.5 gains the magenta nothing more: its next-nearest point is on the plateau)
ENC_SW = 4.0                    # ... and the two encoder filaments with their vias this much further down the diagonals (SW): 153-155's magenta NE deep via at (40.0, 44.6)
#                                 sat on the lip of the hatch screw boss at (40, 46) -- its only clear spot was the boss's own screw bore (via_scan156: solid all round within 3 mm,
#                                 0.16 / 0.28 mm3 of hull under the via 1 SE / 1 S of it); at (37.9, 41.1) the cavity is open, the three other vias are clear 4 mm all round
VIA_HIDE = 5.0                  # THE TRACES' EAST ENDS HIDDEN (build 156; Willy 18:21 'the blue and magenta traces reach a little too far and are too close to the outside
#                                 edges on both cases'): the lit slot + the guide's stem stop this short of the east via (the vias sit in the bore's lead rooms, 3.1 / 3.5
#                                 from the outline: they cannot move), the groove and the lead channel run on under the crown; the trace's end pad 8+ from the outline
VIA_DEEP = FLOOR + 1.0          # L's encoder-run vias: drilled from the groove's floor to 1 over the hatch (into the cavity; fittings asserts each opens)

# THE SLIDE DOORS (build 155; Willy 14:40 'replace the pi usb doors with tambour slide door built into the wall printing in place - use oval shape
#                                 hole', 14:43 'all openings that have covers currently - slide direction optimized for each area where it best fits -
#                                 one oval per port'): print-in-place sliders in a pocket inside the wall, over an oval port (velvet/doors.py). The
#                                 charging windows had no cover and keep their rounded rectangles.
DOOR_SKIN, DOOR_GAP, DOOR_GAP_FLOOR, DOOR_GAP_BRIDGE, DOOR_BACK = 0.8, 0.3, 0.2, 0.25, 1.0   # the outer skin (2 perimeters) | side play | the play under the pocket's
#                                 top face (rim-down that face is the print floor: the first layer sticks lightly, the first slide breaks it) | the play over the
#                                 pocket's floor (bridged) | the back wall behind the tracks (a pad fused on the inside where the wall is thinner)
DOOR_TAB = (2.0, 1.8, 1.2)      # THE HANDLE (Willy 16:50 'handle should pop out not sink in'): a thumb tab on the door's face, this wide along the stroke x tall, standing this
#                                 much proud of the wall's face; it rides through the port hole itself (a tab cannot pass under the 0.8 skin)
PICO_TAB = (5.0, 7.0, 2.0, 3.8)  # A's tab, closed: along x0..x1 (inside the hole's r 2 back corner by 0.4), z0..z1 (the hole's top band). Open it parks at -11.5..-9.5: the hole's
#                                 top-front corner gets a TAIL (PICO_TAIL) 0.3 round it, since the stroke is 0.5 longer than the hole is wide
PICO_TAIL = -11.8               # the tail's front end along the wall (x), z = the tab's band + 0.3
PICO_HOLE = (16.0, 9.0, 2.0)    # the Pico plug's hole in place of the PLUG_W x PLUG_H window: a ROUNDED RECTANGLE w x h, corner r (Willy 16:27 'stadium / rounded rectangle
#                                 ... which passes 12x8': the 12 x 8 overmold's corner (6, 4) sits 0.2 inside an r 2 corner; a true stadium's end arc would cut it off). 9 tall
#                                 because the palm's surface behind that wall slopes down toward the front (4.25 over the window's centre 26 forward, sect155.py); 16 wide
#                                 (not 14) so that open, A's end face still shows 1.0 inside the hole for the nail that closes it while the 12 wide plug clears it by 1.0
PICO_LEAF = (1.4, 1.0)          # the leaves' thicknesses, A (outer) and B (inner). A ONE-PLATE door is 2 mm short of clearing the plug either way along that wall (a room
#                                 behind the wall at -25, the rest pin's pull notch at +25.5) and the wall is straight for 40 mm each way (no corner for a tambour to turn):
#                                 TWO leaves in two tracks, B's lug riding in a groove in A's hidden face (lost motion: A alone first, then it drags B; both faces plain)
PICO_A, PICO_B = (-1.0, 9.5), (-9.5, 1.5)   # the leaves CLOSED, along the wall from the oval's centre (+ = toward the back): A 10.5, B 11.0, 2.5 overlap
PICO_TRAVEL = 16.5              # A's stroke toward the front: open, A lies at -17.5..-7.0, B at -18.7..-7.7 -- the plug (12 wide) clears by 1.0, A's end face shows in the oval for the nail
PICO_GROOVE = (-0.2, 8.6, 1.5, 0.8)   # the groove in A's inner face (along, closed; half-height; depth -- 0.6 of face left): B's lug runs in it, A alone for 7.3 of its stroke
PICO_LUG = (0.1, 1.3, 1.2)      # B's lug on its outer face (along, closed; half-height), 0.3 off the groove's floor and its front end when printed
PICO_POCKET = (-19.0, 9.75)     # the pocket along the wall: B's open end + DOOR_GAP_BRIDGE .. A's closed end + 0.25
PICO_PAD_Z = (-5.0, 3.8)        # the pad behind the tracks, in the ROLLED door frame: it must cover the cavity band (hull z -2.9..2.4 behind the wall; the palm skin backs the
#                                 wall above it, the slab below) at both ends of the 29 mm pocket, which the 5 deg roll moves 1.7 down at the front and 0.9 up at the back
DOOR_SPEC = dict(hole=PICO_HOLE, pocket=PICO_POCKET, a=PICO_A, b=PICO_B, leaf=PICO_LEAF, tab=PICO_TAB, tail=PICO_TAIL, groove=PICO_GROOVE, lug=PICO_LUG, pad_z=PICO_PAD_Z, travel=PICO_TRAVEL, plug_w=PLUG_W)   # the Pico's slide door (155) as one spec
CHG_SPEC = dict(DOOR_SPEC, pad_z=(-5.3, 5.3))   # THE CHARGING DOORS (162; Willy 22:04 'tambour those doors - lets not take forever to build them because we already know how'): the same two leaves
#                                 behind a PICO_HOLE (16 x 9 r 2) in the wall at each charging window (R's back face, L's inner face; CHG_W x CHG_H's flared window gone); the pad
#                                 behind the wall the pocket's height (the Pico's was clipped to the palm skin's cavity band); the hatch's charger fence is relieved for it
PI_COVER = (1.5, 0.8)           # THE OTG COVER (162; Willy 22:04 'a nice cover on that pi usb cable access, a friction fitting one low profile'): an oval plug WALL deep with four
#                                 crush ribs (the bezel's press fit, COVER_CLR a side) and a brim this wide x thick on the wall's face, its edge rounded
BTN_TOP = 3.5                   # MEASURED (Willy 22:50 '3.5 mm pcb from top of button'; 162 guessed 4.3): the PiTFT's tactile buttons (Willy 22:09 'standard tactile buttons 1mm tall' = the plunger): a 6 x 6 x 4.3 part, its plunger's top
#                                 this far over the HAT's PCB top (the Adafruit STEP's posts stand 8.8: tall ones, 4.3 over the glass)
CAP = (0.8, 0.8, 0.7, 0.15)     # THE BUTTON CAPS (162; Willy 'custom button covers'): proud of the bezel's top | the foot past the slot a side | the foot's thickness | clearance to the slot
PI_OVAL = (14.2, 12.6)          # the Pi OTG oval in place of the USB_W x USB_H window, centred on the port: passes 12 x 6.5 with 0.2. Narrow because the tent pin's room
#                                 in the back wall ends 7.8 inward of the OTG (x 19.1; the pocket ends 0.4 short of it), tall to pass the overmold's corners; its bottom
#                                 clipped PI_SILL over the hatch (the port sits 4.1 over it) -- an arch rather than an oval, the port is too low for a full one
# (155's lift door -- PI_PLATE / PI_SILL / PI_LIFT_CLR / PI_TAB_Z -- dropped in 161: Willy 21:02 'drop this door - forget about it - its not needed')
