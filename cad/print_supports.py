"""Designed supports for a part held off the bed by a protrusion (a hinge barrel, ears, fences): a ray up from the bed every RES mm finds
the part's underside; wherever that is an overhang (flatter than 60 deg) within LIFT of the bed, a column stands from the bed to GAP under
it (down to a layer line) -- so the walls follow flat faces AND the rounded edges climbing off them -- kept XY_GAP clear of whatever reaches
lower beside it. Columns are kept on the region's outline (WALL thick) and a waffle PITCH apart. Higher overhangs stay with the slicer.
Merge the walls into the part's object (one object: the slicer keeps its supports out of them and bridges the waffle's cells).
    python print_supports.py part.stl [out.stl]      # writes part + walls (default part_supported.stl); --test runs the self-check"""
import sys, numpy as np, trimesh, manifold3d as mf
from scipy import ndimage

LIFT, GAP, LAYER, WALL, PITCH, XY_GAP, RES = 8.0, 0.2, 0.2, 0.8, 8.0, 0.4, 0.4   # PITCH 8: cells bridge 7.2


def walls(m, lift=LIFT, gap=GAP, layer=LAYER, wall=WALL, pitch=PITCH, xy_gap=XY_GAP, res=RES):
    """The support walls for mesh m in its print pose (any z offset). Returns (mesh or None, (area mm2, tallest wall))."""
    (x0, y0, z0), (x1, y1, _) = m.bounds
    xs, ys = np.arange(x0, x1 + res, res), np.arange(y0, y1 + res, res)           # the columns' centres
    f = res / 2                                                                     # rays on the half grid: every column's centre, edges, corners
    xf, yf = xs[0] - f + f * np.arange(2 * len(xs) + 1), ys[0] - f + f * np.arange(2 * len(ys) + 1)
    X, Y = np.meshgrid(xf, yf, indexing="ij")
    o = np.c_[X.ravel(), Y.ravel(), np.full(X.size, z0 - 1.0)]
    loc, ri, ti = m.ray.intersects_location(o, np.tile([0.0, 0.0, 1.0], (len(o), 1)), multiple_hits=False)
    h = np.full(X.size, np.inf); h[ri] = loc[:, 2] - z0
    nz = np.zeros(X.size); nz[ri] = m.face_normals[ti, 2]
    Hf, Nf = h.reshape(X.shape), nz.reshape(X.shape)
    H, N = Hf[1::2, 1::2], Nf[1::2, 1::2]                                           # at the centres
    need = (N < -0.5) & (H > 0.3) & (H < lift)                                      # an overhang low over the bed
    R = int(np.ceil(xy_gap / f)) + 2
    Hp, lim = np.pad(Hf, R, constant_values=np.inf), np.full(Hf.shape, np.inf)
    for di in range(-R, R + 1):                                                     # over the footprint (|offset| <= 1 half-step): the underside's
        for dj in range(-R, R + 1):                                                 # true lowest sample; beyond it, a 45 deg cone of clearance off the
            d = np.hypot(max(0, abs(di) - 2), max(0, abs(dj) - 2)) * f              # footprint box, a sample's step short (an edge 0.02 past the footprint
            if d <= xy_gap + 1e-9:                                                  # is 0 off, not 0.2): a neighbour d off may sit d lower -- the walls climb a round
                lim = np.minimum(lim, Hp[R + di:R + di + Hf.shape[0], R + dj:R + dj + Hf.shape[1]] + d)
    lim = lim[1::2, 1::2]
    top = np.floor((lim - gap) / layer + 1e-6) * layer
    ok = need & (top >= layer)
    n_w, n_p = max(1, round(wall / res)), max(2, round(pitch / res))
    I, J = np.indices(ok.shape)
    keep = ok & ((ok & ~ndimage.binary_erosion(ok, iterations=n_w)) | (I % n_p < n_w) | (J % n_p < n_w))
    if not keep.any():
        return None, (0.0, 0.0)
    boxes = []
    for i in range(keep.shape[0]):                                                  # runs along y of one height -> one box each
        js = np.flatnonzero(keep[i])
        if not len(js): continue
        cuts = np.flatnonzero((np.diff(js) != 1) | (np.diff(top[i, js]) != 0)) + 1
        for run in np.split(js, cuts):
            t = float(top[i, run[0]])
            boxes.append(mf.Manifold.cube([res, res * len(run), t]).translate([xs[i] - res / 2, ys[run[0]] - res / 2, z0]))
    u = mf.Manifold.batch_boolean(boxes, mf.OpType.Add).to_mesh()
    w = trimesh.Trimesh(np.asarray(u.vert_properties)[:, :3], np.asarray(u.tri_verts), process=True)
    return w, (float(keep.sum() * res * res), float(top[keep].max()))


def _test():
    """A 40 x 40 x 2 plate held 2 up by a 4 x 40 rib at one edge: walls under the rest, 1.8 tall, clear of the rib and the plate."""
    plate = trimesh.creation.box((40, 40, 2)); plate.apply_translation([0, 0, 3])
    rib = trimesh.creation.box((4, 40, 2)); rib.apply_translation([-18, 0, 1])
    part = trimesh.boolean.union([plate, rib])
    w, rep = walls(part)
    assert w is not None and abs(rep[1] - 1.8) < 1e-9, rep
    assert w.bounds[1][2] <= 2.0 - GAP + 1e-6 and w.bounds[0][2] >= -1e-6, w.bounds
    assert w.bounds[0][0] >= -16 + XY_GAP - 1e-6, w.bounds                      # clear of the rib
    assert trimesh.boolean.intersection([w, part]).volume < 1e-6
    print("print_supports self-check ok:", rep)


if __name__ == "__main__":
    if "--test" in sys.argv:
        _test(); sys.exit()
    src = sys.argv[1]; dst = sys.argv[2] if len(sys.argv) > 2 else src.rsplit(".", 1)[0] + "_supported.stl"
    m = trimesh.load(src); m.merge_vertices()
    w, rep = walls(m)
    print(f"walls under {rep[0] / 100:.1f} cm2, the tallest {rep[1]:.1f}")
    trimesh.util.concatenate([m, w] if w is not None else [m]).export(dst); print("wrote", dst)
