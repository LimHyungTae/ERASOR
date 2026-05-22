"""Minimal PR/RR evaluator. Uses a hand-rolled ASCII PCD reader to dodge pypcd's
numpy-2 incompatibilities, then mirrors analysis_py3.py's PR/RR computation.
"""

import argparse
import os
import sys

import numpy as np
from sklearn.neighbors import NearestNeighbors
from tabulate import tabulate
from tqdm import tqdm

DYNAMIC_CLASSES = {252, 253, 254, 255, 256, 257, 258, 259}


def read_pcd_ascii(path):
    with open(path, "r") as f:
        fields = None
        n_points = None
        data_mode = None
        for line in f:
            line = line.strip()
            if line.startswith("#"):
                continue
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("POINTS"):
                n_points = int(line.split()[1])
            elif line.startswith("DATA"):
                data_mode = line.split()[1]
                break
        assert data_mode == "ascii", f"Need ASCII PCD, got {data_mode} at {path}"
        assert fields is not None and n_points is not None
        arr = np.loadtxt(f, dtype=np.float64, max_rows=n_points)
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    cols = {name: arr[:, i] for i, name in enumerate(fields)}
    xyz = np.stack([cols["x"], cols["y"], cols["z"]], axis=1).astype(np.float32)
    intensity = cols["intensity"]
    return xyz, intensity


def labels(intensity_np):
    label = intensity_np.astype(np.uint32)
    sem = label & 0xFFFF
    return sem


def count_dyn_stat(sem):
    nd = int(np.sum(np.isin(sem, list(DYNAMIC_CLASSES))))
    return sem.shape[0] - nd, nd


def overlap_report(gt_xyz, est_xyz, voxelsize=0.2):
    """est -> GT NN distance distribution. Most static est points should land
    well inside one voxel of a GT point if the maps overlap correctly."""
    nn = NearestNeighbors(n_neighbors=1, algorithm="kd_tree").fit(gt_xyz)
    d, _ = nn.kneighbors(est_xyz)
    d = d.reshape(-1)
    half = 0.5 * voxelsize
    one = voxelsize
    print(
        f"est->GT dist: median={np.median(d):.4f}m  p90={np.percentile(d,90):.4f}m  "
        f"p99={np.percentile(d,99):.4f}m  max={d.max():.4f}m"
    )
    print(
        f"  fraction <0.5*v ({half:.2f}m): {np.mean(d < half)*100:.2f}%  "
        f"<1*v ({one:.2f}m): {np.mean(d < one)*100:.2f}%  "
        f"<2*v ({2*one:.2f}m): {np.mean(d < 2*one)*100:.2f}%"
    )


def evaluate(gt_xyz, gt_sem, est_xyz, est_sem, voxelsize=0.2):
    ns_gt, nd_gt = count_dyn_stat(gt_sem)
    ns_est, nd_est = count_dyn_stat(est_sem)

    nn = NearestNeighbors(n_neighbors=1, algorithm="kd_tree").fit(est_xyz)
    dists, idx = nn.kneighbors(gt_xyz)
    dists = dists.reshape(-1)
    idx = idx.reshape(-1)

    thr = voxelsize * np.sqrt(3) / 2
    is_in = dists < thr

    gt_sem_in = gt_sem[is_in]
    est_sem_in = est_sem[idx[is_in]]

    gt_is_dyn = np.isin(gt_sem_in, list(DYNAMIC_CLASSES))
    est_is_dyn = np.isin(est_sem_in, list(DYNAMIC_CLASSES))

    num_static_preserved = int(np.sum((~gt_is_dyn) & (~est_is_dyn)))
    num_dynamic_preserved = int(np.sum(gt_is_dyn & est_is_dyn))

    pr = num_static_preserved / ns_gt * 100.0
    rr = (nd_gt - num_dynamic_preserved) / nd_gt * 100.0 if nd_gt > 0 else 0.0
    f1 = 2 * (pr / 100) * (rr / 100) / ((pr / 100) + (rr / 100)) if (pr + rr) > 0 else 0.0

    return {
        "gt_static": ns_gt, "gt_dynamic": nd_gt,
        "est_static": ns_est, "est_dynamic": nd_est,
        "preserved_static": num_static_preserved,
        "preserved_dynamic": num_dynamic_preserved,
        "PR": pr, "RR": rr, "F1": f1,
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--gt", required=True)
    p.add_argument("--est", required=True)
    p.add_argument("--voxelsize", type=float, default=0.2)
    args = p.parse_args()

    assert os.path.isfile(args.gt), args.gt
    assert os.path.isfile(args.est), args.est

    print(f"GT : {args.gt}")
    print(f"Est: {args.est}")
    gt_xyz, gt_int = read_pcd_ascii(args.gt)
    est_xyz, est_int = read_pcd_ascii(args.est)
    gt_sem = labels(gt_int)
    est_sem = labels(est_int)

    overlap_report(gt_xyz, est_xyz, voxelsize=args.voxelsize)
    r = evaluate(gt_xyz, gt_sem, est_xyz, est_sem, voxelsize=args.voxelsize)
    print(tabulate(
        [[r["gt_static"], r["gt_dynamic"], r["est_static"], r["est_dynamic"],
          r["preserved_static"], r["preserved_dynamic"],
          f'{r["PR"]:.3f}', f'{r["RR"]:.3f}', f'{r["F1"]:.4f}']],
        headers=["gt_S", "gt_D", "est_S", "est_D", "kept_S", "kept_D", "PR%", "RR%", "F1"],
        tablefmt="orgtbl",
    ))


if __name__ == "__main__":
    main()
