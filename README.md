# CurrentFeature Odometry

MATLAB reference implementation and exported data for:

**Bias-Eliminated PnP for Stereo Visual Odometry: Provably Consistent and Large-Scale Localization**

This repository contains the MATLAB back-end code for CurrentFeature Odometry and a small KITTI sequence 04 demo exported from OV2SLAM. The full exported data package is intentionally kept outside Git and should be downloaded separately.

## What This Repository Runs

The included MATLAB pipeline does **not** run feature extraction from raw stereo images. Instead, it consumes exported OV2SLAM front-end data:

- per-frame 2D feature tracks and point IDs,
- stereo keyframe left/right matches,
- KITTI/Oxford/EuRoC ground-truth or reference pose files where available.

The runner re-estimates the relative pose of each current frame against the latest keyframe using CurrentFeature Odometry:

1. triangulate 3D points from the current stereo keyframe,
2. estimate triangulation uncertainty,
3. run bias-eliminated weighted PnP,
4. refine poses with local epipolar bundle adjustment,
5. compose the trajectory and report ATE/RPE.

## Layout

```text
src/
  pnp/                 Bias-Eli-W PnP, L1 PnP outlier sorting, triangulation
  ba/                  local epipolar bundle adjustment
  io/                  readers for OV2SLAM, stereo matches, KITTI/EuRoC poses
  eval/                ATE/RPE and trajectory alignment tools
  utils/               SO(3)/se(3) helpers
  baselines/pca_pnp/   PCA-PnP fallback used when very few matches are available

scripts/
  run_kitti_currentfeature.m  run one KITTI sequence
  run_batch_kitti.m           run multiple KITTI sequences

data/
  ov2slam_data/        small bundled OV2SLAM demo data for KITTI seq04
  gt_poses/            bundled KITTI seq04 ground-truth pose file
  baselines/           placeholder for optional baseline trajectories

legacy/
  live_scripts/        original MATLAB Live Scripts kept for reference
  experiments/         older exploratory simulation functions

docs/
  DATA_FORMAT.md       exported data format notes
  CODE_MAP.md          paper-to-code map
  OPEN_SOURCE_TODO.md  release checklist
```

## Requirements

- MATLAB
- Optimization Toolbox (`lsqnonlin`, `linprog`)
- Symbolic Math Toolbox for the PCA-PnP fallback path

The main Bias-Eli-W PnP path uses `lsqnonlin` and `linprog`. The PCA-PnP fallback is called when fewer than 40 matches are available.

## Reproduction Notes for the Current Release

**When reproducing this released implementation (PnP11) under the recorded configuration and environment, use the re-evaluated `released_pnp11` results in the [KITTI comparison](docs/reproducibility/kitti_version_comparison.json) as the reference outputs.** These results are specific to this code version; they are not replacement values for the published paper tables.

For grayscale KITTI sequence 04, the paper values are not reproduced by the current release:

| Source | Translation ATE, SE(3)-aligned (m) | Translation RPE, adjacent frames (m) |
| --- | ---: | ---: |
| Paper Table II | 0.726 | 0.0197 |
| Current release, repeated and independently checked | **1.056770** | **0.0217528** |

The paper's ATE has historical support: the archived trajectory gives `0.725646` m, and an isolated rerun of PnP7 from commit `71f6969` gives `0.725606` m with the same inputs, calibration, BA, and evaluation. The historical and current results therefore need to be identified by implementation version. The correspondence between implementations, the described method, and all paper experiments remains under review.

The PnP TLS thresholds also belong to different residual definitions:

| Implementation | Refinement residual | PnP TLS threshold |
| --- | --- | --- |
| Historical PnP7 at commit `71f6969` | Depth-weighted normalized reprojection error | `5e-5` |
| Released PnP11 | Covariance-whitened reprojection error | `1e3` (1000) |

These thresholds operate on different residual scales and are **not directly interchangeable**. For this release, retain the default `pnpTlsDelta = 1e3` in [`scripts/run_kitti_currentfeature.m`](scripts/run_kitti_currentfeature.m); the separate epipolar BA threshold is `baTlsDelta = 3e-4`. Pose initialization and feature selection also changed. This comparison does not isolate the contribution of the threshold or whitening to the trajectory difference.

Our earlier README interpretation of `5e-5` as a suspected typo is withdrawn. The evidence establishes a difference between historical and released **implementations**; the checked arXiv v1 and accepted manuscript both describe covariance-whitened refinement. See [reproducibility status, limitations, and evidence](docs/REPRODUCIBILITY.md) for comparison details and the scope of the residual statistics.

## Quick Start

Open MATLAB in the repository root and run:

```matlab
init_currentfeature_paths;
results = run_kitti_currentfeature(4, true);
```

From a terminal:

```powershell
matlab -batch "init_currentfeature_paths; run_kitti_currentfeature(4, true);"
```

Run multiple KITTI grayscale sequences:

```matlab
init_currentfeature_paths;
batch_results = run_batch_kitti(true, [0 2:10]);
```

The batch command expects the full data package. The Git repository only includes the small `seq04` demo to keep the clone lightweight.

Quick smoke test on only the first few frames:

```matlab
init_currentfeature_paths;
opts = struct('maxFrames', 3, 'verbose', false);
results = run_kitti_currentfeature(4, true, opts);
```

`maxFrames` is only for checking that code and data are wired correctly. Use full sequences for meaningful ATE/RPE numbers.

The runner prints ATE and RPE for:

- `EIV-Initial`: Bias-Eli-W PnP pose tracking before BA,
- `EIV-BA`: after local epipolar BA,
- `OV2SLAM`: the exported OV2SLAM front-end trajectory.

## Data Notes

The repository tracks only these demo inputs:

- `data/ov2slam_data/ov2slam_data_kitti_04/`
- `data/ov2slam_data/ov2slam_data_kitti_04_gray/`
- `data/gt_poses/kitti_gt_pose/04.txt`

The full exported data should be downloaded separately and extracted into `data/` with the same layout:

```text
https://ug.link/yuanas/filemgr/share-download/?id=61f195c8666048d4a85c719b77b7c7bd
```

Each `data/ov2slam_data/ov2slam_data_kitti_XX[_gray]` folder contains:

- `ov2slam_pnp_data_seqXX.txt`: per-frame 2D tracks, associated 3D/map points, scales, point IDs, outlier flags,
- `ov2slam_stereo_matches.txt`: stereo keyframe left/right matches and point IDs,
- `ov2slam_front_end_pose_data.txt`: OV2SLAM front-end trajectory in KITTI 3x4 row format.

See [docs/DATA_FORMAT.md](docs/DATA_FORMAT.md) for details.

## Citation

If this code or the exported data are useful, please cite:

G. Zeng, Y. Shen, Z. Hong, Y. Hong, V. Ila, G. Shi, and J. Wu, "Bias-Eliminated PnP for Stereo Visual Odometry: Provably Consistent and Large-Scale Localization," IEEE Robotics and Automation Letters, vol. 10, no. 11, pp. 11840-11847, Nov. 2025.

```bibtex
@article{zeng2025bias,
  title={Bias-Eliminated PnP for Stereo Visual Odometry: Provably Consistent and Large-Scale Localization},
  author={Zeng, Guangyang and Shen, Yuan and Hong, Ziyang and Hong, Yuze and Ila, Viorela and Shi, Guodong and Wu, Junfeng},
  journal={IEEE Robotics and Automation Letters},
  year={2025},
  volume={10},
  number={11},
  pages={11840-11847},
  publisher={IEEE}
}
```
