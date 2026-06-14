# CurrentFeature Odometry

MATLAB reference implementation and exported data for:

**Bias-Eliminated PnP for Stereo Visual Odometry: Provably Consistent and Large-Scale Localization**

This repository contains the MATLAB back-end code for CurrentFeature Odometry and a small KITTI sequence 04 demo exported from OV2SLAM. The full exported data package is intentionally kept outside Git and should be downloaded separately.

## What This Repository Runs

The included MATLAB pipeline does **not** run feature extraction from raw stereo images. Instead, it consumes exported OV2SLAM front-end data:

- per-frame 2D feature tracks and point IDs,
- stereo keyframe left/right matches,
- OV2SLAM front-end trajectory poses,
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

The full exported data should be downloaded separately and extracted into `data/` with the same layout. Each `data/ov2slam_data/ov2slam_data_kitti_XX[_gray]` folder contains:

- `ov2slam_pnp_data_seqXX.txt`: per-frame 2D tracks, associated 3D/map points, scales, point IDs, outlier flags,
- `ov2slam_stereo_matches.txt`: stereo keyframe left/right matches and point IDs,
- `ov2slam_front_end_pose_data.txt`: OV2SLAM front-end trajectory in KITTI 3x4 row format.

See [docs/DATA_FORMAT.md](docs/DATA_FORMAT.md) for details.

## Important Release Notes

This folder is an open-source candidate, not yet a polished public release:

- Choose a license before publishing.
- Upload the full exported data package to external storage and add the public download URL.
- Confirm redistribution rights for the bundled demo and externally hosted data.
- Clean mojibake comments in legacy MATLAB files if you want the source to look publication-ready.

## Citation

If this code is useful, please cite the paper once the final bibliographic information is available.
