# Data

This Git repository includes only a small KITTI sequence 04 demo so the code can be tested without downloading the full dataset.

Tracked demo files:

- `ov2slam_data/ov2slam_data_kitti_04/`
- `ov2slam_data/ov2slam_data_kitti_04_gray/`
- `gt_poses/kitti_gt_pose/04.txt`

The full exported data package is not tracked in Git. Download it from:

```text
TODO: add public cloud download URL
```

After extracting the full package, the expected layout is:

```text
data/
  ov2slam_data/
    ov2slam_data_kitti_00/
    ov2slam_data_kitti_00_gray/
    ...
  gt_poses/
    kitti_gt_pose/
    euroc_gt_pose/
  baselines/
    orb_slam3_est_results/
```

The MATLAB runners use the same relative paths for the bundled demo and the full data package.
