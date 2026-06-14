# Data Format

This repository uses feature and pose data exported from OV2SLAM. The MATLAB runners do not read raw images.

Only a small KITTI sequence 04 demo is tracked in Git. The full exported data package should be downloaded separately and extracted into `data/` using the same directory layout.

## OV2SLAM Per-Frame PnP Tracks

Path pattern:

```text
data/ov2slam_data/ov2slam_data_kitti_XX[_gray]/ov2slam_pnp_data_seqXX.txt
```

Reader:

```matlab
frames = read_ov2slam_data(path);
```

Each frame is stored as an 8-line block:

1. left image x coordinates,
2. left image y coordinates,
3. associated 3D/map point X coordinates,
4. associated 3D/map point Y coordinates,
5. associated 3D/map point Z coordinates,
6. feature scales,
7. globally unique point IDs,
8. outlier flags.

The reader returns a struct array with:

```matlab
frames(i).image_pts   % N x 2 pixel coordinates
frames(i).world_pts   % N x 3 exported map/world points
frames(i).scales      % N x 1 feature scales
frames(i).point_ids   % N x 1 point IDs
frames(i).outliers    % N x 1 outlier flags, 0 means inlier
```

CurrentFeature Odometry primarily uses `image_pts`, `point_ids`, and `outliers`. The exported `world_pts` are kept for reference and compatibility.

## Stereo Keyframe Matches

Path:

```text
data/ov2slam_data/ov2slam_data_kitti_XX[_gray]/ov2slam_stereo_matches.txt
```

Reader:

```matlab
stereo_frames = read_stereo_matches(path);
```

Each keyframe is stored as a 6-line block:

1. keyframe frame ID,
2. left image x coordinates,
3. left image y coordinates,
4. right image x coordinates,
5. right image y coordinates,
6. globally unique point IDs.

The reader returns:

```matlab
stereo_frames(i).frame_id   % frame index from the original sequence
stereo_frames(i).left_pts   % N x 2 left image pixels
stereo_frames(i).right_pts  % N x 2 right image pixels
stereo_frames(i).point_ids  % N x 1 point IDs
```

These stereo matches are the "current feature" source. The runner triangulates them in the current keyframe and matches them to later frames by `point_ids`.

## Front-End Poses

Path:

```text
data/ov2slam_data/ov2slam_data_kitti_XX[_gray]/ov2slam_front_end_pose_data.txt
```

The pose file follows KITTI odometry format: one pose per line, with 12 values representing a row-major 3x4 transform. It is read by:

```matlab
poses = read_kitti_pose(path);
```

## Ground Truth

KITTI ground truth is stored in:

```text
data/gt_poses/kitti_gt_pose/XX.txt
```

It uses the same 3x4 KITTI pose format.

EuRoC ground-truth utilities are kept under:

```text
data/gt_poses/euroc_gt_pose/
```

The Git repository currently includes only:

```text
data/gt_poses/kitti_gt_pose/04.txt
```

## Coordinate Conventions

The KITTI runner uses normalized pinhole coordinates after subtracting principal point and dividing by focal length. Stereo baseline parameters are defined in `scripts/run_kitti_currentfeature.m` in the `kitti_calibration` helper.
