# Paper-to-Code Map

## CurrentFeature Odometry Runner

- `scripts/run_kitti_currentfeature.m`

Consumes exported OV2SLAM data, tracks each ordinary frame relative to the latest keyframe, runs Bias-Eli-W PnP, triggers local epipolar BA at new keyframes, composes the trajectory, and reports ATE/RPE.

## Triangulation and Uncertainty

- `src/pnp/uncertain_aware_triangulation2_used.m`

Implements the linear least-squares stereo triangulation and first-order uncertainty propagation used by the PnP estimator.

## Bias-Eliminated Weighted PnP

- `src/pnp/weighted_pose_est_three_views_add_3D_uncertainty11.m`
- `src/pnp/EIV_PnP2.m`
- `src/pnp/L1_norm_PnP.m`
- `src/pnp/L1_norm_opt_lp.m`

The main wrapper performs:

1. stereo triangulation from keyframe left/right matches,
2. feature-noise variance estimation,
3. L1-norm PnP sorting for outlier rejection,
4. bias-eliminated EIV PnP,
5. weighted LM refinement with a truncated least-squares residual.

`EIV_PnP2.m` contains the bias-eliminated linear estimator.

## Epipolar Bundle Adjustment

- `src/ba/Epipolar_BA_Multi.m`

Runs local BA over the latest two keyframes and intermediate ordinary frames. It optimizes relative poses with point-to-epipolar-line residuals rather than optimizing 3D map points.

## Evaluation

- `src/eval/calculate_ate.m`
- `src/eval/calculate_rpe.m`
- `src/eval/alignTrajectorySE3.m`
- `src/eval/write_metrics_to_excel.m`

These utilities evaluate absolute trajectory error and relative pose error.

## Baselines and Fallbacks

- `src/baselines/pca_pnp/`

PCA-PnP fallback used in the current wrapper when there are too few matches for the main weighted estimator.

## Legacy Material

- `legacy/live_scripts/`
- `legacy/experiments/`

These files are preserved for traceability. The recommended public entry point is `scripts/run_kitti_currentfeature.m`.
