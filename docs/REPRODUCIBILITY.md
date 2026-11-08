# Reproducibility Status

Updated 2026-11-08. This note records verified code and experiment observations. It does not designate a verified release for all paper experiments or replace a journal correction.

The table comparison uses the locally available eight-page accepted manuscript. The separately checked [arXiv v1](https://arxiv.org/abs/2504.17410v1), dated 2025-04-24, has ten pages: its KITTI Table II contains the same values, but it has no KITTI ablation table. References below to Table III's KITTI ablation therefore refer to the accepted manuscript, not arXiv v1 (whose Table III is Oxford). The publisher version of record has not been independently checked.

## Reference results for the current release

For reproduction of released PnP11, use the `released_pnp11` fields in the [version comparison](reproducibility/kitti_version_comparison.json), with the recorded inputs, parameters, and evaluation. In particular, grayscale KITTI 04 has translation ATE `1.056770` m and RPE `0.0217528` m in the checked environment. The paper's `0.726` m and `0.0197` m should not be used as expected outputs of this release.

This recommendation concerns the current code's reference outputs. It neither invalidates the archived historical trajectory nor substitutes new measurements into the paper tables. The historical-to-current residual weighting change is verified in code; both inspected manuscripts describe covariance whitening, so a change in covariance definition between those two manuscripts is not established as its cause.

## Correction to the earlier README interpretation

The statement in commit `8cfafd3` that the PnP threshold `5e-5` appeared to be a typographical error was premature. Historical code at [`71f6969`](https://github.com/LIAS-CUHKSZ/CurrentFeature-Odometry/commit/71f6969) uses this threshold with `weighted_pose_est_three_views_add_3D_uncertainty7`. That function refines depth-weighted normalized reprojection errors. Released PnP11 instead refines covariance-whitened errors and uses `1e3`.

The two versions additionally differ in initialization and feature selection:

| Operation | Historical PnP7 | Released PnP11 |
| --- | --- | --- |
| Main-path initial pose | L1 PnP; EIV calls commented out | Bias-eliminated EIV after L1 sorting |
| Additional point rejection | 10% rejection code commented out | Rejects the worst `ceil(0.1 * n)` matches |
| LM residual weighting | `depth^(-1.5)` per point | Inverse square root of propagated covariance |
| PnP TLS threshold | `5e-5` | `1e3` |

Consequently, changing only PnP11's threshold to `5e-5` does not reconstruct PnP7.

For the released residual definition, a check across 20 KITTI runs found no recorded scalar components clipped by `1e3` at initialization or after LM refinement. Applying `5e-5` to those same whitened residuals would clip more than 99.99% of components. These counterfactual statistics exclude intermediate LM trial steps and do not represent either actual optimization with `5e-5` or an assessment of historical depth-weighted residuals.

## Controlled grayscale KITTI 04 check

All fresh runs used the same exported input tracks, calibration, depth range `[0.5, 100]` m, epipolar BA implementation and threshold `3e-4`, trajectory composition, MATLAB R2023a, one computational thread, and seed 0. The historical comparison changed the PnP function and its matching threshold. No trajectory scale was fitted.

| Source | Translation ATE, SE(3)-aligned (m) | Translation RPE, adjacent frames (m) |
| --- | ---: | ---: |
| Paper Table II | 0.726 | 0.0197 |
| Archived trajectory in `71f6969` | 0.725646 | 0.0196454 |
| Fresh historical PnP7, `5e-5` | 0.725606 | 0.0196458 |
| Fresh released PnP11, `1e3` | 1.056770 | 0.0217528 |

Both fresh runs contain all 271 poses with no missing frames. An independent NumPy evaluation agrees with the reported MATLAB metrics. The historical ATE rounds to the paper value, while historical RPE still has a last-digit difference from the table. Repeated released-PnP11 runs produced identical saved trajectories.

This experiment establishes the effect of the **PnP implementation as a whole**, not the contribution of the TLS threshold alone. A saved trajectory and runner in the same commit are useful provenance evidence but do not by themselves prove which configuration generated every table entry.

## Uniform comparison across the 20 KITTI groups

The same historical PnP7 file from `71f6969` and threshold `5e-5` were used for every group in Table II, keeping the current inputs, calibration, BA, and evaluator fixed. All 20 runs completed with full coverage and passed independent metric evaluation. They were compared with 20 previously validated runs of released PnP11 with `1e3`. This section uses "PnP7" specifically for that commit: later commits changed the function with the same name.

| Mean translation error (m) | Paper, recalculated from printed rows | Historical PnP7 rerun | Released PnP11 rerun |
| --- | ---: | ---: | ---: |
| Color ATE | 3.433300 | 3.440674 | 3.135652 |
| Color RPE | 0.023870 | 0.023791 | 0.024323 |
| Grayscale ATE | 2.195000 | 2.196984 | 2.247406 |
| Grayscale RPE | 0.020020 | 0.020006 | 0.019843 |

Historical reruns match the paper's displayed precision for 13/20 ATE entries and 17/20 RPE entries; released reruns match 0/20 and 1/20, respectively. These counts do not measure statistical significance or method correctness. Mean grayscale ATE **before BA** is 14.888116 m for historical PnP7 and 7.237970 m for released PnP11, versus 17.611900 m from the printed Table III rows. Thus neither comparison establishes complete reproduction of the ablation table.

[Machine-readable comparison, configuration, and source/input hashes](reproducibility/kitti_version_comparison.json) are provided for inspection. The underlying comparison was independently checked against 40 MAT results and 240 translation metrics, including pre-BA, post-BA, and exported OV2 trajectories. These measurements are an audit of available implementations, not replacement values for the published tables.

## Method-to-code correspondence still needs verification

The locally available accepted manuscript describes a bias-eliminated initial estimate and covariance-whitened refinement in Section V-B, and 10% rejection in Section IV. Historical PnP7 does not perform those steps as described. Its numerical agreement for grayscale 04 therefore does not establish it as a faithful implementation of the published method.

Released PnP11 contains those mechanisms, but complete equation-level and experimental correspondence has not been established. For example, its linear EIV solve includes depth weights, and `scripts/run_kitti_currentfeature.m` does not enforce the manuscript's maximum of five ordinary frames per BA window. These differences need to be checked against the final experimental records and version of record.

The audit keeps historical and released results separate, uses uniform configurations across sequences, and preserves original records. No implementation is selected separately for each sequence merely because it is closer to the paper number. Algorithm defaults and paper tables have not been altered by this documentation correction.

## Scope and next verification steps

- Trace each reported main-table and ablation result to its input data, saved trajectory, runner, code revision, and parameters.
- Use the completed uniform 20-group comparison above to resolve the remaining code-to-experiment provenance questions.
- Check the initial-PnP and post-BA results separately; do not infer a PnP contribution solely from the final trajectory ATE.
- Distinguish re-evaluation of exported baseline trajectories from independently rerunning ORB-SLAM3, OV2SLAM, EPnP, or PCA-PnP.
- Confirm the final published text and submission/revision experiment records with the authors before deciding whether a journal correction is needed.

Oxford experiments, simulation claims, and the erratic-motion example are not certified by the KITTI checks above. Any correction to published scientific claims should be assessed with the authors and journal editors using the complete evidence.
