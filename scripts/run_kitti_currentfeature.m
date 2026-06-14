function results = run_kitti_currentfeature(seq, use_gray, opts)
%RUN_KITTI_CURRENTFEATURE Run CurrentFeature Odometry on exported KITTI data.
%
%   results = run_kitti_currentfeature(0, true)
%
% This runner consumes OV2SLAM-exported feature tracks, stereo keyframe
% matches, and front-end poses. It does not run image feature extraction.

if nargin < 1 || isempty(seq)
    seq = 0;
end
if nargin < 2 || isempty(use_gray)
    use_gray = true;
end
if nargin < 3 || isempty(opts)
    opts = struct();
end
opts = default_options(opts);

repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(repo_root, 'src')));

seq_str = sprintf('%02d', seq);
gray_suffix = '';
if use_gray
    gray_suffix = '_gray';
end

ov2_base_path = fullfile(repo_root, 'data', 'ov2slam_data', ...
    sprintf('ov2slam_data_kitti_%s%s', seq_str, gray_suffix));
frames_path = fullfile(ov2_base_path, sprintf('ov2slam_pnp_data_seq%s.txt', seq_str));
stereo_path = fullfile(ov2_base_path, 'ov2slam_stereo_matches.txt');
ov2_poses_path = fullfile(ov2_base_path, 'ov2slam_front_end_pose_data.txt');
gt_path = fullfile(repo_root, 'data', 'gt_poses', 'kitti_gt_pose', [seq_str '.txt']);

assert(isfile(frames_path), 'Missing frame track file: %s', frames_path);
assert(isfile(stereo_path), 'Missing stereo keyframe match file: %s', stereo_path);
assert(isfile(ov2_poses_path), 'Missing OV2SLAM pose file: %s', ov2_poses_path);
assert(isfile(gt_path), 'Missing KITTI ground-truth pose file: %s', gt_path);

frames = read_ov2slam_data(frames_path);
stereo_frames = read_stereo_matches(stereo_path);
gt_poses = read_kitti_pose(gt_path);
ov2slam_poses = read_kitti_pose(ov2_poses_path);

[K, R_l1_r1, t_l1_r1, T_20] = kitti_calibration(seq, use_gray);
fx = K(1, 1);
fy = K(2, 2);
cx = K(1, 3);
cy = K(2, 3);

num_frames = min(length(frames), size(gt_poses, 3) - 1);
if isfinite(opts.maxFrames)
    num_frames = min(num_frames, opts.maxFrames);
end
num_stereo_frames = length(stereo_frames);

relative_poses = cell(num_frames, 1);
relative_poses_ba = cell(num_frames, 1);
relative_poses_gt = cell(num_frames, 1);
relative_poses_ov2 = cell(num_frames, 1);

errors_rotation = [];
errors_translation = [];
errors_rotation_ba = [];
errors_translation_ba = [];
errors_rotation_ov2 = [];
errors_translation_ov2 = [];
missing_frames = [];

current_kf_idx = 1;
if opts.rngSeed >= 0
    rng(opts.rngSeed);
end

fprintf('Running KITTI seq%s%s with %d frames and %d keyframes.\n', ...
    seq_str, gray_suffix, num_frames, num_stereo_frames);

for frame_idx = 1:num_frames
    if current_kf_idx > num_stereo_frames
        warning('No more keyframes at frame %d. Stopping.', frame_idx);
        break;
    end

    if opts.useInliers
        inlier_index = find(frames(frame_idx).outliers == 0);
        frame_point_pts = frames(frame_idx).image_pts(inlier_index, :);
        current_frame_ids = frames(frame_idx).point_ids(inlier_index);
    else
        frame_point_pts = frames(frame_idx).image_pts;
        current_frame_ids = frames(frame_idx).point_ids;
    end

    kf_ids = stereo_frames(current_kf_idx).point_ids;
    [common_ids, frame_indices, kf_indices] = intersect(current_frame_ids, kf_ids);

    if isempty(common_ids)
        warning('Frame %d: no common points with keyframe %d.', frame_idx, current_kf_idx);
        missing_frames(end + 1) = frame_idx; %#ok<AGROW>
        continue;
    end

    frame_pts = frame_point_pts(frame_indices, :)';
    kf_left_pts = stereo_frames(current_kf_idx).left_pts(kf_indices, :)';
    kf_right_pts = stereo_frames(current_kf_idx).right_pts(kf_indices, :)';

    disparity = abs(kf_left_pts(1, :) - kf_right_pts(1, :));
    baseline = abs(t_l1_r1(1));
    min_disparity = baseline * fx / opts.maxDepth;
    max_disparity = baseline * fx / opts.minDepth;
    valid_idx = find(disparity >= min_disparity & disparity <= max_disparity);

    if isempty(valid_idx)
        warning('Frame %d: no valid matches after depth filtering.', frame_idx);
        missing_frames(end + 1) = frame_idx; %#ok<AGROW>
        continue;
    end

    u1 = kf_left_pts(1, valid_idx);
    v1 = kf_left_pts(2, valid_idx);
    u2 = kf_right_pts(1, valid_idx);
    v2 = kf_right_pts(2, valid_idx);
    u3 = frame_pts(1, valid_idx);
    v3 = frame_pts(2, valid_idx);

    x1_h = [(u2 - cx) / fx; (v2 - cy) / fy; ones(1, numel(valid_idx))];
    x2_h = [(u1 - cx) / fx; (v1 - cy) / fy; ones(1, numel(valid_idx))];
    x3_h = [(u3 - cx) / fx; (v3 - cy) / fy; ones(1, numel(valid_idx))];

    is_keyframe = check_if_keyframe(frame_idx, stereo_frames);

    pose_f = T_20 * gt_poses(:, :, frame_idx + 1) / T_20;
    pose_kf = T_20 * gt_poses(:, :, stereo_frames(current_kf_idx).frame_id + 1) / T_20;
    gt_relative_pose = pose_kf \ pose_f;

    pose_f_ov2 = ov2slam_poses(:, :, frame_idx + 1);
    pose_kf_ov2 = ov2slam_poses(:, :, stereo_frames(current_kf_idx).frame_id + 1);
    ov2_relative_pose = pose_kf_ov2 \ pose_f_ov2;

    [~, ~, ~, R21, t21] = weighted_pose_est_three_views_add_3D_uncertainty11( ...
        x3_h, x2_h, x1_h, R_l1_r1, t_l1_r1, opts.pnpTlsDelta, frame_idx, current_kf_idx);

    est_T = [R21', -R21' * t21; 0, 0, 0, 1];

    relative_poses{frame_idx}.pose = est_T;
    relative_poses{frame_idx}.kf_flag = is_keyframe;
    relative_poses_gt{frame_idx}.pose = gt_relative_pose;
    relative_poses_gt{frame_idx}.kf_flag = is_keyframe;
    relative_poses_ov2{frame_idx}.pose = ov2_relative_pose;
    relative_poses_ov2{frame_idx}.kf_flag = is_keyframe;
    relative_poses_ba{frame_idx}.kf_flag = is_keyframe;

    [rot_err, trans_err] = pose_error(est_T, gt_relative_pose);
    [rot_err_ov2, trans_err_ov2] = pose_error(ov2_relative_pose, gt_relative_pose);
    errors_rotation(end + 1) = rot_err; %#ok<AGROW>
    errors_translation(end + 1) = trans_err; %#ok<AGROW>
    errors_rotation_ov2(end + 1) = rot_err_ov2; %#ok<AGROW>
    errors_translation_ov2(end + 1) = trans_err_ov2; %#ok<AGROW>

    if opts.verbose
        fprintf('Frame %d -> KF %d (frame_id=%d): %d common, %d valid.\n', ...
            frame_idx, current_kf_idx, stereo_frames(current_kf_idx).frame_id, ...
            numel(common_ids), numel(valid_idx));
    end

    if is_keyframe && current_kf_idx < num_stereo_frames
        previous_kf_idx = current_kf_idx;
        current_kf_idx = current_kf_idx + 1;

        ba_start = stereo_frames(previous_kf_idx).frame_id + 1;
        ba_end = stereo_frames(current_kf_idx).frame_id;
        init_poses = relative_poses(ba_start:ba_end);

        if any(cellfun(@isempty, init_poses))
            warning('Skipping BA for KF %d because some initial poses are missing.', current_kf_idx);
        else
            ba_frame_num = max(0, ba_end - ba_start);
            frame_feature_arrays = cell(ba_frame_num, 1);
            for i = 1:ba_frame_num
                frame_feature_arrays{i} = frames(ba_start + i - 1);
            end

            kframe_feature_arrays = cell(2, 1);
            kframe_feature_arrays{1} = stereo_frames(previous_kf_idx);
            kframe_feature_arrays{2} = stereo_frames(current_kf_idx);

            opt_poses = Epipolar_BA_Multi(init_poses, eye(3), t_l1_r1, ...
                frame_feature_arrays, kframe_feature_arrays, K, K, opts.baTlsDelta);

            for i = 1:length(opt_poses)
                frame_id = stereo_frames(previous_kf_idx).frame_id + i;
                relative_poses_ba{frame_id}.pose = opt_poses{i};
                relative_poses_ba{frame_id}.kf_flag = relative_poses{frame_id}.kf_flag;

                [rot_err_ba, trans_err_ba] = pose_error(opt_poses{i}, ...
                    relative_poses_gt{frame_id}.pose);
                errors_rotation_ba(end + 1) = rot_err_ba; %#ok<AGROW>
                errors_translation_ba(end + 1) = trans_err_ba; %#ok<AGROW>
            end
        end
    end
end

[relative_poses, relative_poses_gt, relative_poses_ov2] = fill_missing_pose_cells( ...
    relative_poses, relative_poses_gt, relative_poses_ov2);

for i = 1:length(relative_poses)
    if isempty(relative_poses_ba{i}) || ~isfield(relative_poses_ba{i}, 'pose')
        relative_poses_ba{i} = relative_poses{i};
    end
end

[poses, poses_ba, poses_gt, poses_ov2] = compose_trajectories( ...
    relative_poses, relative_poses_ba, relative_poses_gt, relative_poses_ov2);

metrics = evaluate_trajectories(poses_gt, poses, poses_ba, poses_ov2);
print_metrics(seq_str, gray_suffix, metrics);

if opts.saveResults
    results_dir = fullfile(repo_root, 'results');
    if ~isfolder(results_dir)
        mkdir(results_dir);
    end
    result_file = fullfile(results_dir, sprintf('seq%s%s_results.mat', seq_str, gray_suffix));
    save(result_file, 'poses', 'poses_ba', 'poses_gt', 'poses_ov2', 'metrics', 'missing_frames');
    fprintf('Saved results to %s\n', result_file);
end

results = struct();
results.seq = seq;
results.use_gray = use_gray;
results.metrics = metrics;
results.poses = poses;
results.poses_ba = poses_ba;
results.poses_gt = poses_gt;
results.poses_ov2 = poses_ov2;
results.missing_frames = missing_frames;
results.relative_errors.initial.rotation_deg = errors_rotation;
results.relative_errors.initial.translation_m = errors_translation;
results.relative_errors.ba.rotation_deg = errors_rotation_ba;
results.relative_errors.ba.translation_m = errors_translation_ba;
results.relative_errors.ov2.rotation_deg = errors_rotation_ov2;
results.relative_errors.ov2.translation_m = errors_translation_ov2;
end

function opts = default_options(opts)
opts = set_default(opts, 'useInliers', true);
opts = set_default(opts, 'minDepth', 0.5);
opts = set_default(opts, 'maxDepth', 100);
opts = set_default(opts, 'pnpTlsDelta', 1e3);
opts = set_default(opts, 'baTlsDelta', 0.3e-3);
opts = set_default(opts, 'rngSeed', -1);
opts = set_default(opts, 'verbose', true);
opts = set_default(opts, 'saveResults', false);
opts = set_default(opts, 'maxFrames', inf);
end

function opts = set_default(opts, name, value)
if ~isfield(opts, name) || isempty(opts.(name))
    opts.(name) = value;
end
end

function [K, R_l1_r1, t_l1_r1, T_20] = kitti_calibration(seq, use_gray)
R_l1_r1 = eye(3);
if seq >= 0 && seq <= 2
    K = [7.188560000000e+02 0 6.071928000000e+02;
         0 7.188560000000e+02 1.852157000000e+02;
         0 0 1];
    if use_gray
        t_l1_r1 = [0.537165718864418; 0; 0];
        T_20 = eye(4);
    else
        t_l1_r1 = [0.5323; 0; 0];
        T_20 = [1 0 0 0.0631; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    end
elseif seq == 3
    K = [7.215377000000e+02 0 6.095593000000e+02;
         0 7.215377000000e+02 1.728540000000e+02;
         0 0 1];
    if use_gray
        t_l1_r1 = [0.537150588250621; 0; 0];
        T_20 = eye(4);
    else
        t_l1_r1 = [0.5327; 0; 0];
        T_20 = [1 0 0 0.0622; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    end
elseif seq >= 4 && seq <= 10
    K = [7.070912000000e+02 0 6.018873000000e+02;
         0 7.070912000000e+02 1.831104000000e+02;
         0 0 1];
    if use_gray
        t_l1_r1 = [0.537150653267924; 0; 0];
        T_20 = eye(4);
    else
        t_l1_r1 = [0.5433; 0; 0];
        T_20 = [1 0 0 0.0663; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    end
else
    error('Unsupported KITTI sequence: %d', seq);
end
end

function is_keyframe = check_if_keyframe(frame_idx, stereo_frames)
frame_ids = [stereo_frames.frame_id];
is_keyframe = any(frame_ids == frame_idx);
end

function [rot_err, trans_err] = pose_error(T_est, T_gt)
R_err = T_est(1:3, 1:3)' * T_gt(1:3, 1:3);
rot_arg = (trace(R_err) - 1) / 2;
rot_arg = max(-1, min(1, rot_arg));
rot_err = real(acosd(rot_arg));
trans_err = norm(T_est(1:3, 4) - T_gt(1:3, 4));
end

function [rel, rel_gt, rel_ov2] = fill_missing_pose_cells(rel, rel_gt, rel_ov2)
for i = 1:length(rel)
    if isempty(rel{i}) || ~isfield(rel{i}, 'pose')
        rel{i}.pose = eye(4);
        rel{i}.kf_flag = false;
    end
    if isempty(rel_gt{i}) || ~isfield(rel_gt{i}, 'pose')
        rel_gt{i}.pose = eye(4);
        rel_gt{i}.kf_flag = rel{i}.kf_flag;
    end
    if isempty(rel_ov2{i}) || ~isfield(rel_ov2{i}, 'pose')
        rel_ov2{i}.pose = eye(4);
        rel_ov2{i}.kf_flag = rel{i}.kf_flag;
    end
end
end

function [poses, poses_ba, poses_gt, poses_ov2] = compose_trajectories( ...
    rel, rel_ba, rel_gt, rel_ov2)
N = length(rel);
poses = repmat(eye(4), 1, 1, N + 1);
poses_ba = repmat(eye(4), 1, 1, N + 1);
poses_gt = repmat(eye(4), 1, 1, N + 1);
poses_ov2 = repmat(eye(4), 1, 1, N + 1);

pose_kf = eye(4);
pose_ba_kf = eye(4);
pose_gt_kf = eye(4);
pose_ov2_kf = eye(4);

for i = 2:N + 1
    poses(:, :, i) = pose_kf * rel{i - 1}.pose;
    poses_ba(:, :, i) = pose_ba_kf * rel_ba{i - 1}.pose;
    poses_gt(:, :, i) = pose_gt_kf * rel_gt{i - 1}.pose;
    poses_ov2(:, :, i) = pose_ov2_kf * rel_ov2{i - 1}.pose;

    if rel{i - 1}.kf_flag
        pose_kf = poses(:, :, i);
        pose_ba_kf = poses_ba(:, :, i);
        pose_gt_kf = poses_gt(:, :, i);
        pose_ov2_kf = poses_ov2(:, :, i);
    end
end
end

function metrics = evaluate_trajectories(poses_gt, poses, poses_ba, poses_ov2)
metrics = struct();
[~, ~, metrics.initial.ate_wo.trans_rmse, metrics.initial.ate_wo.rot_rmse] = ...
    calculate_ate(poses_gt, poses, 0);
[~, ~, metrics.ba.ate_wo.trans_rmse, metrics.ba.ate_wo.rot_rmse] = ...
    calculate_ate(poses_gt, poses_ba, 0);
[~, ~, metrics.ov2.ate_wo.trans_rmse, metrics.ov2.ate_wo.rot_rmse] = ...
    calculate_ate(poses_gt, poses_ov2, 0);

[~, ~, metrics.initial.ate_align.trans_rmse, metrics.initial.ate_align.rot_rmse] = ...
    calculate_ate(poses_gt, poses, 1);
[~, ~, metrics.ba.ate_align.trans_rmse, metrics.ba.ate_align.rot_rmse] = ...
    calculate_ate(poses_gt, poses_ba, 1);
[~, ~, metrics.ov2.ate_align.trans_rmse, metrics.ov2.ate_align.rot_rmse] = ...
    calculate_ate(poses_gt, poses_ov2, 1);

[~, ~, metrics.initial.rpe.trans_rmse, metrics.initial.rpe.rot_rmse] = ...
    calculate_rpe(poses_gt, poses, 1);
[~, ~, metrics.ba.rpe.trans_rmse, metrics.ba.rpe.rot_rmse] = ...
    calculate_rpe(poses_gt, poses_ba, 1);
[~, ~, metrics.ov2.rpe.trans_rmse, metrics.ov2.rpe.rot_rmse] = ...
    calculate_rpe(poses_gt, poses_ov2, 1);
end

function print_metrics(seq_str, gray_suffix, metrics)
fprintf('\nKITTI sequence %s%s\n', seq_str, gray_suffix);
fprintf('========== ATE without alignment ==========\n');
fprintf('Method        Translation(m)    Rotation(deg)\n');
fprintf('EIV-Initial   %12.4f    %12.4f\n', ...
    metrics.initial.ate_wo.trans_rmse, metrics.initial.ate_wo.rot_rmse);
fprintf('EIV-BA        %12.4f    %12.4f\n', ...
    metrics.ba.ate_wo.trans_rmse, metrics.ba.ate_wo.rot_rmse);
fprintf('OV2SLAM       %12.4f    %12.4f\n', ...
    metrics.ov2.ate_wo.trans_rmse, metrics.ov2.ate_wo.rot_rmse);

fprintf('\n========== ATE with alignment ==========\n');
fprintf('Method        Translation(m)    Rotation(deg)\n');
fprintf('EIV-Initial   %12.4f    %12.4f\n', ...
    metrics.initial.ate_align.trans_rmse, metrics.initial.ate_align.rot_rmse);
fprintf('EIV-BA        %12.4f    %12.4f\n', ...
    metrics.ba.ate_align.trans_rmse, metrics.ba.ate_align.rot_rmse);
fprintf('OV2SLAM       %12.4f    %12.4f\n', ...
    metrics.ov2.ate_align.trans_rmse, metrics.ov2.ate_align.rot_rmse);

fprintf('\n========== RPE ==========\n');
fprintf('Method        Translation(m)    Rotation(deg)\n');
fprintf('EIV-Initial   %12.4f    %12.4f\n', ...
    metrics.initial.rpe.trans_rmse, metrics.initial.rpe.rot_rmse);
fprintf('EIV-BA        %12.4f    %12.4f\n', ...
    metrics.ba.rpe.trans_rmse, metrics.ba.rpe.rot_rmse);
fprintf('OV2SLAM       %12.4f    %12.4f\n', ...
    metrics.ov2.rpe.trans_rmse, metrics.ov2.rpe.rot_rmse);
end
