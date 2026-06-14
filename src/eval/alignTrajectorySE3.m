function [T_est_aligned,T_align] = alignTrajectorySE3(T_gt, T_est)
    % Aligns the estimated trajectory (T_est) to the ground truth trajectory (T_gt)
    % using SE(3) alignment (rigid body transformation).
    %
    % Inputs:
    %   T_gt: Ground truth trajectory (4 ¡Á 4 ¡Á N array of poses)
    %   T_est: Estimated trajectory (4 ¡Á 4 ¡Á N array of poses)
    %
    % Outputs:
    %   T_est_aligned: Aligned estimated trajectory (4 ¡Á 4 ¡Á N array of poses)

    % Number of poses
    N = size(T_gt, 3);

    % Extract translation components (3 ¡Á N matrices)
    P_gt = reshape(T_gt(1:3, 4, :), 3, N); % Ground truth translations
    P_est = reshape(T_est(1:3, 4, :), 3, N); % Estimated translations

    % Compute centroids
    centroid_gt = mean(P_gt, 2); % Centroid of ground truth
    centroid_est = mean(P_est, 2); % Centroid of estimated trajectory

    % Center the translations
    P_gt_centered = P_gt - centroid_gt; % Centered ground truth translations
    P_est_centered = P_est - centroid_est; % Centered estimated translations

    % Compute the cross-covariance matrix
    H = P_est_centered * P_gt_centered';

    % Compute the optimal rotation using Singular Value Decomposition (SVD)
    [U, ~, V] = svd(H);
    R_align = V * U';

    % Ensure the rotation matrix is proper (det(R) = 1)
    if det(R_align) < 0
        V(:, end) = -V(:, end);
        R_align = V * U';
    end

    % Compute the optimal translation
    t_align = centroid_gt - R_align * centroid_est;

    % Construct the alignment transformation matrix
    T_align = eye(4);
    T_align(1:3, 1:3) = R_align;
    T_align(1:3, 4) = t_align;

    % Apply the alignment to the estimated trajectory
    T_est_aligned = zeros(size(T_est));
    for i = 1:N
        T_est_aligned(:, :, i) = T_align * T_est(:, :, i);
    end
end