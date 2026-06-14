function [mean_trans, mean_rot, rmse_trans, rmse_rot,poses1_align,poses2_align] = calculate_ate(poses1, poses2, need_align)
% Calculate the absolute trajectory error between two 4x4xN trajectory matrices
% Input:
%   poses1: 4x4xN ground truth poses
%   poses2: 4x4xN estimated poses
% Output:
%   mean_trans: mean translational error (m)
%   mean_rot: mean rotational error (deg)
%   rmse_trans: RMSE of translational error (m)
%   rmse_rot: RMSE of rotational error (deg)
    if need_align ==1
        [poses2,T_align] = alignTrajectorySE3(poses1, poses2);
    end
    poses1_align = poses1;
    poses2_align = poses2;
    n = size(poses1, 3);
    trans_error = zeros(n, 1);
    rot_error = zeros(n, 1);
    
    for i = 1:n
        % Calculate error transformation
        % E = inv(P1) * P2
        E = inv(poses1(:,:,i)) * poses2(:,:,i);
        
        % Translation error (Euclidean distance)
        trans_error(i) = norm(E(1:3,4));
        
        % Rotation error
        R_err = E(1:3,1:3);
        rot_error(i) = rad2deg(norm(logm(R_err))/sqrt(2));
    end
    
    % Calculate statistics
    mean_trans = mean(trans_error);
    mean_rot = mean(rot_error);
    rmse_trans = sqrt(mean(trans_error.^2));
    rmse_rot = sqrt(mean(rot_error.^2));
end