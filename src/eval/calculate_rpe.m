function [mean_trans, mean_rot, rmse_trans, rmse_rot] = calculate_rpe(poses1, poses2, delta)
% Calculate the relative pose error between two 4x4xN trajectory matrices
% Input:
%   poses1: 4x4xN ground truth poses
%   poses2: 4x4xN estimated poses
%   delta: frame interval for relative error calculation
% Output:
%   mean_trans: mean translational error (m)
%   mean_rot: mean rotational error (deg)
%   rmse_trans: RMSE of translational error (m)
%   rmse_rot: RMSE of rotational error (deg)

    n = size(poses1, 3);
    trans_error = zeros(n-delta, 1);
    rot_error = zeros(n-delta, 1);
    
    for i = 1:(n-delta)
        j = i + delta;
        
        % Calculate relative poses
        P1_rel = inv(poses1(:,:,i)) * poses1(:,:,j);
        P2_rel = inv(poses2(:,:,i)) * poses2(:,:,j);
        
        % Calculate error transformation
        E = inv(P1_rel) * P2_rel;
        
        % Translation error
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