function opt_poses = Epipolar_BA_Multi(init_poses, baseline_R,baseline_t, frame_feature_arrays, kframe_feature_arrays,K_l,K_r,BA_tls_delta)
% 输入:
% init_poses: cell数组，包含所有帧相对于kf1的位姿初值
% baseline: 双目基线变换
% frame_feature_arrays: 普通帧数据，包含左目2D点和特征点ID
% kframe_feature_arrays: 关键帧数据，包含左右目2D点和特征点ID

    % 优化器配置
    options = optimoptions('lsqnonlin', 'Display', 'off', ...
                          'Algorithm', 'levenberg-marquardt');

    % 构造初始优化变量
    init_kexi = [];
    for k = 1:length(init_poses)
        R = init_poses{k}.pose(1:3,1:3);
        t = init_poses{k}.pose(1:3,4);
        init_kexi = [init_kexi; anti_skew_symmetric(logm(R)); t];
    end
    init0 = anti_skew_symmetric(logm(baseline_R(1:3,1:3)));
    init_kexi = [init_kexi; init0];
    debug_myfunc = myfun(init_kexi);
    % 优化
    x = lsqnonlin(@myfun, init_kexi, [], [], options);
    debug_myfunc2 = myfun(x);
    % size(myfun(x),1)
    % 恢复优化后的位姿
    opt_poses = cell(length(init_poses), 1);
    for j1 = 1:length(init_poses)
        idx1 = (j1-1)*6 + 1;
        opt_poses{j1} = cal_T_from_kexi(x(idx1:idx1+5));
    end

    function g = myfun(kexi)
        g = [];
        % delta=0.3*10^(-3); % kitti
        % delta=0.3*10^(-4); % euroc
        delta = BA_tls_delta;      
        fx_l = K_l(1, 1);
        fy_l = K_l(2, 2);
        cx_l = K_l(1, 3);
        cy_l = K_l(2, 3);

        fx_r = K_r(1, 1);
        fy_r = K_r(2, 2);
        cx_r = K_r(1, 3);
        cy_r = K_r(2, 3);
        % 恢复所有位姿
        num_poses = (length(kexi)-3)/6;
        poses = cell(num_poses, 1);
        for i = 1:num_poses
            idx = (i-1)*6 + 1;
            poses{i} = cal_T_from_kexi(kexi(idx:idx+5));
        end
        
        % 构造双目变换
        % T0 = [expm(skew_symmetric(kexi(end-2:end))),[0.5433;0;0];0,0,0,1];
        T0 = [expm(skew_symmetric(kexi(end-2:end))),baseline_t;0,0,0,1];
        % 1. KF2(最后一帧)与其他所有帧之间的约束
        kf2_pts_l = kframe_feature_arrays{2}.left_pts';  % KF2左目点
        kf2_pts_r = kframe_feature_arrays{2}.right_pts'; % KF2右目点
        kf2_ids = kframe_feature_arrays{2}.point_ids;
        % 归一化KF2的点
        kf2_pts_l_norm = [(kf2_pts_l(1,:) - cx_l)/fx_l;
                         (kf2_pts_l(2,:) - cy_l)/fy_l;
                         ones(1,size(kf2_pts_l,2))];
        kf2_pts_r_norm = [(kf2_pts_r(1,:) - cx_r)/fx_r;
                         (kf2_pts_r(2,:) - cy_r)/fy_r;
                         ones(1,size(kf2_pts_r,2))];        
        % % 1.0 KF2左右眼约束
        % T_rel = T0; 
        % % T_rel = inv(T0) ;
        % l = cal_E_from_T(T_rel)' * kf2_pts_l_norm(:,:);
        % d = sum(l .* kf2_pts_r_norm(:,:)) ./ sqrt(l(1,:).^2 + l(2,:).^2);
        % g = [g; huber_loss(d(:), delta)]; 


        
        % 1.1 KF2与普通帧的约束
        for frame_idx = 1:length(frame_feature_arrays)
            frame_pts = frame_feature_arrays{frame_idx}.image_pts';
            frame_ids = frame_feature_arrays{frame_idx}.point_ids;
            
            % 归一化普通帧的点
            frame_pts_norm = [(frame_pts(1,:) - cx_l)/fx_l;
                            (frame_pts(2,:) - cy_l)/fy_l;
                            ones(1,size(frame_pts,2))];
            
            % 找到共同点
            [~, idx_frame, idx_kf2] = intersect(frame_ids, kf2_ids);
            
            if ~isempty(idx_frame)
                % KF2左目到普通帧左目
                T_rel = inv(poses{end}) * poses{frame_idx}; % T_KF2_F1/T_KF2_F2
                % T_rel = inv(poses{frame_idx}) *poses{end} ;
                l = cal_E_from_T(T_rel)' * kf2_pts_l_norm(:,idx_kf2);
                d = sum(l .* frame_pts_norm(:,idx_frame)) ./ sqrt(l(1,:).^2 + l(2,:).^2);
                g = [g; truncate_loss(d(:), delta)];
                
                % KF2右目到普通帧左目
                T_rel = inv(T0) * inv(poses{end}) * poses{frame_idx};
                % T_rel = inv(poses{frame_idx}) * (poses{end}) * (T0)   ;
                l = cal_E_from_T(T_rel)' * kf2_pts_r_norm(:,idx_kf2);
                d = sum(l .* frame_pts_norm(:,idx_frame)) ./ sqrt(l(1,:).^2 + l(2,:).^2);
                g = [g; truncate_loss(d(:), delta)];
            end
        end
        
        % 1.2 KF2与KF1的约束
        kf1_pts_l = kframe_feature_arrays{1}.left_pts';  % KF1左目点
        kf1_pts_r = kframe_feature_arrays{1}.right_pts'; % KF1右目点
        kf1_ids = kframe_feature_arrays{1}.point_ids;
        % 归一化KF1的点
        kf1_pts_l_norm = [(kf1_pts_l(1,:) - cx_l)/fx_l;
                         (kf1_pts_l(2,:) - cy_l)/fy_l;
                         ones(1,size(kf1_pts_l,2))];
        kf1_pts_r_norm = [(kf1_pts_r(1,:) - cx_r)/fx_r;
                         (kf1_pts_r(2,:) - cy_r)/fy_r;
                         ones(1,size(kf1_pts_r,2))];
        % % 1.2.0 KF1左右眼约束
        % T_rel = T0; 
        % % T_rel = inv(T0) ;
        % l = cal_E_from_T(T_rel)' * kf1_pts_l_norm(:,:);
        % d = sum(l .* kf1_pts_r_norm(:,:)) ./ sqrt(l(1,:).^2 + l(2,:).^2);
        % g = [g; huber_loss(d(:), delta)];         

        
        [~, idx_kf1, idx_kf2] = intersect(kf1_ids, kf2_ids);
        
        if ~isempty(idx_kf1)

            % KF2左目到KF1左目
            T_rel = inv(poses{end});
            % T_rel = (poses{end});
            l = cal_E_from_T(T_rel)' * kf2_pts_l_norm(:,idx_kf2);
            d = sum(l .* kf1_pts_l_norm(:,idx_kf1)) ./ sqrt(l(1,:).^2 + l(2,:).^2);
            g = [g; truncate_loss(d(:), delta)];
            
            % KF2左目到KF1右目
            T_rel = inv(poses{end}) * T0;
            % T_rel = inv(T0) * poses{end};
            l = cal_E_from_T(T_rel)' * kf2_pts_l_norm(:,idx_kf2);
            d = sum(l .* kf1_pts_r_norm(:,idx_kf1)) ./ sqrt(l(1,:).^2 + l(2,:).^2);
            g = [g; truncate_loss(d(:), delta)];
            
            % KF2右目到KF1左目
            T_rel = inv(T0) * inv(poses{end});
            % T_rel = poses{end}* T0;
            l = cal_E_from_T(T_rel)' * kf2_pts_r_norm(:,idx_kf2);
            d = sum(l .* kf1_pts_l_norm(:,idx_kf1)) ./ sqrt(l(1,:).^2 + l(2,:).^2);
            g = [g; truncate_loss(d(:), delta)];
            
            % KF2右目到KF1右目
            T_rel = inv(T0) * inv(poses{end}) * T0;
            % T_rel = inv(T0) * (poses{end}) * T0;
            l = cal_E_from_T(T_rel)' * kf2_pts_r_norm(:,idx_kf2);
            d = sum(l .* kf1_pts_r_norm(:,idx_kf1)) ./ sqrt(l(1,:).^2 + l(2,:).^2);
            g = [g; truncate_loss(d(:), delta)];
        end
        
        % 2. 普通帧之间的约束
        for i = 1:length(frame_feature_arrays)-1
            pts_i = frame_feature_arrays{i}.image_pts';
            ids_i = frame_feature_arrays{i}.point_ids;
            
            % 归一化第i帧点
            pts_i_norm = [(pts_i(1,:) - cx_l)/fx_l;
                         (pts_i(2,:) - cy_l)/fy_l;
                         ones(1,size(pts_i,2))];
            
            for j = i+1:length(frame_feature_arrays)
                pts_j = frame_feature_arrays{j}.image_pts';
                ids_j = frame_feature_arrays{j}.point_ids;
                
                % 归一化第j帧点
                pts_j_norm = [(pts_j(1,:) - cx_l)/fx_l;
                             (pts_j(2,:) - cy_l)/fy_l;
                             ones(1,size(pts_j,2))];
                
                [~, idx_i, idx_j] = intersect(ids_i, ids_j);
                
                if ~isempty(idx_i)
                    T_rel = inv(poses{j}) * poses{i};
                    % T_rel = inv(poses{i})*(poses{j});
                    l = cal_E_from_T(T_rel)' * pts_j_norm(:,idx_j);
                    d = sum(l .* pts_i_norm(:,idx_i)) ./ sqrt(l(1,:).^2 + l(2,:).^2);
                    g = [g; truncate_loss(d(:), delta)];
                end
            end
        end
        
        % 3. 普通帧与KF1的约束
        for frame_idx = 1:length(frame_feature_arrays)
            frame_pts = frame_feature_arrays{frame_idx}.image_pts';
            frame_ids = frame_feature_arrays{frame_idx}.point_ids;
            
            % 归一化普通帧点
            frame_pts_norm = [(frame_pts(1,:) - cx_l)/fx_l;
                            (frame_pts(2,:) - cy_l)/fy_l;
                            ones(1,size(frame_pts,2))];
            
            [~, idx_kf1, idx_frame] = intersect(kf1_ids, frame_ids);
            
            if ~isempty(idx_kf1)
                % 普通帧左目到KF1左目
                T_rel = inv(poses{frame_idx});
                % T_rel = (poses{frame_idx});
                l = cal_E_from_T(T_rel)' * frame_pts_norm(:,idx_frame);
                d = sum(l .* kf1_pts_l_norm(:,idx_kf1)) ./ sqrt(l(1,:).^2 + l(2,:).^2);
                g = [g; truncate_loss(d(:), delta)];
                
                % 普通帧左目到KF1右目
                T_rel = inv(poses{frame_idx}) * T0;
                % T_rel = inv(T0) * poses{frame_idx};
                l = cal_E_from_T(T_rel)' * frame_pts_norm(:,idx_frame);
                d = sum(l .* kf1_pts_r_norm(:,idx_kf1)) ./ sqrt(l(1,:).^2 + l(2,:).^2);
                g = [g; truncate_loss(d(:), delta)];
            end
        end        
    end
end

function E = cal_E_from_T(T)
    E = skew_symmetric(T(1:3,4)) * T(1:3,1:3);
end

function T = cal_T_from_kexi(kexi)
    T = [expm(skew_symmetric(kexi(1:3))) kexi(4:6); 0,0,0,1];
end

function S = skew_symmetric(s)
    S = [0 -s(3) s(2); s(3) 0 -s(1); -s(2) s(1) 0];
end

function L = huber_loss(e, delta)
    % Huber 损失函数
    % 输入:
    %   e: 误差值 (可以是标量或向量)
    %   delta: 阈值参数 (标量)
    % 输出:
    %   L: Huber 损失值 (与输入 e 的维度相同)

    % 条件判断
    abs_e = abs(e); % 误差的绝对值
    L = zeros(size(e)); % 预分配输出数组

    % 条件1: 对于 |e| <= delta
    mask1 = abs_e <= delta;
    L(mask1) = sqrt(0.5 * e(mask1).^2);

    % 条件2: 对于 |e| > delta
    mask2 = abs_e > delta;
    L(mask2) = sqrt(delta * (abs_e(mask2) - 0.5 * delta));
end

function L = truncate_loss(e, delta)
    % 截断核函数
    % 输入:
    % e: 误差值 (可以是标量或向量)
    % delta: 阈值参数 (标量)
    % 输出:
    % L: 截断损失值 (与输入 e 的维度相同)
    % 计算误差的绝对值
    abs_e = abs(e);
    % 应用截断
    L = min(abs_e, delta);
    % 为了保持数值稳定性，对误差进行归一化
    L = L / delta;
    % L =e;
end