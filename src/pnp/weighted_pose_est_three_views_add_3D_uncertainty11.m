% First triangulate 3D points from binocular cameras and estimate their
% uncertainties, 
% and then solve a weighted PnP problem, finally apply a one-step weighted pnp GN iteration.
function [var_est,R_est,t_est,R_GN,t_GN] = weighted_pose_est_three_views_add_3D_uncertainty11(z_h,y_h,x_h,...
    R1,t1,pnp_tls_delta,frame_idx, current_kf_idx)  %,eiv_pt_3d,evi_z_h,evi_y_h,evi_x_h
m=size(y_h,2); % number of point

use_pca = 0;
if size(z_h,2)<40
    use_pca = 1;
    var_est = 1;
end


%% 3D point triangulation and uncertainty estimation
% obtain coordinates in the y camera frame
pt_3d=zeros(3,m);
cov_P=zeros(3,3*m);
for i=1:m
    [pt_3d(:,i),cov_P(:,3*i-2:3*i)]=uncertain_aware_triangulation2_used(y_h(:,i),x_h(:,i),R1',-R1'*t1,1);
end
z_depth = pt_3d(3, :);
w = 1 ./ (z_depth.^(1.5)); % 1xN 鐨勭煩闃碉紝w 鏄? z 杞存繁搴︾殑鍊掓暟
% w = ones(length(z_depth));


pt_3d_temp = pt_3d;
z_h_temp = z_h;
%% EIV pnp pose estimation
if use_pca==1
    [R_est,t_est,R_GN,t_GN]=PnP_PCA(pt_3d,z_h(1:2,:));
    % fprintf('Use PCA method at frame_idx= %d ,current_kf_idx=%d',frame_idx, current_kf_idx);
    return;
else

    [~,~,index] = L1_norm_PnP(pt_3d_temp,z_h_temp(1:2,:));



    % 计算每帧需要保留的点数
    removal_ratio1 = 0.1;
    
    removal_num1 = ceil(removal_ratio1 * m);
    
    remain_num1 = m - removal_num1;
    
    
    % 将index分成两部分
    % 找出index中属于第一帧的索引
    index1 = index(index <= m);
    
    
    % 分别提取两帧中误差小的点
    % 第一帧
    z_h = z_h(:,index1(1:remain_num1));
    y_h = y_h(:,index1(1:remain_num1));
    x_h = x_h(:,index1(1:remain_num1));
    w = w(:,index1(1:remain_num1));

    m=size(z_h,2);



    bar_A=zeros(m,9);
    bar_Y=zeros(3,3);
    H=[1 0 0;0 1 0];
    for i=1:m
        bar_A(i,:)=kron(z_h(:,i)',y_h(:,i)');
        bar_Y=bar_Y+z_h(:,i)*z_h(:,i)'/m;
    end
    Q_m=bar_A'*bar_A/m;
    S_m=kron(bar_Y,[H;0 0 0]);
    var_est=1/max(eig(Q_m\S_m));
    pt_3d=zeros(3,m);
    cov_P=zeros(3,3*m);
    for i=1:m
        [pt_3d(:,i),cov_P(:,3*i-2:3*i)]=uncertain_aware_triangulation2_used(y_h(:,i),x_h(:,i),R1',-R1'*t1,var_est);
    end
    evi_z_h = z_h;
    evi_y_h = y_h;
    evi_x_h = x_h;
    eiv_pt_3d = pt_3d;
    
    [R_est,t_est] = EIV_PnP2(pt_3d,cov_P,z_h(1:2,:),w);

    H=[1 0 0;0 1 0];
    % 权重生成
    cov_PP=zeros(2,2*m);
    e3=[0 0 1];
    for i=1:(m)
        J=(e3*(R_est*pt_3d(:,i)+t_est)*H*R_est-H*(R_est*pt_3d(:,i)+t_est)*e3*R_est)/(e3*(R_est*pt_3d(:,i)+t_est))^2;
        cov_PP(:,2*i-1:2*i)=J*cov_P(:,3*i-2:3*i)*J';
    end
    cov_PP_inv=[];
    for k = 1:(m)  
        cov_PP_inv=blkdiag(cov_PP_inv,cov_PP(:,2*k-1:2*k)^(-0.5));
    end
end




    %% weighted pnp iteration

% 优化器配置
    options = optimoptions('lsqnonlin', 'Display', 'off', ...
                          'Algorithm', 'levenberg-marquardt');

    % 构造初始优化变量
    init_kexi = [anti_skew_symmetric(logm(R_est));t_est];
    init_cost=myfun2(init_kexi);
    % 优化
    opt_kexi = lsqnonlin(@myfun2, init_kexi, [], [], options);
    opt_T=cal_T_from_kexi(opt_kexi);
    R_GN=opt_T(1:3,1:3);
    t_GN=opt_T(1:3,4);
    opt_cost=myfun2(opt_kexi);

    function g = myfun2(kexi)
        T=cal_T_from_kexi(kexi);
        % delta2 = 1*10^(3);
        % delta2 = 1*10^(-1);
        X=pt_3d;
        x=z_h(1:2,:);
        E=[1 0 0;0 1 0];
        e3 = [0;0;1];
        g=E* (T(1:3,1:3)*X+T(1:3,4));
        h=e3'* (T(1:3,1:3)*X+T(1:3,4));
        f=g./h;
        d=cov_PP_inv*(x(:) - f(:));
        % d=(x(:) - f(:));
        g=truncate_loss(d,pnp_tls_delta);
    end
end

function T = cal_T_from_kexi(kexi)
    T = [expm(skew_symmetric(kexi(1:3))) kexi(4:6); 0,0,0,1];
end

function L = truncate_loss(e, delta)
    % 截断核函数
    % 输入:
    %   e: 误差值 (可以是标量或向量)
    %   delta: 阈值参数 (标量)
    % 输出:
    %   L: 截断损失值 (与输入 e 的维度相同)

    % 计算误差的绝对值
    abs_e = abs(e);
    
    % 应用截断
    L = min(abs_e, delta);
    
    % 为了保持数值稳定性，对误差进行归一化
    L = L / delta;
end


function world_points = triangulate_stereo(left_points, right_points, R, t)
    % Input:
    % left_points: 3xN normalized coordinates in left camera
    % right_points: 3xN normalized coordinates in right camera 
    % R: 3x3 rotation matrix from right to left camera
    % t: 3x1 translation vector from right to left camera
    %
    % Output:
    % world_points: 3xN 3D points in left camera coordinate system
    
    % Get number of points
    N = size(left_points, 2);
    
    % Initialize 3D points
    world_points = zeros(3, N);
    
    % Triangulate each point
    for i = 1:N
        % Get normalized coordinates
        x1 = left_points(:,i);
        x2 = right_points(:,i);
        
        % Construct coefficient matrix A
        A = zeros(4,3);
        A(1,:) = [x1(3) 0 -x1(1)];
        A(2,:) = [0 x1(3) -x1(2)];
        A(3,:) = (x2(3)*R(1,:) - x2(1)*R(3,:));
        A(4,:) = (x2(3)*R(2,:) - x2(2)*R(3,:));
        
        b = zeros(4,1);
        b(3:4) = x2([1,2])*t(3) - x2(3)*t([1,2]);
        
        % Solve linear system
        X = A\b;
        
        % Store 3D point
        world_points(:,i) = X;
    end
end
