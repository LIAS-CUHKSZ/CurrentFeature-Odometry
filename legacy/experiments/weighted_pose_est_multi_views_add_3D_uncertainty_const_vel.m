% First triangulate 3D points from binocular cameras and estimate their
% uncertainties, 
% and then solve a weighted PnP problem, finally apply a one-step weighted pnp GN iteration.
function [var_est,R_est,t_est,R_GN,t_GN] = weighted_pose_est_multi_views_add_3D_uncertainty_const_vel(multi_kf,R1,t1,poses_est) 

% multi_kf为当前帧和用到的关键帧的信息（特征点和全局id）
% poses_est为用到的关键帧的（全局）位姿

var_est=1;
z_h=[];
pt_3d=[];
used_id=[];
use_latest_KF=1;  % 是否用最新的关键帧信息
for i=1:length(multi_kf)-1-(1-use_latest_KF)
    [x3_h_wid,x2_h_wid,x1_h_wid] = pt_mathing_optimized(multi_kf{i}, multi_kf{end});  % 当前帧和第i个关键帧匹配
    % 如果某些点在之前关键帧已经三角化了，则不在该关键帧三角化
    [~, ith_kf_used_id, ~] = intersect(x3_h_wid(4,:), used_id);
    x3_h_wid(:,ith_kf_used_id)=[];
    x2_h_wid(:,ith_kf_used_id)=[];
    x1_h_wid(:,ith_kf_used_id)=[];
    if ~isempty(x3_h_wid)
        z_h=[z_h x3_h_wid(1:2,:)];
        T_rel=inv(poses_est{length(multi_kf)-1})*poses_est{i};
        R_rel=T_rel(1:3,1:3);
        t_rel=T_rel(1:3,4);
        for j=1:size(x3_h_wid,2)
            [temp_pt,~]=uncertain_aware_triangulation2_used(x2_h_wid(1:3,j),x1_h_wid(1:3,j),R1',-R1'*t1,var_est);
            pt_3d=[pt_3d R_rel*temp_pt+t_rel];
        end
    end
    used_id=[used_id x3_h_wid(4,:)];
end



m=size(pt_3d,2); % number of point
m_multi=m
use_pca = 0;
if m<30
    use_pca = 1;
    var_est = 1;
% else
%     %% noise variance estimation
%     bar_A=zeros(m,9);
%     bar_Y=zeros(3,3);
%     H=[1 0 0;0 1 0];
%     for i=1:m
%         bar_A(i,:)=kron(z_h(:,i)',y_h(:,i)');
%         bar_Y=bar_Y+z_h(:,i)*z_h(:,i)'/m;
%     end
%     Q_m=bar_A'*bar_A/m;
%     S_m=kron(bar_Y,[H;0 0 0]);
%     var_est=1/max(eig(Q_m\S_m));
end
% 
% 
% %% 3D point triangulation and uncertainty estimation
% % obtain coordinates in the y camera frame
% pt_3d=zeros(3,m);
% cov_P=zeros(3,3*m);
% for i=1:m
%     [pt_3d(:,i),cov_P(:,3*i-2:3*i)]=uncertain_aware_triangulation2_used(y_h(:,i),x_h(:,i),R1',-R1'*t1,var_est);
% end

% pt_3d=zeros(3,m);
% cov_P=zeros(3,3*m);
% for i=1:m
%     [pt_3d(:,i),cov_P(:,3*i-2:3*i)]=uncertain_aware_triangulation2_used(y_h(:,i),x_h(:,i),R1',-R1'*t1,var_est);
% end

% pt_3d2=zeros(3,size(z_h2,2));
% for i=1:size(z_h2,2)
%     [pt_3d2(:,i),~]=uncertain_aware_triangulation2_used(y_h2(:,i),x_h2(:,i),R1',-R1'*t1,var_est);
% end
% 
% 
% pt_3d=[pt_3d R2'*pt_3d2-R2'*t2];
% z_h=[z_h z_h2];


z_depth = pt_3d(3, :);
w = 1 ./ (z_depth.^(1.5)); % 1xN 鐨勭煩闃碉紝w 鏄? z 杞存繁搴︾殑鍊掓暟

% 鎵惧埌鐭╅樀 P 涓殑 Inf 鎴? -Inf 鍊?
inf_indices = isinf(pt_3d);

% 鎵撳嵃鍑哄寘鍚? Inf 鎴? -Inf 鐨勫厓绱犱綅缃?
[row, col] = find(inf_indices);
if ~isempty(row)  % 褰撴壘鍒癐nf鍏冪礌鏃舵墠鎵撳嵃
    disp('Inf 鎴? -Inf 鐨勪綅缃細');
    disp([row, col]);
end


% %% EIV pnp pose estimation
% if use_pca==1
%     [R_est,t_est,~,~]=PnP_PCA(pt_3d,z_h(1:2,:));
% else
% %     [R_est,t_est] = EIV_PnP2(pt_3d,cov_P,z_h(1:2,:),w);
%     [R_est,t_est,~] = L1_norm_PnP(pt_3d,z_h(1:2,:));
% %     removal_ratio=0.06;    % important parameter to be refined!!!
% %     removal_num=ceil(removal_ratio*m);
% %     remain_num=m-removal_num;
% %     pt_3d=pt_3d(:,index(1:remain_num));
% %     z_h=z_h(:,index(1:remain_num));
% %     w=w(index(1:remain_num));
% %     m=size(z_h,2);
% %     cov_PP=zeros(3,3*m);
% %     for i=1:m
% %         cov_PP(:,3*i-2:3*i)=cov_P(:,3*index(i)-2:3*index(i));
% %     end
% %     cov_P=cov_PP;
% %         [R_est,t_est] = EIV_PnP2(pt_3d,cov_P,z_h(1:2,:),w);
% 
% end

% 恒速模型位姿预测
T_est=inv(poses_est{end})*poses_est{end-1};

R_est=T_est(1:3,1:3);
t_est=T_est(1:3,4);



%% weighted pnp iteration

% 优化器配置
    options = optimoptions('lsqnonlin', 'Display', 'off', ...
                          'Algorithm', 'levenberg-marquardt');

    % 构造初始优化变量
    init_kexi = [anti_skew_symmetric(logm(R_est));t_est];

    init_cost=myfun(init_kexi);

    % 优化
    opt_kexi = lsqnonlin(@myfun, init_kexi, [], [], options);
    opt_T=cal_T_from_kexi(opt_kexi);
    R_GN=opt_T(1:3,1:3);
    t_GN=opt_T(1:3,4);

    opt_cost=myfun(opt_kexi);



    function g = myfun(kexi)
        T=cal_T_from_kexi(kexi);
%         delta=0.5*10^(-4);
        delta=2*10^(-2);
        X=pt_3d;
        x=z_h(1:2,:);
        E=[1 0 0;0 1 0];
        e3 = [0;0;1];
        g=E* (T(1:3,1:3)*X+T(1:3,4));
        h=e3'* (T(1:3,1:3)*X+T(1:3,4));
        f=g./h;
        ww=kron(w',[1;1]);
%         d=ww.*(x(:) - f(:));
        d=x(:) - f(:);
        g=truncate_loss(d,delta);
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

function [x3_h_wid,x2_h_wid,x1_h_wid] = pt_mathing_optimized(kf1, kf2)
    [~, kf1_id_l, kf1_id_r] = intersect(kf1.id_l, kf1.id_r);
    
    pt_l = kf1.pt_l(:,kf1_id_l);
    id_l = kf1.id_l(:,kf1_id_l);
    pt_r = kf1.pt_r(:,kf1_id_r);
    
    [~, kf1_id, kf2_id] = intersect(id_l, kf2.id_l);
    
    x3_h = [kf2.pt_l(:,kf2_id); ones(1,length(kf2_id))];
    x2_h = [pt_l(:,kf1_id); ones(1,length(kf1_id))];
    x1_h = [pt_r(:,kf1_id); ones(1,length(kf1_id))];

    % 匹配到的2d点的3d点编号
    x3_h_wid = [x3_h;kf2.id_l(:,kf2_id)];
    x2_h_wid = [x2_h;kf2.id_l(:,kf2_id)];
    x1_h_wid = [x1_h;kf2.id_l(:,kf2_id)];
end
