function poses = read_euroc_pose(filePath)
    % 读取数据文件
    fid = fopen(filePath, 'r');
    if fid == -1
        error('Cannot open file: %s', filePath);
    end
    
    % 读取所有行
    data = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    lines = data{1};
    
    % 跳过第一行表头，从第二行开始处理
    lines = lines(2:end);
    num_poses = length(lines);
    
    % 初始化结构体数组
    poses = struct('timestamp', [], ...
                  'position', [], ...
                  'quaternion', [], ...
                  'velocity', [], ...
                  'angular_velocity_bias', [], ...
                  'acceleration_bias', [], ...
                  'T', []);
    poses = repmat(poses, num_poses, 1);
    
    % 解析每一行数据
    for i = 1:num_poses
        % 使用逗号分割数据
        parts = strsplit(lines{i}, ',');
        
        % 转换为数值
        values = str2double(parts);
        
        % 填充基本字段
        poses(i).timestamp = values(1);
        poses(i).position = values(2:4);
        poses(i).quaternion = values(5:8);
        poses(i).velocity = values(9:11);
        poses(i).angular_velocity_bias = values(12:14);
        poses(i).acceleration_bias = values(15:17);
        
        % 计算变换矩阵 T
        poses(i).T = quaternionToTransform(poses(i).quaternion, poses(i).position);
    end
end

function T = quaternionToTransform(q, p)
    % 输入：
    % q: 四元数 [w, x, y, z]
    % p: 位置向量 [x, y, z]
    
    % 从四元数计算旋转矩阵
    w = q(1); x = q(2); y = q(3); z = q(4);
    
    R = [1-2*y^2-2*z^2,  2*x*y-2*w*z,    2*x*z+2*w*y;
         2*x*y+2*w*z,    1-2*x^2-2*z^2,  2*y*z-2*w*x;
         2*x*z-2*w*y,    2*y*z+2*w*x,    1-2*x^2-2*y^2];
    
    % 构建4x4变换矩阵
    T = eye(4);
    T(1:3, 1:3) = R;
    T(1:3, 4) = p(:);
end