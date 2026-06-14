function poses = read_kitti_pose(filePath)
% 文件路径
% filePath = '00.txt';

% 读取文件内容
posesData = load(filePath);

% 行数，表示帧数
numFrames = size(posesData, 1);

% 初始化一个 3D 矩阵，用于存储每帧的 4x4 变换矩阵
poses = zeros(4, 4, numFrames);

% 遍历每一帧，将每行数据转化为 4x4 矩阵
for i = 1:numFrames
    % 当前行数据，取出 12 个数值
    currentPose = posesData(i, :);
    % 转换成 3x4 矩阵
    T = reshape(currentPose, [4, 3])'; % 注意需要转置
    poses(:, :, i) = [T; 0 0 0 1];
end

% % 打印第1帧的位姿矩阵作为检查
% disp('第一帧的位姿矩阵:');
% disp(poses(:, :, 1));

end