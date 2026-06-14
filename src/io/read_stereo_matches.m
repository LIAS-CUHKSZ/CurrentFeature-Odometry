function [stereo_frames] = read_stereo_matches(filename)
    % 打开文件
    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open file: %s', filename);
    end

    % 初始化存储结构
    stereo_frames = struct('frame_id', [], ...
                          'left_pts', [], ...
                          'right_pts', [], ...
                          'point_ids', []);
    frame_idx = 1;

    % 逐帧读取数据
    while ~feof(fid)
        try
            % 读取帧ID
            frame_id = str2double(fgetl(fid));

            % 读取左眼x,y坐标
            left_x = str2num(fgetl(fid));
            left_y = str2num(fgetl(fid));

            % 读取右眼x,y坐标
            right_x = str2num(fgetl(fid));
            right_y = str2num(fgetl(fid));

            % 读取特征点ID
            point_ids = str2num(fgetl(fid));

            % 检查数据完整性
            if isempty(left_x) || isempty(left_y) || ...
               isempty(right_x) || isempty(right_y) || ...
               isempty(point_ids)
                warning('Incomplete data for frame %d', frame_id);
                continue;
            end

            % 检查所有向量长度是否一致
            if length(left_x) ~= length(left_y) || ...
               length(left_x) ~= length(right_x) || ...
               length(left_x) ~= length(right_y) || ...
               length(left_x) ~= length(point_ids)
                warning('Inconsistent data lengths in frame %d', frame_id);
                continue;
            end

            % 组织数据
            stereo_frames(frame_idx).frame_id = frame_id;
            stereo_frames(frame_idx).left_pts = [left_x; left_y]';   % N×2矩阵
            stereo_frames(frame_idx).right_pts = [right_x; right_y]'; % N×2矩阵
            stereo_frames(frame_idx).point_ids = point_ids';

            frame_idx = frame_idx + 1;

        catch ME
            warning('Error reading frame %d: %s', frame_idx, ME.message);
            break;
        end
    end

    fclose(fid);

    % 如果没有读取到任何帧，返回错误
    if frame_idx == 1
        error('No valid frames read from file');
    end
end

