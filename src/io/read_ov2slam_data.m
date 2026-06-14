% function [frames] = read_ov2slam_data(filename)
%     % 打开文件
%     fid = fopen(filename, 'r');
%     if fid == -1
%         error('Cannot open file');
%     end
% 
%     frames = struct('image_pts', [], 'world_pts', [], 'scales', [], 'outliers', []);
%     frame_idx = 1;
% 
%     % 逐行读取
%     while ~feof(fid)
%         % 读取7行数据
%         try
%             % 读取图像坐标
%             line1 = str2num(fgetl(fid));  % x坐标
%             line2 = str2num(fgetl(fid));  % y坐标
% 
%             % 读取世界坐标
%             line3 = str2num(fgetl(fid));  % X坐标
%             line4 = str2num(fgetl(fid));  % Y坐标
%             line5 = str2num(fgetl(fid));  % Z坐标
% 
%             % 读取尺度和外点标记
%             line6 = str2num(fgetl(fid));  % 尺度
%             line7 = str2num(fgetl(fid));  % 外点标记
% 
%             % 检查数据是否完整
%             if isempty(line1) || isempty(line2) || isempty(line3) || ...
%                isempty(line4) || isempty(line5) || isempty(line6) || isempty(line7)
%                 continue;
%             end
% 
%             % 组织数据
%             image_pts = [line1; line2]';   % N×2矩阵
%             world_pts = [line3; line4; line5]';  % N×3矩阵
%             scales = line6';    % N×1向量
%             outliers = line7';  % N×1向量
% 
%             % 存储到结构体
%             frames(frame_idx).image_pts = image_pts;
%             frames(frame_idx).world_pts = world_pts;
%             frames(frame_idx).scales = scales;
%             frames(frame_idx).outliers = outliers;
% 
%             frame_idx = frame_idx + 1;
%         catch
%             warning('Error reading frame %d', frame_idx);
%             break;
%         end
%     end
% 
%     fclose(fid);
% 
%     % 如果没有读取到任何帧，返回错误
%     if frame_idx == 1
%         error('No valid frames read from file');
%     end
% end
function [frames] = read_ov2slam_data(filename)
    % 打开文件
    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open file');
    end
    
    frames = struct('image_pts', [], 'world_pts', [], 'scales', [], 'point_ids', [], 'outliers', []);
    frame_idx = 1;
    
    % 逐行读取
    while ~feof(fid)
        try
            % 读取8行数据
            % 读取图像坐标x和y
            x_coords = str2num(fgetl(fid));
            y_coords = str2num(fgetl(fid));
            
            % 读取世界坐标X、Y、Z
            X_coords = str2num(fgetl(fid));
            Y_coords = str2num(fgetl(fid));
            Z_coords = str2num(fgetl(fid));
            
            % 读取尺度
            scales = str2num(fgetl(fid));
            
            % 读取3D点ID
            point_ids = str2num(fgetl(fid));
            
            % 读取外点标记
            outliers = str2num(fgetl(fid));
            
            % 检查数据是否完整
            if isempty(x_coords) || isempty(y_coords) || ...
               isempty(X_coords) || isempty(Y_coords) || isempty(Z_coords) || ...
               isempty(scales) || isempty(point_ids) || isempty(outliers)
                continue;
            end
            
            % 组织数据
            image_pts = [x_coords; y_coords]';   % N×2矩阵
            world_pts = [X_coords; Y_coords; Z_coords]';  % N×3矩阵
            
            % 存储到结构体
            frames(frame_idx).image_pts = image_pts;
            frames(frame_idx).world_pts = world_pts;
            frames(frame_idx).scales = scales';
            frames(frame_idx).point_ids = point_ids';
            frames(frame_idx).outliers = outliers';
            
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