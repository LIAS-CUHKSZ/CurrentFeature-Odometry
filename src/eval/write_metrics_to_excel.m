function write_metrics_to_excel(poses_gt, poses_ov2, poses_ours, filename, seq_num)
    % ATE without alignment
    [~, ~, ov2_ate_m_wo, ov2_ate_deg_wo] = calculate_ate(poses_gt, poses_ov2, 0);
    [~, ~, our_ate_m_wo, our_ate_deg_wo] = calculate_ate(poses_gt, poses_ours, 0);
    
    % ATE with alignment
    [~, ~, ov2_ate_m_w, ov2_ate_deg_w] = calculate_ate(poses_gt, poses_ov2, 1);
    [~, ~, our_ate_m_w, our_ate_deg_w] = calculate_ate(poses_gt, poses_ours, 1);

    % RPE
    [~, ~, ov2_rpe_trans, ov2_rpe_rot] = calculate_rpe(poses_gt, poses_ov2, 1);
    [~, ~, our_rpe_trans, our_rpe_rot] = calculate_rpe(poses_gt, poses_ours, 1);
    ov2_rpe_rot = mean(ov2_rpe_rot);
    our_rpe_rot = mean(our_rpe_rot);

    try
        if ~isfile(filename)
            % ATE/M 表头
            ate_m_headers1 = {'ATE/M', 'w Align', '', '', 'wo Align', '', ''};
            ate_m_headers2 = {'KITTI', 'ORBSLAM3', 'OV2SLAM', 'Ours', 'ORBSLAM3', 'OV2SLAM', 'Ours'};
            
            % ATE/DEG 表头
            ate_deg_headers1 = {'ATE/DEG', 'w Align', '', '', 'wo Align', '', ''};
            ate_deg_headers2 = {'KITTI', 'ORBSLAM3', 'OV2SLAM', 'Ours', 'ORBSLAM3', 'OV2SLAM', 'Ours'};
            
            % RPE 表头
            rpe_headers1 = {'RPE/M', '', '', '', 'RPE/DEG', '', ''};
            rpe_headers2 = {'KITTI', 'ORBSLAM3', 'OV2SLAM', 'Ours', 'ORBSLAM3', 'OV2SLAM', 'Ours'};
            
            % 写入表头
            writecell(ate_m_headers1, filename, 'Sheet', 1, 'Range', 'A1');
            writecell(ate_m_headers2, filename, 'Sheet', 1, 'Range', 'A2');
            writecell(ate_deg_headers1, filename, 'Sheet', 1, 'Range', 'I1');
            writecell(ate_deg_headers2, filename, 'Sheet', 1, 'Range', 'I2');
            writecell(rpe_headers1, filename, 'Sheet', 1, 'Range', 'Q1');
            writecell(rpe_headers2, filename, 'Sheet', 1, 'Range', 'Q2');
        end

        % 准备当前行数据
        row_num = num2str(seq_num + 3);
        
        % 准备ATE/M数据行
        ate_m_row = {sprintf('seq%02d', seq_num), ...
                    '', ov2_ate_m_w, our_ate_m_w, ...    % w align
                    '', ov2_ate_m_wo, our_ate_m_wo};     % wo align
        
        % 准备ATE/DEG数据行
        ate_deg_row = {sprintf('seq%02d', seq_num), ...
                      '', ov2_ate_deg_w, our_ate_deg_w, ...    % w align
                      '', ov2_ate_deg_wo, our_ate_deg_wo};     % wo align
        
        % 准备RPE数据行
        rpe_row = {sprintf('seq%02d', seq_num), ...
                  '', ov2_rpe_trans, our_rpe_trans, ...    % RPE/M
                  '', ov2_rpe_rot, our_rpe_rot};           % RPE/DEG
        
        % 写入数据
        writecell(ate_m_row, filename, 'Sheet', 1, 'Range', ['A' row_num]);
        writecell(ate_deg_row, filename, 'Sheet', 1, 'Range', ['I' row_num]);
        writecell(rpe_row, filename, 'Sheet', 1, 'Range', ['Q' row_num]);
        
        % 输出所有结果用于验证
        fprintf('\nSequence %02d Results:\n', seq_num);
        fprintf('OV2 ATE RMSE (Translation) with Align: %.4f meters\n', ov2_ate_m_w);
        fprintf('OV2 ATE RMSE (Rotation) with Align: %.4f degrees\n', ov2_ate_deg_w);
        fprintf('OV2 ATE RMSE (Translation) without Align: %.4f meters\n', ov2_ate_m_wo);
        fprintf('OV2 ATE RMSE (Rotation) without Align: %.4f degrees\n', ov2_ate_deg_wo);
        fprintf('OV2 RPE RMSE (Translation): %.4f meters\n', ov2_rpe_trans);
        fprintf('OV2 RPE RMSE (Rotation): %.4f degrees\n', ov2_rpe_rot);
        
        fprintf('\nOurs ATE RMSE (Translation) with Align: %.4f meters\n', our_ate_m_w);
        fprintf('Ours ATE RMSE (Rotation) with Align: %.4f degrees\n', our_ate_deg_w);
        fprintf('Ours ATE RMSE (Translation) without Align: %.4f meters\n', our_ate_m_wo);
        fprintf('Ours ATE RMSE (Rotation) without Align: %.4f degrees\n', our_ate_deg_wo);
        fprintf('Ours RPE RMSE (Translation): %.4f meters\n', our_rpe_trans);
        fprintf('Ours RPE RMSE (Rotation): %.4f degrees\n', our_rpe_rot);
        
    catch ME
        fprintf('Error writing to Excel file: %s\n', ME.message);
        rethrow(ME);
    end
end