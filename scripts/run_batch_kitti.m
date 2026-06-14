function batch_results = run_batch_kitti(use_gray, seqs)
%RUN_BATCH_KITTI Run the KITTI runner for multiple sequences.

if nargin < 1 || isempty(use_gray)
    use_gray = true;
end
if nargin < 2 || isempty(seqs)
    seqs = [0 2:10];
end

repo_root = fileparts(fileparts(mfilename('fullpath')));
batch_results = struct();

for i = 1:numel(seqs)
    seq = seqs(i);
    seq_str = sprintf('%02d', seq);
    gray_suffix = '';
    if use_gray
        gray_suffix = '_gray';
    end

    data_dir = fullfile(repo_root, 'data', 'ov2slam_data', ...
        sprintf('ov2slam_data_kitti_%s%s', seq_str, gray_suffix));
    if ~isfolder(data_dir)
        warning('Skipping seq%s%s because data folder is missing.', seq_str, gray_suffix);
        continue;
    end

    fprintf('\n\n===== Running seq%s%s =====\n', seq_str, gray_suffix);
    result = run_kitti_currentfeature(seq, use_gray);
    batch_results(i).seq = seq; %#ok<AGROW>
    batch_results(i).use_gray = use_gray; %#ok<AGROW>
    batch_results(i).metrics = result.metrics; %#ok<AGROW>
end
end
