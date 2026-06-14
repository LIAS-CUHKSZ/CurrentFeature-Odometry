function repo_root = init_currentfeature_paths()
%INIT_CURRENTFEATURE_PATHS Add project MATLAB paths.
%   Run this once from the repository root before executing scripts.

repo_root = fileparts(mfilename('fullpath'));

addpath(genpath(fullfile(repo_root, 'src')));
addpath(fullfile(repo_root, 'scripts'));

fprintf('CurrentFeature Odometry paths initialized at:\n%s\n', repo_root);
end
