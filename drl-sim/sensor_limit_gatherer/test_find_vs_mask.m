M = 0 : 1e-7 : 1;

% Mask
tic;

F_mask = M(M > 0.5);

t_mask = toc;

fprintf('For mask: %.3f\n', t_mask);

% Find
tic;

F_find = M(find(M > 0.5));

t_find = toc;

fprintf('For find: %.3f\n', t_find);