gt_times = GT(1, 1):0.1:GT(end, 1);

gt_x = interp1(GT(:, 1), GT(:, 2), gt_times, 'linear');
gt_y = interp1(GT(:, 1), GT(:, 3), gt_times, 'linear');
GT = [gt_times', gt_x', gt_y'];