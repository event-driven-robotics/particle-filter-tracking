
%extract relevant information
GT = importdata(gt_file);
GT = [GT(:, 5), GT(:, 2), GT(:, 3)];
CPU = importdata(cpu_file);
CPU = [CPU(:, 7), CPU(:, 4), CPU(:, 5)];
SPIN = importdata(spin_file);
SPIN = [SPIN(:, 7), SPIN(:, 4), SPIN(:, 5)];

%clean timestamps
start_time = max([GT(1, 1) CPU(1, 1) SPIN(1, 1)]);
GT(:, 1) = GT(:, 1) - start_time;
CPU(:, 1) = CPU(:, 1) - start_time;
SPIN(:, 1) = SPIN(:, 1) - start_time;
end_time = min([GT(end, 1) CPU(end, 1) SPIN(end, 1)]);

%remove ts synch errors
SPIN = SPIN([true; diff(SPIN(:, 1))>0], :);

r_ts = 0: resolution : end_time;
r_gt = interp1(GT(:, 1), GT(:, 2), r_ts, 'linear');
r_cpu = interp1(CPU(:, 1), CPU(:, 2), r_ts, 'nearest');
r_spin = interp1(SPIN(:, 1), SPIN(:, 2), r_ts, 'nearest');


figure(2); clf; hold on;
plot(r_ts, r_gt, '-y', 'linewidth', 5);
plot(r_ts, r_cpu, '--b');
plot(r_ts, r_spin, '-g');
ylabel('X position (pixels)');
%legend('Ground Truth', 'CPU implementation', 'SpiNNaker Implementation');
xlabel('Time (s)');

%start the process
npts = ceil(0.1 / resolution);
latency_cpu = [];
latency_spin = [];

disp('Running for CPU ... ');

for i = 1 : floor(npts / 2) : length(r_ts) - npts
    
    goal_value = mean(r_gt(i:i+npts));
    
    %find the line of the GT
    mb_gt = polyfit(r_ts(i:i+npts), r_gt(i:i+npts), 1);
    t_gt = (goal_value - mb_gt(2)) / mb_gt(1);
    
    f2_x1 = min(r_ts(i:i+npts)); f2_x2 = max(r_ts(i:i+npts));
    f2_y1 = min(r_gt(i:i+npts)); f2_y2 = max(r_gt(i:i+npts));
    
    %find the line of the CPU
    try
        j = i;
        while(abs(mean(r_cpu(j:j+npts)) - goal_value) > abs(mean(r_cpu(j+1:j+npts+1)) - goal_value))
            j = j + 1;
        end
    catch
        disp(['Breaking due to end of series' num2str(j +npts+1) ' ' num2str(length(r_ts))]);
        break;
    end
    
    mb_cpu = polyfit(r_ts(j:j+npts), r_cpu(j:j+npts), 1);
    t_cpu = (goal_value - mb_cpu(2)) / mb_cpu(1);
    
    f2_x1 = min([f2_x1 r_ts(j:j+npts)]); f2_x2 = max([f2_x2 r_ts(j:j+npts)]);
    f2_y1 = min([f2_y1 r_cpu(j:j+npts)]); f2_y2 = max([f2_y2 r_cpu(j:j+npts)]);
    
    if abs(mb_gt(1)) < 100
        continue;
    end

    if t_cpu > t_gt && sign(mb_gt(1)) == sign(mb_cpu(1)) && abs(mb_cpu(1)) > 100 && t_cpu - t_gt < lat_threshold
        latency_cpu(end+1, :) = [r_ts(i+npts/2), t_cpu - t_gt];
        figure(2);
        axis([f2_x1 f2_x2 f2_y1 f2_y2]);
    end

    
end

disp('Running for SpiNNaker ... ');

for i = 1 : floor(npts / 2) : length(r_ts) - npts
        
    goal_value = mean(r_gt(i:i+npts));
    
    %find the line of the GT  
    mb_gt = polyfit(r_ts(i:i+npts), r_gt(i:i+npts), 1);
    t_gt = (goal_value - mb_gt(2)) / mb_gt(1);
    
    f2_x1 = min(r_ts(i:i+npts)); f2_x2 = max(r_ts(i:i+npts));
    f2_y1 = min(r_gt(i:i+npts)); f2_y2 = max(r_gt(i:i+npts));
    
    %find the line of the SPIN
    try
        j = i;
        while(abs(mean(r_spin(j:j+npts)) - goal_value) > abs(mean(r_spin(j+1:j+npts+1)) - goal_value))
            j = j + 1;
        end
    catch
        disp(['Breaking due to end of series' num2str(j +npts+1) ' ' num2str(length(r_ts))]);
        break
    end
    mb_spin = polyfit(r_ts(j:j+npts), r_spin(j:j+npts), 1);
    t_spin = (goal_value - mb_spin(2)) / mb_spin(1);
    
    f2_x1 = min([f2_x1 r_ts(j:j+npts)]); f2_x2 = max([f2_x2 r_ts(j:j+npts)]);
    f2_y1 = min([f2_y1 r_spin(j:j+npts)]); f2_y2 = max([f2_y2 r_spin(j:j+npts)]);

    if abs(mb_gt(1)) < 100
        continue;
    end

    if t_spin > t_gt && sign(mb_gt(1)) == sign(mb_spin(1)) && abs(mb_spin(1)) > 100 && t_spin - t_gt < lat_threshold
        latency_spin(end+1, :) = [r_ts(i+npts/2), t_spin - t_gt];
        figure(2);
        axis([f2_x1 f2_x2 f2_y1 f2_y2]);
    end    
    
end

try; close(1); end

dlmwrite(latency_cpu_file, latency_cpu, 'delimiter', ' ', 'precision', 4);
dlmwrite(latency_spin_file, latency_spin, 'delimiter', ' ', 'precision', 4);

disp('done');






