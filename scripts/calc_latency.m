
if(isfile(latency_file))
    disp('Output file already exists. Please choose another location');
    return;
end

resolution = 0.01;

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
r_cpu = interp1(CPU(:, 1), CPU(:, 2), r_ts, 'PCHIP');
r_spin = interp1(SPIN(:, 1), SPIN(:, 2), r_ts, 'PCHIP');


figure(2); clf; hold on;
plot(r_ts, r_gt, '-y', 'linewidth', 5);
plot(r_ts, r_cpu, '--b');
plot(r_ts, r_spin, '-g');
ylabel('X position (pixels)');
legend('Ground Truth', 'CPU implementation', 'SpiNNaker Implementation');
xlabel('Time (s)');

%start the process
npts = ceil(0.1 / resolution);
latency = [];

for i = 1 : floor(npts / 2) : length(r_ts) - npts
    
    figure(1); clf; hold on;
    
    goal_value = mean(r_gt(i:i+npts));
    
    %find the line of the GT
    plot(r_ts(i:i+npts), r_gt(i:i+npts), 'xm', 'linewidth', 5);    
    mb_gt = polyfit(r_ts(i:i+npts), r_gt(i:i+npts), 1);
    leg1 = plot(r_ts(i:i+npts), polyval(mb_gt, r_ts(i:i+npts)), '-m');
    t_gt = (goal_value - mb_gt(2)) / mb_gt(1);
    
    f2_x1 = min(r_ts(i:i+npts)); f2_x2 = max(r_ts(i:i+npts));
    f2_y1 = min(r_gt(i:i+npts)); f2_y2 = max(r_gt(i:i+npts));
    
    %find the line of the CPU
    try
        j = i;
%         while(abs(mean(r_cpu(j:j+npts)) - goal_value) > 2)
%             j = j+1;
%         end
        while(abs(mean(r_cpu(j:j+npts)) - goal_value) > abs(mean(r_cpu(j+1:j+npts+1)) - goal_value))
            j = j + 1;
        end
    catch
        disp(['Breaking due to end of series' num2str(j +npts+1) ' ' num2str(length(r_ts))]);
        break;
    end
    
    plot(r_ts(j:j+npts), r_cpu(j:j+npts), 'xb', 'linewidth', 5);
    mb_cpu = polyfit(r_ts(j:j+npts), r_cpu(j:j+npts), 1);
    leg2 =plot(r_ts(j:j+npts), polyval(mb_cpu, r_ts(j:j+npts)), '-b');
    t_cpu = (goal_value - mb_cpu(2)) / mb_cpu(1);
    
    f2_x1 = min([f2_x1 r_ts(j:j+npts)]); f2_x2 = max([f2_x2 r_ts(j:j+npts)]);
    f2_y1 = min([f2_y1 r_cpu(j:j+npts)]); f2_y2 = max([f2_y2 r_cpu(j:j+npts)]);
    
    %find the line of the SPIN
    try
        j = i;
%         while(abs(mean(r_spin(j:j+npts)) - goal_value) > 1)
%             j = j+1;
%         end
        while(abs(mean(r_spin(j:j+npts)) - goal_value) > abs(mean(r_spin(j+1:j+npts+1)) - goal_value))
            j = j + 1;
        end
    catch
        disp(['Breaking due to end of series' num2str(j +npts+1) ' ' num2str(length(r_ts))]);
        break
    end
    plot(r_ts(j:j+npts), r_spin(j:j+npts), 'xg', 'linewidth', 5);
    mb_spin = polyfit(r_ts(j:j+npts), r_spin(j:j+npts), 1);
    leg3 =plot(r_ts(j:j+npts), polyval(mb_spin, r_ts(j:j+npts)), '-g');
    t_spin = (goal_value - mb_spin(2)) / mb_spin(1);
    
    f2_x1 = min([f2_x1 r_ts(j:j+npts)]); f2_x2 = max([f2_x2 r_ts(j:j+npts)]);
    f2_y1 = min([f2_y1 r_spin(j:j+npts)]); f2_y2 = max([f2_y2 r_spin(j:j+npts)]);
    
    %plot the final positions for comparison
    plot(t_gt, goal_value, 'ok', 'markersize', 10);
    plot(t_cpu, goal_value, 'xb', 'markersize', 10);
    plot(t_spin, goal_value, 'xg', 'markersize', 10);
    legend([leg1 leg2 leg3], 'GT', 'CPU', 'SpiNNaker', 'location', 'northeastoutside');
    
    if(t_cpu < t_gt || t_spin < t_gt)
        continue;
    end
    
    if(sign(mb_gt(1)) ~= sign(mb_cpu(1)) || sign(mb_gt(1)) ~= sign(mb_spin(1)))
        continue;
    end
    
    figure(2);
    axis([f2_x1 f2_x2 f2_y1 f2_y2]);
    figure(1);
    
    latency(end+1, :) = [r_ts(i+npts/2), t_cpu - t_gt, t_spin - t_gt];
    continue;
    
    try
        c = waitforbuttonpress;
        if c
            c = get(1, 'CurrentCharacter');
            %uint32(get(1, 'currentcharacter'))
        else
            mousep = get(gca, 'currentpoint');
            y = round(mousep(2, 2));
            x = round(mousep(2, 1));
            c = -1;
        end
    catch
        break
    end
    
    if c == 27 %ESC
        close(1);
        break
    elseif c == 32 %space
        continue
    elseif c == 13
        latency(end+1, :) = [r_ts(i+npts/2), t_cpu - t_gt, t_spin - t_gt];
    end
    
    
end

try; close(1); end

dlmwrite(latency_file, latency, 'delimiter', ' ', 'precision', 4);

disp('done (use plot_latency to see the results)');






