

disp('Importing groud truth ... ');
GT = importdata(gt_file);
GT = [GT(:, 5), GT(:, 2), GT(:, 3)];

start_time = GT(1, 1);
GT(:, 1) = GT(:, 1) - start_time;
fix_GT;

figure(1); clf; hold on;
plot(GT(:, 1), GT(:, 2), '-', 'linewidth', 5, 'color', c3);

xlabel('Time (s)');
ylabel('X position (pixels)');

title('Select first point');
drawnow;

c = 1;
while c ~= 0   
    c = waitforbuttonpress;
end

mousep = get(gca, 'currentpoint');
%y = mousep(2, 2);
ts_start = mousep(2, 1);

title('Select second point');
drawnow;
c = 1;
while c ~= 0   
    c = waitforbuttonpress;
end
mousep = get(gca, 'currentpoint');
%y = mousep(2, 2);
ts_end = mousep(2, 1);

ts_inc = GT(:, 1) > ts_start & GT(:, 1) < ts_end;
plot(GT(ts_inc, 1), GT(ts_inc, 2), '-gx');
drawnow;

dx = diff(GT(ts_inc, 2));
dy = diff(GT(ts_inc, 3));
dp = sum(sqrt(dx.^2 + dy.^2));
disp(['Total distance is ' num2str(dp) ' pixels']);
dt = ts_end - ts_start;
disp(['Total time is ' num2str(dt) ' seconds']);
dpdt = dp / dt;
disp(['Average speed is ' num2str(dpdt) ' pixels / second']);

disp('Importing events ... ');
EVENTS = importdata(raw_file);
stamps = EVENTS(:, 7) - start_time;
stamps = stamps(stamps > ts_start & stamps < ts_end);
dvdt = length(stamps) / dt;
disp(['Event-rate is ' num2str(floor(dvdt*0.001)) 'k events / second']);

output = [dp dt length(stamps) dpdt dvdt];

dlmwrite(stats_file, output, 'delimiter', ' ', 'precision', '%10.4f');


