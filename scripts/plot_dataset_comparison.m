
figure(1); clf; hold on;
ts_start = 0; ts_end = 20;
dataset_dirs = {'C:\Users\AGlover\Documents\workspace\dump\trackingcomparison3\20_512', ...
'C:\Users\AGlover\Documents\workspace\dump\trackingcomparison2\difficult1'};
linestyles = {'--', '-'};
linewidths = {2, 1};

for ic = 1:size(dataset_dirs, 2)
    
    gt_file = [dataset_dirs{ic} '\ATIS\data.log.GT'];
    raw_file = [dataset_dirs{ic} '\ATIS\data.log.txt'];

    disp('Importing groud truth ... ');
    GT = importdata(gt_file);
    GT = [GT(:, 5), GT(:, 2), GT(:, 3)];
    start_time = GT(1, 1);
    GT(:, 1) = GT(:, 1) - start_time;
    fix_GT;

    ts_inc = GT(:, 1) > ts_start & GT(:, 1) < ts_end;

    
    subplot(2, 1, 1); hold on;
    dx = diff(GT(ts_inc, 2));
    dy = diff(GT(ts_inc, 3));
    dt = diff(GT(ts_inc, 1));
    temp_timevalues = GT(ts_inc, 1);
    temp_timevalues = temp_timevalues(1:end-1); %./ diff(GT(ts_inc, 1))
    plot(temp_timevalues, sqrt(dx.^2 + dy.^2)./dt, 'color', c(3+ic, :), ...
        'linestyle', linestyles{ic}, 'linewidth', linewidths{ic});
    drawnow;
    
    subplot(2, 1, 2); hold on;
    disp('Importing events ... ');
    EVENTS = importdata(raw_file);
    stamps = EVENTS(:, 7) - start_time;
    stamps = stamps(stamps > ts_start & stamps < ts_end);
    batch_size = floor(size(stamps, 1) / 1000);
    event_rate_vector = [];
    event_rate_times = [];
    for i = 1:1000
        time1 = stamps((i-1)*batch_size+1);
        time2 = stamps(i*batch_size);
        event_rate_vector(end+1) = batch_size / (time2 - time1);
        event_rate_times(end+1) = time2;
    end
    plot(event_rate_times, event_rate_vector/1e6, 'color', c(3+ic, :), ...
        'linestyle', linestyles{ic}, 'linewidth', linewidths{ic});
    drawnow;
    
end

%name the axes
subplot(2, 1, 2);
xlabel('Time (s)');
ylabel('Event-rate (1e6 ev./s)');
box on;
subplot(2, 1, 1);
ylabel('Target Speed (pix./s)');
box on;

%set to correct font
set(findall(gcf,'-property','FontSize'),'FontSize',16);
set(findall(gcf,'-property','FontName'),'FontName','Times');

legend('Trial n_p=20, n_v=512', 'Unconstrained Motion', 'location', 'northwest');
set(legend, 'fontsize', 12);

%move the y label over
subplot(2, 1, 2);
sp1_labl_pos = get(get(gca, 'ylabel'), 'position');
subplot(2, 1, 1);
sp2_labl_pos = get(get(gca, 'ylabel'), 'position');
sp2_labl_pos(1) = sp1_labl_pos(1);
set(get(gca, 'ylabel'), 'position', sp2_labl_pos);

set(1,'Units','Inches');
set(1, 'position', [2.4028 7.2708 8.7569 5.1042]);

disp('Done');


        

