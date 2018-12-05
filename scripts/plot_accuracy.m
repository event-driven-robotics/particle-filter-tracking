
nps = {20; 150; 400};
nvs = {256; 512; 1024};

disp('Loading data ... ');

lat_cpu = ones(length(nvs), length(nps));
lat_cpu_std = ones(length(nvs), length(nps));
acc_cpu = ones(length(nvs), length(nps));
acc_cpu_std = ones(length(nvs), length(nps));
lat_spin = ones(length(nvs), length(nps));
lat_spin_std = ones(length(nvs), length(nps));
acc_spin = ones(length(nvs), length(nps));
acc_spin_std = ones(length(nvs), length(nps));

speed = ones(length(nvs), length(nps));
eventrate = ones(length(nvs), length(nps));

for i = 1:length(nvs)
    for j = 1:length(nps)
        
        stats = dlmread([root_dir num2str(nps{j}) '_' num2str(nvs{i}) '\data_stats.txt']);
        result = dlmread([root_dir num2str(nps{j}) '_' num2str(nvs{i}) '\accuracy.txt']);
        
        lat_cpu(i, j) = result(1, 1);
        lat_spin(i, j) = result(1, 3);
        acc_cpu(i, j) = result(2, 1);
        acc_spin(i, j) = result(2, 3);
        
        lat_cpu_std(i, j) = result(1, 2);
        lat_spin_std(i, j) = result(1, 4);
        acc_cpu_std(i, j) = result(2, 2);
        acc_spin_std(i, j) = result(2, 4);
        
        speed(i, j) = stats(4);
        eventrate(i, j) = stats(5)*0.001;
        
        
        
    end
end

disp('Accuracy CPU (pixels)');
disp(acc_cpu);
disp('Accuracy SpiNNaker (pixels)');
disp(acc_spin);
disp(' ');
disp('Latency CPU (ms)');
disp(lat_cpu);
disp('Latency SpiNNaker (ms)');
disp(lat_spin);

disp(' ');
disp('Average Target Speed');
disp(speed);
disp('Average Event Rate');
disp(eventrate);


figure(1); clf; hold on;
acc_table = [acc_cpu', acc_spin'];
acc_std_table = [acc_cpu_std', acc_spin_std'];
hb = bar(acc_table);
drawnow;

for ib = 1:numel(hb)
    %XData property is the tick labels/group centers; XOffset is the offset
    %of each distinct group
    xData = hb(ib).XData+hb(ib).XOffset;
    errorbar(xData,acc_table(:,ib),acc_std_table(:,ib),'k.')
end

y_limits = get(gca, 'ylim');
y_limits(1) = 0;
ylim(y_limits);


legend('CPU[1]-256', 'CPU[1]-512', 'CPU[1]-1024', ...
    'SPIN-256', 'SPIN-512', 'SPIN-1024', ...
    'location', 'northwest');
xticks([1 2 3]);
xticklabels({'20'; '150'; '400'})
xlabel('Number of Particles (n_p)');
ylabel('Accuracy (pixels)');

figure(2); clf; hold on;
lat_table = [lat_cpu', lat_spin'];
lat_std_table = [lat_cpu_std', lat_spin_std'];
hb = bar(lat_table);
drawnow;

for ib = 1:numel(hb)
    %XData property is the tick labels/group centers; XOffset is the offset
    %of each distinct group
    xData = hb(ib).XData+hb(ib).XOffset;
    errorbar(xData,lat_table(:,ib),lat_std_table(:,ib),'k.')
end
y_limits = get(gca, 'ylim');
y_limits(1) = 0;
ylim(y_limits);

legend('CPU[1]-256', 'CPU[1]-512', 'CPU[1]-1024', ...
    'SPIN-256', 'SPIN-512', 'SPIN-1024', ...
    'location', 'northwest');
xticks([1 2 3]);
xticklabels({'20'; '150'; '400'})
xlabel('Number of Particles (n_p)');
ylabel('Latency (ms)');


if ~PUBLISH
    return
end

disp('Saving Figure 1');
set(1,'Units','Inches');
pos = get(1,'Position');
set(1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(1,'accuracy_all.pdf','-dpdf','-r0','-fillpage')

disp('Saving Figure 2');
set(2,'Units','Inches');
pos = get(2,'Position');
set(2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(2,'latency_all.pdf','-dpdf','-r0','-fillpage')








