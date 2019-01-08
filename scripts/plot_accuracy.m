
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
number_of_detections = ones(length(nvs), length(nps));
length_of_data = ones(length(nvs), length(nps));

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
        number_of_detections(i, j) = stats(3);
        time_of_data(i, j) = stats(2);
           
    end
end

disp('Accuracy CPU (pixels)');
disp(acc_cpu); 
disp(acc_cpu_std);
disp('Accuracy SpiNNaker (pixels)');
disp(acc_spin);
disp(acc_spin_std);
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

disp('Average Data Length (seconds)');
disp(mean(mean(time_of_data)));
disp('Average Number of Detections'); format longg;
disp(mean(mean(number_of_detections)));

latencies = [reshape([lat_cpu; [0 0 0]], [12, 1]), reshape([lat_spin; [0 0 0]], [12, 1])];
laten_std = [reshape([lat_cpu_std; [0 0 0]], [12, 1]), reshape([lat_spin_std; [0 0 0]], [12, 1])];
std_ind = [1:3 5:7 9:11]; 
figure(1); clf; hold on;
hb = bar(latencies(1:end-1, :));
drawnow;
for i = 1:numel(hb)
    %XData property is the tick labels/group centers; XOffset is the offset
    %of each distinct group
    hb(i).FaceColor = c(i, :);
    xData = hb(i).XData+hb(i).XOffset;
    xData = xData(std_ind);
    errorbar(xData,latencies(std_ind,i), laten_std(std_ind,i),'k.');
    drawnow;
end
y_limits = get(gca, 'ylim');
y_limits(1) = 0;
ylim(y_limits);

xticks([2 6 10]);
xticklabels({'20', '150', '400'});
xlabel('Number of Particles (n_p)');
ylabel('Latency (ms)');
h = text(1, 100, 'n_v = 256');
set(h, 'rotation', 60);
h = text(2, 100, 'n_v = 512');
set(h, 'rotation', 60);
h = text(3, 100, 'n_v = 1024');
set(h, 'rotation', 60);
legend('CPU', 'SpiNNaker', 'location', 'northwest');

set(findall(gcf,'-property','FontSize'),'FontSize',12);
set(findall(gcf,'-property','FontName'),'FontName','Times');
set(legend, 'fontsize', 8);

set(1,'Units','Inches');
set(1, 'position', [0.3681    8.4236    7    3.4931]);
box on;


accuracies = [reshape([acc_cpu; [0 0 0]], [12, 1]), reshape([acc_spin; [0 0 0]], [12, 1])];
accura_std = [reshape([acc_cpu_std; [0 0 0]], [12, 1]), reshape([acc_spin_std; [0 0 0]], [12, 1])];
std_ind = [1:3 5:7 9:11]; 
figure(2); clf; hold on;
hb = bar(accuracies(1:end-1, :));
drawnow;
for i = 1:numel(hb)
    %XData property is the tick labels/group centers; XOffset is the offset
    %of each distinct group
    hb(i).FaceColor = c(i, :);
    xData = hb(i).XData+hb(i).XOffset;
    xData = xData(std_ind);
    errorbar(xData, accuracies(std_ind,i), accura_std(std_ind,i),'k.');
    drawnow;
end
y_limits = get(gca, 'ylim');
y_limits(1) = 0;
ylim(y_limits);

xticks([2 6 10]);
xticklabels({'20', '150', '400'});
xlabel('Number of Particles (n_p)');
ylabel('Tracking Error (pixels)');
h = text(1, 20, 'n_v = 256');
set(h, 'rotation', 60);
h = text(2, 20, 'n_v = 512');
set(h, 'rotation', 60);
h = text(3, 20, 'n_v = 1024');
set(h, 'rotation', 60);
legend('CPU', 'SpiNNaker', 'location', 'northwest');

set(findall(gcf,'-property','FontSize'),'FontSize',12);
set(findall(gcf,'-property','FontName'),'FontName','Times');
set(legend, 'fontsize', 8);

set(2,'Units','Inches');
set(2, 'position', [0.3819    4.0000    7    3.4931]);
box on;

figure(3); clf; hold on;
speed = reshape(speed, [9, 1]);
eventrate = reshape(eventrate, [9 1]);
markers = {'o'; 'o'; 'o'; 'square'; 'square'; 'square'; '^'; '^'; '^'};
markersizes = {3; 6; 9; 3; 6; 9; 3; 6; 9};
for i = [1 4 7 2 3 5 6 8 9]
    h = plot(speed(i), eventrate(i), '.', 'marker', markers{i}, ... 
        'markersize', markersizes{i}, 'color', 'k');
end
legend('n_p = 20', 'n_p = 150', 'n_p = 400', 'location', 'northwest');


xlabel('Average Target Speed (px/s)');
ylabel('Average event-rate (kv/s)');
set(gca, 'xlim', [0 400]);
set(gca, 'ylim', [0 400]);

set(findall(gcf,'-property','FontSize'),'FontSize',16);
set(findall(gcf,'-property','FontName'),'FontName','Times');
set(legend, 'fontsize', 12);
box on;












