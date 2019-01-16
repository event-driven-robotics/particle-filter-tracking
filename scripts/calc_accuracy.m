
%extract relevant information
GT = importdata(gt_file);
GT = [GT(:, 5), GT(:, 2), GT(:, 3)];
fix_GT;
CPU = importdata(cpu_file);
CPU = [CPU(:, 7), CPU(:, 4), CPU(:, 5)];
SPIN = importdata(spin_file);
SPIN = [SPIN(:, 7), SPIN(:, 4), SPIN(:, 5)];
LAT_CPU = importdata(latency_cpu_file);
LAT_SPIN = importdata(latency_spin_file);

%LAT_AVG = mean(LAT);
%LAT_VAR = var(LAT);

%clean timestamps
start_time = max([GT(1, 1) CPU(1, 1) SPIN(1, 1)]);
GT(:, 1) = GT(:, 1) - start_time;
CPU(:, 1) = CPU(:, 1) - start_time;
SPIN(:, 1) = SPIN(:, 1) - start_time;
end_time = min([GT(end, 1) CPU(end, 1) SPIN(end, 1) time_period(2)]);

%remove ts synch errors
while(sum(diff(SPIN(:, 1)) <= 0)> 0)
    SPIN = SPIN([true; diff(SPIN(:, 1))>0], :);
end

%perform interpolation
r_ts = 0: resolution : end_time;
r_gtx = interp1(GT(:, 1), GT(:, 2), r_ts, 'linear', 'extrap');
r_gty = interp1(GT(:, 1), GT(:, 3), r_ts, 'linear', 'extrap');


cpu_lat = interp1(LAT_CPU(:, 1), LAT_CPU(:, 2), CPU(:, 1), 'nearest', 'extrap');
CPU(:, 1) = CPU(:, 1) - cpu_lat;
%sortrows(CPU);
r_cpux = interp1(CPU(:, 1), CPU(:, 2), r_ts, 'linear');
r_cpuy = interp1(CPU(:, 1), CPU(:, 3), r_ts, 'linear');

spin_lat = interp1(LAT_SPIN(:, 1), LAT_SPIN(:, 2), SPIN(:, 1), 'nearest', 'extrap');
SPIN(:, 1) = SPIN(:, 1) - spin_lat;
%SPIN = sortrows(SPIN);
r_spinx = interp1(SPIN(:, 1), SPIN(:, 2), r_ts, 'linear');
r_spiny = interp1(SPIN(:, 1), SPIN(:, 3), r_ts, 'linear');

figure(1); clf; hold on;
plot(r_ts, r_gtx, 'color', c3);
plot(r_ts, r_cpux, 'color', c1);
plot(r_ts, r_spinx, 'color', c2);

error_cpu = (r_cpux - r_gtx).^2 + (r_cpuy - r_gty).^2;
error_spin = (r_spinx - r_gtx).^2 + (r_spiny - r_gty).^2;

%figure(2); clf; hold on;
plot(r_ts, sqrt(error_cpu), 'color', c4);
plot(r_ts, sqrt(error_spin), 'color', c5);

rms_cpu = sqrt(mean(error_cpu));
rms_spin = sqrt(mean(error_spin));

%disp(['RMS CPU ' num2str(rms_cpu)]);
%disp(['RMS SpiNNaker ' num2str(rms_spin)]);
output = [mean(LAT_CPU(:, 2)*1000) sqrt(var(LAT_CPU(:, 2)*1000)) mean(LAT_SPIN(:, 2)*1000) sqrt(var(LAT_SPIN(:, 2)*1000)); 
    mean(sqrt(error_cpu)) sqrt(var(sqrt(error_cpu))) mean(sqrt(error_spin)) sqrt(var(sqrt(error_spin)))];

disp(output);
dlmwrite(accuracy_file, output, 'delimiter', ' ', 'precision', 4);






