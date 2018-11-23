gt_file = 'C:\Users\AGlover\Documents\workspace\dump\trackingcomparison\ATIS\data_00002\data.log.GT';
cpu_file = 'C:\Users\AGlover\Documents\workspace\dump\trackingcomparison\cpu\data_00002\data.log.txt';
spin_file = 'C:\Users\AGlover\Documents\workspace\dump\trackingcomparison\spinnaker\data_00002\data.log.txt';
latency_file = 'latency.txt';

time_period = [4 20];
resolution = 0.01;

%extract relevant information
GT = importdata(gt_file);
GT = [GT(:, 5), GT(:, 2), GT(:, 3)];
CPU = importdata(cpu_file);
CPU = [CPU(:, 7), CPU(:, 4), CPU(:, 5)];
SPIN = importdata(spin_file);
SPIN = [SPIN(:, 7), SPIN(:, 4), SPIN(:, 5)];
LAT = importdata(latency_file);

%clean timestamps
start_time = min([GT(1, 1) CPU(1, 1) SPIN(1, 1)]);
GT(:, 1) = GT(:, 1) - start_time;
CPU(:, 1) = CPU(:, 1) - start_time;
SPIN(:, 1) = SPIN(:, 1) - start_time;

%remove ts synch errors
SPIN = SPIN([true; diff(SPIN(:, 1))>0], :);

%perform interpolation
r_ts = time_period(1): resolution : time_period(2);
r_gtx = interp1(GT(:, 1), GT(:, 2), r_ts, 'linear');
r_gty = interp1(GT(:, 1), GT(:, 3), r_ts, 'linear');



cpu_lat = interp1(LAT(:, 1), LAT(:, 2), CPU(:, 1), 'nearest', 'extrap');
CPU(:, 1) = CPU(:, 1) - cpu_lat;
sortrows(CPU);
r_cpux = interp1(CPU(:, 1), CPU(:, 2), r_ts, 'PCHIP');
r_cpuy = interp1(CPU(:, 1), CPU(:, 3), r_ts, 'PCHIP');

spin_lat = interp1(LAT(:, 1), LAT(:, 3), SPIN(:, 1), 'nearest', 'extrap');
SPIN(:, 1) = SPIN(:, 1) - spin_lat;
sortrows(SPIN);
r_spinx = interp1(SPIN(:, 1), SPIN(:, 2), r_ts, 'PCHIP');
r_spiny = interp1(SPIN(:, 1), SPIN(:, 3), r_ts, 'PCHIP');

c = hot(3) * 0.85;
c1 = c(1, :);
c2 = c(2, :);
c3 = c(3, :);

% figure(1); clf; hold on;
% 
% plot(r_ts, r_gt, 'color', c3);
% plot(r_ts, r_cpu, 'color', c1);
% plot(r_ts, r_spin, 'color', c2);

rms_cpu = sqrt(mean((r_cpux - r_gtx).^2 + (r_cpuy - r_gty).^2) );
rms_spin = sqrt(mean((r_spinx - r_gtx).^2 + (r_spiny - r_gty).^2));
disp(['RMS SPU ' num2str(rms_cpu)]);
disp(['RMS SpiNNaker ' num2str(rms_spin)]);






