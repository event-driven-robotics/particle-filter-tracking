
try
    GT = importdata(gt_file);
catch
    GT = [0 0 0 0 0]
end
        
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

figure(1); clf;
sp1 = subplot(2, 1, 1); hold on;
plot(GT(:, 1), GT(:, 2), '-', 'linewidth', 5, 'color', c3);
plot(CPU(:, 1), CPU(:, 2), '-', 'linewidth', 2, 'color', c1);
plot(SPIN(:, 1), SPIN(:, 2), '--', 'linewidth', 2, 'color', c2);

ylabel('X position (pixels)');
ylim1 = get(gca, 'ylim');
box on;



sp2 = subplot(2, 1, 2); hold on;
plot(GT(:, 1), GT(:, 3), '-', 'linewidth', 5, 'color', c3);
plot(CPU(:, 1), CPU(:, 3), '-', 'linewidth', 2, 'color', c1);
plot(SPIN(:, 1), SPIN(:, 3), '--', 'linewidth', 2, 'color', c2);

xlabel('Time (s)');
ylabel('Y position (pixels)');
ylim2 = get(gca, 'ylim');
legend('Ground Truth', 'CPU-20-512', 'SPINN-150-256', 'location', 'northwest');

fixed_ylim = [min(ylim1(1), ylim2(1)), max(ylim1(2), ylim2(2))];
set(sp1, 'ylim', fixed_ylim);
set(sp1, 'xlim', time_period);
set(sp2, 'ylim', fixed_ylim);
set(sp2, 'xlim', time_period);
box on;
set(1,'Units','Inches');
set(1, 'position', [0.3681    8.4236    7    3.4931]);


set(findall(gcf,'-property','FontSize'),'FontSize',12);
set(findall(gcf,'-property','FontName'),'FontName','Times');
set(legend, 'fontsize', 8);

figure(2); clf; hold on;

plot(GT(:, 1), GT(:, 3), '-', 'linewidth', 5, 'color', c3);
plot(CPU(:, 1), CPU(:, 3), '-', 'linewidth', 2, 'color', c1);
plot(SPIN(:, 1), SPIN(:, 3), '--', 'linewidth', 2, 'color', c2);

xlabel('Time (s)');
ylabel('Y position (pixels)');
set(gca, 'ylim', [110 170]);
set(gca, 'xlim', [12.5 13.4]);
legend('Ground Truth', 'CPU', 'SpiNNaker', 'location', 'northwest');

set(findall(gcf,'-property','FontSize'),'FontSize',16);
set(findall(gcf,'-property','FontName'),'FontName','Times');
set(legend, 'fontsize', 12);
set(2,'Units','Inches');
set(2, 'position', [2.4 8 7.95 4.375]);
box on;
