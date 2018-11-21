gt_file = 'C:\Users\AGlover\Documents\workspace\dump\trackingcomparison\ATIS\data_00002\data.log.GT';
cpu_file = 'C:\Users\AGlover\Documents\workspace\dump\trackingcomparison\cpu\data_00002\data.log.txt';
spin_file = 'C:\Users\AGlover\Documents\workspace\dump\trackingcomparison\spinnaker\data_00002\data.log.txt';
time_period = [4 20];

GT = importdata(gt_file);
GT = [GT(:, 5), GT(:, 2), GT(:, 3)];
CPU = importdata(cpu_file);
CPU = [CPU(:, 7), CPU(:, 4), CPU(:, 5)];
SPIN = importdata(spin_file);
SPIN = [SPIN(:, 7), SPIN(:, 4), SPIN(:, 5)];

%clean timestamps
start_time = min([GT(1, 1) CPU(1, 1) SPIN(1, 1)]);
GT(:, 1) = GT(:, 1) - start_time;
CPU(:, 1) = CPU(:, 1) - start_time;
SPIN(:, 1) = SPIN(:, 1) - start_time;

c = hot(3) * 0.85;
c1 = c(1, :);
c2 = c(2, :);
c3 = c(3, :);

figure(1); clf;
sp1 = subplot(2, 1, 1); hold on;
plot(GT(:, 1), GT(:, 2), '-', 'linewidth', 5, 'color', c3);
plot(CPU(:, 1), CPU(:, 2), '-', 'linewidth', 2, 'color', c1);
plot(SPIN(:, 1), SPIN(:, 2), '--', 'linewidth', 2, 'color', c2);

ylabel('X position (pixels)');
ylim1 = get(gca, 'ylim');

sp2 = subplot(2, 1, 2); hold on;
plot(GT(:, 1), GT(:, 3), '-', 'linewidth', 5, 'color', c3);
plot(CPU(:, 1), CPU(:, 3), '-', 'linewidth', 2, 'color', c1);
plot(SPIN(:, 1), SPIN(:, 3), '--', 'linewidth', 2, 'color', c2);

xlabel('Time (s)');
ylabel('Y position (pixels)');
ylim2 = get(gca, 'ylim');

legend('Ground Truth', 'CPU[1]-256', 'SPINN-256', 'location', 'northwest');

fixed_ylim = [min(ylim1(1), ylim2(1)), max(ylim1(2), ylim2(2))];
set(sp1, 'ylim', fixed_ylim);
set(sp1, 'xlim', time_period);
set(sp2, 'ylim', fixed_ylim);
set(sp2, 'xlim', time_period);



set(gcf, 'position', [3.3417 10.3083 9.7250 5.1000]);


set(findall(gcf,'-property','FontSize'),'FontSize',12);
set(findall(gcf,'-property','FontType'),'FontType','Times');

disp('Saving Figure 1');
set(1,'Units','Inches');
pos = get(1,'Position');
set(1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(1,'tracking.pdf','-dpdf','-r0','-fillpage')

figure(2); clf; hold on;

plot(GT(:, 1), GT(:, 3), '-', 'linewidth', 5, 'color', c3);
plot(CPU(:, 1), CPU(:, 3), '-', 'linewidth', 2, 'color', c1);
plot(SPIN(:, 1), SPIN(:, 3), '--', 'linewidth', 2, 'color', c2);

xlabel('Time (s)');
ylabel('Y position (pixels)');
set(gca, 'ylim', [110 160]);
set(gca, 'xlim', [11.3 12.3]);

disp('Saving Figure 2');
set(2,'Units','Inches');
pos = get(2,'Position');
set(2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(2,'tracking_inset.pdf','-dpdf','-r0','-fillpage')