latency_cpu = dlmread(latency_cpu_file);
latency_spin = dlmread(latency_spin_file);
latency_spin(:, 2) = latency_spin(:, 2) * 1000;
latency_cpu(:, 2) = latency_cpu(:, 2) * 1000;

figure(3); clf; hold on;
plot(latency_cpu(:, 1), latency_cpu(:, 2), 'color', c1);
plot(latency_spin(:, 1), latency_spin(:, 2), 'color', c2);
line([latency_cpu(1, 1) latency_cpu(end, 1)], [mean(latency_cpu(:, 2)) mean(latency_cpu(:, 2))], 'linestyle', ':', 'color', c1);
line([latency_spin(1, 1) latency_spin(end, 1)], [mean(latency_spin(:, 2)) mean(latency_spin(:, 2))], 'linestyle', ':', 'color', c2);

xlabel('Time (s)');
ylabel('Latency (ms)');
legend('CPU', 'SPIN');

%set(gcf, 'position', [3.3417 10.3083 9.7250 5.1000]);
set(findall(gcf,'-property','FontSize'),'FontSize',12);
set(findall(gcf,'-property','FontName'),'FontName','Times');
set(legend, 'fontsize', 8);

    