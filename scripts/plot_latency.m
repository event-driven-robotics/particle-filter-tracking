latency = dlmread(latency_file);
latency(:, 2:3) = latency(:, 2:3) * 1000;



figure(3); clf; hold on;
plot(latency(:, 1), latency(:, 2), 'color', c1);
plot(latency(:, 1), latency(:, 3), 'color', c2);
line([latency(1, 1) latency(end, 1)], [mean(latency(:, 2)) mean(latency(:, 2))], 'linestyle', ':', 'color', c1);
line([latency(1, 1) latency(end, 1)], [mean(latency(:, 3)) mean(latency(:, 3))], 'linestyle', ':', 'color', c2);

xlabel('Time (s)');
ylabel('Latency (ms)');
legend('CPU[1]-256', 'SPIN-256');

%set(gcf, 'position', [3.3417 10.3083 9.7250 5.1000]);
set(findall(gcf,'-property','FontSize'),'FontSize',12);
set(findall(gcf,'-property','FontType'),'FontType','Times');

if ~PUBLISH
    return
end
%save figures

disp('Saving Figure 1');
set(1,'Units','Inches');
pos = get(1,'Position');
set(1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(1,'latency.pdf','-dpdf','-r0')
    