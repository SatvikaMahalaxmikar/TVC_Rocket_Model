function plot_results(log, P, outfile, titlestr)
%PLOT_RESULTS  Headless summary figure (rendered off-screen, saved as PNG).
if nargin < 4, titlestr = 'TVC model rocket - closed loop'; end
M  = flight_metrics(log, P);
t  = log.t;
fl = log.dbg(:,5) == 2;
t0 = P.t_ign - 0.2;
tb = P.t_ign + P.tc_t(end) + 0.5;

f = figure('Visible', 'off', 'Position', [100 100 1400 900], 'Color', 'w');
tl = tiledlayout(f, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, titlestr);

nexttile; plot(t, M.tilt, t, M.tilt_err, 'LineWidth', 1.2); grid on; xlim([t0 tb]);
xline(P.t_ign + P.tc_t(end), 'k:'); legend('tilt from vertical', 'error to command', 'Location', 'best');
ylabel('deg'); title('Attitude');

nexttile; plot(t, rad2deg(log.X(:,14:15)), 'LineWidth', 1.2); hold on;
plot(t, rad2deg(log.cmd), '--', 'LineWidth', 0.8); yline(rad2deg(P.lim)*[1 -1], 'r:'); grid on; xlim([t0 tb]);
legend('\delta_y', '\delta_z', 'cmd y', 'cmd z', 'Location', 'best'); ylabel('deg'); title('Gimbal (limit \pm7 deg)');

nexttile; plot(t, rad2deg(log.X(:,11:13)), 'LineWidth', 1.0); grid on; xlim([t0 tb]);
legend('p', 'q', 'r', 'Location', 'best'); ylabel('deg/s'); title('Body rates');

nexttile; plot(t, log.X(:,1:3), 'LineWidth', 1.2); grid on; xlim([t0 P.t_end]);
legend('x', 'y', 'z (alt)', 'Location', 'best'); ylabel('m'); title('Position');

nexttile; plot(t, log.X(:,4:6), 'LineWidth', 1.2); grid on; xlim([t0 P.t_end]);
legend('v_x', 'v_y', 'v_z', 'Location', 'best'); ylabel('m/s'); title('Velocity');

nexttile; plot(t, M.alpha, 'LineWidth', 1.2); grid on; xlim([t0 tb]); ylabel('deg'); title('Angle of attack');

nexttile; plot(t, M.est_err, 'LineWidth', 1.2); grid on; xlim([t0 P.t_end]); ylabel('deg'); title('Attitude estimate error');

nexttile; plot(t, log.gyro(:,2:3), t, log.X(:,12:13), 'LineWidth', 0.8); grid on; xlim([t0 tb]);
legend('gyro q', 'gyro r', 'true q', 'true r', 'Location', 'best'); ylabel('rad/s'); title('Gyro vs truth');

nexttile; plot(t, log.acc(:,1), t, log.fb(:,1), 'LineWidth', 0.8); grid on; xlim([t0 tb]);
legend('accel x meas', 'true', 'Location', 'best'); ylabel('m/s^2'); title('Axial accelerometer');

exportgraphics(f, outfile, 'Resolution', 130);
close(f);
end
