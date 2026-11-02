%STUDY_MONTECARLO  Closed loop vs open loop (gimbal locked) under random disturbances.
%   Randomised: pad tilt, steady wind, gust, thrust scale, thrust misalignment,
%   CG offset, IMU noise/bias realisation. Headless; writes ../results/montecarlo_matlab.*
set(0, 'DefaultFigureVisible', 'off');
N    = 30;
base = rocket_params();
rng(2026, 'twister');

res = zeros(N, 2, 4);      % run x {closed, open} x {max tilt err, tilt@BO, apogee, drift}
cases = zeros(N, 8);
for i = 1:N
    P = base;
    azw = 2*pi*rand;  P.wind_ss  = 6*rand*[cos(azw); sin(azw); 0];
    azg = 2*pi*rand;  P.gust_amp = 10*rand*[cos(azg); sin(azg); 0];
    P.T_scale = min(max(1 + 0.05*randn, 0.85), 1.15);
    P.eps     = deg2rad(0.4)*randn(2,1);
    P.cg_off  = [0; 0.0015*randn(2,1)];
    P.tilt0   = deg2rad(3)*rand;   P.az0 = 2*pi*rand;
    cases(i,:) = [P.wind_ss(1:2).' P.gust_amp(1:2).' P.T_scale rad2deg(P.eps).' rad2deg(P.tilt0)];
    nz = gen_noise(100 + i, P);
    for c = 1:2
        P.ctrl_on = (c == 1);
        L = run_sim_matlab(P, nz);
        M = flight_metrics(L, P);
        % open loop never enters the controller's boost mode only if ctrl_on=0 -> mode logic still runs
        res(i, c, :) = [M.max_tilt_err_boost, M.final_tilt, M.apogee, M.lateral_drift];
    end
    fprintf('run %2d: closed max err %5.1f deg | open max err %6.1f deg tilt@BO %6.1f deg\n', ...
        i, res(i,1,1), res(i,2,1), res(i,2,2));
end

okc = res(:,1,1) < 15 & res(:,1,2) < 10;
oko = res(:,2,1) < 15 & res(:,2,2) < 10;
fprintf('\nsuccess (max tilt err < 15 deg and tilt at burnout < 10 deg):\n');
fprintf('  closed loop %d/%d, open loop %d/%d\n', sum(okc), N, sum(oko), N);
fprintf('closed loop: tilt err max  median %.2f  p95 %.2f  worst %.2f deg\n', ...
    median(res(:,1,1)), prctile(res(:,1,1), 95), max(res(:,1,1)));
fprintf('closed loop: apogee mean %.1f m  std %.1f m;  open loop: mean %.1f m\n', ...
    mean(res(:,1,3)), std(res(:,1,3)), mean(res(:,2,3)));

T = table((1:N).', res(:,1,1), res(:,1,2), res(:,1,3), res(:,2,1), res(:,2,2), res(:,2,3), ...
    'VariableNames', {'run','cl_max_tilt_err','cl_tilt_burnout','cl_apogee','ol_max_tilt_err','ol_tilt_burnout','ol_apogee'});
writetable(T, fullfile('..', 'results', 'montecarlo_matlab.csv'));

f = figure('Visible', 'off', 'Position', [100 100 1200 450], 'Color', 'w');
tiledlayout(f, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; histogram(res(:,1,1), 0:2:60); hold on; histogram(res(:,2,1), 0:2:60); grid on;
legend('closed loop', 'open loop'); xlabel('max boost tilt error [deg]'); ylabel('runs'); title('Attitude error');
nexttile; histogram(res(:,1,3), 15); hold on; histogram(res(:,2,3), 15); grid on;
legend('closed loop', 'open loop'); xlabel('apogee [m]'); title('Apogee');
nexttile; scatter(res(:,1,4), res(:,2,4), 36, 'filled'); grid on;
xlabel('closed-loop lateral drift at burnout [m]'); ylabel('open-loop [m]'); title('Lateral drift');
exportgraphics(f, fullfile('..', 'results', 'montecarlo_matlab.png'), 'Resolution', 130); close(f);
