%STUDY_SWEEPS  How sample rate, servo limits, gimbal travel and gyro noise shape the controller.
%   Headless. One parameter is degraded at a time, everything else nominal.
%   Writes ../results/sweeps_matlab.csv and ../results/sweeps_matlab.png
set(0, 'DefaultFigureVisible', 'off');
base = rocket_params();

studies = {
  'Ts_ms',        [5 10 20 40 60 80],          @(P,v) setfield(setfield(P,'Ts',v*1e-3),'a_f',min(1,0.3*v/10));  % keep filter time constant
  'servo_rate_dps',[250 100 50 25 12],         @(P,v) setfield(P,'rate',deg2rad(v));
  'servo_bw_Hz',  [10 5 3 2 1],                @(P,v) setfield(P,'wn_s',2*pi*v);
  'gimbal_lim_deg',[7 3 2 1 0.5],              @(P,v) setfield(P,'lim',deg2rad(v));
  'gyro_noise_x', [1 5 20 50 100],             @(P,v) setfield(P,'sig_g',base.sig_g*v);
  'gust_mps',     [4 8 12 16 20],              @(P,v) setfield(P,'gust_amp',[0; v; 0]);
};

rows = {};
for s = 1:size(studies, 1)
    name = studies{s,1};  vals = studies{s,2};  mod = studies{s,3};
    for v = vals
        P  = mod(base, v);
        nz = gen_noise(1, P);
        L  = run_sim_matlab(P, nz);
        M  = flight_metrics(L, P);
        ok = M.max_tilt_err_boost < 20 && M.final_tilt < 15;
        rows(end+1,:) = {name, v, M.rms_tilt_err_boost, M.max_tilt_err_boost, M.max_gimbal_deg, ...
            100*M.sat_fraction, M.final_tilt, M.apogee, ok}; %#ok<SAGROW>
        fprintf('%-15s %8.3g  rms %6.2f  max %6.2f  sat %5.1f%%  tilt@BO %6.2f  apogee %6.1f  ok=%d\n', ...
            name, v, rows{end,3}, rows{end,4}, rows{end,6}, rows{end,7}, rows{end,8}, ok);
    end
end

T = cell2table(rows, 'VariableNames', {'study','value','rms_tilt_err_deg','max_tilt_err_deg', ...
    'max_gimbal_deg','saturated_pct','tilt_at_burnout_deg','apogee_m','ok'});
writetable(T, fullfile('..', 'results', 'sweeps_matlab.csv'));

f = figure('Visible', 'off', 'Position', [100 100 1400 700], 'Color', 'w');
tiledlayout(f, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for s = 1:size(studies, 1)
    nexttile;
    sel = strcmp(T.study, studies{s,1});
    yyaxis left;  plot(T.value(sel), T.max_tilt_err_deg(sel), 'o-', 'LineWidth', 1.3); ylabel('max tilt error [deg]');
    yyaxis right; plot(T.value(sel), T.saturated_pct(sel), 's--', 'LineWidth', 1.0); ylabel('gimbal saturated [%]');
    title(strrep(studies{s,1}, '_', ' ')); grid on;
    if s <= 4 && studies{s,2}(1) > studies{s,2}(end), set(gca, 'XDir', 'reverse'); end
end
exportgraphics(f, fullfile('..', 'results', 'sweeps_matlab.png'), 'Resolution', 130); close(f);
disp(T);
