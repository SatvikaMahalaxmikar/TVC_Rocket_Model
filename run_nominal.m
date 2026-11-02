%TEST_NOMINAL  Headless nominal run of the script simulation; prints headline numbers.
set(0, 'DefaultFigureVisible', 'off');
P  = rocket_params();
nz = gen_noise(1, P);
tic; log = run_sim_matlab(P, nz); fprintf('sim wall time %.2f s\n', toc);
M = flight_metrics(log, P);
fprintf('mass: wet %.4f kg, prop %.4f kg, impulse %.1f Ns, mean thrust %.1f N\n', ...
    P.m_dry + P.mp0, P.mp0, P.Itot, P.Itot/P.tc_t(end));
fprintf('apogee %.1f m @ t=%.2f s, burnout speed %.1f m/s\n', M.apogee, M.t_apogee, M.v_burnout);
fprintf('boost tilt error: max %.2f deg, rms %.2f deg\n', M.max_tilt_err_boost, M.rms_tilt_err_boost);
fprintf('max gimbal %.2f deg, saturated %.1f %% of boost, max body rate %.1f deg/s\n', ...
    M.max_gimbal_deg, 100*M.sat_fraction, M.max_rate_dps);
fprintf('attitude estimate error max %.3f deg, max alpha in boost %.2f deg, tilt at burnout %.2f deg\n', ...
    M.max_est_err, M.max_alpha_boost, M.final_tilt);
