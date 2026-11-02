%COMPARE_SIMULINK_VS_SCRIPT  Same plant, same noise: Simulink model vs script loop.
set(0, 'DefaultFigureVisible', 'off');
P  = rocket_params();
nz = gen_noise(1, P);
build_simulink_model(P);

tic; ls = run_sim_matlab(P, nz); t_script = toc;
tic; lm = run_simulink(P, nz);   t_slx = toc;
fprintf('script %.1f s, simulink %.1f s\n', t_script, t_slx);
fprintf('samples: script %d, simulink %d\n', numel(ls.t), numel(lm.t));

n = min(numel(ls.t), numel(lm.t));
dX = max(abs(ls.X(1:n,:) - lm.X(1:n,:)), [], 1);
fprintf('max |X_script - X_simulink| per state:\n'); disp(dX);
fprintf('max |cmd diff| = %.3e rad, max |gyro diff| = %.3e, max |dbg diff| = %.3e\n', ...
    max(abs(ls.cmd(1:n,:) - lm.cmd(1:n,:)), [], 'all'), ...
    max(abs(ls.gyro(1:n,:) - lm.gyro(1:n,:)), [], 'all'), ...
    max(abs(ls.dbg(1:n,:) - lm.dbg(1:n,:)), [], 'all'));
Ms = flight_metrics(ls, P);  Mm = flight_metrics(lm, P);
fprintf('apogee: script %.3f m, simulink %.3f m\n', Ms.apogee, Mm.apogee);
