%EXPORT_FOR_PYTHON  Write the shared noise sequence and the MATLAB reference run to CSV.
%   Python loads the same noise, re-runs the closed loop and compares sample by sample.
set(0, 'DefaultFigureVisible', 'off');
P   = rocket_params();
nz  = gen_noise(1, P);
log = run_sim_matlab(P, nz);

out = fullfile('..', 'results');
writematrix(nz.n, fullfile(out, 'noise_seed1.csv'));
writematrix(nz.b, fullfile(out, 'noise_seed1_bias.csv'));
A = [log.t, log.X, log.fb, log.gyro, log.acc, log.cmd, log.dbg];
hdr = ['t', compose(',X%d', 1:15), compose(',fb%d', 1:3), compose(',gyro%d', 1:3), ...
       compose(',acc%d', 1:3), compose(',cmd%d', 1:2), compose(',dbg%d', 1:8)];
fid = fopen(fullfile(out, 'matlab_reference_log.csv'), 'w');
fprintf(fid, '%s\n', strjoin(string(hdr), ''));
fclose(fid);
writematrix(A, fullfile(out, 'matlab_reference_log.csv'), 'WriteMode', 'append');
plot_results(log, P, fullfile(out, 'matlab_nominal.png'), 'MATLAB script simulation (nominal, seed 1)');

% Simulink run of the same case for the cross-check figure
build_simulink_model(P);
lm = run_simulink(P, nz);
plot_results(lm, P, fullfile(out, 'simulink_nominal.png'), 'Simulink model (nominal, seed 1)');
fprintf('exported %d samples x %d columns\n', size(A, 1), size(A, 2));
