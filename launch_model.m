%LAUNCH_MODEL  Interactive entry point: opens rocket_tvc.slx with everything it needs in the
%   base workspace, so the Run button in the Simulink editor works as is.
%   Also runs the model once and shows the result figure.
P  = rocket_params();
nz = gen_noise(1, P);
X0 = initial_state(P);
NZ = nz.n;
BV = nz.b;

build_simulink_model(P);          % regenerate rocket_tvc.slx from code (keeps model and parameters in sync)
open_system('rocket_tvc');        % opens the editor window

log = run_simulink(P, nz);        % one run, same call the Run button makes
M   = flight_metrics(log, P);
fprintf('apogee %.1f m, boost tilt error max %.2f deg, max gimbal %.2f deg\n', ...
    M.apogee, M.max_tilt_err_boost, M.max_gimbal_deg);

png = fullfile(tempdir, 'rocket_tvc_launch.png');
plot_results(log, P, png, 'Simulink model - nominal flight');
figure('Name', 'Nominal flight', 'Position', [80 80 1300 840]);
imshow(imread(png));
