function mdl = build_simulink_model(P)
%BUILD_SIMULINK_MODEL  Programmatically create rocket_tvc.slx (no GUI is opened).
%
%   Closed loop:
%
%     Clock -> Plant (6-DOF + servo, MATLAB Function) -> Integrator (15 states) -+
%                ^ cmd                  | fb                                      |
%             UnitDelay(Ts)              v                                         |
%                ^                      IMU (Ts, noise + bias) --> FC (Ts) ---------+--> back to UnitDelay
%
%   The plant block calls the same plant_deriv.m that run_sim_matlab.m uses and
%   the integrator is solved with fixed-step ode4 (classical RK4, 1 ms), so the
%   Simulink and script results are numerically identical. The flight computer
%   and IMU are discrete blocks at 100 Hz; the UnitDelay is the 1-sample
%   computation delay.

if nargin < 1, P = rocket_params(); end
mdl = 'rocket_tvc';
if bdIsLoaded(mdl), close_system(mdl, 0); end
here = fileparts(mfilename('fullpath'));
slxfile = fullfile(here, [mdl '.slx']);
if isfile(slxfile), delete(slxfile); end            % avoid the model-name shadowing warning
new_system(mdl);                                    % created in memory, never opened

% ---- solver -----------------------------------------------------------
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', num2str(P.dt, '%.10g'), 'StopTime', num2str(P.t_end, '%.10g'), ...
    'ReturnWorkspaceOutputs', 'on', 'SaveTime', 'off', 'SaveOutput', 'off', ...
    'SignalLogging', 'off');

% ---- blocks -------------------------------------------------------------
add_block('simulink/Sources/Clock', [mdl '/Clock'], 'Position', [30 40 60 60]);

plantCode = [ ...
    "function [Xdot, fb] = fcn(t, X, cmd)" ...
    "[Xdot, fb] = plant_deriv(t, X, cmd, P);"];
add_mfn(mdl, 'Plant', plantCode, {'P'}, [150 60 260 140]);

add_block('simulink/Continuous/Integrator', [mdl '/Integrator'], ...
    'InitialCondition', 'X0', 'Position', [320 80 360 120]);

imuCode = [ ...
    "function [gyro, acc] = fcn(X, fb)" ...
    "persistent k" ...
    "if isempty(k), k = 0; end" ...
    "[gyro, acc] = imu_model(X(11:13), fb, NZ(k+1, :), BV, P);" ...
    "k = k + 1;"];
add_mfn(mdl, 'IMU', imuCode, {'P', 'NZ', 'BV'}, [430 150 540 230]);

fcCode = [ ...
    "function [cmd, dbg] = fcn(gyro, acc)" ...
    "[cmd, dbg] = flight_computer(gyro, acc, P);"];
add_mfn(mdl, 'FC', fcCode, {'P'}, [600 150 710 230]);

% Zero-order holds sample the continuous plant at Ts; the IMU and FC blocks
% inherit that discrete rate (100 Hz).
Tss = num2str(P.Ts, '%.10g');
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/ZOH_X'],  'SampleTime', Tss, 'Position', [380 150 410 170]);
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/ZOH_fb'], 'SampleTime', Tss, 'Position', [380 190 410 210]);

add_block('simulink/Discrete/Unit Delay', [mdl '/CmdDelay'], ...
    'InitialCondition', 'zeros(2,1)', 'SampleTime', num2str(P.Ts, '%.10g'), ...
    'Position', [150 200 190 240]);
set_param([mdl '/CmdDelay'], 'Orientation', 'left');

% ---- logging (decimated to the 100 Hz sample grid) ---------------------
logs = {'logX', 15, 10; 'logFb', 3, 10; 'logGyro', 3, 1; 'logAcc', 3, 1; 'logCmd', 2, 1; 'logDbg', 8, 1};
for i = 1:size(logs, 1)
    add_block('simulink/Sinks/To Workspace', [mdl '/' logs{i,1}], ...
        'VariableName', logs{i,1}, 'SaveFormat', 'Timeseries', ...
        'Decimation', num2str(logs{i,3}), 'Position', [760 20+40*i 840 40+40*i]);
end

% ---- wiring --------------------------------------------------------------
add_line(mdl, 'Clock/1',      'Plant/1');
add_line(mdl, 'Integrator/1', 'Plant/2');
add_line(mdl, 'CmdDelay/1',   'Plant/3');
add_line(mdl, 'Plant/1',      'Integrator/1');
add_line(mdl, 'Integrator/1', 'ZOH_X/1');
add_line(mdl, 'Plant/2',      'ZOH_fb/1');
add_line(mdl, 'ZOH_X/1',      'IMU/1');
add_line(mdl, 'ZOH_fb/1',     'IMU/2');
add_line(mdl, 'IMU/1',        'FC/1');
add_line(mdl, 'IMU/2',        'FC/2');
add_line(mdl, 'FC/1',         'CmdDelay/1');
add_line(mdl, 'Integrator/1', 'logX/1');
add_line(mdl, 'Plant/2',      'logFb/1');
add_line(mdl, 'IMU/1',        'logGyro/1');
add_line(mdl, 'IMU/2',        'logAcc/1');
add_line(mdl, 'FC/1',         'logCmd/1');
add_line(mdl, 'FC/2',         'logDbg/1');

save_system(mdl, fullfile(here, [mdl '.slx']));
end

function add_mfn(mdl, name, codeLines, paramNames, pos)
blk = [mdl '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = strjoin(codeLines, newline);
for i = 1:numel(paramNames)
    d = Stateflow.Data(ch);
    d.Name  = paramNames{i};
    d.Scope = 'Parameter';
end
end
