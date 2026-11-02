function log = run_simulink(P, nz)
%RUN_SIMULINK  Simulate rocket_tvc.slx headlessly and return the same log struct as run_sim_matlab.
mdl  = 'rocket_tvc';
here = fileparts(mfilename('fullpath'));
if ~bdIsLoaded(mdl)
    load_system(fullfile(here, [mdl '.slx']));      % load only, never open_system
end
set_param(mdl, 'StopTime', num2str(P.t_end, '%.10g'));

in = Simulink.SimulationInput(mdl);
in = in.setVariable('P',  P);
in = in.setVariable('X0', initial_state(P));
in = in.setVariable('NZ', nz.n);
in = in.setVariable('BV', nz.b);
out = sim(in);

get = @(name) squeeze(out.(name).Data);
tsX = out.logX;
log.t    = tsX.Time(:);
log.X    = reshape(tsX.Data, 15, []).';
log.fb   = reshape(out.logFb.Data,   3, []).';
log.gyro = reshape(out.logGyro.Data, 3, []).';
log.acc  = reshape(out.logAcc.Data,  3, []).';
log.cmd  = reshape(out.logCmd.Data,  2, []).';
log.dbg  = reshape(out.logDbg.Data,  8, []).';
end
