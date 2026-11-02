function M = flight_metrics(log, P)
%FLIGHT_METRICS  Derived quantities and headline numbers from a simulation log.
%   M.tilt_err  [deg] angle between true nose axis and commanded direction
%   M.tilt      [deg] true angle between nose axis and vertical
%   M.est_err   [deg] attitude estimate error (angle between true and estimated nose axis)
%   M.alpha     [deg] true angle of attack

N  = numel(log.t);
tl = log.t - P.t_ign;
M.tilt_err = zeros(N,1);  M.tilt = zeros(N,1);  M.est_err = zeros(N,1);  M.alpha = zeros(N,1);
for i = 1:N
    q  = log.X(i, 7:10).'/norm(log.X(i, 7:10));
    R  = quat_to_dcm(q);
    xb = R(:,1);
    M.tilt(i) = acosd(min(max(xb(3), -1), 1));
    ucmd = guidance(max(tl(i), 0), P);
    M.tilt_err(i) = acosd(min(max(xb.'*ucmd, -1), 1));
    qe = log.dbg(i, 1:4).';
    xe = quat_to_dcm(qe/norm(qe))*[1;0;0];
    M.est_err(i) = acosd(min(max(xb.'*xe, -1), 1));
    vrel = R.'*(log.X(i, 4:6).' - wind_model(log.t(i), P));
    M.alpha(i) = atan2d(hypot(vrel(2), vrel(3)), vrel(1));
    if norm(vrel) < 10, M.alpha(i) = 0; end      % undefined at near-zero airspeed
end
M.est_err(log.dbg(:,5) == 0) = 0;                % estimator not yet initialised (pad calibration)
M.tl = tl;

flying = log.dbg(:,5) == 2;                          % controller in BOOST mode
M.t_liftoff = P.t_ign;
[~, ia] = max(log.X(:,3));
M.apogee = log.X(ia,3);
M.t_apogee = log.t(ia);
M.v_burnout = norm(log.X(find(flying, 1, 'last'), 4:6));
M.max_tilt_err_boost = max(M.tilt_err(flying));
M.rms_tilt_err_boost = sqrt(mean(M.tilt_err(flying).^2));
M.max_gimbal_deg = rad2deg(max(max(abs(log.X(:,14:15)))));
M.sat_fraction = mean(any(abs(log.cmd(flying,:)) >= P.lim - 1e-9, 2));
M.max_rate_dps = rad2deg(max(max(abs(log.X(:,12:13)))));
M.final_tilt = M.tilt(find(flying,1,'last'));
M.max_est_err = max(M.est_err);
M.max_alpha_boost = max(M.alpha(flying));
M.lateral_drift = hypot(log.X(find(flying,1,'last'),1), log.X(find(flying,1,'last'),2));
end
