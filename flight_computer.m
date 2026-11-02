function [cmd, dbg] = flight_computer(gyro, acc, P)
%FLIGHT_COMPUTER  Navigation + guidance + control, executed once per sample.
%
%   Inputs  gyro [rad/s], acc [m/s^2]  raw IMU samples (body frame)
%   Outputs cmd  [pitch; yaw] gimbal command [rad]
%           dbg  [q(4); mode; e_y; e_z; t_launch]  estimator / controller telemetry
%
%   Modes: 0 PAD-CAL  average gyro (bias) and accel (levelling)
%          1 ARMED    strapdown attitude, wait for lift-off acceleration
%          2 BOOST    closed loop attitude control, gain-scheduled by thrust
%          3 COAST    gimbal centred after burn-out detection
%
%   Navigation: during powered flight the accelerometer measures thrust, not
%   gravity, so it cannot correct attitude. Attitude is therefore gyro
%   strapdown propagation from the pad-levelled initial attitude with the
%   pad-calibrated gyro bias.
%
%   Control: tilt error e = body-x x u_cmd (expressed in body axes, independent
%   of roll), PID in angular-acceleration units, then converted to gimbal angle
%   with the thrust-dependent control effectiveness  tau = -l*T*delta.
%   Keep `clear flight_computer` between independent runs (persistent state).

persistent mode cnt gsum asum q gprev bias fxf ie tl
if isempty(mode)
    mode = 0;  cnt = 0;
    gsum = zeros(3,1);  asum = zeros(3,1);
    q = [1; 0; 0; 0];   gprev = zeros(3,1);  bias = zeros(3,1);
    fxf = P.g0;  ie = zeros(2,1);  tl = 0;
end

cmd = zeros(2,1);
e   = zeros(2,1);

if mode == 0
    gsum = gsum + gyro;
    asum = asum + acc;
    cnt  = cnt + 1;
    if cnt >= round(P.t_cal/P.Ts)
        bias  = gsum/cnt;
        q     = level_quat(asum/norm(asum));
        gprev = gyro - bias;
        fxf   = asum(1)/cnt;
        mode  = 1;
    end
else
    % --- attitude propagation (trapezoidal body rates) -------------------
    wc = gyro - bias;
    q  = quat_mult(q, rotvec_quat(0.5*(wc + gprev)*P.Ts));
    q  = q/norm(q);
    gprev = wc;
    fxf = fxf + P.a_f*(acc(1) - fxf);

    % --- mode logic ------------------------------------------------------
    if mode == 2
        tl = tl + P.Ts;
    elseif mode == 1 && acc(1) > P.a_launch
        mode = 2;  tl = 0;  ie = zeros(2,1);
    end
    if mode == 2 && tl > 0.5 && fxf < P.a_burnout
        mode = 3;
    end

    % --- control ---------------------------------------------------------
    if mode == 2 && P.ctrl_on > 0
        u_b = quat_to_dcm(q).'*guidance(tl, P);
        e   = [-u_b(3); u_b(2)];
        a_cmd = P.Kp*e - P.Kd*wc(2:3) + P.Ki*ie;

        [~, m, s_cg, I] = mass_props(tl, P);          % nominal motor / mass schedule
        l   = P.s_eng - s_cg;
        gain = I(2)/(m*l*max(fxf, P.fx_min));          % I/(l T), T = m * a_x
        dc  = -gain*a_cmd;
        cmd = max(min(dc, P.lim), -P.lim);

        free = abs(dc) < P.lim;                        % conditional integration
        ie(free) = ie(free) + e(free)*P.Ts;
        ie = max(min(ie, P.ie_max), -P.ie_max);
    end
end
dbg = [q; mode; e; tl];
end

function q = level_quat(up_b)
% Minimal rotation taking the measured "up" direction (body frame) to inertial +z.
ax = cross(up_b, [0; 0; 1]);
s  = norm(ax);
ang = atan2(s, up_b(3));
q = [cos(ang/2); ax/max(s, 1e-12)*sin(ang/2)];
end

function q = rotvec_quat(phi)
a = norm(phi);
if a < 1e-9
    q = [1; 0.5*phi];
else
    q = [cos(a/2); phi/a*sin(a/2)];
end
end
