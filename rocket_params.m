function P = rocket_params()
%ROCKET_PARAMS  Vehicle, motor, actuator, sensor and GNC parameters (SI units).
%
%   Frames
%     Inertial : x east, y north, z up (flat Earth), origin at the pad.
%     Body     : x forward along the nose, y/z complete a right-handed set.
%                At lift-off body-x points along inertial +z.
%     Station  : s = distance from the nose tip, measured aft [m].
%
%   Every field is a double (scalar / column / row) so the same struct can be
%   used by MATLAB Function blocks in Simulink (code-generation friendly).

P.g0 = 9.80665;

% ---- timing ----------------------------------------------------------
P.dt    = 1e-3;     % plant integration step (fixed-step RK4)
P.Ts    = 1e-2;     % IMU / flight-computer sample time (100 Hz)
P.t_ign = 1.0;      % ignition time (pad phase before it is used for IMU calibration)
P.t_end = 13.0;     % end of simulation

% ---- geometry and aerodynamics --------------------------------------
P.L    = 0.80;                  % overall length
P.d    = 0.060;                 % body diameter
P.Aref = pi*P.d^2/4;
P.s_cp = 0.475;                 % centre of pressure (fixed, low-speed estimate)
P.CNa  = 3.0;                   % normal-force slope [1/rad]
P.CA   = 0.55;                  % axial (drag) coefficient
P.Cmq  = -4.0;                  % pitch/yaw damping coefficient
P.Clp  = -15.0;                 % roll damping coefficient (small fixed fins; roll is NOT actively controlled)
P.rho0 = 1.225;
P.H    = 8500;                  % density scale height
P.s_eng = 0.78;                 % gimbal pivot station

% ---- mass properties ------------------------------------------------
P.m_dry   = 0.75;               % everything except propellant
P.s_dry   = 0.46;               % CG of dry vehicle
P.Iyy_dry = 0.040;              % pitch/yaw inertia of dry vehicle about its own CG
P.Ixx     = 6e-4;               % roll inertia (constant)
P.s_prop  = 0.74;               % propellant station (treated as point mass)

% ---- motor: representative G-class thrust curve (piecewise linear) ---
P.tc_t = [0 0.05 0.15 0.5 1.5 2.1 2.3 2.4];
P.tc_T = [0 95   75   62  55  45  20  0];
P.Isp  = 170;
P.T_scale = 1.0;                % thrust multiplier (motor variability)
P.Itot = sum(diff(P.tc_t).*(P.tc_T(1:end-1)+P.tc_T(2:end))/2);   % total impulse [N s]
P.mp0  = P.Itot/(P.Isp*P.g0);                                   % propellant mass

% ---- environment (deterministic nominal case) -----------------------
P.wind_ss  = [3; 0; 0];          % steady wind velocity [m/s], inertial
P.gust_amp = [0; 4; 0];          % 1-cos gust amplitude [m/s]
P.gust_t0  = 1.6;                % gust start (absolute time)
P.gust_T   = 1.0;                % gust duration

% ---- build errors (constant, unknown to the controller) -------------
P.eps    = deg2rad([0.4; -0.3]); % thrust misalignment, pitch/yaw
P.cg_off = [0; 0.001; 0.0005];   % lateral CG offset from the body axis [m]

% ---- initial condition ----------------------------------------------
P.tilt0 = deg2rad(2);            % pad tilt from vertical
P.az0   = deg2rad(30);           % direction of that tilt

% ---- TVC actuator (servo + linkage) ---------------------------------
P.lim  = deg2rad(7);             % gimbal travel limit (each axis)
P.rate = deg2rad(250);           % gimbal rate limit
P.wn_s = 2*pi*10;                % servo bandwidth (first-order lag) [rad/s]

% ---- IMU --------------------------------------------------------------
P.sig_g   = 0.010;               % gyro white noise (1-sigma per sample) [rad/s]
P.bsig_g  = 0.005;               % gyro turn-on bias (1-sigma) [rad/s]
P.sig_a   = 0.15;                % accelerometer noise [m/s^2]
P.bsig_a  = 0.05;                % accelerometer bias [m/s^2]
P.gyro_fs = 35;                  % gyro full scale [rad/s]

% ---- guidance ---------------------------------------------------------
P.guid_mode = 1;                 % 0 = hold vertical, 1 = pitch-over / return
P.pitch_c = deg2rad(8);          % commanded tilt during the manoeuvre
P.az_c    = 0;                   % azimuth of that tilt
P.t_kick  = 0.8;                 % manoeuvre start (time since lift-off)
P.T_kick  = 0.5;                 % smooth-step duration
P.t_ret   = 1.6;                 % return-to-vertical start

% ---- flight computer --------------------------------------------------
P.ctrl_on   = 1;                 % 0 = open loop (gimbal locked at centre)
P.t_cal     = 0.5;               % pad calibration time (gyro bias + levelling)
P.a_launch  = 25;                % lift-off detection threshold on accel-x [m/s^2]
P.a_burnout = 8;                 % burn-out detection threshold [m/s^2]
P.a_f       = 0.3;               % accel-x low-pass coefficient
P.fx_min    = 15;                % floor on the thrust-acceleration estimate
P.wn_c  = 12;                    % attitude-loop natural frequency [rad/s]
P.zeta  = 0.8;
P.Kp    = P.wn_c^2;
P.Kd    = 2*P.zeta*P.wn_c;
P.Ki    = 250;
P.ie_max = 0.05;                 % integrator clamp [rad s]
end
