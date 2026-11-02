function [xdot, fb] = rocket_dynamics(t, x, delta, P)
%ROCKET_DYNAMICS  6-DOF rigid-body equations of a TVC rocket.
%   x     = [r(3); v(3); q(4); w(3)]   inertial pos/vel, body->inertial quat, body rates
%   delta = [pitch; yaw] gimbal angles [rad] (actual, after the servo)
%   fb    = specific force in the body frame (what an ideal accelerometer reads)
%
%   Modelled: thrust with gimbal + static misalignment, time-varying mass /
%   CG / inertia, linear normal-force aerodynamics at the CP, axial drag,
%   pitch damping, exponential atmosphere, wind, pad constraint.
%   Not modelled: jet damping, roll control, rail, Earth rotation.

r = x(1:3);  v = x(4:6);  q = x(7:10)/norm(x(7:10));  w = x(11:13);
Rbi = quat_to_dcm(q);

[T, m, s_cg, I] = mass_props(t - P.t_ign, P);

% --- thrust ------------------------------------------------------------
dy = delta(1) + P.eps(1);
dz = delta(2) + P.eps(2);
Ft = T*[cos(dz)*cos(dy); sin(dz)*cos(dy); -sin(dy)];
r_eng = [s_cg - P.s_eng; -P.cg_off(2); -P.cg_off(3)];   % CG -> gimbal pivot
r_cp  = [s_cg - P.s_cp;  -P.cg_off(2); -P.cg_off(3)];   % CG -> centre of pressure

% --- aerodynamics ------------------------------------------------------
vrel = Rbi.'*(v - wind_model(t, P));
V    = norm(vrel);
Vs   = max(V, 1e-3);
qd   = 0.5*P.rho0*exp(-max(r(3), 0)/P.H)*V^2;
Fa   = -qd*P.Aref*[P.CA*vrel(1); P.CNa*vrel(2); P.CNa*vrel(3)]/Vs;
Mdmp = qd*P.Aref*P.d*(P.d/(2*max(V, 5)))*[P.Clp*w(1); P.Cmq*w(2); P.Cmq*w(3)];
Ma   = cross(r_cp, Fa) + Mdmp;

% --- equations of motion ---------------------------------------------
Fb   = Ft + Fa;
fb   = Fb/m;
M    = cross(r_eng, Ft) + Ma;
wdot = (M - cross(w, I.*w))./I;
vdot = Rbi*fb + [0; 0; -P.g0];
qdot = 0.5*quat_mult(q, [0; w]);

% --- pad: held in place until thrust exceeds weight ----------------------
if r(3) <= 0 && v(3) <= 0 && vdot(3) < 0
    xdot = zeros(13, 1);
    fb   = Rbi.'*[0; 0; P.g0];
else
    xdot = [v; vdot; qdot; wdot];
end
end
