function [Xdot, fb] = plant_deriv(t, X, cmd, P)
%PLANT_DERIV  Rocket + gimbal servo, 15 states.
%   X   = [x(13); delta(2)]
%   cmd = gimbal command [pitch; yaw] [rad] (held between flight-computer samples)
%
%   Servo: first-order lag, command saturation (travel limit) and rate limit.
%   This exact function is used by the script simulation and by the Simulink
%   model, so both integrate identical equations.

delta = X(14:15);
[xd, fb] = rocket_dynamics(t, X(1:13), delta, P);
c  = max(min(cmd, P.lim), -P.lim);
dd = max(min(P.wn_s*(c - delta), P.rate), -P.rate);
Xdot = [xd; dd];
end
