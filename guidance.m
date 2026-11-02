function u = guidance(tl, P)
%GUIDANCE  Commanded nose direction (inertial unit vector) vs time since lift-off.
%   Vertical, then a smooth tilt of P.pitch_c towards azimuth P.az_c, then a
%   smooth return to vertical before burn-out.
if P.guid_mode == 0
    th = 0;
else
    th = P.pitch_c*(sstep((tl - P.t_kick)/P.T_kick) - sstep((tl - P.t_ret)/P.T_kick));
end
u = [sin(th)*cos(P.az_c); sin(th)*sin(P.az_c); cos(th)];
end

function y = sstep(x)
x = min(max(x, 0), 1);
y = x*x*(3 - 2*x);
end
