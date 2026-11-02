function X0 = initial_state(P)
%INITIAL_STATE  15-state vector: rocket on the pad, tilted by P.tilt0 towards
%   azimuth P.az0, rates zero, gimbal centred.
q_up   = [cos(-pi/4); 0; sin(-pi/4); 0];                 % body-x -> inertial +z
ax     = [-sin(P.az0); cos(P.az0); 0];
q_tilt = [cos(P.tilt0/2); ax*sin(P.tilt0/2)];
q0     = quat_mult(q_tilt, q_up);
X0     = [0; 0; 0;  0; 0; 0;  q0;  0; 0; 0;  0; 0];
end
