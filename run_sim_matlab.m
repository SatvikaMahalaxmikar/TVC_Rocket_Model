function log = run_sim_matlab(P, nz)
%RUN_SIM_MATLAB  Closed-loop 6-DOF simulation (fixed-step RK4, 100 Hz flight computer).
%
%   Timing: the flight computer samples the IMU at t_k, computes the command and
%   that command reaches the servo at t_k + Ts (one-sample computation delay).
%   Between samples the command is held (zero-order hold).
%
%   log.t [N x 1], log.X [N x 15], log.fb, log.gyro, log.acc, log.cmd [N x 3/2/3/2], log.dbg [N x 8]

clear flight_computer
nS   = round(P.t_end/P.Ts);
nsub = round(P.Ts/P.dt);
X    = initial_state(P);
cmd_applied = zeros(2,1);

log.t    = (0:nS).'*P.Ts;
log.X    = zeros(nS+1, 15);
log.fb   = zeros(nS+1, 3);
log.gyro = zeros(nS+1, 3);
log.acc  = zeros(nS+1, 3);
log.cmd  = zeros(nS+1, 2);
log.dbg  = zeros(nS+1, 8);

for k = 0:nS
    t = k*P.Ts;
    [~, fb] = plant_deriv(t, X, cmd_applied, P);
    [gyro, acc] = imu_model(X(11:13), fb, nz.n(k+1, :), nz.b, P);
    [cmd_new, dbg] = flight_computer(gyro, acc, P);

    log.X(k+1, :)    = X.';
    log.fb(k+1, :)   = fb.';
    log.gyro(k+1, :) = gyro.';
    log.acc(k+1, :)  = acc.';
    log.cmd(k+1, :)  = cmd_new.';
    log.dbg(k+1, :)  = dbg.';

    if k == nS, break; end
    for j = 0:nsub-1
        X = rk4_step(t + j*P.dt, X, cmd_applied, P);
    end
    cmd_applied = cmd_new;
end
end

function X = rk4_step(t, X, cmd, P)
h  = P.dt;
k1 = plant_deriv(t,       X,            cmd, P);
k2 = plant_deriv(t + h/2, X + h/2*k1,   cmd, P);
k3 = plant_deriv(t + h/2, X + h/2*k2,   cmd, P);
k4 = plant_deriv(t + h,   X + h*k3,     cmd, P);
X  = X + h/6*(k1 + 2*k2 + 2*k3 + k4);
end
