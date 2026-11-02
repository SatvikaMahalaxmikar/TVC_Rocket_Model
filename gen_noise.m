function nz = gen_noise(seed, P)
%GEN_NOISE  Reproducible unit-normal IMU noise, shared by MATLAB, Simulink and Python.
%   nz.n  [N x 6] white noise per sample (gyro xyz, accel xyz)
%   nz.b  [1 x 6] turn-on bias draw
rng(seed, 'twister');
N    = round(P.t_end/P.Ts) + 1;
nz.n = randn(N, 6);
nz.b = randn(1, 6);
end
