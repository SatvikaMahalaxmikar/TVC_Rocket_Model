function [gyro, acc] = imu_model(w, fb, n, b, P)
%IMU_MODEL  Gyro + accelerometer at the CG: bias + white noise, gyro saturation.
%   w  true body rates, fb true specific force
%   n  1x6 unit-normal noise sample, b  1x6 unit-normal turn-on bias draw
gyro = w  + P.bsig_g*b(1:3).' + P.sig_g*n(1:3).';
gyro = max(min(gyro, P.gyro_fs), -P.gyro_fs);
acc  = fb + P.bsig_a*b(4:6).' + P.sig_a*n(4:6).';
end
