"""Guidance and flight computer (navigation + control), one call per IMU sample.

Modes: 0 PAD-CAL   average gyro (bias) and accel (levelling)
       1 ARMED     strapdown attitude, wait for lift-off acceleration
       2 BOOST     closed-loop attitude control, gain-scheduled by thrust
       3 COAST     gimbal centred after burn-out detection

Navigation: during powered flight the accelerometer measures thrust, not gravity,
so it cannot correct attitude. Attitude is gyro strapdown propagation from the
pad-levelled initial attitude using the pad-calibrated gyro bias.

Control: tilt error e = body-x x u_cmd (in body axes, independent of roll), PID in
angular-acceleration units, converted to a gimbal angle with the thrust-dependent
control effectiveness  tau = -l*T*delta,  T estimated as m_nominal * a_x.
"""
from __future__ import annotations

import math

import numpy as np

from .dynamics import mass_props
from .params import Params
from .quat import quat_mult, quat_to_dcm


def _sstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def guidance(tl: float, P: Params):
    """Commanded nose direction (inertial unit vector) vs time since lift-off."""
    if P.guid_mode == 0:
        th = 0.0
    else:
        th = P.pitch_c * (_sstep((tl - P.t_kick) / P.T_kick) - _sstep((tl - P.t_ret) / P.T_kick))
    return np.array([math.sin(th) * math.cos(P.az_c), math.sin(th) * math.sin(P.az_c), math.cos(th)])


def _level_quat(up_b):
    ax = np.array([up_b[1] * 1.0 - 0.0, 0.0 - up_b[0] * 1.0, 0.0])      # up_b x [0, 0, 1]
    s = float(np.linalg.norm(ax))
    ang = math.atan2(s, up_b[2])
    return np.concatenate([[math.cos(ang / 2)], ax / max(s, 1e-12) * math.sin(ang / 2)])


def _rotvec_quat(phi):
    a = float(np.linalg.norm(phi))
    if a < 1e-9:
        return np.concatenate([[1.0], 0.5 * phi])
    return np.concatenate([[math.cos(a / 2)], phi / a * math.sin(a / 2)])


class FlightComputer:
    def __init__(self, P: Params):
        self.P = P
        self.mode = 0
        self.cnt = 0
        self.gsum = np.zeros(3)
        self.asum = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.gprev = np.zeros(3)
        self.bias = np.zeros(3)
        self.fxf = P.g0
        self.ie = np.zeros(2)
        self.tl = 0.0

    def step(self, gyro, acc):
        """Return (cmd[2] gimbal command [rad], dbg[8] = q(4), mode, e_y, e_z, t_launch)."""
        P = self.P
        cmd = np.zeros(2)
        e = np.zeros(2)

        if self.mode == 0:
            self.gsum = self.gsum + gyro
            self.asum = self.asum + acc
            self.cnt += 1
            if self.cnt >= round(P.t_cal / P.Ts):
                self.bias = self.gsum / self.cnt
                self.q = _level_quat(self.asum / np.linalg.norm(self.asum))
                self.gprev = gyro - self.bias
                self.fxf = self.asum[0] / self.cnt
                self.mode = 1
        else:
            wc = gyro - self.bias
            self.q = quat_mult(self.q, _rotvec_quat(0.5 * (wc + self.gprev) * P.Ts))
            self.q = self.q / np.linalg.norm(self.q)
            self.gprev = wc
            self.fxf = self.fxf + P.a_f * (acc[0] - self.fxf)

            if self.mode == 2:
                self.tl += P.Ts
            elif self.mode == 1 and acc[0] > P.a_launch:
                self.mode = 2
                self.tl = 0.0
                self.ie = np.zeros(2)
            if self.mode == 2 and self.tl > 0.5 and self.fxf < P.a_burnout:
                self.mode = 3

            if self.mode == 2 and P.ctrl_on > 0:
                u_b = quat_to_dcm(self.q).T @ guidance(self.tl, P)
                e = np.array([-u_b[2], u_b[1]])
                a_cmd = P.Kp * e - P.Kd * wc[1:3] + P.Ki * self.ie

                _, m, s_cg, I = mass_props(self.tl, P)      # nominal motor / mass schedule
                l = P.s_eng - s_cg
                gain = I[1] / (m * l * max(self.fxf, P.fx_min))   # I/(l T), T = m * a_x
                dc = -gain * a_cmd
                cmd = np.clip(dc, -P.lim, P.lim)

                free = np.abs(dc) < P.lim                   # conditional integration (anti-windup)
                self.ie = np.where(free, self.ie + e * P.Ts, self.ie)
                self.ie = np.clip(self.ie, -P.ie_max, P.ie_max)

        dbg = np.concatenate([self.q, [self.mode], e, [self.tl]])
        return cmd, dbg
