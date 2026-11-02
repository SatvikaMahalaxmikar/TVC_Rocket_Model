"""6-DOF rigid-body equations of a TVC rocket plus the gimbal servo (15 states).

State X = [r(3), v(3), q(4), w(3), delta(2)]: inertial position/velocity,
body->inertial quaternion, body rates, actual gimbal angles [pitch, yaw].

Modelled: thrust with gimbal + static misalignment, time-varying mass / CG /
inertia, linear normal-force aerodynamics at the CP, axial drag, pitch/yaw/roll
damping, exponential atmosphere, wind, pad constraint.
Not modelled: jet damping, roll control, launch rail, Earth rotation.
"""
from __future__ import annotations

import math

import numpy as np

from .params import Params
from .quat import cross, quat_mult, quat_to_dcm


def mass_props(tm: float, P: Params):
    """Thrust [N], mass [kg], CG station [m], inertia [Ixx, Iyy, Izz] at time tm since ignition."""
    T = 0.0
    Icum = 0.0
    if tm > 0:
        tc_t, tc_T = P.tc_t, P.tc_T
        for i in range(len(tc_t) - 1):
            t0, t1 = tc_t[i], tc_t[i + 1]
            T0, T1 = tc_T[i], tc_T[i + 1]
            if tm >= t1:
                Icum += 0.5 * (T0 + T1) * (t1 - t0)
            else:
                sl = (T1 - T0) / (t1 - t0)
                dt = tm - t0
                T = T0 + sl * dt
                Icum += T0 * dt + 0.5 * sl * dt * dt
                break
    T = P.T_scale * T                       # thrust variation; mass flow unchanged
    mp = P.mp0 * (1 - Icum / P.Itot)
    m = P.m_dry + mp
    s_cg = (P.m_dry * P.s_dry + mp * P.s_prop) / m
    Iyy = P.Iyy_dry + P.m_dry * (s_cg - P.s_dry) ** 2 + mp * (s_cg - P.s_prop) ** 2
    return T, m, s_cg, np.array([P.Ixx, Iyy, Iyy])


def wind_model(t: float, P: Params):
    """Steady wind plus one 1-cos gust (inertial frame, m/s)."""
    wind = P.wind_ss
    if P.gust_t0 <= t < P.gust_t0 + P.gust_T:
        wind = wind + P.gust_amp * 0.5 * (1 - math.cos(2 * math.pi * (t - P.gust_t0) / P.gust_T))
    return wind


def rocket_dynamics(t, x, delta, P: Params):
    """Return (xdot[13], fb[3]); fb is the body-frame specific force an ideal accelerometer reads."""
    r, v, w = x[0:3], x[3:6], x[10:13]
    q = x[6:10] / np.linalg.norm(x[6:10])
    Rbi = quat_to_dcm(q)

    T, m, s_cg, I = mass_props(t - P.t_ign, P)

    # thrust
    dy = delta[0] + P.eps[0]
    dz = delta[1] + P.eps[1]
    Ft = T * np.array([math.cos(dz) * math.cos(dy), math.sin(dz) * math.cos(dy), -math.sin(dy)])
    r_eng = np.array([s_cg - P.s_eng, -P.cg_off[1], -P.cg_off[2]])   # CG -> gimbal pivot
    r_cp = np.array([s_cg - P.s_cp, -P.cg_off[1], -P.cg_off[2]])     # CG -> centre of pressure

    # aerodynamics
    vrel = Rbi.T @ (v - wind_model(t, P))
    V = float(np.linalg.norm(vrel))
    Vs = max(V, 1e-3)
    qd = 0.5 * P.rho0 * math.exp(-max(r[2], 0.0) / P.H) * V * V
    Fa = -qd * P.Aref * np.array([P.CA * vrel[0], P.CNa * vrel[1], P.CNa * vrel[2]]) / Vs
    Mdmp = qd * P.Aref * P.d * (P.d / (2 * max(V, 5.0))) * np.array([P.Clp * w[0], P.Cmq * w[1], P.Cmq * w[2]])
    Ma = cross(r_cp, Fa) + Mdmp

    # equations of motion
    fb = (Ft + Fa) / m
    M = cross(r_eng, Ft) + Ma
    wdot = (M - cross(w, I * w)) / I
    vdot = Rbi @ fb + np.array([0.0, 0.0, -P.g0])
    qdot = 0.5 * quat_mult(q, np.array([0.0, w[0], w[1], w[2]]))

    # pad: held in place until thrust exceeds weight
    if r[2] <= 0 and v[2] <= 0 and vdot[2] < 0:
        return np.zeros(13), Rbi.T @ np.array([0.0, 0.0, P.g0])
    return np.concatenate([v, vdot, qdot, wdot]), fb


def plant_deriv(t, X, cmd, P: Params):
    """Rocket + first-order servo with travel and rate limits. Returns (Xdot[15], fb[3])."""
    delta = X[13:15]
    xd, fb = rocket_dynamics(t, X[:13], delta, P)
    c = np.clip(cmd, -P.lim, P.lim)
    dd = np.clip(P.wn_s * (c - delta), -P.rate, P.rate)
    return np.concatenate([xd, dd]), fb


def rk4_step(t, X, cmd, P: Params):
    h = P.dt
    k1 = plant_deriv(t, X, cmd, P)[0]
    k2 = plant_deriv(t + h / 2, X + h / 2 * k1, cmd, P)[0]
    k3 = plant_deriv(t + h / 2, X + h / 2 * k2, cmd, P)[0]
    k4 = plant_deriv(t + h, X + h * k3, cmd, P)[0]
    return X + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def initial_state(P: Params):
    q_up = np.array([math.cos(-math.pi / 4), 0.0, math.sin(-math.pi / 4), 0.0])   # body-x -> inertial +z
    ax = np.array([-math.sin(P.az0), math.cos(P.az0), 0.0])
    q_tilt = np.concatenate([[math.cos(P.tilt0 / 2)], ax * math.sin(P.tilt0 / 2)])
    q0 = quat_mult(q_tilt, q_up)
    X0 = np.zeros(15)
    X0[6:10] = q0
    return X0


def imu_model(w, fb, n, b, P: Params):
    """Gyro + accelerometer at the CG: bias + white noise, gyro saturation."""
    gyro = w + P.bsig_g * b[0:3] + P.sig_g * n[0:3]
    gyro = np.clip(gyro, -P.gyro_fs, P.gyro_fs)
    acc = fb + P.bsig_a * b[3:6] + P.sig_a * n[3:6]
    return gyro, acc
