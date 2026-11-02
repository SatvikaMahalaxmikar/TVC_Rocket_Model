"""Closed-loop 6-DOF simulation: fixed-step RK4 plant, 100 Hz flight computer.

Timing: the flight computer samples the IMU at t_k, computes the command, and
that command reaches the servo at t_k + Ts (one-sample computation delay).
Between samples the command is held (zero-order hold).
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .dynamics import imu_model, initial_state, plant_deriv, rk4_step
from .gnc import FlightComputer
from .params import Params


@dataclass
class Noise:
    n: np.ndarray   # [N, 6] unit-normal white noise (gyro xyz, accel xyz)
    b: np.ndarray   # [6]    unit-normal turn-on bias draw


def gen_noise(seed: int, P: Params) -> Noise:
    """Reproducible noise with numpy (NOT identical to MATLAB's; use load_noise for cross-checks)."""
    rng = np.random.default_rng(seed)
    N = round(P.t_end / P.Ts) + 1
    return Noise(rng.standard_normal((N, 6)), rng.standard_normal(6))


def load_noise(results_dir, name: str = "noise_seed1") -> Noise:
    """Load the noise sequence written by MATLAB's export_for_python.m."""
    d = Path(results_dir)
    return Noise(np.loadtxt(d / f"{name}.csv", delimiter=","),
                 np.loadtxt(d / f"{name}_bias.csv", delimiter=","))


def run_sim(P: Params, nz: Noise) -> dict:
    """Return a log dict with t [N], X [N,15], fb, gyro, acc [N,3], cmd [N,2], dbg [N,8]."""
    fc = FlightComputer(P)
    nS = round(P.t_end / P.Ts)
    nsub = round(P.Ts / P.dt)
    X = initial_state(P)
    cmd_applied = np.zeros(2)

    log = dict(t=np.arange(nS + 1) * P.Ts, X=np.zeros((nS + 1, 15)), fb=np.zeros((nS + 1, 3)),
               gyro=np.zeros((nS + 1, 3)), acc=np.zeros((nS + 1, 3)),
               cmd=np.zeros((nS + 1, 2)), dbg=np.zeros((nS + 1, 8)))

    for k in range(nS + 1):
        t = k * P.Ts
        fb = plant_deriv(t, X, cmd_applied, P)[1]
        gyro, acc = imu_model(X[10:13], fb, nz.n[k], nz.b, P)
        cmd_new, dbg = fc.step(gyro, acc)

        log["X"][k], log["fb"][k], log["gyro"][k] = X, fb, gyro
        log["acc"][k], log["cmd"][k], log["dbg"][k] = acc, cmd_new, dbg
        if k == nS:
            break
        for j in range(nsub):
            X = rk4_step(t + j * P.dt, X, cmd_applied, P)
        cmd_applied = cmd_new
    return log
