"""Derived quantities and headline numbers from a simulation log."""
from __future__ import annotations

import math

import numpy as np

from .dynamics import wind_model
from .gnc import guidance
from .params import Params
from .quat import quat_to_dcm


def _ang(a, b) -> float:
    return math.degrees(math.acos(min(max(float(a @ b), -1.0), 1.0)))


def flight_metrics(log: dict, P: Params) -> dict:
    t, X, dbg = log["t"], log["X"], log["dbg"]
    N = len(t)
    tl = t - P.t_ign
    tilt, tilt_err, est_err, alpha = (np.zeros(N) for _ in range(4))
    for i in range(N):
        q = X[i, 6:10] / np.linalg.norm(X[i, 6:10])
        R = quat_to_dcm(q)
        xb = R[:, 0]
        tilt[i] = _ang(xb, np.array([0.0, 0.0, 1.0]))
        tilt_err[i] = _ang(xb, guidance(max(tl[i], 0.0), P))
        qe = dbg[i, :4]
        est_err[i] = _ang(xb, quat_to_dcm(qe / np.linalg.norm(qe))[:, 0])
        vrel = R.T @ (X[i, 3:6] - wind_model(t[i], P))
        alpha[i] = 0.0 if np.linalg.norm(vrel) < 10 else math.degrees(math.atan2(math.hypot(vrel[1], vrel[2]), vrel[0]))
    est_err[dbg[:, 4] == 0] = 0.0

    flying = dbg[:, 4] == 2
    last = np.flatnonzero(flying)[-1]
    ia = int(np.argmax(X[:, 2]))
    return dict(
        tilt=tilt, tilt_err=tilt_err, est_err=est_err, alpha=alpha, tl=tl,
        apogee=X[ia, 2], t_apogee=t[ia],
        v_burnout=float(np.linalg.norm(X[last, 3:6])),
        max_tilt_err_boost=tilt_err[flying].max(),
        rms_tilt_err_boost=float(np.sqrt(np.mean(tilt_err[flying] ** 2))),
        max_gimbal_deg=math.degrees(np.abs(X[:, 13:15]).max()),
        sat_fraction=float(np.mean(np.any(np.abs(log["cmd"][flying]) >= P.lim - 1e-9, axis=1))),
        max_rate_dps=math.degrees(np.abs(X[:, 11:13]).max()),
        final_tilt=tilt[last],
        max_est_err=est_err.max(),
        max_alpha_boost=alpha[flying].max(),
        lateral_drift=float(math.hypot(X[last, 0], X[last, 1])),
    )
