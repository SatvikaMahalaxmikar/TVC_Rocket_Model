"""Summary figure matching the MATLAB plot_results layout (headless Agg backend)."""
from __future__ import annotations

import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .metrics import flight_metrics  # noqa: E402
from .params import Params  # noqa: E402


def plot_results(log: dict, P: Params, outfile, title: str = "TVC model rocket - closed loop"):
    M = flight_metrics(log, P)
    t, X = log["t"], log["X"]
    d = np.degrees
    t0, tb = P.t_ign - 0.2, P.t_ign + P.tc_t[-1] + 0.5
    burn_end = P.t_ign + P.tc_t[-1]

    fig, ax = plt.subplots(3, 3, figsize=(14, 9), constrained_layout=True)
    fig.suptitle(title)
    a = ax[0, 0]; a.plot(t, M["tilt"], label="tilt from vertical"); a.plot(t, M["tilt_err"], label="error to command")
    a.axvline(burn_end, color="k", ls=":"); a.set(xlim=(t0, tb), ylabel="deg", title="Attitude"); a.legend()
    a = ax[0, 1]; a.plot(t, d(X[:, 13]), label="δ_y"); a.plot(t, d(X[:, 14]), label="δ_z")
    a.plot(t, d(log["cmd"][:, 0]), "--", lw=0.8, label="cmd y"); a.plot(t, d(log["cmd"][:, 1]), "--", lw=0.8, label="cmd z")
    for s in (1, -1):
        a.axhline(s * math.degrees(P.lim), color="r", ls=":")
    a.set(xlim=(t0, tb), ylabel="deg", title="Gimbal (limit ±7°)"); a.legend()
    a = ax[0, 2]; a.plot(t, d(X[:, 10:13])); a.set(xlim=(t0, tb), ylabel="deg/s", title="Body rates"); a.legend(["p", "q", "r"])
    a = ax[1, 0]; a.plot(t, X[:, 0:3]); a.set(xlim=(t0, P.t_end), ylabel="m", title="Position"); a.legend(["x", "y", "z (alt)"])
    a = ax[1, 1]; a.plot(t, X[:, 3:6]); a.set(xlim=(t0, P.t_end), ylabel="m/s", title="Velocity"); a.legend(["vx", "vy", "vz"])
    a = ax[1, 2]; a.plot(t, M["alpha"]); a.set(xlim=(t0, tb), ylabel="deg", title="Angle of attack")
    a = ax[2, 0]; a.plot(t, M["est_err"]); a.set(xlim=(t0, P.t_end), ylabel="deg", title="Attitude estimate error")
    a = ax[2, 1]; a.plot(t, log["gyro"][:, 1:3], lw=0.8); a.plot(t, X[:, 11:13], lw=0.8)
    a.set(xlim=(t0, tb), ylabel="rad/s", title="Gyro vs truth"); a.legend(["gyro q", "gyro r", "true q", "true r"])
    a = ax[2, 2]; a.plot(t, log["acc"][:, 0], lw=0.8, label="accel x meas"); a.plot(t, log["fb"][:, 0], lw=0.8, label="true")
    a.set(xlim=(t0, tb), ylabel="m/s²", title="Axial accelerometer"); a.legend()
    for a in ax.flat:
        a.grid(True)
    fig.savefig(outfile, dpi=130)
    plt.close(fig)
