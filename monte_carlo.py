"""Monte Carlo: closed loop vs open loop (gimbal locked) under random disturbances.

Randomised: pad tilt, steady wind, gust, thrust scale, thrust misalignment, CG offset,
IMU noise/bias realisation. Runs in parallel (one process per CPU core).

Usage:  .venv\\Scripts\\python python\\scripts\\monte_carlo.py [N]
"""
import csv
import dataclasses
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from rocket_tvc import Params, flight_metrics, gen_noise, run_sim  # noqa: E402

RES = Path(__file__).resolve().parents[2] / "results"
deg = math.pi / 180


def make_case(i: int) -> Params:
    rng = np.random.default_rng(2026 + i)
    azw, azg = rng.uniform(0, 2 * math.pi, 2)
    return dataclasses.replace(
        Params(),
        wind_ss=6 * rng.uniform() * np.array([math.cos(azw), math.sin(azw), 0.0]),
        gust_amp=10 * rng.uniform() * np.array([math.cos(azg), math.sin(azg), 0.0]),
        T_scale=float(np.clip(1 + 0.05 * rng.standard_normal(), 0.85, 1.15)),
        eps=0.4 * deg * rng.standard_normal(2),
        cg_off=np.array([0.0, *(0.0015 * rng.standard_normal(2))]),
        tilt0=3 * deg * rng.uniform(),
        az0=2 * math.pi * rng.uniform(),
    )


def run_case(i: int):
    P = make_case(i)
    nz = gen_noise(100 + i, P)
    out = []
    for ctrl in (1, 0):
        Pc = dataclasses.replace(P, ctrl_on=ctrl)
        M = flight_metrics(run_sim(Pc, nz), Pc)
        out.append((M["max_tilt_err_boost"], M["final_tilt"], M["apogee"], M["lateral_drift"]))
    return out


if __name__ == "__main__":
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    with ProcessPoolExecutor(max_workers=os.cpu_count()) as ex:
        res = np.array(list(ex.map(run_case, range(N))))          # [N, 2, 4]

    ok = lambda r: (r[:, 0] < 15) & (r[:, 1] < 10)                # noqa: E731
    cl, ol = res[:, 0], res[:, 1]
    print(f"success (max tilt err < 15 deg and tilt at burnout < 10 deg): closed {ok(cl).sum()}/{N}, open {ok(ol).sum()}/{N}")
    print(f"closed loop tilt error max: median {np.median(cl[:, 0]):.2f}  p95 {np.percentile(cl[:, 0], 95):.2f}  worst {cl[:, 0].max():.2f} deg")
    print(f"apogee closed {cl[:, 2].mean():.1f} +/- {cl[:, 2].std():.1f} m, open {ol[:, 2].mean():.1f} m")

    with open(RES / "montecarlo_python.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["run", "cl_max_tilt_err", "cl_tilt_burnout", "cl_apogee", "cl_drift",
                    "ol_max_tilt_err", "ol_tilt_burnout", "ol_apogee", "ol_drift"])
        for i, r in enumerate(res, 1):
            w.writerow([i, *r[0], *r[1]])

    fig, ax = plt.subplots(1, 3, figsize=(12, 4.2), constrained_layout=True)
    ax[0].hist(cl[:, 0], bins=np.arange(0, 62, 2), alpha=.7, label="closed loop")
    ax[0].hist(ol[:, 0], bins=np.arange(0, 62, 2), alpha=.7, label="open loop")
    ax[0].set(xlabel="max boost tilt error [deg]", ylabel="runs", title="Attitude error"); ax[0].legend()
    ax[1].hist(cl[:, 2], bins=15, alpha=.7, label="closed loop"); ax[1].hist(ol[:, 2], bins=15, alpha=.7, label="open loop")
    ax[1].set(xlabel="apogee [m]", title="Apogee"); ax[1].legend()
    ax[2].scatter(cl[:, 3], ol[:, 3]); ax[2].set(xlabel="closed-loop lateral drift at burnout [m]", ylabel="open-loop [m]", title="Lateral drift")
    for a in ax:
        a.grid(True)
    fig.savefig(RES / "montecarlo_python.png", dpi=130)
