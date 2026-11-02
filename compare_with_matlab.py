"""Run the Python closed loop on MATLAB's noise and diff it against MATLAB's reference log.

Prerequisite: run matlab/export_for_python.m once (writes ../results/*.csv).
Usage:  .venv\\Scripts\\python python\\scripts\\compare_with_matlab.py
"""
import time
from pathlib import Path

import numpy as np

from rocket_tvc import Params, flight_metrics, load_noise, run_sim
from rocket_tvc.plotting import plot_results

RES = Path(__file__).resolve().parents[2] / "results"

P = Params()
nz = load_noise(RES)
t0 = time.time()
log = run_sim(P, nz)
print(f"python sim wall time {time.time() - t0:.1f} s")

ref = np.loadtxt(RES / "matlab_reference_log.csv", delimiter=",", skiprows=1)
cols = {"X": slice(1, 16), "fb": slice(16, 19), "gyro": slice(19, 22), "acc": slice(22, 25),
        "cmd": slice(25, 27), "dbg": slice(27, 35)}
assert np.allclose(ref[:, 0], log["t"])

print(f"{'signal':6s} {'max |python - matlab|':>24s} {'max |matlab|':>14s}")
worst = 0.0
for name, sl in cols.items():
    diff = np.abs(log[name] - ref[:, sl]).max()
    worst = max(worst, diff)
    print(f"{name:6s} {diff:24.3e} {np.abs(ref[:, sl]).max():14.3e}")

M = flight_metrics(log, P)
print(f"\napogee {M['apogee']:.3f} m, boost tilt error max {M['max_tilt_err_boost']:.3f} deg "
      f"rms {M['rms_tilt_err_boost']:.3f} deg, max gimbal {M['max_gimbal_deg']:.3f} deg")
plot_results(log, P, RES / "python_nominal.png", "Python simulation (nominal, seed 1, MATLAB noise)")
print("worst difference over all signals: %.3e" % worst)
