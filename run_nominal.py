"""Run the nominal closed-loop flight in Python, print headline numbers, save a figure.

Usage:  .venv\\Scripts\\python python\\scripts\\run_nominal.py [seed]
Standalone: uses numpy noise, so it does not need the MATLAB CSV exports.
"""
import sys
import time
from pathlib import Path

from rocket_tvc import Params, flight_metrics, gen_noise, run_sim
from rocket_tvc.plotting import plot_results

RES = Path(__file__).resolve().parents[2] / "results"
seed = int(sys.argv[1]) if len(sys.argv) > 1 else 1

P = Params()
t0 = time.time()
log = run_sim(P, gen_noise(seed, P))
print(f"simulated {P.t_end:.0f} s of flight in {time.time() - t0:.1f} s wall time")

M = flight_metrics(log, P)
print(f"mass: wet {P.m_dry + P.mp0:.4f} kg, prop {P.mp0:.4f} kg, impulse {P.Itot:.1f} Ns")
print(f"apogee {M['apogee']:.1f} m at t={M['t_apogee']:.2f} s, burnout speed {M['v_burnout']:.1f} m/s")
print(f"boost tilt error: max {M['max_tilt_err_boost']:.2f} deg, rms {M['rms_tilt_err_boost']:.2f} deg")
print(f"max gimbal {M['max_gimbal_deg']:.2f} deg of {P.lim * 180 / 3.14159265:.0f}, "
      f"saturated {100 * M['sat_fraction']:.1f} % of boost")
print(f"attitude estimate error max {M['max_est_err']:.2f} deg")

out = RES / f"python_run_seed{seed}.png"
plot_results(log, P, out, f"Python simulation - nominal flight (seed {seed})")
print("figure:", out)
