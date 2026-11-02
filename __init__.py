"""6-DOF simulation and GNC for a thrust-vector-controlled model rocket.

Python counterpart of the MATLAB/Simulink model in ../matlab. The two share
equations, parameters and noise sequences, so results can be compared sample by
sample (see scripts/compare_with_matlab.py).
"""
from .params import Params
from .sim import run_sim, gen_noise, load_noise
from .metrics import flight_metrics

__all__ = ["Params", "run_sim", "gen_noise", "load_noise", "flight_metrics"]
