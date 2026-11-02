import dataclasses
import math
from pathlib import Path

import numpy as np
import pytest

from rocket_tvc import Params, flight_metrics, gen_noise, load_noise, run_sim
from rocket_tvc.dynamics import initial_state, mass_props, plant_deriv
from rocket_tvc.quat import quat_to_dcm

RES = Path(__file__).resolve().parents[2] / "results"


def short(P, t_end=5.0):
    return dataclasses.replace(P, t_end=t_end)


def test_dcm_is_rotation():
    q = np.array([0.3, -0.5, 0.2, 0.7]); q /= np.linalg.norm(q)
    R = quat_to_dcm(q)
    assert np.allclose(R @ R.T, np.eye(3)) and math.isclose(np.linalg.det(R), 1.0)


def test_mass_properties():
    P = Params()
    assert mass_props(0.0, P)[1] == pytest.approx(P.m_dry + P.mp0)
    T, m, s_cg, I = mass_props(P.tc_t[-1] + 0.1, P)
    assert T == 0 and m == pytest.approx(P.m_dry) and s_cg == pytest.approx(P.s_dry)
    # CG moves forward during the burn (propellant is aft of the dry CG)
    assert mass_props(1.0, P)[2] > mass_props(2.3, P)[2]
    # numerical impulse equals the analytic total impulse
    ts = np.linspace(0, P.tc_t[-1], 20001)
    thrust = np.array([mass_props(t, P)[0] for t in ts])
    assert np.trapezoid(thrust, ts) == pytest.approx(P.Itot, rel=1e-4)


def test_motor_scaling_keeps_mass_flow():
    P = dataclasses.replace(Params(), T_scale=1.1)
    assert mass_props(1.0, P)[0] == pytest.approx(1.1 * mass_props(1.0, Params())[0])
    assert mass_props(1.0, P)[1] == pytest.approx(mass_props(1.0, Params())[1])


def test_pad_holds_before_ignition():
    P = Params()
    X = initial_state(P)
    Xd, fb = plant_deriv(0.2, X, np.zeros(2), P)
    assert np.allclose(Xd[:13], 0)
    assert fb[0] == pytest.approx(P.g0 * math.cos(P.tilt0), rel=1e-3)   # accel reads +g along the nose


def test_closed_loop_tracks_and_keeps_gimbal_inside_limits():
    P = short(Params())
    log = run_sim(P, gen_noise(3, P))
    M = flight_metrics(log, P)
    assert M["max_tilt_err_boost"] < 6.0
    assert M["max_gimbal_deg"] <= math.degrees(P.lim) + 1e-6
    assert M["max_est_err"] < 1.0


def test_open_loop_is_worse():
    P = short(Params(), 6.0)
    nz = gen_noise(3, P)
    cl = flight_metrics(run_sim(P, nz), P)
    ol = flight_metrics(run_sim(dataclasses.replace(P, ctrl_on=0), nz), P)
    assert ol["max_tilt_err_boost"] > 5 * cl["max_tilt_err_boost"]


def test_slow_flight_computer_destabilises():
    P = short(dataclasses.replace(Params(), Ts=0.06, a_f=1.0))
    M = flight_metrics(run_sim(P, gen_noise(3, P)), P)
    assert M["max_tilt_err_boost"] > 20.0


@pytest.mark.skipif(not (RES / "matlab_reference_log.csv").exists(), reason="run matlab/export_for_python.m first")
def test_matches_matlab_reference():
    P = Params()
    log = run_sim(P, load_noise(RES))
    ref = np.loadtxt(RES / "matlab_reference_log.csv", delimiter=",", skiprows=1)
    assert np.abs(log["X"] - ref[:, 1:16]).max() < 1e-8
    assert np.abs(log["cmd"] - ref[:, 25:27]).max() < 1e-10
