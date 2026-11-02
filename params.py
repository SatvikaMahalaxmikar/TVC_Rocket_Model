"""Vehicle, motor, actuator, sensor and GNC parameters (SI units).

Frames
    Inertial : x east, y north, z up (flat Earth), origin at the pad.
    Body     : x forward along the nose, y/z complete a right-handed set.
               At lift-off body-x points along inertial +z.
    Station  : s = distance from the nose tip, measured aft [m].
Quaternions are [w, x, y, z] and rotate body -> inertial.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

deg = math.pi / 180.0


def _arr(*v):
    return field(default_factory=lambda: np.array(v, dtype=float))


@dataclass
class Params:
    g0: float = 9.80665

    # timing
    dt: float = 1e-3          # plant integration step (fixed-step RK4)
    Ts: float = 1e-2          # IMU / flight-computer sample time (100 Hz)
    t_ign: float = 1.0        # ignition time (pad phase before is used for IMU calibration)
    t_end: float = 13.0

    # geometry and aerodynamics
    L: float = 0.80
    d: float = 0.060
    s_cp: float = 0.475
    CNa: float = 3.0
    CA: float = 0.55
    Cmq: float = -4.0
    Clp: float = -15.0        # roll damping from small fixed fins; roll is NOT actively controlled
    rho0: float = 1.225
    H: float = 8500.0
    s_eng: float = 0.78       # gimbal pivot station

    # mass properties
    m_dry: float = 0.75
    s_dry: float = 0.46
    Iyy_dry: float = 0.040
    Ixx: float = 6e-4
    s_prop: float = 0.74

    # motor: representative G-class thrust curve (piecewise linear)
    tc_t: np.ndarray = _arr(0, 0.05, 0.15, 0.5, 1.5, 2.1, 2.3, 2.4)
    tc_T: np.ndarray = _arr(0, 95, 75, 62, 55, 45, 20, 0)
    Isp: float = 170.0
    T_scale: float = 1.0      # thrust multiplier (motor variability)

    # environment
    wind_ss: np.ndarray = _arr(3, 0, 0)
    gust_amp: np.ndarray = _arr(0, 4, 0)
    gust_t0: float = 1.6
    gust_T: float = 1.0

    # build errors (unknown to the controller)
    eps: np.ndarray = _arr(0.4 * deg, -0.3 * deg)     # thrust misalignment pitch/yaw
    cg_off: np.ndarray = _arr(0, 0.001, 0.0005)       # lateral CG offset [m]

    # initial condition
    tilt0: float = 2 * deg
    az0: float = 30 * deg

    # TVC actuator
    lim: float = 7 * deg
    rate: float = 250 * deg
    wn_s: float = 2 * math.pi * 10

    # IMU
    sig_g: float = 0.010
    bsig_g: float = 0.005
    sig_a: float = 0.15
    bsig_a: float = 0.05
    gyro_fs: float = 35.0

    # guidance
    guid_mode: int = 1        # 0 = hold vertical, 1 = pitch-over / return
    pitch_c: float = 8 * deg
    az_c: float = 0.0
    t_kick: float = 0.8
    T_kick: float = 0.5
    t_ret: float = 1.6

    # flight computer
    ctrl_on: int = 1
    t_cal: float = 0.5
    a_launch: float = 25.0
    a_burnout: float = 8.0
    a_f: float = 0.3
    fx_min: float = 15.0
    wn_c: float = 12.0
    zeta: float = 0.8
    Ki: float = 250.0
    ie_max: float = 0.05

    # derived (recomputed by __post_init__, also after dataclasses.replace)
    Aref: float = field(init=False)
    Itot: float = field(init=False)
    mp0: float = field(init=False)
    Kp: float = field(init=False)
    Kd: float = field(init=False)

    def __post_init__(self):
        self.Aref = math.pi * self.d ** 2 / 4
        self.Itot = float(np.sum(np.diff(self.tc_t) * (self.tc_T[:-1] + self.tc_T[1:]) / 2))
        self.mp0 = self.Itot / (self.Isp * self.g0)
        self.Kp = self.wn_c ** 2
        self.Kd = 2 * self.zeta * self.wn_c
