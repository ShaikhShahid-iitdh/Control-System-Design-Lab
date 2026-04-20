from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class FurutaParams:
    g: float
    mp: float
    Lr: float
    lp: float
    Jr: float
    Jp: float
    br: float
    bp: float
    km: float
    voltage_limit: float


def as_params(d: dict) -> FurutaParams:
    return FurutaParams(**d)


def control_voltage(x: np.ndarray, k: np.ndarray, voltage_limit: float | None = None) -> float:
    v = float(-k @ x)
    if voltage_limit is None:
        return v
    return float(np.clip(v, -voltage_limit, voltage_limit))


def furuta_open_loop_dynamics(x: np.ndarray, u: float, p: FurutaParams) -> np.ndarray:
    theta, alpha, theta_dot, alpha_dot = x

    m11 = p.Jr + p.mp * (p.Lr**2 + (p.lp**2) * (np.sin(alpha) ** 2))
    m12 = p.mp * p.Lr * p.lp * np.cos(alpha)
    m22 = p.Jp + p.mp * (p.lp**2)

    h1 = -p.mp * p.Lr * p.lp * np.sin(alpha) * (alpha_dot**2)
    h2 = -p.mp * p.g * p.lp * np.sin(alpha)

    tau = p.km * u

    rhs1 = tau - p.br * theta_dot - h1
    rhs2 = -p.bp * alpha_dot - h2

    det_m = m11 * m22 - m12 * m12
    theta_ddot = (m22 * rhs1 - m12 * rhs2) / det_m
    alpha_ddot = (-m12 * rhs1 + m11 * rhs2) / det_m

    return np.array([theta_dot, alpha_dot, theta_ddot, alpha_ddot], dtype=float)


def nonlinear_closed_loop_rhs(t: float, x: np.ndarray, k: np.ndarray, p: FurutaParams) -> np.ndarray:
    _ = t
    u = control_voltage(x, k, p.voltage_limit)
    return furuta_open_loop_dynamics(x, u, p)


def numerical_linearization(p: FurutaParams, eps: float = 1e-6) -> tuple[np.ndarray, np.ndarray]:
    x_eq = np.zeros(4, dtype=float)
    u_eq = 0.0

    a = np.zeros((4, 4), dtype=float)
    b = np.zeros((4, 1), dtype=float)

    for i in range(4):
        dx = np.zeros(4, dtype=float)
        dx[i] = eps
        f_plus = furuta_open_loop_dynamics(x_eq + dx, u_eq, p)
        f_minus = furuta_open_loop_dynamics(x_eq - dx, u_eq, p)
        a[:, i] = (f_plus - f_minus) / (2.0 * eps)

    f_plus_u = furuta_open_loop_dynamics(x_eq, u_eq + eps, p)
    f_minus_u = furuta_open_loop_dynamics(x_eq, u_eq - eps, p)
    b[:, 0] = (f_plus_u - f_minus_u) / (2.0 * eps)

    return a, b
