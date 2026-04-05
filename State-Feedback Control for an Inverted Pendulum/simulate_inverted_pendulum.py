import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import place_poles


@dataclass
class ModelData:
    A: np.ndarray
    B: np.ndarray
    state_names: List[str]
    voltage_limit: float


def load_linearized_model() -> ModelData:
    """
    Linearized model around the unstable (upright) equilibrium.

    State vector convention used below:
    x = [theta_base, theta_pendulum, theta_base_dot, theta_pendulum_dot]^T

    Values are taken from the previous lab report (Section 8.2).
    """
    A = np.array(
        [
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 123.98, -1.577, 0.0],
            [0.0, 111.62, -0.7253, 0.0],
        ]
    )

    B = np.array([[0.0], [0.0], [56.38], [25.98]])

    state_names = [
        "Base angle (rad)",
        "Pendulum angle from upright (rad)",
        "Base angular velocity (rad/s)",
        "Pendulum angular velocity (rad/s)",
    ]

    voltage_limit = 12.0

    return ModelData(A=A, B=B, state_names=state_names, voltage_limit=voltage_limit)


def candidate_pole_sets() -> Dict[str, np.ndarray]:
    """Reasonable pole sets from conservative to aggressive."""
    return {
        "conservative": np.array([-2.0, -2.6, -3.2, -4.0]),
        "moderate": np.array([-3.5, -4.2, -5.0, -6.0]),
        "fast": np.array([-5.0, -6.0, -7.5, -9.0]),
        "very_fast": np.array([-7.0, -8.5, -10.0, -12.0]),
    }


def design_gain(A: np.ndarray, B: np.ndarray, poles: np.ndarray) -> np.ndarray:
    placed = place_poles(A, B, poles)
    return placed.gain_matrix


def simulate_linear_closed_loop(
    A: np.ndarray,
    B: np.ndarray,
    K: np.ndarray,
    x0: np.ndarray,
    t_final: float = 5.0,
    dt: float = 0.002,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    Acl = A - B @ K

    def dyn(_t: float, x: np.ndarray) -> np.ndarray:
        return Acl @ x

    t_eval = np.arange(0.0, t_final + dt, dt)
    sol = solve_ivp(dyn, (0.0, t_final), x0, t_eval=t_eval, rtol=1e-7, atol=1e-9)

    x = sol.y.T
    u = -(x @ K.T).flatten()
    return sol.t, x, u


def nonlinear_dynamics_placeholder(_t: float, x: np.ndarray, u: float) -> np.ndarray:
    """
    Furuta pendulum nonlinear dynamics in coupled form:

    [a, b cos(alpha)] [theta_ddot] = [tau + b alpha_dot^2 sin(alpha) - b_eq theta_dot]
    [b cos(alpha), c] [alpha_ddot]   [d sin(alpha)]

    The constants are identified so that linearization near alpha=0 matches the
    A and B matrices used in this script.
    """
    theta, alpha, theta_dot, alpha_dot = x

    # Match local linear mapping [theta_ddot, alpha_ddot]^T = invM * [tau, d*alpha]^T
    # with B and A angle-coupling entries from the report.
    inv_m11 = 56.38
    inv_m21 = 25.98
    inv_m12 = inv_m21
    d = 123.98 / inv_m12
    inv_m22 = 111.62 / d

    inv_m = np.array([[inv_m11, inv_m12], [inv_m21, inv_m22]], dtype=float)
    m0 = np.linalg.inv(inv_m)
    a = float(m0[0, 0])
    b = float(m0[0, 1])
    c = float(m0[1, 1])

    # Viscous term estimated from A33 so local damping matches linear model.
    b_eq = 1.577 / inv_m11

    mass_matrix = np.array(
        [[a, b * np.cos(alpha)], [b * np.cos(alpha), c]],
        dtype=float,
    )
    rhs = np.array(
        [
            u + b * (alpha_dot**2) * np.sin(alpha) - b_eq * theta_dot,
            d * np.sin(alpha),
        ],
        dtype=float,
    )
    theta_ddot, alpha_ddot = np.linalg.solve(mass_matrix, rhs)

    return np.array([theta_dot, alpha_dot, theta_ddot, alpha_ddot], dtype=float)


def simulate_nonlinear_closed_loop(
    K: np.ndarray,
    x0: np.ndarray,
    dyn_fn: Callable[[float, np.ndarray, float], np.ndarray],
    t_final: float = 5.0,
    dt: float = 0.002,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    def dyn(t: float, x: np.ndarray) -> np.ndarray:
        u = float(-(K @ x).item())
        return dyn_fn(t, x, u)

    def divergence_event(_t: float, x: np.ndarray) -> float:
        # Stop integration when trajectories leave a physically meaningful range.
        return 25.0 - float(np.max(np.abs(x)))

    divergence_event.terminal = True
    divergence_event.direction = -1

    t_eval = np.arange(0.0, t_final + dt, dt)
    sol = solve_ivp(
        dyn,
        (0.0, t_final),
        x0,
        t_eval=t_eval,
        rtol=1e-6,
        atol=1e-8,
        method="LSODA",
        events=divergence_event,
    )

    x = sol.y.T
    u = np.array([float(-(K @ xi).item()) for xi in x])
    return sol.t, x, u


def settling_time(t: np.ndarray, y: np.ndarray, tol: float = 0.02) -> float:
    final = y[-1]
    band = tol * max(1e-6, abs(final) + 1e-6)
    idx = np.where(np.abs(y - final) > band)[0]
    if len(idx) == 0:
        return 0.0
    last = idx[-1]
    if last == len(t) - 1:
        return float(t[-1])
    return float(t[last + 1])


def score_response(
    t: np.ndarray,
    x: np.ndarray,
    u: np.ndarray,
    voltage_limit: float,
) -> Dict[str, float]:
    pend = x[:, 1]
    st = settling_time(t, pend)
    max_angle = float(np.max(np.abs(pend)))
    max_voltage = float(np.max(np.abs(u)))
    stable = float(np.all(np.isfinite(x)) and np.max(np.abs(x)) < 50.0)
    within_voltage = float(max_voltage <= voltage_limit)

    return {
        "settling_time": st,
        "max_abs_pendulum_angle": max_angle,
        "max_abs_voltage": max_voltage,
        "stable": stable,
        "within_voltage": within_voltage,
    }


def optimize_poles(model: ModelData) -> Tuple[str, np.ndarray, np.ndarray, Dict[str, float]]:
    tests = candidate_pole_sets()
    x0 = np.array([0.0, np.pi / 10.0, 0.0, 0.0])

    best_name = ""
    best_poles = None
    best_K = None
    best_metrics = None
    best_cost = np.inf

    for name, poles in tests.items():
        K = design_gain(model.A, model.B, poles)
        t, x, u = simulate_linear_closed_loop(model.A, model.B, K, x0)
        m = score_response(t, x, u, model.voltage_limit)

        penalty = 0.0
        if m["within_voltage"] < 0.5:
            penalty += 100.0 * (m["max_abs_voltage"] - model.voltage_limit)
        if m["stable"] < 0.5:
            penalty += 1e6

        cost = 3.0 * m["settling_time"] + 5.0 * m["max_abs_pendulum_angle"] + penalty
        if cost < best_cost:
            best_cost = cost
            best_name = name
            best_poles = poles
            best_K = K
            best_metrics = m

    return best_name, best_poles, best_K, best_metrics


def ensure_outputs_dir() -> Path:
    out = Path(__file__).parent / "outputs"
    out.mkdir(parents=True, exist_ok=True)
    return out


def plot_state_comparison(
    t_lin: np.ndarray,
    x_lin: np.ndarray,
    t_non: np.ndarray,
    x_non: np.ndarray,
    title_suffix: str,
    out_path: Path,
) -> None:
    plt.figure(figsize=(10, 5))
    plt.plot(t_lin, x_lin[:, 0], "b-", label="linear")
    plt.plot(t_non, x_non[:, 0], "r--", label="nonlinear")
    plt.plot(t_lin, x_lin[:, 1], "g-", label="linear pendulum")
    plt.plot(t_non, x_non[:, 1], "m--", label="nonlinear pendulum")
    plt.grid(True, alpha=0.3)
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (rad)")
    plt.title(f"Base and Pendulum Angles ({title_suffix})")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=160)
    plt.close()


def plot_velocity_comparison(
    t_lin: np.ndarray,
    x_lin: np.ndarray,
    t_non: np.ndarray,
    x_non: np.ndarray,
    title_suffix: str,
    out_path: Path,
) -> None:
    plt.figure(figsize=(10, 5))
    plt.plot(t_lin, x_lin[:, 2], "b-", label="linear base vel")
    plt.plot(t_non, x_non[:, 2], "r--", label="nonlinear base vel")
    plt.plot(t_lin, x_lin[:, 3], "g-", label="linear pendulum vel")
    plt.plot(t_non, x_non[:, 3], "m--", label="nonlinear pendulum vel")
    plt.grid(True, alpha=0.3)
    plt.xlabel("Time (s)")
    plt.ylabel("Angular velocity (rad/s)")
    plt.title(f"Angular Velocities ({title_suffix})")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=160)
    plt.close()


def plot_voltage_comparison(
    t_lin: np.ndarray,
    u_lin: np.ndarray,
    t_non: np.ndarray,
    u_non: np.ndarray,
    voltage_limit: float,
    title_suffix: str,
    out_path: Path,
) -> None:
    plt.figure(figsize=(10, 5))
    plt.plot(t_lin, u_lin, "b-", label="linear")
    plt.plot(t_non, u_non, "r--", label="nonlinear")
    plt.axhline(voltage_limit, color="k", linestyle=":", linewidth=1.2, label="+limit")
    plt.axhline(-voltage_limit, color="k", linestyle=":", linewidth=1.2, label="-limit")
    plt.grid(True, alpha=0.3)
    plt.xlabel("Time (s)")
    plt.ylabel("Input voltage (V)")
    plt.title(f"Control Voltage ({title_suffix})")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=160)
    plt.close()


def save_summary_csv(rows: List[Dict[str, float]], out_path: Path) -> None:
    headers = [
        "case",
        "model",
        "settling_time",
        "max_abs_pendulum_angle",
        "max_abs_voltage",
        "stable",
        "within_voltage",
    ]
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writeheader()
        for r in rows:
            writer.writerow(r)


def main() -> None:
    model = load_linearized_model()
    best_name, poles, K, best_metrics = optimize_poles(model)

    print("Selected design:", best_name)
    print("Poles:", poles)
    print("K:", K)
    print("Metrics at theta0=pi/10:", best_metrics)

    out_dir = ensure_outputs_dir()

    initial_conditions = {
        "pi_40": np.array([0.0, np.pi / 40.0, 0.0, 0.0]),
        "pi_10": np.array([0.0, np.pi / 10.0, 0.0, 0.0]),
        "pi_4": np.array([0.0, np.pi / 4.0, 0.0, 0.0]),
    }

    all_rows: List[Dict[str, float]] = []

    for tag, x0 in initial_conditions.items():
        print(f"Simulating case: {tag}")
        t_lin, x_lin, u_lin = simulate_linear_closed_loop(model.A, model.B, K, x0)
        m_lin = score_response(t_lin, x_lin, u_lin, model.voltage_limit)
        all_rows.append({"case": tag, "model": "linear", **m_lin})

        t_non, x_non, u_non = simulate_nonlinear_closed_loop(K, x0, nonlinear_dynamics_placeholder)
        m_non = score_response(t_non, x_non, u_non, model.voltage_limit)
        all_rows.append({"case": tag, "model": "nonlinear", **m_non})

        plot_state_comparison(
            t_lin,
            x_lin,
            t_non,
            x_non,
            title_suffix=tag,
            out_path=out_dir / f"angles_{tag}.png",
        )
        plot_velocity_comparison(
            t_lin,
            x_lin,
            t_non,
            x_non,
            title_suffix=tag,
            out_path=out_dir / f"velocities_{tag}.png",
        )
        plot_voltage_comparison(
            t_lin,
            u_lin,
            t_non,
            u_non,
            voltage_limit=model.voltage_limit,
            title_suffix=tag,
            out_path=out_dir / f"voltage_{tag}.png",
        )

    save_summary_csv(all_rows, out_dir / "summary_metrics.csv")
    print("Saved plots and summary to:", out_dir)


if __name__ == "__main__":
    main()
