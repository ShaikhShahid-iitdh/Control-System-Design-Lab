from __future__ import annotations

from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are

from config import A_EXACT, B_EXACT, Q_LQR, R_LQR, STATE_FEEDBACK_TEST_CASES, T_END, DT, PARAMS
from furuta_dynamics import (
    as_params,
    control_voltage,
    nonlinear_closed_loop_rhs,
)


def settling_time(t: np.ndarray, y: np.ndarray, tol: float = 0.02) -> float:
    if np.allclose(y, 0.0):
        return 0.0
    band = tol * np.max(np.abs(y))
    idx = np.where(np.abs(y) > band)[0]
    if len(idx) == 0:
        return 0.0
    if idx[-1] >= len(t) - 1:
        return float(t[-1])
    return float(t[idx[-1] + 1])


def compute_lqr_gain(a: np.ndarray, b: np.ndarray, q: np.ndarray, r: np.ndarray) -> np.ndarray:
    p = solve_continuous_are(a, b, q, r)
    return np.asarray(np.linalg.inv(r) @ b.T @ p).reshape(-1)


def simulate_linear(a_cl: np.ndarray, x0: np.ndarray, t_eval: np.ndarray) -> np.ndarray:
    def linear_rhs(t: float, x: np.ndarray) -> np.ndarray:
        _ = t
        return a_cl @ x

    solution = solve_ivp(
        linear_rhs,
        (t_eval[0], t_eval[-1]),
        x0,
        t_eval=t_eval,
        rtol=1e-8,
        atol=1e-10,
    )
    if not solution.success:
        raise RuntimeError(solution.message)
    return solution.y


def simulate_nonlinear(k_vec: np.ndarray, x0: np.ndarray, p, t_eval: np.ndarray) -> np.ndarray:
    solution = solve_ivp(
        lambda t, x: nonlinear_closed_loop_rhs(t, x, k_vec, p),
        (t_eval[0], t_eval[-1]),
        x0,
        t_eval=t_eval,
        rtol=1e-8,
        atol=1e-10,
        max_step=DT,
    )
    if not solution.success:
        raise RuntimeError(solution.message)
    return solution.y


def plot_quantity(t_eval: np.ndarray, linear_sets: list[np.ndarray], nonlinear_sets: list[np.ndarray], ylabel_top: str, ylabel_bottom: str, output_path: Path, use_degrees: bool = True) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    colors = plt.get_cmap("tab10").colors
    labels = [
        r"$\theta_p(0)=\pi/40$",
        r"$\theta_p(0)=\pi/10$",
        r"$\theta_p(0)=\pi/4$",
    ]

    for idx, (linear_states, nonlinear_states) in enumerate(zip(linear_sets, nonlinear_sets)):
        color = colors[idx]
        if use_degrees:
            axes[0].plot(t_eval, np.rad2deg(linear_states[0]), linestyle="--", color=color, label=f"Linear {labels[idx]}")
            axes[0].plot(t_eval, np.rad2deg(nonlinear_states[0]), linestyle="-", color=color, label=f"Nonlinear {labels[idx]}")
            axes[1].plot(t_eval, np.rad2deg(linear_states[1]), linestyle="--", color=color, label=f"Linear {labels[idx]}")
            axes[1].plot(t_eval, np.rad2deg(nonlinear_states[1]), linestyle="-", color=color, label=f"Nonlinear {labels[idx]}")
        else:
            axes[0].plot(t_eval, linear_states[0], linestyle="--", color=color, label=f"Linear {labels[idx]}")
            axes[0].plot(t_eval, nonlinear_states[0], linestyle="-", color=color, label=f"Nonlinear {labels[idx]}")
            axes[1].plot(t_eval, linear_states[1], linestyle="--", color=color, label=f"Linear {labels[idx]}")
            axes[1].plot(t_eval, nonlinear_states[1], linestyle="-", color=color, label=f"Nonlinear {labels[idx]}")

    axes[0].set_ylabel(ylabel_top)
    axes[1].set_ylabel(ylabel_bottom)
    axes[1].set_xlabel("Time (s)")
    axes[0].grid(True, alpha=0.3)
    axes[1].grid(True, alpha=0.3)
    axes[0].legend(fontsize=8, ncol=2)
    axes[1].legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def run() -> None:
    root = Path(__file__).resolve().parent.parent
    out_dir = root / "report" / "figures"
    out_dir.mkdir(parents=True, exist_ok=True)

    p = as_params(PARAMS)
    n_steps = int(round(T_END / DT))
    t_eval = np.linspace(0.0, T_END, n_steps + 1)

    k_vec = compute_lqr_gain(A_EXACT, B_EXACT, Q_LQR, R_LQR)
    a_cl = A_EXACT - B_EXACT @ k_vec.reshape(1, -1)

    linear_cases: list[np.ndarray] = []
    nonlinear_cases: list[np.ndarray] = []
    summary_lines = [
        "Closed-loop comparison summary",
        "==============================",
        f"LQR gain K: {k_vec}",
        f"Initial conditions used: {STATE_FEEDBACK_TEST_CASES}",
        "",
    ]

    for idx, x0 in enumerate(STATE_FEEDBACK_TEST_CASES, start=1):
        x_lin = simulate_linear(a_cl, x0, t_eval)
        x_nl = simulate_nonlinear(k_vec, x0, p, t_eval)
        linear_cases.append(x_lin)
        nonlinear_cases.append(x_nl)

        u_lin = np.array([control_voltage(x_lin[:, i], k_vec, None) for i in range(x_lin.shape[1])])
        u_nl = np.array([control_voltage(x_nl[:, i], k_vec, p.voltage_limit) for i in range(x_nl.shape[1])])

        max_diff = np.max(np.abs(x_nl - x_lin), axis=1)
        summary_lines.extend([
            f"Case {idx}: x0 = {x0}",
            f"  Max |nonlinear - linear| theta = {max_diff[0]:.6e} rad ({np.rad2deg(max_diff[0]):.4f} deg)",
            f"  Max |nonlinear - linear| alpha = {max_diff[1]:.6e} rad ({np.rad2deg(max_diff[1]):.4f} deg)",
            f"  Max |nonlinear - linear| theta_dot = {max_diff[2]:.6e} rad/s ({np.rad2deg(max_diff[2]):.4f} deg/s)",
            f"  Max |nonlinear - linear| alpha_dot = {max_diff[3]:.6e} rad/s ({np.rad2deg(max_diff[3]):.4f} deg/s)",
            f"  Linear alpha settling = {settling_time(t_eval, x_lin[1]):.4f} s",
            f"  Nonlinear alpha settling = {settling_time(t_eval, x_nl[1]):.4f} s",
            f"  Peak |u| linear = {np.max(np.abs(u_lin)):.4f} V",
            f"  Peak |u| nonlinear = {np.max(np.abs(u_nl)):.4f} V",
            "",
        ])

    plot_quantity(
        t_eval,
        [x[[0, 1], :] for x in linear_cases],
        [x[[0, 1], :] for x in nonlinear_cases],
        "Base angle theta (deg)",
        "Pendulum angle alpha (deg)",
        out_dir / "angles_compare.png",
        use_degrees=True,
    )

    plot_quantity(
        t_eval,
        [x[[2, 3], :] for x in linear_cases],
        [x[[2, 3], :] for x in nonlinear_cases],
        "Base angular rate theta_dot (deg/s)",
        "Pendulum angular rate alpha_dot (deg/s)",
        out_dir / "rates_compare.png",
        use_degrees=True,
    )

    fig, ax = plt.subplots(figsize=(10, 4.5))
    colors = plt.get_cmap("tab10").colors
    labels = [
        r"$\theta_p(0)=\pi/40$",
        r"$\theta_p(0)=\pi/10$",
        r"$\theta_p(0)=\pi/4$",
    ]
    for idx, (x_lin, x_nl) in enumerate(zip(linear_cases, nonlinear_cases)):
        u_lin = np.array([control_voltage(x_lin[:, i], k_vec, None) for i in range(x_lin.shape[1])])
        u_nl = np.array([control_voltage(x_nl[:, i], k_vec, p.voltage_limit) for i in range(x_nl.shape[1])])
        ax.plot(t_eval, u_lin, linestyle="--", color=colors[idx], label=f"Linear {labels[idx]}")
        ax.plot(t_eval, u_nl, linestyle="-", color=colors[idx], label=f"Nonlinear {labels[idx]}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Input voltage (V)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(out_dir / "voltage_compare.png", dpi=200)
    plt.close(fig)

    summary_lines.extend([
        "Notes:",
        "- Linear model uses the exact A, B matrices from the prerequisite report.",
        "- Nonlinear model is simulated with the same LQR gain and actuator saturation.",
    ])
    (root / "report" / "results_summary.txt").write_text("\n".join(summary_lines), encoding="utf-8")

    print("Simulation complete.")
    print(f"Saved plots in: {out_dir}")
    print("Saved summary in: report/results_summary.txt")


if __name__ == "__main__":
    run()
