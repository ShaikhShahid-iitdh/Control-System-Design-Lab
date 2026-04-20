import numpy as np

# Exact linearized model and LQR weights reported in the prerequisite exercise.
A_EXACT = np.array(
    [[0.0, 0.0, 1.0, 0.0],
     [0.0, 0.0, 0.0, 1.0],
     [0.0, 123.98, -1.577, 0.0],
     [0.0, 111.62, -0.7253, 0.0]],
    dtype=float,
)

B_EXACT = np.array([[0.0], [0.0], [56.38], [25.98]], dtype=float)

Q_LQR = np.diag([5.0, 300.0, 1.0, 20.0])
R_LQR = np.array([[5.0]], dtype=float)

STATE_FEEDBACK_TEST_CASES = np.array(
    [
        [0.0, np.pi / 40.0, 0.0, 0.0],
        [0.0, np.pi / 10.0, 0.0, 0.0],
        [0.0, np.pi / 4.0, 0.0, 0.0],
    ],
    dtype=float,
)

T_END = 8.0
DT = 0.002

# Furuta pendulum parameters (update with your laboratory setup values if different).
PARAMS = {
    "g": 9.81,
    "mp": 0.127,
    "Lr": 0.085,
    "lp": 0.129,
    "Jr": 1.20e-3,
    "Jp": 1.00e-3,
    "br": 2.4e-3,
    "bp": 2.4e-3,
    "km": 0.042,
    "voltage_limit": 10.0,
}
