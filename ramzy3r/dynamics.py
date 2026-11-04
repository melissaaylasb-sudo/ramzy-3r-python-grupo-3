"""Exploratory 3R dynamics derived from energy and published parameters.

See Ramzy et al. (2024), Fig. 1 and Table 1 (DH frames), Sec. 2.2,
Eq. (13) (kinetic energy), Eqs. (15)–(18) (potential and dissipation)
and Table 2 (values). This is a reconstruction with the assumptions below.

The CAD inertia diagonal is treated as being about the COM, expressed in
the link DH axes. COM offsets use the orientation of DH frame i and the
origin of frame i-1 (the proximal joint). This puts X2 and X3 within their
segments. Some Appendix A terms instead add l_i to X_i, so the authors'
intended origin and signs require confirmation. Y1 is vertical and Z1
lateral under this frame choice. Table 2 prints Y4=0 but omits Z2. We
assume Z2=0 and do not relabel Y4. All distances are in m and angles in rad.

The reported 1.66 and 1.3 values are interpreted as equivalent joint
torsional stiffness (N m/rad) and damping (N m s/rad) respectively. These
units are inferred. The elastic rest pose is q=0. Changing these scales
is a numerical sensitivity case without a calibrated pressure law.
"""

import numpy as np
from scipy.integrate import solve_ivp

from .kinematics import frames

MASS_KG = np.array([0.77189, 0.215, 0.14752])
COM_M = np.array([[0.0, 0.040, 0.0245], [0.157, 0.0, 0.0], [0.125, 0.0, 0.0]])
INERTIA_KG_M2 = np.array(
    [
        [2.47e-4, 1.83e-3, 1.86e-3],
        [3.82e-5, 1.60e-3, 1.59e-3],
        [4.26e-5, 7.45e-4, 7.25e-4],
    ]
)
GRAVITY_M_S2 = 9.81
STIFFNESS_NM_RAD = 1.66
DAMPING_NMS_RAD = 1.30


def _mass_and_gravity(q):
    """Compute M, the gravitational generalized force, and potential."""
    # Transposes deliberately omit conjugation to enable complex-step dM/dq.
    dtype = np.result_type(q, np.float64)
    transforms = frames(q)
    origins = transforms[:-1, :3, 3]
    axes = transforms[:-1, :3, 2]
    inertia = np.zeros((3, 3), dtype=dtype)
    gravity = np.zeros(3, dtype=dtype)
    potential = 0.0

    for i, transform in enumerate(transforms[1:]):
        rotation = transform[:3, :3]
        center = origins[i] + rotation @ COM_M[i]
        active = slice(0, i + 1)
        j_linear = np.cross(axes[active], center - origins[active]).T
        j_angular = axes[active].T
        world_inertia = rotation @ np.diag(INERTIA_KG_M2[i]) @ rotation.T
        inertia[active, active] += (
            MASS_KG[i] * j_linear.T @ j_linear
            + j_angular.T @ world_inertia @ j_angular
        )
        weight = MASS_KG[i] * GRAVITY_M_S2
        gravity[active] += weight * j_linear[2]
        potential += weight * center[2]

    return inertia, gravity, potential


def _velocity_bias(q, qdot):
    """Evaluate the quadratic Christoffel contraction from derivatives of M."""
    step = 1e-20
    derivatives = np.empty((3, 3, 3))
    for k in range(3):
        perturbed = q.astype(complex)
        perturbed[k] += 1j * step
        derivatives[:, :, k] = _mass_and_gravity(perturbed)[0].imag / step
    return (
        np.einsum("ijk,j,k->i", derivatives, qdot, qdot)
        - 0.5 * np.einsum("jki,j,k->i", derivatives, qdot, qdot)
    )


def simulate_dynamics(*, damping_scale=1.0, stiffness_scale=1.0, duration=3.0):
    """Simulate unforced motion with fixed initial state and check energy balance.

    Dynamics follow M qdd + velocity_bias + gravity + k*q + c*qdot = 0.
    The augmented state integrates c*qdot.T*qdot, the dissipated work.
    """
    if not np.isfinite(duration) or duration <= 0.0:
        raise ValueError("duration must be finite and positive")
    scales = np.array([damping_scale, stiffness_scale], dtype=float)
    if not np.all(np.isfinite(scales)) or np.any(scales < 0.0):
        raise ValueError("damping and stiffness scales must be finite and nonnegative")

    damping = DAMPING_NMS_RAD * damping_scale
    stiffness = STIFFNESS_NM_RAD * stiffness_scale
    initial = np.r_[np.deg2rad([20.0, 30.0, -40.0]), np.zeros(4)]
    times = np.linspace(0.0, duration, max(2, round(duration * 200) + 1))

    def derivative(_time, state):
        q, qdot = state[:3], state[3:6]
        inertia, gravity, _ = _mass_and_gravity(q)
        torque = -(_velocity_bias(q, qdot) + gravity + stiffness * q + damping * qdot)
        acceleration = np.linalg.solve(inertia, torque)
        return np.r_[qdot, acceleration, damping * np.dot(qdot, qdot)]

    # Equivalent damping may introduce a short time scale relative to motion.
    solution = solve_ivp(
        derivative, (0.0, duration), initial, t_eval=times,
        method="Radau", rtol=1e-8, atol=1e-10,
    )
    if not solution.success:
        raise RuntimeError(solution.message)
    q = solution.y[:3].T
    qdot = solution.y[3:6].T
    dissipated = solution.y[6]
    energy = np.empty(len(times))
    eigenvalues = np.empty(len(times))
    asymmetry = np.empty(len(times))
    for i, (position, velocity) in enumerate(zip(q, qdot)):
        inertia, _, potential = _mass_and_gravity(position)
        energy[i] = (
            0.5 * velocity @ inertia @ velocity
            + potential
            + 0.5 * stiffness * np.dot(position, position)
        )
        eigenvalues[i] = np.linalg.eigvalsh(inertia)[0]
        asymmetry[i] = np.max(np.abs(inertia - inertia.T))

    return {
        "t": solution.t,
        "q": q,
        "qdot": qdot,
        "energy_j": energy,
        "dissipated_j": dissipated,
        "balance_residual_j": energy + dissipated - energy[0],
        "min_mass_eigenvalue": float(eigenvalues.min()),
        "max_mass_asymmetry": float(asymmetry.max()),
        "damping_nm_s_rad": damping,
        "stiffness_nm_rad": stiffness,
    }
