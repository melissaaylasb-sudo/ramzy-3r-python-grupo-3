"""Experimento de seguimiento cartesiano en una planta cinemática ideal."""

from collections.abc import Callable

import numpy as np
from numpy.typing import NDArray

VectorMap = Callable[[NDArray[np.float64]], NDArray[np.float64]]


def _joint_reference(t, duration, case):
    """Trayectoria periódica alcanzable y su derivada analítica, en rad y rad/s."""
    omega = 2 * np.pi / duration
    phase = omega * t
    if case == "regular":
        elbow_center, elbow_amplitude = -1.05, -0.23
    elif case == "near_singular":
        elbow_center, elbow_amplitude = -0.61, -0.60
    else:
        raise ValueError(f"Caso desconocido: {case}")

    angles = np.column_stack(
        (
            0.35 + 0.35 * np.sin(phase),
            0.35 + 0.15 * np.sin(phase + 0.7),
            elbow_center + elbow_amplitude * np.cos(phase),
        )
    )
    rates = omega * np.column_stack(
        (
            0.35 * np.cos(phase),
            0.15 * np.cos(phase + 0.7),
            -elbow_amplitude * np.sin(phase),
        )
    )
    return angles, rates


def simulate_control(
    fk: VectorMap,
    jac: VectorMap,
    case: str = "regular",
    method: str = "pinv",
    *,
    dt: float = 0.01,
    measurement_bias_m: float = 0.0,
) -> dict:
    """Compara control por velocidad resuelta con pseudoinversa o amortiguamiento.

    fk(q) devuelve la posición XYZ en metros. jac(q) devuelve el jacobiano de
    posición 3×3 en m/rad. El estado articular se integra por Euler explícito.
    Se aplica el mismo error inicial y límite de velocidad a ambos métodos.
    La referencia cercana a singularidad llega a q3=-0.01 rad en t=4 s.
    Un sesgo negativo opcional en la medición de x alcanza su máximo en
    t=4 s y permite estudiar sensibilidad sin alterar la posición real.
    """
    if method not in {"pinv", "dls"}:
        raise ValueError(f"Método desconocido: {method}")
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("dt debe ser positivo y finito")
    if not np.isfinite(measurement_bias_m) or measurement_bias_m < 0:
        raise ValueError("measurement_bias_m debe ser finito y no negativo")

    duration = 8.0
    gain = 6.0  # 1/s
    damping = 0.020  # m/rad
    speed_limit = 0.60  # rad/s por articulación
    initial_offset = np.deg2rad([3.0, -3.0, 4.0])
    relative_svd_cutoff = 1e-12
    n_steps = int(round(duration / dt))
    if n_steps < 1 or not np.isclose(n_steps * dt, duration, rtol=0, atol=1e-12):
        raise ValueError("dt debe dividir el horizonte de 8 s")

    t = np.linspace(0.0, duration, n_steps + 1)
    q_reference, qdot_reference = _joint_reference(t, duration, case)
    desired = np.array([fk(q_ref) for q_ref in q_reference])
    reference_jacobians = np.array([jac(q_ref) for q_ref in q_reference])
    desired_velocity = (reference_jacobians @ qdot_reference[..., None])[..., 0]
    desired_sigma_min = np.linalg.svd(reference_jacobians, compute_uv=False)[:, -1]
    bias_phase = np.clip((t - duration / 2) / 0.4, -1.0, 1.0)
    measurement_bias = np.zeros_like(desired)
    measurement_bias[:, 0] = -0.5 * measurement_bias_m * (1 + np.cos(np.pi * bias_phase))
    q = np.empty_like(q_reference)
    actual = np.empty_like(desired)
    qdot = np.empty_like(q)
    qdot_requested = np.empty_like(q)
    sigma_min = np.empty_like(t)
    q[0] = q_reference[0] + initial_offset

    for i in range(n_steps + 1):
        qi = q[i]
        actual[i] = fk(qi)
        u, s, vh = np.linalg.svd(jac(qi))
        sigma_min[i] = s[-1]
        if method == "dls":
            inverse_weights = s / (s * s + damping * damping)
        else:
            inverse_weights = np.divide(
                1.0,
                s,
                out=np.zeros_like(s),
                where=s > relative_svd_cutoff * s[0],
            )

        measured = actual[i] + measurement_bias[i]
        velocity = desired_velocity[i] + gain * (desired[i] - measured)
        qdot_requested[i] = vh.T @ (inverse_weights * (u.T @ velocity))
        qdot[i] = np.clip(qdot_requested[i], -speed_limit, speed_limit)
        if i < n_steps:
            q[i + 1] = qi + dt * qdot[i]

    return {
        "case": case,
        "method": method,
        "dt": dt,
        "gain": gain,
        "damping": damping if method == "dls" else 0.0,
        "speed_limit": speed_limit,
        "measurement_bias_m": measurement_bias_m,
        "measurement_bias": measurement_bias,
        "initial_offset_rad": initial_offset,
        "relative_svd_cutoff": relative_svd_cutoff,
        "t": t,
        "q": q,
        "q_reference": q_reference,
        "qdot": qdot,
        "qdot_requested": qdot_requested,
        "actual": actual,
        "desired": desired,
        "desired_velocity": desired_velocity,
        "error": np.linalg.norm(desired - actual, axis=1),
        "sigma_min": sigma_min,
        "desired_sigma_min": desired_sigma_min,
        "saturated": np.any(np.abs(qdot_requested) > speed_limit, axis=1),
    }
