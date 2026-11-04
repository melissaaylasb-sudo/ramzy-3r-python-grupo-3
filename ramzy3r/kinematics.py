"""Modelo nominal 3R (Tabla 1, Tabla 2 y ecuaciones 6–8). SI y radianes."""
from dataclasses import dataclass

import numpy as np

L1, L2, L3 = 0.040, 0.290, 0.240


def dh(theta, alpha, a, d):
    """Transformación DH estándar Rz(theta) Tz(d) Tx(a) Rx(alpha)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0., sa, ca, d],
        [0., 0., 0., 1.],
    ])


def frames(q):
    """Marcos [base, 1, 2, 3] expresados en la base."""
    q1, q2, q3 = q
    t1 = dh(q1, np.pi / 2, 0., L1)
    t2 = t1 @ dh(q2, 0., L2, 0.)
    t3 = t2 @ dh(q3, 0., L3, 0.)
    return np.stack([np.eye(4), t1, t2, t3])


def fk(q):
    """Posición del efector nominal en metros, por expresión cerrada."""
    q1, q2, q3 = q
    r = L2 * np.cos(q2) + L3 * np.cos(q2 + q3)
    z = L1 + L2 * np.sin(q2) + L3 * np.sin(q2 + q3)
    return np.array([r * np.cos(q1), r * np.sin(q1), z])


def jacobian(q):
    """Jacobiano traslacional en m/rad."""
    q1, q2, q3 = q
    c1, s1 = np.cos(q1), np.sin(q1)
    r = L2 * np.cos(q2) + L3 * np.cos(q2 + q3)
    h = L2 * np.sin(q2) + L3 * np.sin(q2 + q3)
    s3 = L3 * np.sin(q2 + q3)
    c3 = L3 * np.cos(q2 + q3)
    return np.array([[-r * s1, -h * c1, -s3 * c1],
                     [r * c1, -h * s1, -s3 * s1],
                     [0., r, c3]])


def jacobian_fd(q, step=1e-6):
    """Estimación independiente por diferencias centrales."""
    q = np.asarray(q, dtype=float)
    offsets = step * np.eye(3)
    return np.column_stack([(fk(q + dq) - fk(q - dq)) / (2 * step)
                            for dq in offsets])


def wrap(q):
    return (np.asarray(q) + np.pi) % (2 * np.pi) - np.pi


@dataclass(frozen=True)
class IKSolution:
    q: np.ndarray
    radial_sign: int
    elbow_sign: int
    base_free: bool


def inverse_kinematics(position, tol=1e-12):
    """Soluciones de posición sin límites articulares; lista vacía si inalcanzable."""
    x, y, z = np.asarray(position, dtype=float)
    if not np.all(np.isfinite([x, y, z])):
        raise ValueError("El objetivo debe tener coordenadas finitas.")
    rho = np.hypot(x, y)
    h = z - L1
    d = (rho**2 + h**2 - L2**2 - L3**2) / (2 * L2 * L3)
    if d < -1. - tol or d > 1. + tol:
        return []
    if abs(abs(d) - 1.) <= tol:
        d = np.copysign(1., d)
    s = np.sqrt(max(0., 1. - d * d))
    on_axis = rho <= tol
    angle = 0. if on_axis else np.arctan2(y, x)
    solutions = []
    for radial_sign in (1,) if on_axis else (1, -1):
        q1 = angle if radial_sign == 1 else angle + np.pi
        r = radial_sign * rho
        for elbow_sign in (1,) if s == 0. else (1, -1):
            q3 = np.arctan2(elbow_sign * s, d)
            q2 = np.arctan2(h, r) - np.arctan2(L3 * np.sin(q3), L2 + L3 * d)
            solutions.append(IKSolution(wrap([q1, q2, q3]), radial_sign,
                                        elbow_sign if s > 0. else 0, on_axis))
    return solutions


def determinant(q):
    _, q2, q3 = q
    r = L2 * np.cos(q2) + L3 * np.cos(q2 + q3)
    return -L2 * L3 * r * np.sin(q3)


# Tolerancia relativa única del proyecto: un valor singular con
# sigma_i <= SINGULAR_RTOL * sigma_1 se trata como cero numérico, tanto al
# declarar el rango como al truncar la pseudoinversa del control.
SINGULAR_RTOL = 1e-10


def singular_values(q):
    """Valores singulares exactos del jacobiano traslacional, en m/rad.

    J = Rz(q1) @ [[0, -h, -a], [r, 0, 0], [0, r, b]], con a = L3 sin(q2+q3)
    y b = L3 cos(q2+q3). La primera columna es ortogonal a las otras dos, de
    modo que un valor singular es |r| (dirección tangencial, eje de la base)
    y los otros dos son los del bloque plano 2R [[-h, -a], [r, b]], cuya
    norma de Frobenius al cuadrado T y determinante D dependen solo de q3.

    Devuelve (sigma_1, sigma_2, sigma_3) de mayor a menor. sigma_codo se
    obtiene como D / sigma_mas para no perder precisión relativa cerca de
    q3 = 0 o q3 = pi, donde la SVD numérica queda limitada por eps * sigma_1.
    """
    sigma_base, sigma_plus, sigma_elbow = singular_value_families(q)
    return np.sort([sigma_base, sigma_plus, sigma_elbow])[::-1]


def singular_value_families(q):
    """(|r|, sigma_mas, sigma_codo): valores singulares según su origen.

    Acepta q2 y q3 escalares o arreglos (q1 no interviene), para construir
    mapas sin recorrer la malla en Python.
    """
    _, q2, q3 = q
    r = L2 * np.cos(q2) + L3 * np.cos(q2 + q3)
    t = L2**2 + 2 * L3**2 + 2 * L2 * L3 * np.cos(q3)
    d = L2 * L3 * np.abs(np.sin(q3))
    gap = np.sqrt(np.maximum(t * t - 4 * d * d, 0.))
    sigma_plus = np.sqrt(0.5 * (t + gap))
    return np.abs(r), sigma_plus, d / sigma_plus


def numerical_rank(sigma, rtol=SINGULAR_RTOL):
    """Rango con tolerancia relativa al mayor valor singular."""
    sigma = np.asarray(sigma, dtype=float)
    if sigma.size == 0 or sigma.max() == 0.:
        return 0
    return int(np.count_nonzero(sigma > rtol * sigma.max()))


def singularity_measures(q, near_threshold=0.020):
    """Indicadores continuos de cercanía a singularidad en la pose q.

    near_threshold, en m/rad, marca la zona donde sigma_min ya es comparable
    al amortiguamiento del control DLS (0.020 m/rad por defecto). La clave
    "familia" vale "ninguna" fuera de esa zona y, dentro de ella, indica qué
    familia limita: "codo" (sin q3 = 0), "eje_base" (r = 0) o "interseccion".
    """
    sigma_base, sigma_plus, sigma_elbow = singular_value_families(q)
    sigma = np.sort([sigma_base, sigma_plus, sigma_elbow])[::-1]
    sigma_min = sigma[-1]
    rank = numerical_rank(sigma)
    zero = SINGULAR_RTOL * sigma[0]
    if sigma_base <= zero and sigma_elbow <= zero:
        family = "interseccion"
    elif rank < 3 or sigma_min < near_threshold:
        family = "eje_base" if sigma_base < sigma_elbow else "codo"
    else:
        family = "ninguna"
    return {
        "sigma": sigma,
        "sigma_eje_base": sigma_base,
        "sigma_codo": sigma_elbow,
        "sigma_min": sigma_min,
        "manipulabilidad": sigma.prod(),
        "condicion": np.inf if sigma_min == 0. else sigma[0] / sigma_min,
        "rango": rank,
        "familia": family,
    }


def singular_cases():
    return {
        "regular": np.deg2rad([30., 60., -60.]),
        "codo_extend": np.array([0.2, 0.3, 0.]),
        "codo_plegado": np.array([0.2, 0.3, np.pi]),
        "eje_base": np.array([0.2, np.arctan2(L2, L3), np.pi / 2]),
        "interseccion": np.array([0.2, np.pi / 2, 0.]),
    }
