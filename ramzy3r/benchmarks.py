"""Auditoría por componente de las tablas publicadas. Los blancos se preservan."""
import csv
from pathlib import Path

import numpy as np

from .kinematics import fk

DATA = Path(__file__).resolve().parents[1] / "data"


def rows(name):
    with (DATA / name).open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def xyz(records, prefix):
    return np.array([[float(row[key]) if row[key] else np.nan
                      for key in (f"{prefix}_{axis}_mm" for axis in "xyz")]
                     for row in records])


def compare(pred, reference):
    residual = pred - reference
    metrics = {}
    for axis, column in zip("xyz", residual.T):
        valid = column[np.isfinite(column)]
        metrics[axis] = {"n": int(valid.size),
                         "mae_mm": float(np.mean(np.abs(valid))) if valid.size else None,
                         "rmse_mm": float(np.sqrt(np.mean(valid**2))) if valid.size else None}
    return metrics


def static_benchmark():
    records = rows("tabla3.csv")
    q = np.deg2rad([[float(row[f"theta{i}_deg"]) for i in (1, 2, 3)] for row in records])
    nominal = 1000. * np.array([fk(config) for config in q])
    exp = xyz(records, "experimental")
    theory = xyz(records, "theoretical")
    # Sensibilidad de la teoría impresa a +5 mm en l2, sin ajustar el modelo.
    c1, c2, _ = np.cos(q).T
    s1, s2, _ = np.sin(q).T
    variant = nominal + 5. * np.column_stack([c1 * c2, s1 * c2, s2])
    mask = np.array([int(row["id"]) != 26 for row in records])
    return records, nominal, {
        "nominal_vs_experimental": compare(nominal, exp),
        "nominal_vs_experimental_sin_fila_26_sensibilidad": compare(nominal[mask], exp[mask]),
        "teoria_impresa_vs_experimental": compare(theory, exp),
        "nominal_vs_teoria_impresa": compare(nominal, theory),
        "filas_1_a_18_nominal_vs_teoria_impresa": compare(nominal[:18], theory[:18]),
        "filas_1_a_18_l2_mas_5mm_vs_teoria_impresa_sensibilidad": compare(variant[:18], theory[:18]),
        "radios_xy_filas_19_a_25_mm": {
            "nominal": np.linalg.norm(nominal[18:25, :2], axis=1).tolist(),
            "teoria_impresa": np.linalg.norm(theory[18:25, :2], axis=1).tolist(),
        },
    }


def dynamic_benchmark():
    records = rows("tabla4.csv")
    return records, compare(xyz(records, "theoretical"), xyz(records, "experimental"))
