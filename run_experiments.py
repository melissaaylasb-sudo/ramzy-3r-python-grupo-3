"""Ejecuta y documenta los experimentos reproducibles del modelo 3R."""
import argparse
import csv
import json
from pathlib import Path
import platform

import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
import numpy as np
import scipy

from ramzy3r.benchmarks import dynamic_benchmark, static_benchmark, xyz
from ramzy3r.control import simulate_control
from ramzy3r.kinematics import (L1, L2, L3, determinant, fk, frames, inverse_kinematics,
                                jacobian, jacobian_fd, singular_cases)

SEED = 5285


def write_csv(path, header, data):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(data)


def save_figure(fig, filename):
    fig.savefig(filename, dpi=160)
    plt.close(fig)


def kinematics_experiment(out):
    random = np.random.default_rng(SEED)
    qs = random.uniform(-np.pi, np.pi, (128, 3))
    dh_error = max(np.linalg.norm(frames(q)[-1, :3, 3] - fk(q)) for q in qs)
    jac_error = max(np.max(np.abs(jacobian(q) - jacobian_fd(q))) for q in qs)
    det_error = max(abs(determinant(q) - np.linalg.det(jacobian(q))) for q in qs)
    cases = singular_cases()
    ik_cases = {name: fk(q) for name, q in cases.items()}
    ik_cases.update({"inalcanzable_exterior": [0.60, 0., L1],
                     "inalcanzable_interior": [0., 0., L1]})
    solutions = []
    counts = {}
    for name, target in ik_cases.items():
        branches = inverse_kinematics(target)
        counts[name] = len(branches)
        for branch in branches:
            residual = np.linalg.norm(fk(branch.q) - target)
            solutions.append([name, *target, *np.rad2deg(branch.q),
                              branch.radial_sign, branch.elbow_sign, branch.base_free, residual])
    write_csv(out / "inversa.csv", ["caso", "x_m", "y_m", "z_m", "q1_deg", "q2_deg",
              "q3_deg", "signo_radio", "signo_codo", "base_libre", "residuo_m"], solutions)
    table = []
    for name, q in cases.items():
        sv = np.linalg.svd(jacobian(q), compute_uv=False)
        table.append([name, *np.rad2deg(q), determinant(q), int(np.sum(sv > 1e-10)), *sv])
    write_csv(out / "singularidades.csv", ["caso", "q1_deg", "q2_deg", "q3_deg", "det",
              "rango_tol_1e-10", "sigma1", "sigma2", "sigma3"], table)
    q3 = np.r_[np.geomspace(1e-6, .7, 120), 0.]
    sv_min = [np.linalg.svd(jacobian([.2, .3, v]), compute_uv=False)[-1] for v in q3]
    write_csv(out / "barrido_codo.csv", ["q3_rad", "sigma_min_m_rad"], zip(q3, sv_min))
    fig, ax = plt.subplots(figsize=(7, 4), layout="constrained")
    ax.loglog(q3[:-1], sv_min[:-1])
    ax.set(xlabel="Distancia angular a q₃ = 0 [rad]", ylabel="σ mínima [m/rad]",
           title="Aproximación a la singularidad de codo")
    ax.grid(True, which="both", alpha=.25)
    save_figure(fig, out / "singularidad.png")
    points = np.array([fk(q) for q in random.uniform(-np.pi, np.pi, (4500, 3))])
    fig = plt.figure(figsize=(7, 5), layout="constrained")
    ax = fig.add_subplot(projection="3d")
    ax.scatter(*points.T, s=2, alpha=.22)
    arm = frames(cases["regular"])[:, :3, 3]
    ax.plot(*arm.T, "o-", color="#711610", label="Pose de referencia")
    ax.set(xlabel="x [m]", ylabel="y [m]", zlabel="z [m]",
           title="Alcance ideal con articulaciones sin límites físicos")
    ax.set_box_aspect([1, 1, 1])
    ax.legend()
    save_figure(fig, out / "espacio_ideal.png")
    return {"seed": SEED, "n_config": len(qs), "fk_vs_dh_max_m": dh_error,
            "jac_vs_fd_max_m_rad": jac_error, "det_max_abs": det_error,
            "ik_max_residual_m": max(row[-1] for row in solutions), "ik_counts": counts,
            "ranks": {row[0]: row[5] for row in table}}


def paper_experiment(out):
    records, nominal, static = static_benchmark()
    header = list(records[0])
    write_csv(out / "tabla3_auditoria.csv", [*header, "nominal_x_mm", "nominal_y_mm", "nominal_z_mm"],
              [[*(record[key] for key in header), *pred] for record, pred in zip(records, nominal)])
    exp, theory = xyz(records, "experimental"), xyz(records, "theoretical")
    ids = np.array([int(row["id"]) for row in records])
    fig, axs = plt.subplots(3, 2, figsize=(11, 7), sharex=True, layout="constrained")
    for i, (axis, (ax, err_ax)) in enumerate(zip("xyz", axs)):
        available = np.isfinite(exp[:, i])
        ax.plot(ids[available], exp[available, i], "o", label="Medición publicada")
        ax.plot(ids[available], theory[available, i], "x", label="Teoría impresa")
        ax.plot(ids[available], nominal[available, i], "+", label="Modelo nominal")
        ax.set_ylabel(f"{axis} [mm]")
        ax.grid(alpha=.25)
        err_ax.axhline(0., color="0.5", linewidth=0.8)
        err_ax.plot(ids[available], (theory - exp)[available, i], "x", color="C1")
        err_ax.plot(ids[available], (nominal - exp)[available, i], "+", color="C2")
        err_ax.set_ylabel(f"Residuo en {axis} [mm]")
        err_ax.grid(alpha=.25)
    axs[0, 0].set_title("Coordenadas observadas")
    axs[0, 1].set_title("Predicción menos medición")
    axs[0, 0].legend(ncol=1, fontsize=8)
    for ax in axs[-1]:
        ax.set_xlabel("Fila de la Tabla 3")
    fig.suptitle("Comparación por componente observado")
    save_figure(fig, out / "tabla3.png")
    dynamic, dynamic_metrics = dynamic_benchmark()
    times = [float(row["time_s"]) for row in dynamic]
    fig, axs = plt.subplots(3, 1, figsize=(8, 6), sharex=True, layout="constrained")
    for i, (axis, ax) in enumerate(zip("xyz", axs)):
        ax.plot(times, xyz(dynamic, "experimental")[:, i], "o", label="Medición")
        ax.plot(times, xyz(dynamic, "theoretical")[:, i], "x", label="Simulación publicada")
        ax.set_ylabel(f"{axis} [mm]")
        ax.grid(alpha=.25)
    axs[0].legend()
    axs[-1].set_xlabel("Tiempo publicado [s]")
    fig.suptitle("Tabla 4 · Muestras publicadas, sin interpolación")
    save_figure(fig, out / "tabla4.png")
    return {"tabla3": static, "tabla4_muestra_publicada": dynamic_metrics}


def control_experiment(out):
    summary = {}
    scenarios = (
        ("regular", "regular", 0., "Caso regular"),
        ("near_singular", "near_singular", 0., "Cerca de singularidad"),
        ("near_singular_bias", "near_singular", .001, "Cerca de singularidad con sesgo de 1 mm"),
    )
    for case, trajectory, bias, title in scenarios:
        fig = plt.figure(figsize=(13, 4.2))
        grid = fig.add_gridspec(1, 3, left=.025, right=.98, bottom=.15, top=.87,
                                wspace=.43, width_ratios=[1.16, 1., 1.])
        path_ax = fig.add_subplot(grid[0], projection="3d")
        err_ax = fig.add_subplot(grid[1])
        vel_ax = fig.add_subplot(grid[2])
        for method in ("pinv", "dls"):
            d = simulate_control(fk, jacobian, trajectory, method, measurement_bias_m=bias)
            t, desired, actual = d["t"], d["desired"], d["actual"]
            err = np.linalg.norm(desired - actual, axis=1)
            maxvel = np.max(np.abs(d["qdot"]), axis=1)
            if method == "pinv":
                path_ax.plot(*desired.T, "--", color="black", label="Referencia")
            path_ax.plot(*actual.T, label=method)
            err_ax.plot(t, 1000. * err, label=method)
            vel_ax.plot(t, maxvel, label=method)
            summary[f"{case}_{method}"] = {
                "rmse_mm": float(1000. * np.sqrt(np.mean(err**2))),
                "max_error_mm": float(1000. * err.max()),
                "max_error_t_ge_1s_mm": float(1000. * err[t >= 1.].max()),
                "max_measurement_bias_m": float(bias),
                "max_speed_rad_s": float(maxvel.max()),
                "max_requested_speed_rad_s": float(np.max(np.abs(d["qdot_requested"]))),
                "saturated_samples": int(np.count_nonzero(d["saturated"])),
                "min_sigma_actual_m_rad": float(np.min(d["sigma_min"])),
                "min_sigma_reference_m_rad": float(np.min(d["desired_sigma_min"])),
                "dt_s": float(d["dt"]), "gain": float(d["gain"]),
                "damping": float(d["damping"]), "speed_limit_rad_s": float(d["speed_limit"]),
            }
            data = np.column_stack([t, desired, actual, d["q"], d["qdot"],
                                    d["qdot_requested"], err, d["sigma_min"], d["desired_sigma_min"],
                                    d["measurement_bias"][:, 0], d["saturated"].astype(int)])
            header = ["t_s", *[f"deseada_{a}_m" for a in "xyz"], *[f"real_{a}_m" for a in "xyz"],
                      *[f"q{i}_rad" for i in range(1, 4)], *[f"qdot{i}_rad_s" for i in range(1, 4)],
                      *[f"qdot{i}_solicitada_rad_s" for i in range(1, 4)], "error_m",
                      "sigma_min_real", "sigma_min_referencia", "sesgo_x_m", "saturado"]
            write_csv(out / f"control_{case}_{method}.csv", header, data)
        path_ax.set(xlabel="x [m]", ylabel="y [m]", zlabel="z [m]")
        err_ax.set(xlabel="Tiempo [s]", ylabel="Error de posición [mm]")
        vel_ax.set(xlabel="Tiempo [s]", ylabel="Máx. |velocidad articular| [rad/s]")
        for ax in (path_ax, err_ax, vel_ax):
            ax.legend(fontsize=8)
        fig.suptitle(f"Control cinemático · {title}")
        save_figure(fig, out / f"control_{case}.png")
    time_step_results = []
    for dt in (.020, .010, .005):
        for method in ("pinv", "dls"):
            d = simulate_control(fk, jacobian, "near_singular", method, dt=dt, measurement_bias_m=.001)
            time_step_results.append([dt, method, 1000. * np.sqrt(np.mean(d["error"]**2)),
                                      np.max(np.abs(d["qdot_requested"])),
                                      np.count_nonzero(d["saturated"])])
    write_csv(out / "control_sensibilidad_paso.csv", ["dt_s", "metodo", "rmse_mm",
              "max_velocidad_solicitada_rad_s", "muestras_saturadas"], time_step_results)
    return summary


def dynamics_experiment(out):
    from ramzy3r.dynamics import simulate_dynamics

    summary = {}
    fig, axs = plt.subplots(2, 1, figsize=(8, 6), sharex=True, layout="constrained")
    for label, damping, stiffness in (("amortiguado", 1., 1.), ("conservativo", 0., 1.),
                                      ("rigidez_0p5", 1., .5), ("rigidez_1p5", 1., 1.5)):
        d = simulate_dynamics(damping_scale=damping, stiffness_scale=stiffness)
        t, energy = d["t"], d["energy_j"]
        loss, balance = d["dissipated_j"], d["balance_residual_j"]
        axs[0].plot(t, energy - energy[0], label=label)
        axs[1].plot(t, balance, label=label)
        data = np.column_stack([t, d["q"], d["qdot"], energy, loss, balance])
        write_csv(out / f"dinamica_{label}.csv", ["t_s", "q1_rad", "q2_rad", "q3_rad",
                  "qdot1_rad_s", "qdot2_rad_s", "qdot3_rad_s", "energia_j",
                  "disipacion_j", "residuo_balance_j"], data)
        summary[label] = {"max_abs_balance_j": float(np.max(np.abs(balance))),
                          "cambio_energia_j": float(energy[-1] - energy[0]),
                          "min_autovalor_m_kg_m2": float(np.min(d["min_mass_eigenvalue"])),
                          "max_asimetria_m_kg_m2": float(np.max(d["max_mass_asymmetry"])),
                          "rigidez_nm_rad_asumida": float(d["stiffness_nm_rad"]),
                          "amortiguamiento_nm_s_rad_asumido": float(d["damping_nm_s_rad"])}
    axs[0].set_ylabel("E(t) − E(0) [J]")
    axs[1].set(xlabel="Tiempo [s]", ylabel="Residuo de balance [J]")
    for ax in axs:
        ax.legend()
        ax.grid(alpha=.25)
    fig.suptitle("Extensión dinámica · Respuesta libre reconstruida")
    save_figure(fig, out / "dinamica.png")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("resultados"))
    parser.add_argument("--sin-dinamica", action="store_true", help="Omitir la extensión dinámica.")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    result = {
        "entorno": {"python": platform.python_version(), "numpy": np.__version__,
                    "scipy": scipy.__version__, "matplotlib": matplotlib.__version__},
        "geometria_m": [L1, L2, L3],
        "cinematica": kinematics_experiment(args.out),
        "articulo": paper_experiment(args.out),
        "control": control_experiment(args.out),
    }
    if not args.sin_dinamica:
        result["dinamica"] = dynamics_experiment(args.out)
    report = args.out / "resumen.json"
    report.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"Resumen y evidencias en {args.out.resolve()}")


if __name__ == "__main__":
    main()
