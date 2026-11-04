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
from ramzy3r.control import DLS_DAMPING, simulate_control
from ramzy3r.kinematics import (L1, L2, L3, SINGULAR_RTOL, determinant, fk, frames,
                                inverse_kinematics, jacobian, jacobian_fd, numerical_rank,
                                singular_cases, singular_value_families, singular_values,
                                singularity_measures)

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
    singular = singularity_experiment(out, qs, cases)
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
            **singular}


def _sweep_plot(axs, distance, analytic, svd, sigma_max, title, xlabel):
    value_ax, error_ax = axs
    value_ax.loglog(distance, analytic, label="Fórmula cerrada")
    value_ax.loglog(distance[::8], svd[::8], "o", markersize=3.5, fillstyle="none",
                    label="SVD numérica")
    value_ax.axhline(DLS_DAMPING, color="#711610", linewidth=0.9, label="λ del DLS = 0.020 m/rad")
    value_ax.set(ylabel="σ mínima [m/rad]", title=title)
    relative = np.maximum(np.abs(svd - analytic) / analytic, 1e-17)
    error_ax.loglog(distance, relative, color="C1", label="|σ SVD − σ exacta| / σ exacta")
    error_ax.loglog(distance, np.finfo(float).eps * sigma_max / analytic, "--", color="0.45",
                    linewidth=0.9, label="ε · σ₁ / σ mín")
    error_ax.set(xlabel=xlabel, ylabel="Error relativo SVD", ylim=(1e-17, 1.))
    for ax in axs:
        ax.grid(True, which="both", alpha=.25)


def singularity_experiment(out, qs, cases):
    """Valores singulares analíticos, barridos hacia ambas familias y mapas."""
    sigma_error = max(np.max(np.abs(singular_values(q) - np.linalg.svd(jacobian(q), compute_uv=False)))
                      for q in qs)
    table, ranks, families = [], {}, {}
    for name, q in cases.items():
        m = singularity_measures(q, DLS_DAMPING)
        sv = np.linalg.svd(jacobian(q), compute_uv=False)
        ranks[name], families[name] = m["rango"], m["familia"]
        table.append([name, *np.rad2deg(q), determinant(q), m["manipulabilidad"], *m["sigma"],
                      m["sigma_eje_base"], m["sigma_codo"], *sv, m["condicion"], m["rango"],
                      numerical_rank(sv), m["familia"]])
    write_csv(out / "singularidades.csv", ["caso", "q1_deg", "q2_deg", "q3_deg", "det_m3",
              "manipulabilidad_m3", "sigma1_m_rad", "sigma2_m_rad", "sigma3_m_rad",
              "sigma_eje_base_m_rad", "sigma_codo_m_rad", "sigma1_svd_m_rad", "sigma2_svd_m_rad",
              "sigma3_svd_m_rad", "condicion", "rango_analitico", "rango_svd", "familia"], table)

    # Codo: q3 -> 0 con q2 = 0.3 rad, donde |r| ≈ 0.5 m no interviene.
    elbow_slope = L2 * L3 / np.hypot(L2 + L3, L3)
    q3 = np.geomspace(1e-14, .7, 169)
    elbow = np.array([singular_values([.2, .3, v])[-1] for v in q3])
    elbow_svd = np.array([np.linalg.svd(jacobian([.2, .3, v]), compute_uv=False)[-1] for v in q3])
    write_csv(out / "barrido_codo.csv", ["q3_rad", "sigma_min_analitica_m_rad",
              "sigma_min_svd_m_rad", "error_relativo_svd"],
              zip(q3, elbow, elbow_svd, np.abs(elbow_svd - elbow) / elbow))

    # Eje de la base: r -> 0 desplazando q2 desde la pose eje_base, con q3 = pi/2 fijo.
    q2_axis = cases["eje_base"][1]
    base_slope = np.hypot(L2, L3)
    delta = np.geomspace(1e-14, .3, 161)
    axis_q = [[.2, q2_axis + v, np.pi / 2] for v in delta]
    axis = np.array([singular_values(q)[-1] for q in axis_q])
    axis_svd = np.array([np.linalg.svd(jacobian(q), compute_uv=False)[-1] for q in axis_q])
    axis_r = np.array([singular_value_families(q)[0] for q in axis_q])
    write_csv(out / "barrido_eje.csv", ["delta_q2_rad", "r_m", "sigma_min_analitica_m_rad",
              "sigma_min_svd_m_rad", "error_relativo_svd"],
              zip(delta, axis_r, axis, axis_svd, np.abs(axis_svd - axis) / axis))

    fig, axs = plt.subplots(2, 2, figsize=(11, 6.4), sharex="col", layout="constrained",
                            height_ratios=[1.6, 1.])
    _sweep_plot(axs[:, 0], q3, elbow, elbow_svd, singular_values([.2, .3, 0.])[0],
                "Familia de codo · sin q₃ = 0", "Distancia angular a q₃ = 0 [rad]")
    _sweep_plot(axs[:, 1], delta, axis, axis_svd, singular_values(cases["eje_base"])[0],
                "Familia del eje de la base · r = 0", "Desplazamiento de q₂ desde r = 0 [rad]")
    axs[0, 0].legend(fontsize=8)
    axs[1, 0].legend(fontsize=8, loc="upper right")
    fig.suptitle("Aproximación a las dos familias singulares")
    save_figure(fig, out / "singularidad.png")

    # Mapa articular: q1 no interviene en ningún valor singular.
    grid = np.linspace(-np.pi, np.pi, 721)
    q2g, q3g = np.meshgrid(grid, grid, indexing="xy")
    base_g, _, elbow_g = singular_value_families((None, q2g, q3g))
    sigma_g = np.minimum(base_g, elbow_g)
    r_g = L2 * np.cos(q2g) + L3 * np.cos(q2g + q3g)

    # Sección cartesiana (radio con signo, z): sigma_min solo depende de la posición,
    # porque ambas ramas del codo comparten |sin q3| y |r| es la distancia al eje.
    rho = np.linspace(-.56, .56, 561)
    height = L1 + np.linspace(-.56, .56, 561)
    rho_g, z_g = np.meshgrid(rho, height, indexing="xy")
    cos_q3 = (rho_g**2 + (z_g - L1)**2 - L2**2 - L3**2) / (2 * L2 * L3)
    reachable = np.abs(cos_q3) <= 1.
    _, _, elbow_c = singular_value_families((None, 0., np.arccos(np.clip(cos_q3, -1., 1.))))
    sigma_c = np.where(reachable, np.minimum(np.abs(rho_g), elbow_c), np.nan)
    # Fracción de volumen: cada celda de la sección barre un anillo de volumen ∝ |rho|.
    weight = np.where(reachable, np.abs(rho_g), 0.)
    near = reachable & (sigma_c < DLS_DAMPING)
    near_base = near & (np.abs(rho_g) <= elbow_c)
    volume = {"fraccion_volumen_sigma_menor_lambda": float(weight[near].sum() / weight.sum()),
              "fraccion_volumen_cerca_eje_base": float(weight[near_base].sum() / weight.sum()),
              "fraccion_volumen_cerca_codo": float(weight[near & ~near_base].sum() / weight.sum())}

    fig, axs = plt.subplots(1, 2, figsize=(12, 5), layout="constrained")
    levels = np.linspace(-4, np.log10(.4), 25)
    image = axs[0].contourf(np.rad2deg(q2g), np.rad2deg(q3g),
                            np.log10(np.maximum(sigma_g, 1e-4)), levels=levels, extend="min")
    axs[0].contour(np.rad2deg(q2g), np.rad2deg(q3g), r_g, levels=[0.], colors="white",
                   linewidths=1.3)
    for value in (-180., 0., 180.):
        axs[0].axhline(value, color="black", linewidth=1.3)
    axs[0].contour(np.rad2deg(q2g), np.rad2deg(q3g), sigma_g, levels=[DLS_DAMPING],
                   colors="#711610", linewidths=0.9, linestyles="--")
    axs[0].set(xlabel="q₂ [°]", ylabel="q₃ [°]", aspect="equal",
               title="Espacio articular (válido para todo q₁)")
    axs[0].plot([], [], color="black", label="sin q₃ = 0 · codo")
    axs[0].plot([], [], color="white", label="r = 0 · eje de la base")
    axs[0].plot([], [], "--", color="#711610", label="σ mín = λ")
    axs[0].legend(fontsize=8, loc="lower left", facecolor="0.85")
    axs[1].contourf(rho_g, z_g, np.log10(np.maximum(sigma_c, 1e-4)), levels=levels, extend="min")
    axs[1].contour(rho_g, z_g, np.nan_to_num(sigma_c, nan=1.), levels=[DLS_DAMPING],
                   colors="#711610", linewidths=0.9, linestyles="--")
    angle = np.linspace(0, 2 * np.pi, 400)
    for radius in (L2 + L3, L2 - L3):
        axs[1].plot(radius * np.cos(angle), L1 + radius * np.sin(angle), color="black",
                    linewidth=1.3)
    axs[1].axvline(0., color="white", linewidth=1.3)
    axs[1].plot(0., L1, "k+", markersize=9)
    axs[1].set(xlabel="Radio con signo r [m]", ylabel="z [m]", aspect="equal",
               title="Sección cartesiana que contiene el eje z")
    bar = fig.colorbar(image, ax=axs, label="σ mínima [m/rad]", shrink=.85, ticks=[-4, -3, -2, -1])
    bar.ax.set_yticklabels(["≤ 10⁻⁴", "10⁻³", "10⁻²", "10⁻¹"])
    fig.suptitle("Cercanía a singularidad · σ mínima analítica")
    save_figure(fig, out / "mapa_singularidades.png")

    return {"sigma_analitica_vs_svd_max_m_rad": float(sigma_error),
            "rango_rtol": SINGULAR_RTOL, "ranks": ranks, "familias": families,
            "pendiente_codo_m_rad": float(elbow_slope), "pendiente_eje_m_rad": float(base_slope),
            "barrido_codo_error_relativo_svd_max": float(np.max(np.abs(elbow_svd - elbow) / elbow)),
            "barrido_eje_error_relativo_svd_max": float(np.max(np.abs(axis_svd - axis) / axis)),
            "umbral_cercania_m_rad": DLS_DAMPING, **volume}


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
