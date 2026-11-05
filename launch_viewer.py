r"""Abre un visor 3D interactivo para los resultados locales del manipulador 3R.

Uso en PowerShell, desde la carpeta del proyecto::

    .\.venv\Scripts\python.exe .\launch_viewer.py

El visor usa únicamente la interfaz local (127.0.0.1) y los CSV ya generados.
"""

from __future__ import annotations

import argparse
import csv
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import webbrowser

from ramzy3r.kinematics import L1, L2, L3, frames


PROJECT = Path(__file__).resolve().parent
VIEWER = PROJECT / "viewer"
CASES = (
    ("regular", "pinv", "Regular · pseudoinversa"),
    ("regular", "dls", "Regular · amortiguado"),
    ("near_singular", "pinv", "Cerca de singularidad · pseudoinversa"),
    ("near_singular", "dls", "Cerca de singularidad · amortiguado"),
    ("near_singular_bias", "pinv", "Singularidad + sesgo · pseudoinversa"),
    ("near_singular_bias", "dls", "Singularidad + sesgo · amortiguado"),
)

# Los nombres coinciden con kinematics_experiment() y las filas de inversa.csv.
# Los dos objetivos sin solución no aparecen en el CSV, por lo que se conservan
# explícitamente aquí con las mismas coordenadas de run_experiments.py.
INVERSE_CASES = (
    ("regular", "Regular", None),
    ("codo_extend", "Codo alineado · extendido", None),
    ("codo_plegado", "Codo alineado · plegado", None),
    ("eje_base", "Sobre el eje", None),
    ("interseccion", "Eje y borde", None),
    ("inalcanzable_exterior", "Fuera del alcance · lejos", [0.60, 0., L1]),
    ("inalcanzable_interior", "Fuera del alcance · cerca", [0., 0., L1]),
)


def newest_results() -> Path:
    candidates = [
        directory
        for directory in PROJECT.glob("resultados_locales*")
        if directory.is_dir() and (directory / "resumen.json").is_file()
    ]
    if not candidates:
        raise FileNotFoundError(
            "No hay una corrida local terminada. Ejecuta run_experiments.py primero."
        )
    return max(candidates, key=lambda directory: (directory / "resumen.json").stat().st_mtime)


def _vector(row: dict[str, str], prefix: str) -> list[float]:
    return [float(row[f"{prefix}_{axis}_m"]) for axis in "xyz"]


def load_case(results: Path, scenario: str, method: str, label: str,
              summary: dict) -> dict:
    case_id = f"{scenario}_{method}"
    csv_path = results / f"control_{case_id}.csv"
    samples: list[dict] = []
    previous_time = -math.inf
    with csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            t = float(row["t_s"])
            if not math.isfinite(t) or t <= previous_time:
                raise ValueError(f"Tiempo no creciente en {csv_path.name}: {t}")
            previous_time = t
            q = [float(row[f"q{i}_rad"]) for i in range(1, 4)]
            joints = frames(q)[:, :3, 3].tolist()
            actual = _vector(row, "real")
            desired = _vector(row, "deseada")
            error_m = float(row["error_m"])
            sigma_min = float(row["sigma_min_real"])
            if not all(math.isfinite(value) for value in
                       [t, *q, *actual, *desired, error_m, sigma_min]):
                raise ValueError(f"Dato no finito en {csv_path.name}, t={t}")
            if max(abs(a - b) for a, b in zip(joints[-1], actual)) > 1e-10:
                raise ValueError(
                    f"La geometría y la posición real discrepan en {csv_path.name}, t={t}"
                )
            samples.append({
                "t": t,
                "q": q,
                "joints": joints,
                "desired": desired,
                "actual": actual,
                "error_mm": 1000.0 * error_m,
                "sigma_min": sigma_min,
                "saturated": bool(int(float(row["saturado"]))),
            })
    if len(samples) < 2:
        raise ValueError(f"{csv_path.name} contiene menos de dos muestras")
    return {
        "id": case_id,
        "scenario": scenario,
        "method": method,
        "label": label,
        "metrics": summary["control"][case_id],
        "samples": samples,
    }


def _limit_pose(target: list[float], rho: float, h: float, d: float) -> dict:
    """Postura límite ilustrativa; no es una solución del objetivo imposible."""
    q1 = math.atan2(target[1], target[0]) if rho > 0. else 0.
    q2 = math.atan2(h, rho) if d > 0. else 0.
    q3 = 0. if d > L2 + L3 else math.pi
    q = [q1, q2, q3]
    joints = frames(q)[:, :3, 3].tolist()
    actual = joints[-1]
    return {
        "q": q,
        "joints": joints,
        "actual": actual,
        "distance_to_target_m": math.dist(actual, target),
    }


def load_inverse_cases(results: Path, summary: dict) -> list[dict]:
    """Carga las ternas realmente escritas por la corrida y su geometría 3D."""
    rows_by_case = {case_id: [] for case_id, _, _ in INVERSE_CASES}
    csv_path = results / "inversa.csv"
    with csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            case_id = row["caso"]
            if case_id not in rows_by_case:
                raise ValueError(f"Caso desconocido en {csv_path.name}: {case_id}")
            rows_by_case[case_id].append(row)

    expected_counts = summary["cinematica"]["ik_counts"]
    cases = []
    for case_id, label, unreachable_target in INVERSE_CASES:
        rows = rows_by_case[case_id]
        expected = expected_counts[case_id]
        if len(rows) != expected:
            raise ValueError(
                f"{csv_path.name}: {case_id} tiene {len(rows)} ternas, "
                f"pero resumen.json declara {expected}"
            )
        if unreachable_target is None:
            if not rows:
                raise ValueError(f"{csv_path.name}: faltan ternas para {case_id}")
            target = [float(rows[0][f"{axis}_m"]) for axis in "xyz"]
        else:
            if rows:
                raise ValueError(f"{csv_path.name}: {case_id} debería tener 0 ternas")
            target = unreachable_target.copy()
        if not all(math.isfinite(value) for value in target):
            raise ValueError(f"Objetivo no finito en {case_id}")

        rho = math.hypot(target[0], target[1])
        h = target[2] - L1
        d = math.hypot(rho, h)
        cos_elbow = (d * d - L2 * L2 - L3 * L3) / (2 * L2 * L3)
        postures: list[list[float]] = []
        solutions = []
        for row in rows:
            row_target = [float(row[f"{axis}_m"]) for axis in "xyz"]
            q_deg = [float(row[f"q{i}_deg"]) for i in range(1, 4)]
            recorded_residual = float(row["residuo_m"])
            if not all(math.isfinite(value) for value in [*row_target, *q_deg, recorded_residual]):
                raise ValueError(f"Dato no finito en {csv_path.name}, caso {case_id}")
            if math.dist(row_target, target) > 1e-10:
                raise ValueError(f"Objetivos distintos en {csv_path.name}, caso {case_id}")
            q = [math.radians(angle) for angle in q_deg]
            joints = frames(q)[:, :3, 3].tolist()
            residual = math.dist(joints[-1], target)
            if residual > 1e-9 or abs(residual - recorded_residual) > 1e-9:
                raise ValueError(f"FK no reproduce el objetivo en {csv_path.name}, caso {case_id}")
            elbow = joints[2]
            posture_index = next(
                (index for index, other in enumerate(postures) if math.dist(elbow, other) <= 1e-9),
                None,
            )
            if posture_index is None:
                posture_index = len(postures)
                postures.append(elbow)
            solutions.append({
                "q": q,
                "q_deg": q_deg,
                "joints": joints,
                "radial_sign": int(row["signo_radio"]),
                "elbow_sign": int(row["signo_codo"]),
                "base_free": row["base_libre"].strip().lower() == "true",
                "residual_m": residual,
                "posture": chr(ord("A") + posture_index),
            })

        case = {
            "id": case_id,
            "label": label,
            "target": target,
            "rho": rho,
            "h": h,
            "d": d,
            "D": cos_elbow,
            "solutions": solutions,
            "posture_count": len(postures),
            "base_free": any(solution["base_free"] for solution in solutions),
        }
        if not solutions:
            case["limit_pose"] = _limit_pose(target, rho, h, d)
        cases.append(case)
    return cases


def build_data(results: Path) -> dict:
    results = results.resolve()
    summary_path = results / "resumen.json"
    with summary_path.open("r", encoding="utf-8") as stream:
        summary = json.load(stream)
    cases = [load_case(results, *case, summary) for case in CASES]
    inverse_cases = load_inverse_cases(results, summary)
    return {
        "version": 1,
        "source_folder": results.name,
        "geometry_m": [L1, L2, L3],
        "cases": cases,
        "inverse_cases": inverse_cases,
    }


class ViewerHandler(SimpleHTTPRequestHandler):
    data_json: bytes = b""
    summary_json: bytes = b""
    evidence_png: dict[str, bytes] = {}

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path in ("/data.json", "/summary.json"):
            payload = self.data_json if path == "/data.json" else self.summary_json
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)
            return
        image = self.evidence_png.get(path)
        if image is not None:
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(image)))
            self.end_headers()
            self.wfile.write(image)
            return
        super().do_GET()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, help="Carpeta de resultados; por defecto, la local más reciente")
    parser.add_argument("--port", type=int, default=0, help="Puerto local; 0 elige uno libre")
    parser.add_argument("--no-browser", action="store_true", help="No abrir el navegador automáticamente")
    parser.add_argument("--check", action="store_true", help="Validar los datos y salir")
    args = parser.parse_args()

    results = args.results or newest_results()
    data = build_data(results)
    print(f"Resultados: {results.resolve()}", flush=True)
    print(f"Trayectorias: {len(data['cases'])}; muestras: " +
          ", ".join(f"{case['id']}={len(case['samples'])}" for case in data["cases"]),
          flush=True)
    if args.check:
        return
    if not (VIEWER / "index.html").is_file():
        raise FileNotFoundError(f"No existe la interfaz: {VIEWER / 'index.html'}")

    ViewerHandler.data_json = json.dumps(data, ensure_ascii=False, allow_nan=False,
                                         separators=(",", ":")).encode("utf-8")
    ViewerHandler.summary_json = (results / "resumen.json").read_bytes()
    evidence_names = (
        "mapa_singularidades.png",
        "control_regular.png",
        "control_near_singular.png",
        "control_near_singular_bias.png",
        "tabla3.png",
    )
    ViewerHandler.evidence_png = {
        f"/evidence/{name}": (results / name).read_bytes()
        for name in evidence_names if (results / name).is_file()
    }
    handler = partial(ViewerHandler, directory=str(VIEWER))
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler)
    address = f"http://127.0.0.1:{server.server_port}/"
    print(f"Visor 3D: {address}", flush=True)
    print("Deja esta terminal abierta mientras usas el visor. Ctrl+C lo cierra.", flush=True)
    if not args.no_browser:
        webbrowser.open(address)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
