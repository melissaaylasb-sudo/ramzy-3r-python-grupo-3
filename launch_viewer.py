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


def build_data(results: Path) -> dict:
    results = results.resolve()
    summary_path = results / "resumen.json"
    with summary_path.open("r", encoding="utf-8") as stream:
        summary = json.load(stream)
    cases = [load_case(results, *case, summary) for case in CASES]
    return {
        "version": 1,
        "source_folder": results.name,
        "geometry_m": [L1, L2, L3],
        "cases": cases,
    }


class ViewerHandler(SimpleHTTPRequestHandler):
    data_json: bytes = b""

    def do_GET(self) -> None:
        if self.path.split("?", 1)[0] == "/data.json":
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(self.data_json)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(self.data_json)
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
