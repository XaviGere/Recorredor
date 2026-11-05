# Dashboard Web en Vivo (7 Modos + Historial Lateral de Reportes con Foto + Dashboard Comparativo Analítico)
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import random
import socket
import sys
import threading
import time
from typing import Optional
from urllib.parse import parse_qs, urlparse

# Permitir ejecutar este archivo directamente con el botón "Run Python File" de VS Code / Windsurf
if __package__ is None or __package__ == "":
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sim.config import RobotConfig
from sim.world import (
    create_custom_room,
    create_diamond_room,
    create_hexagonal_room,
    create_l_shaped_ortho_room,
    create_pentagonal_room,
    create_rectangular_room,
    create_trapezoid_room,
    run_simulation,
)

SAVED_SHAPES_FILE = os.path.join(os.path.dirname(__file__), "saved_shapes.json")
SAVED_RUNS_FILE = os.path.join(os.path.dirname(__file__), "saved_runs.json")
SHAPES_LOCK = threading.Lock()
RUNS_LOCK = threading.Lock()

ESP32_LIVE_LOCK = threading.Lock()
ESP32_LIVE_PACKETS: list[dict] = []


def load_saved_shapes() -> dict[str, list[list[float]]]:
    with SHAPES_LOCK:
        if not os.path.exists(SAVED_SHAPES_FILE):
            return {}
        try:
            with open(SAVED_SHAPES_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
                return data if isinstance(data, dict) else {}
        except Exception:
            return {}


def save_custom_shape(name: str, vertices: list[tuple[float, float]]) -> dict[str, list[list[float]]]:
    shapes = load_saved_shapes()
    with SHAPES_LOCK:
        shapes[name] = [[round(float(x), 3), round(float(y), 3)] for x, y in vertices]
        with open(SAVED_SHAPES_FILE, "w", encoding="utf-8") as f:
            json.dump(shapes, f, ensure_ascii=False, indent=2)
    return shapes


def load_saved_runs() -> list[dict]:
    with RUNS_LOCK:
        if not os.path.exists(SAVED_RUNS_FILE):
            return []
        try:
            with open(SAVED_RUNS_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
                return data if isinstance(data, list) else []
        except Exception:
            return []


def append_saved_run(run_record: dict) -> list[dict]:
    runs = load_saved_runs()
    with RUNS_LOCK:
        # Evitar duplicar el mismo seed si ya fue guardado
        if not any(r.get("seed") == run_record.get("seed") for r in runs):
            run_record["run_number"] = len(runs) + 1
            run_record["saved_at"] = time.strftime("%H:%M:%S")
            runs.append(run_record)
            if len(runs) > 120:
                runs = runs[-120:]
            with open(SAVED_RUNS_FILE, "w", encoding="utf-8") as f:
                json.dump(runs, f, ensure_ascii=False)
    return runs


def clear_saved_runs() -> list[dict]:
    with RUNS_LOCK:
        with open(SAVED_RUNS_FILE, "w", encoding="utf-8") as f:
            json.dump([], f)
    return []


def delete_saved_run(run_number: int) -> list[dict]:
    """Elimina un único run del historial persistente por su número de run."""
    runs = load_saved_runs()
    with RUNS_LOCK:
        runs = [r for r in runs if r.get("run_number") != run_number]
        with open(SAVED_RUNS_FILE, "w", encoding="utf-8") as f:
            json.dump(runs, f, ensure_ascii=False)
    return runs


def parse_turn_sequence(seq_str: str) -> list[float]:
    vals: list[float] = []
    for part in seq_str.replace(";", ",").split(","):
        part = part.strip()
        if not part:
            continue
        try:
            vals.append(float(part))
        except ValueError:
            continue
    return vals if vals else [110.0, 145.0, 125.0]


def udp_listener_thread(udp_port: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", udp_port))
    except OSError:
        return
    while True:
        try:
            data, addr = sock.recvfrom(2048)
            payload = json.loads(data.decode("utf-8", errors="ignore").strip())
            payload["source_ip"] = addr[0]
            with ESP32_LIVE_LOCK:
                ESP32_LIVE_PACKETS.append(payload)
                if len(ESP32_LIVE_PACKETS) > 500:
                    ESP32_LIVE_PACKETS.pop(0)
        except Exception:
            continue


def build_simulation_payload(
    room_type: str = "rect",
    mode: str = "classic",
    max_chords: int = 32,
    max_relearns: int = 15,
    turn_mode: str = "random",
    fixed_turn_deg: float = 135.0,
    turn_seq_str: str = "110, 145, 125",
    test_stride_m: float = 0.42,
    test_corner_pass_m: float = 0.12,
    early_stop_perimeter: bool = False,
    seed: Optional[int] = None,
    custom_vertices: Optional[list[tuple[float, float]]] = None,
    custom_name: Optional[str] = None,
) -> dict:
    actual_seed = seed if seed is not None else random.SystemRandom().randint(1, 999999)
    force_ortho = mode in ("recto", "recto_reaprendizaje", "testeador", "incidente")
    relearn_active = mode in ("reaprendizaje", "recto_reaprendizaje", "bordeador", "testeador", "incidente")

    if custom_vertices and len(custom_vertices) >= 3:
        base_label = custom_name or "Figura Personalizada"
        room = create_custom_room(
            custom_vertices,
            name=f"{base_label} (90° Normalizada)" if force_ortho else base_label,
            force_orthogonal_90=force_ortho,
        )
    elif room_type.startswith("saved:"):
        shape_name = room_type.split("saved:", 1)[1]
        saved_map = load_saved_shapes()
        verts_saved = saved_map.get(shape_name, [])
        if len(verts_saved) >= 3:
            pts_tuples = [(float(p[0]), float(p[1])) for p in verts_saved]
            room = create_custom_room(
                pts_tuples,
                name=f"{shape_name} (90° Normalizada)" if force_ortho else shape_name,
                force_orthogonal_90=force_ortho,
            )
        else:
            room = create_rectangular_room(4.0, 3.0)
    elif room_type == "l_ortho":
        room = create_l_shaped_ortho_room()
    elif room_type == "pentagon":
        room = create_pentagonal_room()
    elif room_type == "trapezoid":
        room = create_trapezoid_room()
    elif room_type == "hexagon":
        room = create_hexagonal_room()
    elif room_type == "octagon":
        room = create_diamond_room()
    else:
        room = create_rectangular_room(4.0, 3.0)

    exact_chords = max(8, min(120, int(max_chords)))
    cfg = RobotConfig(
        min_chords_for_map=exact_chords,
        max_chords_explore=exact_chords,
    )

    seq_list = parse_turn_sequence(turn_seq_str)
    res = run_simulation(
        room=room,
        config=cfg,
        initial_pose=None,
        seed=actual_seed,
        mode=mode,
        relearn_enabled=relearn_active,
        max_relearns=max_relearns,
        turn_mode=turn_mode,
        fixed_turn_deg=fixed_turn_deg,
        turn_sequence_deg=seq_list,
        test_stride_m=test_stride_m,
        test_corner_pass_m=test_corner_pass_m,
        early_stop_perimeter=early_stop_perimeter,
        max_steps=95000,
    )
    nav = res.navigator
    est = nav.estimator

    stride = 3
    frames = [
        {"x": round(p.x, 4), "y": round(p.y, 4), "theta": round(p.theta, 4)}
        for p in res.true_poses[::stride]
    ]
    if res.true_poses:
        last_p = res.true_poses[-1]
        frames.append({"x": round(last_p.x, 4), "y": round(last_p.y, 4), "theta": round(last_p.theta, 4)})

    chords_data = []
    for i, c in enumerate(est.chords):
        start_pose_idx, end_pose_idx = (
            res.chord_pose_spans[i]
            if i < len(res.chord_pose_spans)
            else (0, res.perimeter_start_pose_idx)
        )
        chords_data.append(
            {
                "idx": c.idx,
                "x1": round(c.start.x, 4),
                "y1": round(c.start.y, 4),
                "x2": round(c.end.x, 4),
                "y2": round(c.end.y, 4),
                "length": round(c.length, 3),
                "duration_s": round(c.duration_s, 2),
                "start_frame": start_pose_idx // stride,
                "end_frame": max(start_pose_idx // stride + 1, end_pose_idx // stride),
            }
        )

    crossings_data = [{"x": round(p.x, 4), "y": round(p.y, 4)} for p in est.crossings]
    wall_hits_data = [{"x": round(p.x, 4), "y": round(p.y, 4)} for p in est.wall_points]

    history_data = [
        {
            "chords": h.num_chords,
            "crossings": h.num_crossings,
            "total_dist_m": round(h.total_path_length_m, 2),
            "mean_chord_m": round(h.mean_chord_length_m, 3),
            "poly_area": round(h.poly_area_m2, 3),
            "poly_perim": round(h.poly_perimeter_m, 3),
            "ants_area": round(h.ants_pairs_area_m2, 3),
            "mallock_area": round(h.ants_mallock_split_area_m2, 3),
            "crofton_perim": round(h.crofton_perimeter_from_poly_m, 3),
            "converged": h.converged,
        }
        for h in nav.estimates_history
    ]

    final_est = nav.last_estimates
    final_poly_area = round(final_est.poly_area_m2, 3) if final_est else 0.0
    final_poly_perim = round(final_est.poly_perimeter_m, 3) if final_est else 0.0
    final_ants_area = round(final_est.ants_pairs_area_m2, 3) if final_est else 0.0
    area_error_pct = (
        round(abs(final_poly_area - room.true_area_m2) / max(room.true_area_m2, 1e-6) * 100.0, 2)
    )
    perim_error_pct = (
        round(abs(final_poly_perim - room.true_perimeter_m) / max(room.true_perimeter_m, 1e-6) * 100.0, 2)
    )
    precision_pct = round(max(0.0, 100.0 - 0.5 * (area_error_pct + perim_error_pct)), 2)

    initial_deg = round(res.true_poses[0].theta * 180.0 / 3.14159265, 1) if res.true_poses else 0.0

    return {
        "seed": actual_seed,
        "mode": mode,
        "turn_mode": turn_mode,
        "fixed_turn_deg": fixed_turn_deg,
        "turn_seq_str": turn_seq_str,
        "test_stride_m": test_stride_m,
        "test_corner_pass_m": test_corner_pass_m,
        "requested_chords": exact_chords,
        "actual_chords": len(est.chords),
        "wall_bounces": len(est.wall_points),
        "early_stop_enabled": bool(early_stop_perimeter),
        "early_stopped": bool(getattr(nav, "stopped_early", False)),
        "max_relearns": max_relearns,
        "relearn_count": nav.relearn_count,
        "total_relearn_collisions": res.total_relearn_collisions,
        "perimeter_lap_collisions": res.perimeter_lap_collisions,
        "relearn_frames": [idx // stride for idx in res.relearn_events_pose_indices],
        "initial_angle_deg": initial_deg,
        "room_key": room_type,
        "room_name": room.name,
        "true_area_m2": round(room.true_area_m2, 3),
        "true_perimeter_m": round(room.true_perimeter_m, 3),
        "final_poly_area_m2": final_poly_area,
        "final_poly_perim_m": final_poly_perim,
        "final_ants_area_m2": final_ants_area,
        "area_error_pct": area_error_pct,
        "perim_error_pct": perim_error_pct,
        "precision_pct": precision_pct,
        "total_distance_m": round(nav.odom.total_distance_m, 2),
        "robot_radius_m": cfg.robot_radius_m,
        "safe_margin_m": round(cfg.robot_radius_m * 1.2, 4),
        "room_vertices": [{"x": round(v.x, 4), "y": round(v.y, 4)} for v in room.vertices_ccw],
        "frames": frames,
        "perimeter_start_frame": res.perimeter_start_pose_idx // stride,
        "chords": chords_data,
        "crossings": crossings_data,
        "wall_hits": wall_hits_data,
        "reconstructed_perimeter": [
            {"x": round(v.x, 4), "y": round(v.y, 4)} for v in nav.reconstructed_perimeter
        ],
        "safe_waypoints": [
            {"x": round(v.x, 4), "y": round(v.y, 4)} for v in nav.safe_perimeter_waypoints
        ],
        "estimates_history": history_data,
        "min_clearance_cm": round(res.min_perimeter_clearance_m * 100.0, 1),
        "timings": {
            "total_time_s": round(res.total_sim_time_s, 2),
            "exploration_time_s": round(res.exploration_time_s, 2),
            "relearn_time_s": round(res.relearn_time_s, 2),
            "final_lap_time_s": round(res.final_lap_time_s, 2),
        },
    }


HTML_DASHBOARD = """<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<title>Recorredor — Simulador Robótico (7 Modos | Reportes Guardados | Dashboard Comparativo)</title>
<style>
  :root {
    --bg: #0b1120;
    --panel: #1e293b;
    --border: #334155;
    --text: #f8fafc;
    --muted: #94a3b8;
    --accent: #38bdf8;
    --green: #22c55e;
    --amber: #f59e0b;
    --red: #ef4444;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Segoe UI', system-ui, sans-serif; }
  body { background: var(--bg); color: var(--text); padding: 10px; }
  header {
    display: flex; flex-direction: column; gap: 8px;
    background: var(--panel); border: 1px solid var(--border);
    padding: 10px 14px; border-radius: 10px; margin-bottom: 10px;
  }
  .header-top { display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 8px; }
  header h1 { font-size: 0.98rem; font-weight: 700; color: var(--accent); }
  .controls { display: flex; gap: 7px; align-items: center; flex-wrap: wrap; }
  button, select, input[type="number"], input[type="text"] {
    background: #0f172a; color: var(--text); border: 1px solid var(--border);
    padding: 5px 9px; border-radius: 6px; cursor: pointer; font-size: 0.8rem;
  }
  button.primary { background: #16a34a; border-color: #4ade80; font-weight: 700; }
  button.draw-btn { background: #7c3aed; border-color: #a78bfa; font-weight: 600; }
  button.save-btn { background: #0284c7; border-color: #38bdf8; font-weight: 600; }
  button.dash-btn { background: #ea580c; border-color: #fb923c; font-weight: 700; }
  button:hover { filter: brightness(1.15); }
  .main-grid {
    display: grid;
    grid-template-columns: 1.25fr 0.95fr 0.82fr;
    gap: 10px;
    align-items: start;
  }
  .card {
    background: var(--panel); border: 1px solid var(--border);
    border-radius: 10px; padding: 11px; display: flex; flex-direction: column; gap: 8px;
  }
  .card h2 { font-size: 0.84rem; color: var(--muted); text-transform: uppercase; letter-spacing: 0.04em; }
  canvas { background: #090d16; border-radius: 8px; border: 1px solid #1e293b; width: 100%; cursor: crosshair; }
  .metrics-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 7px; }
  .metric-box {
    background: #0f172a; border: 1px solid var(--border); border-radius: 7px; padding: 7px 10px;
  }
  .metric-box .label { font-size: 0.7rem; color: var(--muted); }
  .metric-box .val { font-size: 0.98rem; font-weight: 700; margin-top: 2px; }
  .metric-box .sub { font-size: 0.69rem; color: var(--muted); margin-top: 2px; }
  .legend { display: flex; gap: 10px; flex-wrap: wrap; font-size: 0.74rem; color: var(--muted); }
  .dot { display: inline-block; width: 9px; height: 9px; border-radius: 50%; margin-right: 4px; }
  .status-pill {
    display: inline-block; padding: 3px 9px; border-radius: 999px;
    font-size: 0.72rem; font-weight: 700; background: #0284c7; color: #fff;
  }
  #drawHint {
    display: none; background: #3b0764; border: 1px solid #a855f7; color: #f3e8ff;
    padding: 7px 10px; border-radius: 7px; font-size: 0.78rem;
    justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 6px;
  }
  /* Columna derecha de reportes guardados */
  .reports-sidebar {
    max-height: 675px; overflow-y: auto; display: flex; flex-direction: column; gap: 8px; padding-right: 3px;
  }
  .report-item {
    background: #0f172a; border: 1px solid var(--border); border-radius: 8px; padding: 8px;
    display: grid; grid-template-columns: 112px 1fr; gap: 8px; align-items: center;
  }
  .report-item img {
    width: 112px; height: 80px; object-fit: cover; border-radius: 5px; border: 1px solid #334155; background: #090d16;
  }
  .report-meta { font-size: 0.72rem; line-height: 1.35; color: #cbd5e1; }
  .report-meta strong { color: var(--accent); font-size: 0.78rem; }
  /* Sección inferior: Dashboard Comparativo Analítico */
  .analytics-section {
    margin-top: 12px; background: var(--panel); border: 1px solid var(--border);
    border-radius: 10px; padding: 14px; display: flex; flex-direction: column; gap: 12px;
  }
  .analytics-charts {
    display: grid; grid-template-columns: 1fr 1fr; gap: 12px;
  }
  table.comp-table {
    width: 100%; border-collapse: collapse; font-size: 0.76rem; background: #0f172a; border-radius: 8px; overflow: hidden;
  }
  table.comp-table th, table.comp-table td {
    padding: 6px 8px; border-bottom: 1px solid #1e293b; text-align: left;
  }
  table.comp-table th { background: #162032; color: var(--accent); font-weight: 600; }
  table.comp-table th.sortable { cursor: pointer; user-select: none; }
  table.comp-table th.sortable:hover { background: #1e293b; color: #7dd3fc; }
  table.comp-table th.sorted-asc::after { content: ' ▲'; color: #4ade80; }
  table.comp-table th.sorted-desc::after { content: ' ▼'; color: #4ade80; }
  .run-detail {
    grid-column: 1 / -1; background: #162032; border: 1px solid #334155; border-radius: 6px;
    padding: 8px; font-size: 0.74rem; color: #cbd5e1; display: flex; gap: 12px; align-items: flex-start;
  }
  .run-detail img { width: 260px; border-radius: 6px; border: 1px solid #475569; }
  .run-detail .det-grid { display: grid; grid-template-columns: repeat(2, minmax(150px, 1fr)); gap: 3px 14px; flex: 1; }
  .del-run-btn { background: #7f1d1d; border: none; color: #fca5a5; border-radius: 4px; cursor: pointer; font-size: 0.7rem; padding: 2px 6px; }
  .del-run-btn:hover { background: #b91c1c; color: #fff; }
  .dl-run-btn { background: #14532d; border: none; color: #86efac; border-radius: 4px; cursor: pointer; font-size: 0.7rem; padding: 2px 6px; }
  .dl-run-btn:hover { background: #15803d; color: #fff; }
  .dl-run-big { background: #14532d; border: none; color: #86efac; border-radius: 6px; cursor: pointer; font-size: 0.82rem; padding: 6px 12px; margin-top: 8px; font-weight: 700; }
  .dl-run-big:hover { background: #15803d; color: #fff; }
  .report-item { cursor: pointer; }
  .report-item:hover { border-color: #475569; }
</style>
</head>
<body>

<header>
  <div class="header-top">
    <h1>RECORREDOR — 7 Modos (Clásico | Recto | Reaprendizaje | Recto+Reaprend. | Bordeador | Testeador | Incidente)</h1>
    <div class="controls">
      <button class="primary" onclick="startSimulationNow()">▶ Nuevo Run Ahora</button>
      <button class="draw-btn" onclick="openCustomDrawEditor()">✏️ Dibujar Figura</button>
      <button onclick="togglePause()" id="pauseBtn">⏸ Pausar</button>
      <button onclick="skipToPerimeter()">⏭ Ir a Vuelta Final</button>
      <button class="save-btn" onclick="saveCurrentRunReport(true)">📸 Guardar Reporte ya</button>
      <button class="dash-btn" onclick="scrollToDashboard()">📊 Ver Dashboard Comparativo</button>
      <label style="font-size:0.78rem;color:var(--muted);">Vel:</label>
      <input type="range" id="speedRange" min="1" max="20" value="6" style="width:75px;">
    </div>
  </div>

  <div class="controls">
    <label style="font-size:0.78rem;color:var(--muted);">Figura:</label>
    <select id="roomSelect" onchange="onSelectRoomDropdown()">
      <optgroup label="Predefinidas">
        <option value="rect">1. Rectangular (4.0m x 3.0m)</option>
        <option value="l_ortho">2. Habitación Escalonada 90° (Con Entrante)</option>
        <option value="pentagon">3. Salón Pentagonal Achaflanado</option>
        <option value="trapezoid">4. Habitación Trapezoidal</option>
        <option value="hexagon">5. Aula Hexagonal (6 Paredes)</option>
        <option value="octagon">6. Salón Octogonal</option>
        <option value="custom">✏️ Figura Personalizada Actual</option>
      </optgroup>
      <optgroup label="Mis Figuras Guardadas" id="savedGroup"></optgroup>
    </select>

    <label style="font-size:0.78rem;color:var(--muted);">Modo:</label>
    <select id="modeSelect" onchange="onModeDropdownChanged()">
      <option value="classic">1. Modo Clásico (Poligonal)</option>
      <option value="recto">2. Modo Recto (Ángulos 90°)</option>
      <option value="reaprendizaje">3. Modo Reaprendizaje (Clásico + Recálculo)</option>
      <option value="recto_reaprendizaje" selected>4. Modo Recto + Reaprendizaje (90° + Recálculo)</option>
      <option value="bordeador">5. Modo Bordeador (Prueba ambos lados y elige mejor)</option>
      <option value="testeador">6. Modo Testeador (Sigue pared y tantea esquinas a 90°)</option>
      <option value="incidente">7. Modo Incidente (Rebote + orientación de pared, 90° + Reaprend.)</option>
    </select>

    <label style="font-size:0.78rem;color:var(--muted);">Cuerdas/Pasos:</label>
    <input type="number" id="chordsInput" min="8" max="120" value="32" style="width:54px;">

    <label style="font-size:0.78rem;color:var(--muted);">Máx Reaprend.:</label>
    <input type="number" id="maxRelearnsInput" min="0" max="50" value="15" style="width:50px;">

    <label style="font-size:0.78rem;color:var(--muted);">Variación Rumbo:</label>
    <select id="turnModeSelect" onchange="onTurnModeChanged()">
      <option value="random" selected>Aleatorio Inteligente</option>
      <option value="fixed">Ángulo Fijo (°)</option>
      <option value="sequence">Alternar Lista (°)</option>
    </select>

    <span id="fixedTurnBox" style="display:none;align-items:center;gap:4px;">
      <label style="font-size:0.75rem;color:var(--muted);">Giro (°):</label>
      <input type="number" id="fixedTurnInput" min="15" max="345" value="135" style="width:54px;">
    </span>

    <span id="seqTurnBox" style="display:none;align-items:center;gap:4px;">
      <label style="font-size:0.75rem;color:var(--muted);">Lista (°):</label>
      <input type="text" id="seqTurnInput" value="105, 135, 155" style="width:105px;">
    </span>

    <span id="testParamsBox" style="display:none;align-items:center;gap:4px;">
      <label style="font-size:0.75rem;color:var(--muted);" title="Distancia que el Testeador avanza paralelo a la pared antes de volver a testearla">Paso testeo (m):</label>
      <input type="number" id="testStrideInput" min="0.1" max="2.0" step="0.05" value="0.42" style="width:56px;">
      <label style="font-size:0.75rem;color:var(--muted);" title="Avance mínimo para librar la esquina saliente antes de doblar a buscar la pared">Paso esquina (m):</label>
      <input type="number" id="testCornerInput" min="0.04" max="1.0" step="0.02" value="0.12" style="width:56px;">
    </span>

    <label style="font-size:0.78rem;color:var(--muted);align-items:center;gap:3px;cursor:pointer;" title="Si está activo, el run frena apenas el perímetro estimado coincida con el real (±5%), sin esperar a completar todas las cuerdas. Habilita reaprendizaje automático por si le falta un rincón.">
      <input type="checkbox" id="earlyStopChk"> Corte x Perímetro
    </label>
  </div>
</header>

<div class="main-grid">
  <!-- COLUMNA 1: MAPA EN VIVO -->
  <div class="card">
    <div style="display:flex;justify-content:space-between;align-items:center;">
      <h2 id="roomTitle">Mapa Virtual en Tiempo Real</h2>
      <span class="status-pill" id="fsmBadge">INICIANDO...</span>
    </div>
    <div id="drawHint">
      <span id="drawHintText">Hacé clic alrededor del centro (0,0) para trazar las paredes.</span>
      <div style="display:flex;gap:5px;align-items:center;flex-wrap:wrap;">
        <input type="text" id="saveShapeName" placeholder="Nombre para guardar..." style="width:135px;">
        <button class="save-btn" onclick="saveCurrentCustomShape()">💾 Guardar y Simular</button>
        <button class="primary" onclick="finishCustomDrawAndRun()">▶ Simular Ahora</button>
        <button onclick="undoLastPoint()">↩ Deshacer</button>
        <button onclick="clearCustomPoints()">🗑️ Limpiar</button>
      </div>
    </div>
    <canvas id="mapCanvas" width="720" height="510"></canvas>
    <div class="legend">
      <span><i class="dot" style="background:#64748b"></i>Pared Real</span>
      <span><i class="dot" style="background:#38bdf8"></i>Cuerdas / Sondeos</span>
      <span><i class="dot" style="background:#ef4444"></i>Impactos y Perímetro</span>
      <span><i class="dot" style="background:#f59e0b"></i>Cruces Hormigas</span>
      <span><i class="dot" style="background:#22c55e"></i>Recorrido Final (Sin Diagonal)</span>
    </div>
  </div>

  <!-- COLUMNA 2: TELEMETRÍA Y CONVERGENCIA -->
  <div style="display:flex;flex-direction:column;gap:10px;">
    <div class="card">
      <h2>Telemetría, Tiempos y Métricas</h2>
      <div class="metrics-grid">
        <div class="metric-box">
          <div class="label">Modo Activo y Variación de Rumbo</div>
          <div class="val" id="mSeed">Run #--</div>
          <div class="sub" id="mInitAngle">Salida θ₀: --° | Cuerdas: --</div>
        </div>
        <div class="metric-box">
          <div class="label">Tiempo Total y Desglose por Etapa</div>
          <div class="val" id="mTimeTotal" style="color:#38bdf8">0.00 s</div>
          <div class="sub" id="mTimeBreakdown">Explor.: 0s | Reaprend.: 0s | Vuelta: 0s</div>
        </div>
        <div class="metric-box">
          <div class="label">Métricas de Reaprendizaje y Choques</div>
          <div class="val" id="mRelearnStats" style="color:var(--amber)">0 recálculos | 0 choques</div>
          <div class="sub" id="mRelearnSub">Tiempo reaprend.: 0.00 s | Choques vuelta: 0</div>
        </div>
        <div class="metric-box">
          <div class="label">Rebotes, Cuerdas y Distancia</div>
          <div class="val" id="mChords">0 cuerdas | 0 cruces</div>
          <div class="sub" id="mDist">Distancia: 0.00 m</div>
        </div>
        <div class="metric-box">
          <div class="label">Perímetro: Estimado vs Real</div>
          <div class="val" id="mPerimPoly" style="color:var(--green)">0.000 m</div>
          <div class="sub" id="mPerimTrue">Real: -- m | Error: --%</div>
        </div>
        <div class="metric-box">
          <div class="label">Área: Polígono vs Hormigas vs Real</div>
          <div class="val" id="mAreaPoly" style="color:var(--accent)">0.000 m²</div>
          <div class="sub" id="mAreaAnts">Hormigas: -- m² | Real: -- m²</div>
        </div>
      </div>
    </div>

    <div class="card">
      <h2>Convergencia: Área Polígono (Verde) vs Hormigas (Ámbar)</h2>
      <canvas id="chartCanvas" width="500" height="215"></canvas>
    </div>
  </div>

  <!-- COLUMNA 3: REPORTE LATERAL DE RUNS GUARDADOS CON FOTO DEL MAPA -->
  <div class="card">
    <div style="display:flex;justify-content:space-between;align-items:center;">
      <h2>📋 Historial de Runs (<span id="runCountBadge">0</span>)</h2>
      <div style="display:flex;gap:4px;">
        <button onclick="refreshSavedRunsSidebar()" title="Refrescar lista">🔄</button>
        <button onclick="clearAllSavedRuns()" title="Borrar historial">🗑️</button>
      </div>
    </div>
    <div style="font-size:0.72rem;color:var(--muted);">
      Al completarse cada misión (o tocando 📸), se guarda automáticamente aquí con su foto del mapa completado y sus métricas.
    </div>
    <div class="reports-sidebar" id="reportsSidebar"></div>
  </div>
</div>

<!-- DASHBOARD ANALÍTICO COMPARATIVO (ACTUALIZABLE A DEMANDA) -->
<div class="analytics-section" id="dashboardSection">
  <div style="display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:10px;">
    <div>
      <h2 style="font-size:1rem;color:var(--accent);">📊 DASHBOARD COMPARATIVO DE RUNS Y RELACIONES DE MÉTRICAS</h2>
      <p style="font-size:0.76rem;color:var(--muted);">Compará los runs guardados por Tiempo, Precisión, Rebotes, Error %, Distancia Recorrida y analizá la relación entre dos variables cualesquiera.</p>
    </div>
    <div class="controls">
      <label style="font-size:0.78rem;color:var(--muted);">Filtrar Figura:</label>
      <select id="dashFilterRoom" onchange="updateComparativeDashboard()">
        <option value="ALL">Todas las Figuras</option>
      </select>
      <label style="font-size:0.78rem;color:var(--muted);">Métrica Barras:</label>
      <select id="dashBarMetric" onchange="updateComparativeDashboard()">
        <option value="total_time_s">Tiempo Total (s)</option>
        <option value="precision_pct" selected>Precisión Global (%)</option>
        <option value="area_error_pct">Error de Área (%)</option>
        <option value="perim_error_pct">Error de Perímetro (%)</option>
        <option value="wall_bounces">Rebotes / Choques Totales</option>
        <option value="total_distance_m">Distancia Recorrida (m)</option>
        <option value="relearn_time_s">Tiempo de Reaprendizaje (s)</option>
      </select>
      <label style="font-size:0.78rem;color:var(--muted);">Relación Eje X:</label>
      <select id="dashScatterX" onchange="updateComparativeDashboard()">
        <option value="total_distance_m" selected>Distancia Recorrida (m)</option>
        <option value="total_time_s">Tiempo Total (s)</option>
        <option value="wall_bounces">Rebotes / Impactos</option>
        <option value="relearn_count">Cant. Reaprendizajes</option>
      </select>
      <label style="font-size:0.78rem;color:var(--muted);">vs Eje Y:</label>
      <select id="dashScatterY" onchange="updateComparativeDashboard()">
        <option value="area_error_pct" selected>Error de Área (%)</option>
        <option value="precision_pct">Precisión Global (%)</option>
        <option value="perim_error_pct">Error de Perímetro (%)</option>
        <option value="total_time_s">Tiempo Total (s)</option>
      </select>
      <button class="dash-btn" onclick="updateComparativeDashboard()">🔄 Actualizar Dashboard</button>
    </div>
  </div>

  <div class="analytics-charts">
    <div class="card" style="background:#0f172a;">
      <h2 id="barChartTitle">Comparativa por Run</h2>
      <canvas id="compBarCanvas" width="620" height="250"></canvas>
    </div>
    <div class="card" style="background:#0f172a;">
      <h2 id="scatterChartTitle">Relación entre Variables (Cada punto = 1 Run)</h2>
      <canvas id="compScatterCanvas" width="620" height="250"></canvas>
    </div>
  </div>

  <div style="display:flex;gap:10px;align-items:center;flex-wrap:wrap;font-size:0.78rem;color:var(--muted);">
    <span>Filtrar tabla y gráficos:</span>
    <label>Precisión mínima (%):</label>
    <input type="number" id="dashMinPrecision" min="0" max="100" step="1" value="0" style="width:60px;" onchange="updateComparativeDashboard()" title="Ej: 100 muestra sólo los runs con 100% de precisión">
    <label>Modo:</label>
    <select id="dashFilterMode" onchange="updateComparativeDashboard()">
      <option value="ALL">Todos</option>
      <option value="classic">Clásico</option>
      <option value="recto">Recto</option>
      <option value="reaprendizaje">Reaprendizaje</option>
      <option value="recto_reaprendizaje">Recto+Reaprend.</option>
      <option value="bordeador">Bordeador</option>
      <option value="testeador">Testeador</option>
      <option value="incidente">Incidente</option>
    </select>
    <span style="font-size:0.72rem;color:#64748b;">(Tip: tocá el título de cualquier columna para ordenar la tabla)</span>
  </div>

  <div style="overflow-x:auto;">
    <table class="comp-table">
      <thead>
        <tr>
          <th class="sortable" onclick="sortCompTable('run_number')">Run #</th>
          <th class="sortable" onclick="sortCompTable('saved_at')">Hora</th>
          <th class="sortable" onclick="sortCompTable('mode')">Modo</th>
          <th class="sortable" onclick="sortCompTable('room_name')">Figura</th>
          <th class="sortable" onclick="sortCompTable('requested_chords')">Parámetros (Cuerdas / Rumbo)</th>
          <th class="sortable" onclick="sortCompTable('total_time_s')">Tiempo Total (Expl / Reap / Vuelta)</th>
          <th class="sortable" onclick="sortCompTable('precision_pct')">Precisión %</th>
          <th class="sortable" onclick="sortCompTable('area_error_pct')">Error Área / Perím</th>
          <th class="sortable" onclick="sortCompTable('wall_bounces')">Rebotes (Reap.)</th>
          <th class="sortable" onclick="sortCompTable('total_distance_m')">Distancia</th>
        </tr>
      </thead>
      <tbody id="compTableBody"></tbody>
    </table>
  </div>
</div>

<script>
let simData = null;
let frameIdx = 0;
let paused = false;
let drawingMode = false;
let customPoints = [];
let savedShapesMap = {};
let savedRunsList = [];
let currentRunAutoSaved = false;
let requestCounter = 0;
let compSortKey = 'precision_pct';
let compSortDir = -1;  // -1 = descendente, 1 = ascendente
let expandedRunNumber = null;

const scale = 118;
function toScreen(canvas, x, y) {
  return [canvas.width / 2 + x * scale, canvas.height / 2 - y * scale];
}
function toWorld(canvas, px, py) {
  return [(px - canvas.width / 2) / scale, (canvas.height / 2 - py) / scale];
}

function scrollToDashboard() {
  updateComparativeDashboard();
  document.getElementById('dashboardSection').scrollIntoView({ behavior: 'smooth' });
}

function onTurnModeChanged() {
  const tm = document.getElementById('turnModeSelect').value;
  document.getElementById('fixedTurnBox').style.display = (tm === 'fixed') ? 'inline-flex' : 'none';
  document.getElementById('seqTurnBox').style.display = (tm === 'sequence') ? 'inline-flex' : 'none';
}

async function refreshSavedShapesDropdown(selectShapeKey) {
  try {
    const res = await fetch('/api/shapes?t=' + Date.now());
    const data = await res.json();
    savedShapesMap = data.shapes || {};
    const group = document.getElementById('savedGroup');
    group.innerHTML = '';
    Object.keys(savedShapesMap).forEach((name) => {
      const opt = document.createElement('option');
      opt.value = 'saved:' + name;
      opt.textContent = '💾 ' + name;
      group.appendChild(opt);
    });
    if (selectShapeKey) {
      document.getElementById('roomSelect').value = selectShapeKey;
    }
  } catch (e) {}
}

async function refreshSavedRunsSidebar() {
  try {
    const res = await fetch('/api/runs?t=' + Date.now());
    const data = await res.json();
    savedRunsList = data.runs || [];
    renderReportsSidebar();
    populateDashboardRoomFilter();
  } catch (e) {}
}

async function clearAllSavedRuns() {
  await fetch('/api/runs_clear', { method: 'POST' });
  savedRunsList = [];
  expandedRunNumber = null;
  renderReportsSidebar();
  updateComparativeDashboard();
}

async function deleteSingleRun(runNumber, ev) {
  if (ev) ev.stopPropagation();
  const res = await fetch('/api/runs_delete', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({ run_number: runNumber })
  });
  const out = await res.json();
  savedRunsList = out.runs || [];
  if (expandedRunNumber === runNumber) expandedRunNumber = null;
  renderReportsSidebar();
  populateDashboardRoomFilter();
  updateComparativeDashboard();
}

function toggleRunExpand(runNumber) {
  expandedRunNumber = (expandedRunNumber === runNumber) ? null : runNumber;
  renderReportsSidebar();
}

// Compone una tarjeta de reporte (mapa + métricas) en un canvas offscreen y la
// descarga como PNG lista para pegar en Word / documentación.
async function downloadRunReportImage(runNumber, ev) {
  if (ev) ev.stopPropagation();
  const r = savedRunsList.find(x => x.run_number === runNumber);
  if (!r) return;

  const img = new Image();
  if (r.snapshot_data_url) {
    img.src = r.snapshot_data_url;
    try { await img.decode(); } catch (e) {}
  }

  const W = 960, pad = 28, headerH = 70;
  const mapW = W - pad * 2;
  const mapH = img.src ? Math.round(mapW * (img.naturalHeight || 340) / (img.naturalWidth || 480)) : 300;
  const rowsY = headerH + mapH + 66;
  const rowStep = 26;
  const nRows = 9;
  const H = rowsY + nRows * rowStep + 52;

  const cv = document.createElement('canvas');
  cv.width = W; cv.height = H;
  const ctx = cv.getContext('2d');
  const FONT = "'Segoe UI', Arial, sans-serif";

  // Fondo y marco estilo del dashboard
  ctx.fillStyle = '#0b1220'; ctx.fillRect(0, 0, W, H);
  ctx.strokeStyle = '#334155'; ctx.lineWidth = 2; ctx.strokeRect(1, 1, W - 2, H - 2);

  // Encabezado
  ctx.fillStyle = '#1e293b'; ctx.fillRect(0, 0, W, headerH);
  ctx.fillStyle = '#4ade80'; ctx.font = `bold 26px ${FONT}`;
  ctx.fillText(`RECORREDOR — Reporte Run #${r.run_number}`, pad, 45);
  ctx.textAlign = 'right';
  ctx.fillStyle = '#94a3b8'; ctx.font = `17px ${FONT}`;
  ctx.fillText(`seed ${r.seed}  |  ${r.saved_at || ''}`, W - pad, 45);
  ctx.textAlign = 'left';

  // Mapa con borde
  const my = headerH + 14;
  ctx.fillStyle = '#000'; ctx.fillRect(pad - 2, my - 2, mapW + 4, mapH + 4);
  ctx.strokeStyle = '#475569'; ctx.strokeRect(pad - 2, my - 2, mapW + 4, mapH + 4);
  if (img.src) {
    ctx.imageSmoothingEnabled = true;
    ctx.imageSmoothingQuality = 'high';
    ctx.drawImage(img, pad, my, mapW, mapH);
  }

  // Métricas en dos columnas
  let ty = my + mapH + 36;
  ctx.font = `bold 19px ${FONT}`; ctx.fillStyle = '#e2e8f0';
  ctx.fillText(`MODO: ${String(r.mode).toUpperCase()}    |    FIGURA: ${r.room_name}`, pad, ty);

  const col1 = [
    ['Cuerdas recorridas', `${r.actual_chords}/${r.requested_chords}`],
    ['Tiempo total', `${r.total_time_s} s`],
    ['Tiempo exploración', `${r.exploration_time_s} s`],
    ['Tiempo reaprendizaje', `${r.relearn_time_s} s (${r.relearn_count} recálcs, ${r.total_relearn_collisions} choques)`],
    ['Tiempo vuelta final', `${r.final_lap_time_s} s (${r.perimeter_lap_collisions} colis.)`],
    ['Distancia recorrida', `${r.total_distance_m} m`],
    ['Rebotes / impactos', `${r.wall_bounces}`],
    ['Variación de rumbo', `${r.turn_mode}`],
  ];
  const col2 = [
    ['Área estimada / real', `${r.final_poly_area_m2} / ${r.true_area_m2} m²`],
    ['Área hormigas (estocástica)', `${r.final_ants_area_m2} m²`],
    ['Perímetro est. / real', `${r.final_poly_perim_m} / ${r.true_perimeter_m} m`],
    ['Error de área', `${r.area_error_pct}%`],
    ['Error de perímetro', `${r.perim_error_pct}%`],
    ['PRECISIÓN GLOBAL', `${r.precision_pct}%`],
    ['Reaprendizajes', `${r.relearn_count}`],
    ['Colisiones vuelta final', `${r.perimeter_lap_collisions}`],
    ['Corte por perímetro real', r.early_stopped ? `SÍ — frenó en ${r.actual_chords}/${r.requested_chords} cuerdas` : (r.early_stop_enabled ? 'activo (no alcanzó)' : 'desactivado')],
  ];
  ctx.font = `16px ${FONT}`;
  col1.forEach((row, i) => {
    const y = rowsY + i * rowStep;
    ctx.fillStyle = '#7dd3fc'; ctx.fillText(row[0], pad, y);
    ctx.fillStyle = '#e2e8f0'; ctx.fillText(String(row[1]), pad + 235, y);
  });
  const x2 = pad + W / 2 - 10;
  col2.forEach((row, i) => {
    const y = rowsY + i * rowStep;
    ctx.fillStyle = '#fbbf24'; ctx.fillText(row[0], x2, y);
    ctx.fillStyle = '#e2e8f0'; ctx.fillText(String(row[1]), x2 + 225, y);
  });

  // Pie
  ctx.fillStyle = '#64748b'; ctx.font = `13px ${FONT}`;
  ctx.fillText('Generado por Recorredor Simulador Web — Robot explorador de habitaciones (ESP32 + Python)', pad, H - 18);

  cv.toBlob((blob) => {
    if (!blob) return;
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `recorredor_run${r.run_number}_${r.mode}_${String(r.room_name).replace(/[^\w\-]+/g, '_')}.png`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(a.href), 3000);
  }, 'image/png');
}

function renderReportsSidebar() {
  document.getElementById('runCountBadge').textContent = String(savedRunsList.length);
  const container = document.getElementById('reportsSidebar');
  container.innerHTML = '';
  // Mostrar del más reciente al más antiguo
  const reversed = [...savedRunsList].reverse();
  reversed.forEach((r) => {
    const div = document.createElement('div');
    div.className = 'report-item';
    div.setAttribute('onclick', `toggleRunExpand(${r.run_number})`);
    const thumbSrc = r.snapshot_data_url || '';
    let detailHtml = '';
    if (expandedRunNumber === r.run_number) {
      detailHtml = `
        <div class="run-detail" onclick="event.stopPropagation()">
          <img src="${thumbSrc}" alt="Mapa Run #${r.run_number}"/>
          <div class="det-grid">
            <div><b>Run:</b> #${r.run_number} (seed ${r.seed})</div>
            <div><b>Modo:</b> ${r.mode}</div>
            <div><b>Figura:</b> ${r.room_name}</div>
            <div><b>Cuerdas:</b> ${r.actual_chords}/${r.requested_chords}${r.early_stopped ? ' <span style="color:#fbbf24">⚡corte x perímetro</span>' : ''}</div>
            <div><b>Tiempo total:</b> ${r.total_time_s} s</div>
            <div><b>Exploración:</b> ${r.exploration_time_s} s</div>
            <div><b>Reaprendizaje:</b> ${r.relearn_time_s} s (${r.relearn_count} recálcs, ${r.total_relearn_collisions} choques)</div>
            <div><b>Vuelta final:</b> ${r.final_lap_time_s} s (${r.perimeter_lap_collisions} colis.)</div>
            <div><b>Área est/real:</b> ${r.final_poly_area_m2} / ${r.true_area_m2} m²</div>
            <div><b>Área hormigas:</b> ${r.final_ants_area_m2} m²</div>
            <div><b>Perím est/real:</b> ${r.final_poly_perim_m} / ${r.true_perimeter_m} m</div>
            <div><b>Error área/perím:</b> ${r.area_error_pct}% / ${r.perim_error_pct}%</div>
            <div><b>Precisión:</b> ${r.precision_pct}%</div>
            <div><b>Distancia:</b> ${r.total_distance_m} m</div>
            <div><b>Rebotes:</b> ${r.wall_bounces}</div>
            <div><b>Rumbo:</b> ${r.turn_mode}</div>
          </div>
          <div><button class="dl-run-big" onclick="downloadRunReportImage(${r.run_number}, event)">⬇️ Descargar reporte como imagen PNG</button></div>
        </div>`;
    }
    div.innerHTML = `
      <img src="${thumbSrc}" alt="Mapa Run #${r.run_number}"/>
      <div class="report-meta">
        <div><strong>Run #${r.run_number} (${r.seed})</strong> — <span style="color:#4ade80">${r.mode}</span>
          <button class="dl-run-btn" title="Descargar reporte PNG" onclick="downloadRunReportImage(${r.run_number}, event)">⬇️</button>
          <button class="del-run-btn" title="Borrar este run" onclick="deleteSingleRun(${r.run_number}, event)">🗑️</button>
        </div>
        <div><b>Figura:</b> ${r.room_name}</div>
        <div><b>Paráms:</b> ${r.requested_chords} cuerdas | Rumbo: ${r.turn_mode}${r.early_stopped ? ' | <span style="color:#fbbf24">⚡corte x perímetro</span>' : ''}</div>
        <div><b>Tiempo:</b> ${r.total_time_s}s (Reap: ${r.relearn_time_s}s)</div>
        <div><b>Área:</b> ${r.final_poly_area_m2} / ${r.true_area_m2} m² (Err: ${r.area_error_pct}%)</div>
        <div><b>Perím:</b> ${r.final_poly_perim_m} / ${r.true_perimeter_m} m (Prec: ${r.precision_pct}%)</div>
        <div><b>Dist:</b> ${r.total_distance_m}m | <b>Rebotes:</b> ${r.wall_bounces} (Reap: ${r.relearn_count})</div>
      </div>
      ${detailHtml}
    `;
    container.appendChild(div);
  });
}

async function saveCurrentRunReport(forceJumpToEnd) {
  if (!simData) return;
  if (forceJumpToEnd && frameIdx < simData.frames.length - 1) {
    frameIdx = simData.frames.length - 1;
    drawMap();
  }
  const mapCanvas = document.getElementById('mapCanvas');
  // Miniatura JPEG para el reporte lateral (tamaño suficiente para descarga nítida)
  const thumbCanvas = document.createElement('canvas');
  thumbCanvas.width = 480;
  thumbCanvas.height = 340;
  const tctx = thumbCanvas.getContext('2d');
  tctx.drawImage(mapCanvas, 0, 0, thumbCanvas.width, thumbCanvas.height);
  const dataUrl = thumbCanvas.toDataURL('image/jpeg', 0.85);

  const payload = {
    seed: simData.seed,
    mode: simData.mode,
    room_name: simData.room_name,
    requested_chords: simData.requested_chords,
    actual_chords: simData.actual_chords,
    early_stop_enabled: simData.early_stop_enabled,
    early_stopped: simData.early_stopped,
    turn_mode: simData.turn_mode,
    wall_bounces: simData.wall_bounces,
    relearn_count: simData.relearn_count,
    total_relearn_collisions: simData.total_relearn_collisions,
    perimeter_lap_collisions: simData.perimeter_lap_collisions,
    total_time_s: simData.timings.total_time_s,
    exploration_time_s: simData.timings.exploration_time_s,
    relearn_time_s: simData.timings.relearn_time_s,
    final_lap_time_s: simData.timings.final_lap_time_s,
    true_area_m2: simData.true_area_m2,
    final_poly_area_m2: simData.final_poly_area_m2,
    final_ants_area_m2: simData.final_ants_area_m2,
    true_perimeter_m: simData.true_perimeter_m,
    final_poly_perim_m: simData.final_poly_perim_m,
    area_error_pct: simData.area_error_pct,
    perim_error_pct: simData.perim_error_pct,
    precision_pct: simData.precision_pct,
    total_distance_m: simData.total_distance_m,
    snapshot_data_url: dataUrl
  };

  try {
    const res = await fetch('/api/runs', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(payload)
    });
    const out = await res.json();
    savedRunsList = out.runs || [];
    renderReportsSidebar();
    populateDashboardRoomFilter();
    updateComparativeDashboard();
  } catch (e) {}
}

function populateDashboardRoomFilter() {
  const sel = document.getElementById('dashFilterRoom');
  const currentVal = sel.value;
  const rooms = Array.from(new Set(savedRunsList.map(r => r.room_name)));
  sel.innerHTML = '<option value="ALL">Todas las Figuras</option>';
  rooms.forEach(rn => {
    const opt = document.createElement('option');
    opt.value = rn;
    opt.textContent = rn;
    sel.appendChild(opt);
  });
  if (rooms.includes(currentVal)) sel.value = currentVal;
}

const MODE_COLORS = {
  classic: '#38bdf8',
  recto: '#a855f7',
  reaprendizaje: '#f59e0b',
  recto_reaprendizaje: '#22c55e',
  bordeador: '#ec4899',
  testeador: '#eab308',
  incidente: '#14b8a6'
};

function sortCompTable(key) {
  if (compSortKey === key) {
    compSortDir = -compSortDir;
  } else {
    compSortKey = key;
    compSortDir = (key === 'precision_pct' || key === 'total_time_s' || key === 'run_number') ? -1 : 1;
  }
  updateComparativeDashboard();
}

function updateComparativeDashboard() {
  // Marcar la columna activa con ▲/▼
  document.querySelectorAll('th.sortable').forEach(th => {
    th.classList.remove('sorted-asc', 'sorted-desc');
    if (th.getAttribute('onclick') === `sortCompTable('${compSortKey}')`) {
      th.classList.add(compSortDir === 1 ? 'sorted-asc' : 'sorted-desc');
    }
  });
  const roomFilter = document.getElementById('dashFilterRoom').value;
  const modeFilter = document.getElementById('dashFilterMode').value;
  const minPrec = parseFloat(document.getElementById('dashMinPrecision').value) || 0;
  const barMetric = document.getElementById('dashBarMetric').value;
  const scatterX = document.getElementById('dashScatterX').value;
  const scatterY = document.getElementById('dashScatterY').value;

  const filtered = savedRunsList.filter(r =>
    (roomFilter === 'ALL' || r.room_name === roomFilter) &&
    (modeFilter === 'ALL' || r.mode === modeFilter) &&
    Number(r.precision_pct || 0) >= minPrec
  );

  // Ordenar según la columna clickeada
  filtered.sort((a, b) => {
    const va = a[compSortKey];
    const vb = b[compSortKey];
    const na = Number(va), nb = Number(vb);
    if (!isNaN(na) && !isNaN(nb)) return (na - nb) * compSortDir;
    return String(va ?? '').localeCompare(String(vb ?? '')) * compSortDir;
  });

  // 1. Poblar tabla comparativa
  const tbody = document.getElementById('compTableBody');
  tbody.innerHTML = '';
  filtered.forEach(r => {
    const tr = document.createElement('tr');
    const color = MODE_COLORS[r.mode] || '#38bdf8';
    tr.innerHTML = `
      <td><b>#${r.run_number}</b> (${r.seed})</td>
      <td>${r.saved_at || ''}</td>
      <td><span style="color:${color};font-weight:700;">${r.mode.toUpperCase()}</span></td>
      <td>${r.room_name}</td>
      <td>${r.actual_chords}/${r.requested_chords} cuerdas | ${r.turn_mode}</td>
      <td><b>${r.total_time_s}s</b> (${r.exploration_time_s}s / ${r.relearn_time_s}s / ${r.final_lap_time_s}s)</td>
      <td style="color:#4ade80;font-weight:700;">${r.precision_pct}%</td>
      <td>Área: ${r.area_error_pct}% | Perím: ${r.perim_error_pct}%</td>
      <td>${r.wall_bounces} impactos (${r.relearn_count} reap.)</td>
      <td>${r.total_distance_m} m</td>
    `;
    tbody.appendChild(tr);
  });

  // 2. Dibujar Gráfico de Barras Comparativo
  const bCanvas = document.getElementById('compBarCanvas');
  const bCtx = bCanvas.getContext('2d');
  bCtx.clearRect(0, 0, bCanvas.width, bCanvas.height);

  if (filtered.length === 0) {
    bCtx.fillStyle = '#94a3b8';
    bCtx.font = '13px Segoe UI';
    bCtx.fillText('Aún no hay runs completados para graficar. Dejá terminar un run o tocá 📸 Guardar Reporte.', 24, 120);
  } else {
    const padL = 48, padR = 16, padT = 24, padB = 42;
    const w = bCanvas.width - padL - padR;
    const h = bCanvas.height - padT - padB;
    const vals = filtered.map(r => Number(r[barMetric] || 0));
    const maxV = Math.max(...vals, 1.0) * 1.15;

    const barW = Math.min(44, Math.max(12, (w / filtered.length) * 0.68));
    const stepW = w / filtered.length;

    filtered.forEach((r, idx) => {
      const val = Number(r[barMetric] || 0);
      const bh = (val / maxV) * h;
      const bx = padL + idx * stepW + (stepW - barW) / 2;
      const by = padT + h - bh;
      bCtx.fillStyle = MODE_COLORS[r.mode] || '#38bdf8';
      bCtx.fillRect(bx, by, barW, bh);

      bCtx.fillStyle = '#f8fafc';
      bCtx.font = '10px Segoe UI';
      bCtx.fillText(val.toFixed(1), bx, by - 4);

      bCtx.fillStyle = '#94a3b8';
      bCtx.fillText(`#${r.run_number} ${r.mode.slice(0,5)}`, bx - 2, padT + h + 15);
    });
  }

  // 3. Dibujar Scatter Plot de Relación X vs Y
  const sCanvas = document.getElementById('compScatterCanvas');
  const sCtx = sCanvas.getContext('2d');
  sCtx.clearRect(0, 0, sCanvas.width, sCanvas.height);

  if (filtered.length > 0) {
    const padL = 48, padR = 20, padT = 24, padB = 36;
    const w = sCanvas.width - padL - padR;
    const h = sCanvas.height - padT - padB;

    const xs = filtered.map(r => Number(r[scatterX] || 0));
    const ys = filtered.map(r => Number(r[scatterY] || 0));
    const minX = Math.min(...xs) * 0.9, maxX = Math.max(...xs, minX + 1.0) * 1.08;
    const minY = 0.0, maxY = Math.max(...ys, 1.0) * 1.15;

    sCtx.strokeStyle = '#334155';
    sCtx.strokeRect(padL, padT, w, h);

    filtered.forEach(r => {
      const vx = Number(r[scatterX] || 0);
      const vy = Number(r[scatterY] || 0);
      const px = padL + ((vx - minX) / Math.max(1e-6, maxX - minX)) * w;
      const py = padT + h - ((vy - minY) / Math.max(1e-6, maxY - minY)) * h;

      sCtx.fillStyle = MODE_COLORS[r.mode] || '#38bdf8';
      sCtx.beginPath();
      sCtx.arc(px, py, 6, 0, Math.PI * 2);
      sCtx.fill();
      sCtx.strokeStyle = '#f8fafc';
      sCtx.lineWidth = 1.2;
      sCtx.stroke();

      sCtx.fillStyle = '#cbd5e1';
      sCtx.font = '10px Segoe UI';
      sCtx.fillText(`#${r.run_number}(${r.mode.slice(0,4)})`, px + 8, py + 3);
    });
  }
}

document.getElementById('mapCanvas').addEventListener('click', (ev) => {
  if (!drawingMode) return;
  const canvas = ev.target;
  const rect = canvas.getBoundingClientRect();
  const px = (ev.clientX - rect.left) * (canvas.width / rect.width);
  const py = (ev.clientY - rect.top) * (canvas.height / rect.height);
  let [wx, wy] = toWorld(canvas, px, py);

  const mode = document.getElementById('modeSelect').value;
  if (mode === 'recto' || mode === 'recto_reaprendizaje' || mode === 'testeador' || mode === 'incidente') {
    wx = Math.round(wx * 10) / 10;
    wy = Math.round(wy * 10) / 10;
    if (customPoints.length > 0) {
      const [prevX, prevY] = customPoints[customPoints.length - 1];
      if (Math.abs(wx - prevX) >= Math.abs(wy - prevY)) {
        wy = prevY;
      } else {
        wx = prevX;
      }
    }
  }
  customPoints.push([ Number(wx.toFixed(3)), Number(wy.toFixed(3)) ]);
});

async function onSelectRoomDropdown() {
  const val = document.getElementById('roomSelect').value;
  if (val === 'custom') {
    if (customPoints.length < 3) {
      openCustomDrawEditor();
      return;
    }
  } else if (val.startsWith('saved:')) {
    const name = val.slice(6);
    if (savedShapesMap[name]) {
      customPoints = savedShapesMap[name].map(pt => [pt[0], pt[1]]);
    }
  }
  drawingMode = false;
  document.getElementById('drawHint').style.display = 'none';
  await fetchSimulationData(true);
}

async function onModeDropdownChanged() {
  updateTestParamsVisibility();
  if (drawingMode) {
    updateDrawBannerText();
    return;
  }
  await fetchSimulationData(true);
}

function updateTestParamsVisibility() {
  const mode = document.getElementById('modeSelect').value;
  document.getElementById('testParamsBox').style.display = (mode === 'testeador') ? 'inline-flex' : 'none';
}

function updateDrawBannerText() {
  const mode = document.getElementById('modeSelect').value;
  const isOrtho = (mode === 'recto' || mode === 'recto_reaprendizaje' || mode === 'testeador' || mode === 'incidente');
  document.getElementById('drawHintText').textContent = isOrtho
    ? 'MODO RECTO 90°: Tus clics trazan paredes a 90° exactos.'
    : 'MODO LIBRE: Hacé clic alrededor del centro (0,0) para crear tu figura.';
}

function openCustomDrawEditor() {
  paused = true;
  drawingMode = true;
  customPoints = [];
  document.getElementById('roomSelect').value = 'custom';
  document.getElementById('drawHint').style.display = 'flex';
  updateDrawBannerText();
  document.getElementById('fsmBadge').textContent = 'DIBUJANDO NUEVA HABITACIÓN';
  document.getElementById('fsmBadge').style.background = '#7c3aed';
}

function undoLastPoint() {
  if (customPoints.length > 0) customPoints.pop();
}

function clearCustomPoints() {
  customPoints = [];
}

async function saveCurrentCustomShape() {
  if (customPoints.length < 3) {
    alert('Marcá al menos 3 esquinas en el mapa antes de guardar.');
    return;
  }
  const rawName = document.getElementById('saveShapeName').value.trim();
  const shapeName = rawName || ('Figura_' + Math.floor(100 + Math.random() * 900));
  const mode = document.getElementById('modeSelect').value;

  const res = await fetch('/api/shapes', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({ name: shapeName, vertices: customPoints, mode: mode })
  });
  const out = await res.json();
  if (out.normalized_vertices) {
    customPoints = out.normalized_vertices;
  }
  drawingMode = false;
  document.getElementById('drawHint').style.display = 'none';
  await refreshSavedShapesDropdown('saved:' + shapeName);
  await fetchSimulationData(true);
}

async function finishCustomDrawAndRun() {
  if (customPoints.length < 3) {
    alert('Marcá al menos 3 esquinas alrededor del centro (0,0).');
    return;
  }
  drawingMode = false;
  document.getElementById('drawHint').style.display = 'none';
  document.getElementById('roomSelect').value = 'custom';
  await fetchSimulationData(true);
}

async function startSimulationNow() {
  if (drawingMode) {
    await finishCustomDrawAndRun();
    return;
  }
  await fetchSimulationData(true);
}

async function fetchSimulationData(autoPlayAfterLoad = true) {
  const myReqId = ++requestCounter;
  paused = true;
  frameIdx = 0;
  currentRunAutoSaved = false;
  // Limpiar el run anterior de inmediato: el mapa viejo no puede repetirse
  // ni confundirse con el nuevo mientras el servidor calcula.
  simData = null;

  const badge = document.getElementById('fsmBadge');
  badge.textContent = 'CALCULANDO RUN...';
  badge.style.background = '#d97706';

  const room = document.getElementById('roomSelect').value;
  const mode = document.getElementById('modeSelect').value;
  const chords = parseInt(document.getElementById('chordsInput').value, 10) || 32;
  const maxRelearns = parseInt(document.getElementById('maxRelearnsInput').value, 10);
  const validMaxRelearns = isNaN(maxRelearns) ? 15 : maxRelearns;
  const turnMode = document.getElementById('turnModeSelect').value;
  const fixedDeg = parseFloat(document.getElementById('fixedTurnInput').value) || 135.0;
  const seqStr = document.getElementById('seqTurnInput').value || '105, 135, 155';
  const testStride = parseFloat(document.getElementById('testStrideInput').value) || 0.42;
  const testCorner = parseFloat(document.getElementById('testCornerInput').value) || 0.12;
  const earlyStop = document.getElementById('earlyStopChk').checked;
  const randSeed = Math.floor(Math.random() * 900000) + 10000;

  if (room === 'custom' && customPoints.length < 3) {
    badge.textContent = 'FIGURA PERSONALIZADA VACÍA — dibujá la habitación o elegí otra figura';
    badge.style.background = '#dc2626';
    alert('La figura personalizada tiene menos de 3 puntos. Dibujá la habitación o elegí otra figura antes de iniciar el run.');
    return;
  }

  try {
    let res;
    if (room === 'custom' && customPoints.length >= 3) {
      res = await fetch('/api/sim_custom?t=' + Date.now(), {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
          vertices: customPoints,
          mode: mode,
          max_chords: chords,
          max_relearns: validMaxRelearns,
          turn_mode: turnMode,
          fixed_turn_deg: fixedDeg,
          turn_seq_str: seqStr,
          test_stride_m: testStride,
          test_corner_pass_m: testCorner,
          early_stop: earlyStop,
          seed: randSeed
        })
      });
    } else {
      const params = new URLSearchParams({
        room: room,
        mode: mode,
        max_chords: String(chords),
        max_relearns: String(validMaxRelearns),
        turn_mode: turnMode,
        fixed_turn_deg: String(fixedDeg),
        turn_seq_str: seqStr,
        test_stride_m: String(testStride),
        test_corner_pass_m: String(testCorner),
        early_stop: earlyStop ? '1' : '0',
        seed: String(randSeed),
        t: String(Date.now())
      });
      res = await fetch('/api/sim?' + params.toString());
    }
    if (!res.ok) {
      let msg = 'HTTP ' + res.status;
      try {
        const errBody = await res.json();
        if (errBody && errBody.error) msg = errBody.error;
      } catch (e) {}
      throw new Error(msg);
    }
    const data = await res.json();
    if (data && data.error) throw new Error(data.error);
    if (myReqId !== requestCounter) return;

    simData = data;
    document.getElementById('roomTitle').textContent = simData.room_name;
    document.getElementById('mSeed').textContent = `Run #${simData.seed} (${mode.toUpperCase()})`;
    document.getElementById('mInitAngle').textContent =
      `Salida θ₀: ${simData.initial_angle_deg}° | Rumbo: ${turnMode} | Cuerdas: ${simData.chords.length}/${simData.requested_chords}`;

    const tm = simData.timings;
    document.getElementById('mTimeTotal').textContent = `${tm.total_time_s.toFixed(2)} s totales`;
    document.getElementById('mTimeBreakdown').textContent =
      `Explor.: ${tm.exploration_time_s.toFixed(1)}s | Reap.: ${tm.relearn_time_s.toFixed(1)}s | Vuelta: ${tm.final_lap_time_s.toFixed(1)}s`;

    document.getElementById('mRelearnStats').textContent =
      `${simData.relearn_count}/${simData.max_relearns} recálculos | ${simData.total_relearn_collisions} choques reap.`;
    document.getElementById('mRelearnSub').textContent =
      `Tiempo reaprend.: ${tm.relearn_time_s.toFixed(2)} s | Choques vuelta final: ${simData.perimeter_lap_collisions}`;

    frameIdx = 0;
    paused = !autoPlayAfterLoad;
    document.getElementById('pauseBtn').textContent = paused ? '▶ Reanudar' : '⏸ Pausar';
  } catch (err) {
    console.error(err);
    if (myReqId !== requestCounter) return;
    simData = null;
    badge.textContent = 'ERROR EN EL RUN — ' + String(err.message || err);
    badge.style.background = '#dc2626';
    document.getElementById('pauseBtn').textContent = '⏸ Pausar';
  }
}

function togglePause() {
  if (!simData) return;
  paused = !paused;
  document.getElementById('pauseBtn').textContent = paused ? '▶ Reanudar' : '⏸ Pausar';
}

function skipToPerimeter() {
  if (!simData) return;
  frameIdx = simData.perimeter_start_frame;
  paused = false;
  document.getElementById('pauseBtn').textContent = '⏸ Pausar';
}

function getCompletedChordCount() {
  if (!simData) return 0;
  let count = 0;
  for (let i = 0; i < simData.chords.length; i++) {
    if (frameIdx >= simData.chords[i].end_frame) {
      count = i + 1;
    } else {
      break;
    }
  }
  return count;
}

function drawMap() {
  const canvas = document.getElementById('mapCanvas');
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  ctx.strokeStyle = '#162032';
  ctx.lineWidth = 1;
  for (let gx = -3; gx <= 3; gx += 0.5) {
    const [sx] = toScreen(canvas, gx, 0);
    ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, canvas.height); ctx.stroke();
  }
  for (let gy = -2; gy <= 2; gy += 0.5) {
    const [, sy] = toScreen(canvas, 0, gy);
    ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(canvas.width, sy); ctx.stroke();
  }

  const [ox, oy] = toScreen(canvas, 0, 0);
  ctx.strokeStyle = '#334155';
  ctx.beginPath(); ctx.arc(ox, oy, 6, 0, Math.PI * 2); ctx.stroke();

  if (drawingMode) {
    if (customPoints.length > 0) {
      ctx.strokeStyle = '#a855f7';
      ctx.lineWidth = 3;
      ctx.beginPath();
      customPoints.forEach((pt, idx) => {
        const [sx, sy] = toScreen(canvas, pt[0], pt[1]);
        if (idx === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
      });
      if (customPoints.length >= 3) ctx.closePath();
      ctx.stroke();

      ctx.fillStyle = '#f3e8ff';
      customPoints.forEach((pt) => {
        const [sx, sy] = toScreen(canvas, pt[0], pt[1]);
        ctx.beginPath(); ctx.arc(sx, sy, 5, 0, Math.PI * 2); ctx.fill();
      });
    }
    return;
  }

  if (!simData) return;

  // 1. Paredes Reales
  ctx.strokeStyle = '#64748b';
  ctx.lineWidth = 4;
  ctx.beginPath();
  simData.room_vertices.forEach((v, i) => {
    const [sx, sy] = toScreen(canvas, v.x, v.y);
    if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
  });
  ctx.closePath();
  ctx.stroke();

  const pose = simData.frames[Math.min(frameIdx, simData.frames.length - 1)];
  const completedChords = getCompletedChordCount();
  const inPerimeterPhase = frameIdx >= simData.perimeter_start_frame;

  // 2. Cuerdas recorridas
  if (frameIdx > 0) {
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.65)';
    ctx.lineWidth = 2.0;
    for (let i = 0; i < simData.chords.length; i++) {
      const c = simData.chords[i];
      if (frameIdx >= c.end_frame) {
        const [x1, y1] = toScreen(canvas, c.x1, c.y1);
        const [x2, y2] = toScreen(canvas, c.x2, c.y2);
        ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
      } else if (frameIdx >= c.start_frame && !inPerimeterPhase) {
        const [x1, y1] = toScreen(canvas, c.x1, c.y1);
        const [rx, ry] = toScreen(canvas, pose.x, pose.y);
        ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(rx, ry); ctx.stroke();
        break;
      }
    }
  }

  // 3. Puntos de impacto en pared
  const visibleHits = inPerimeterPhase ? simData.wall_hits.length : Math.min(completedChords, simData.wall_hits.length);
  ctx.fillStyle = '#ef4444';
  for (let i = 0; i < visibleHits; i++) {
    const wp = simData.wall_hits[i];
    const [wx, wy] = toScreen(canvas, wp.x, wp.y);
    ctx.beginPath(); ctx.arc(wx, wy, 5.2, 0, Math.PI * 2); ctx.fill();
  }

  // 4. Cruces del Método de las Hormigas
  const histIdx = completedChords - 1;
  const activeCrossings = (histIdx >= 0 && histIdx < simData.estimates_history.length)
    ? simData.estimates_history[histIdx].crossings
    : 0;
  ctx.fillStyle = '#f59e0b';
  for (let i = 0; i < Math.min(activeCrossings, simData.crossings.length); i++) {
    const cr = simData.crossings[i];
    const [kx, ky] = toScreen(canvas, cr.x, cr.y);
    ctx.beginPath(); ctx.arc(kx, ky, 3.6, 0, Math.PI * 2); ctx.fill();
  }

  // 5. Perímetro Virtual Detectado y rastro del recorrido final (SIN DIAGONAL)
  if (inPerimeterPhase && simData.reconstructed_perimeter.length >= 3) {
    ctx.strokeStyle = '#ef4444';
    ctx.setLineDash([8, 5]);
    ctx.lineWidth = 2.6;
    ctx.beginPath();
    simData.reconstructed_perimeter.forEach((v, i) => {
      const [sx, sy] = toScreen(canvas, v.x, v.y);
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    });
    ctx.closePath();
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.strokeStyle = '#22c55e';
    ctx.lineWidth = 4;
    ctx.beginPath();
    for (let f = simData.perimeter_start_frame; f <= frameIdx; f++) {
      const fp = simData.frames[f];
      const [px, py] = toScreen(canvas, fp.x, fp.y);
      if (f === simData.perimeter_start_frame) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    }
    ctx.stroke();
  }

  // 6. Dibujar el Robot
  const [rx, ry] = toScreen(canvas, pose.x, pose.y);
  const radPx = simData.robot_radius_m * scale;

  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(-pose.theta);
  ctx.fillStyle = inPerimeterPhase ? '#16a34a' : '#0284c7';
  ctx.strokeStyle = '#f8fafc';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(0, 0, radPx, 0, Math.PI * 2); ctx.fill(); ctx.stroke();

  ctx.strokeStyle = '#fde047';
  ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(radPx + 5, 0); ctx.stroke();
  ctx.restore();

  const badge = document.getElementById('fsmBadge');
  if (frameIdx >= simData.frames.length - 1) {
    badge.textContent = `MISIÓN COMPLETADA (${simData.relearn_count} REAPREND. | ${simData.timings.total_time_s}s)`;
    badge.style.background = '#16a34a';
    if (!currentRunAutoSaved) {
      currentRunAutoSaved = true;
      saveCurrentRunReport(false);
    }
  } else if (inPerimeterPhase) {
    badge.textContent = `RECORRIENDO PERÍMETRO A R×1.2 (REAPREND.: ${simData.relearn_count})`;
    badge.style.background = '#15803d';
  } else {
    badge.textContent = `EXPLORANDO [${simData.mode.toUpperCase()}] (${completedChords}/${simData.chords.length})`;
    badge.style.background = '#0284c7';
  }

  const effectiveIdx = inPerimeterPhase
    ? simData.estimates_history.length - 1
    : histIdx;

  if (effectiveIdx >= 0 && effectiveIdx < simData.estimates_history.length) {
    const h = simData.estimates_history[effectiveIdx];
    document.getElementById('mChords').textContent = `${h.chords} cuerdas | ${h.crossings} cruces`;
    document.getElementById('mDist').textContent =
      `Distancia: ${simData.total_distance_m.toFixed(2)} m (Rebotes: ${simData.wall_bounces})`;
    document.getElementById('mPerimPoly').textContent = `${h.poly_perim.toFixed(3)} m`;
    document.getElementById('mPerimTrue').textContent =
      `Real: ${simData.true_perimeter_m.toFixed(3)} m | Precisión: ${simData.precision_pct}%`;
    document.getElementById('mAreaPoly').textContent = `${h.poly_area.toFixed(3)} m²`;
    document.getElementById('mAreaAnts').textContent =
      `Hormigas: ${h.ants_area.toFixed(2)} m² | Real: ${simData.true_area_m2.toFixed(2)} m²`;
  }
}

function drawConvergenceChart() {
  if (!simData || drawingMode) return;
  const canvas = document.getElementById('chartCanvas');
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const padL = 42, padR = 14, padT = 16, padB = 24;
  const w = canvas.width - padL - padR;
  const h = canvas.height - padT - padB;
  const maxY = Math.max(simData.true_area_m2 * 1.45, 16);
  const hist = simData.estimates_history;
  const activeCount = (frameIdx >= simData.perimeter_start_frame)
    ? hist.length
    : Math.min(getCompletedChordCount(), hist.length);

  const toXY = (idx, val) => [
    padL + (idx / Math.max(1, hist.length - 1)) * w,
    padT + h - (Math.min(val, maxY) / maxY) * h
  ];

  const [, trueY] = toXY(0, simData.true_area_m2);
  ctx.strokeStyle = '#94a3b8';
  ctx.setLineDash([5, 5]);
  ctx.beginPath(); ctx.moveTo(padL, trueY); ctx.lineTo(padL + w, trueY); ctx.stroke();
  ctx.setLineDash([]);

  if (activeCount < 1) return;

  ctx.strokeStyle = '#22c55e';
  ctx.lineWidth = 2.2;
  ctx.beginPath();
  for (let i = 0; i < activeCount; i++) {
    const [x, y] = toXY(i, hist[i].poly_area);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }
  ctx.stroke();

  ctx.strokeStyle = '#f59e0b';
  ctx.lineWidth = 2.2;
  ctx.beginPath();
  for (let i = 0; i < activeCount; i++) {
    const [x, y] = toXY(i, hist[i].ants_area);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }
  ctx.stroke();
}

function tick() {
  if (simData && !paused && !drawingMode) {
    const speed = parseInt(document.getElementById('speedRange').value, 10) || 6;
    if (frameIdx < simData.frames.length - 1) {
      frameIdx = Math.min(simData.frames.length - 1, frameIdx + speed);
    }
  }
  drawMap();
  drawConvergenceChart();
  requestAnimationFrame(tick);
}

(async function init() {
  updateTestParamsVisibility();
  await refreshSavedShapesDropdown();
  await refreshSavedRunsSidebar();
  await fetchSimulationData(true);
  updateComparativeDashboard();
  requestAnimationFrame(tick);
})();
</script>
</body>
</html>
"""


class LiveSimRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format: str, *args) -> None:
        return

    def do_HEAD(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.end_headers()

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/shapes":
            shapes = load_saved_shapes()
            body = json.dumps({"shapes": shapes}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if parsed.path == "/api/runs":
            runs = load_saved_runs()
            body = json.dumps({"runs": runs}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if parsed.path == "/api/sim":
            qs = parse_qs(parsed.query)
            room_type = qs.get("room", ["rect"])[0]
            mode = qs.get("mode", ["classic"])[0]
            max_chords = int(qs.get("max_chords", ["32"])[0])
            max_relearns = int(qs.get("max_relearns", ["15"])[0])
            turn_mode = qs.get("turn_mode", ["random"])[0]
            fixed_turn_deg = float(qs.get("fixed_turn_deg", ["135.0"])[0])
            turn_seq_str = qs.get("turn_seq_str", ["105, 135, 155"])[0]
            test_stride_m = float(qs.get("test_stride_m", ["0.42"])[0])
            test_corner_pass_m = float(qs.get("test_corner_pass_m", ["0.12"])[0])
            early_stop = qs.get("early_stop", ["0"])[0] in ("1", "true", "True")
            seed_param = qs.get("seed", [None])[0]
            seed_val = int(seed_param) if seed_param is not None else None
            try:
                payload = build_simulation_payload(
                    room_type=room_type,
                    mode=mode,
                    max_chords=max_chords,
                    max_relearns=max_relearns,
                    turn_mode=turn_mode,
                    fixed_turn_deg=fixed_turn_deg,
                    turn_seq_str=turn_seq_str,
                    test_stride_m=test_stride_m,
                    test_corner_pass_m=test_corner_pass_m,
                    early_stop_perimeter=early_stop,
                    seed=seed_val,
                )
            except Exception as exc:
                body = json.dumps({"error": f"Simulación falló: {exc}"}).encode("utf-8")
                self.send_response(500)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            body = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if parsed.path == "/api/esp32_live":
            with ESP32_LIVE_LOCK:
                body = json.dumps({"packets": ESP32_LIVE_PACKETS[-50:]}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.end_headers()
            self.wfile.write(body)
            return

        body = HTML_DASHBOARD.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/runs":
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length).decode("utf-8")
            record = json.loads(raw)
            runs = append_saved_run(record)
            body = json.dumps({"ok": True, "runs": runs}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if parsed.path == "/api/runs_clear":
            runs = clear_saved_runs()
            body = json.dumps({"ok": True, "runs": runs}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.end_headers()
            self.wfile.write(body)
            return

        if parsed.path == "/api/runs_delete":
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length).decode("utf-8")
            data = json.loads(raw)
            run_number = int(data.get("run_number", -1))
            runs = delete_saved_run(run_number)
            body = json.dumps({"ok": True, "runs": runs}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if parsed.path == "/api/shapes":
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length).decode("utf-8")
            data = json.loads(raw)
            name = str(data.get("name", "Figura_Personalizada")).strip() or "Figura_Personalizada"
            verts = [(float(pt[0]), float(pt[1])) for pt in data.get("vertices", [])]
            mode = str(data.get("mode", "classic"))
            room_obj = create_custom_room(
                verts,
                name=name,
                force_orthogonal_90=mode in ("recto", "recto_reaprendizaje", "testeador", "incidente"),
            )
            norm_verts = [(round(v.x, 3), round(v.y, 3)) for v in room_obj.vertices_ccw]
            shapes = save_custom_shape(name, norm_verts)
            body = json.dumps({
                "ok": True,
                "name": name,
                "normalized_vertices": [[x, y] for x, y in norm_verts],
                "shapes": shapes,
            }).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if parsed.path == "/api/sim_custom":
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length).decode("utf-8")
            data = json.loads(raw)
            verts = [(float(pt[0]), float(pt[1])) for pt in data.get("vertices", [])]
            mode = str(data.get("mode", "classic"))
            max_chords = int(data.get("max_chords", 32))
            max_relearns = int(data.get("max_relearns", 15))
            turn_mode = str(data.get("turn_mode", "random"))
            fixed_turn_deg = float(data.get("fixed_turn_deg", 135.0))
            turn_seq_str = str(data.get("turn_seq_str", "105, 135, 155"))
            test_stride_m = float(data.get("test_stride_m", 0.42))
            test_corner_pass_m = float(data.get("test_corner_pass_m", 0.12))
            early_stop = bool(data.get("early_stop", False))
            seed_val = data.get("seed", None)
            try:
                payload = build_simulation_payload(
                    room_type="custom",
                    mode=mode,
                    max_chords=max_chords,
                    max_relearns=max_relearns,
                    turn_mode=turn_mode,
                    fixed_turn_deg=fixed_turn_deg,
                    turn_seq_str=turn_seq_str,
                    test_stride_m=test_stride_m,
                    test_corner_pass_m=test_corner_pass_m,
                    early_stop_perimeter=early_stop,
                    seed=int(seed_val) if seed_val is not None else None,
                    custom_vertices=verts,
                )
            except Exception as exc:
                body = json.dumps({"error": f"Simulación falló: {exc}"}).encode("utf-8")
                self.send_response(500)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            body = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_response(404)
        self.end_headers()


def main() -> None:
    parser = argparse.ArgumentParser(description="Servidor en vivo multihilo de Recorredor")
    parser.add_argument("--port", type=int, default=8765, help="Puerto HTTP del visualizador")
    parser.add_argument("--udp-port", type=int, default=5005, help="Puerto UDP para telemetría WiFi del ESP32")
    args = parser.parse_args()

    t = threading.Thread(target=udp_listener_thread, args=(args.udp_port,), daemon=True)
    t.start()

    server = ThreadingHTTPServer(("127.0.0.1", args.port), LiveSimRequestHandler)
    print(f"Servidor en vivo de Recorredor activo en http://127.0.0.1:{args.port}")
    print(f"Escuchando telemetria WiFi UDP del ESP32 en puerto {args.udp_port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
