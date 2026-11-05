# API web para Pyodide: expone build_simulation_payload sin dependencias de
# servidor (sin archivos, sin sockets). Las figuras guardadas llegan como
# vértices explícitos desde localStorage del navegador.
import json
import random
from typing import Optional

from sim.config import RobotConfig
from sim.world import (
    create_custom_room,
    create_diamond_room,
    create_figura_743_room,
    create_hexagonal_room,
    create_l_shaped_ortho_room,
    create_pentagonal_room,
    create_rectangular_room,
    create_trapezoid_room,
    run_simulation,
)


def parse_turn_sequence(seq_str: str) -> list:
    vals = []
    for part in seq_str.replace(";", ",").split(","):
        part = part.strip()
        if not part:
            continue
        try:
            vals.append(float(part))
        except ValueError:
            continue
    return vals if vals else [110.0, 145.0, 125.0]


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
    custom_vertices: Optional[list] = None,
    custom_name: Optional[str] = None,
) -> dict:
    actual_seed = seed if seed is not None else random.SystemRandom().randint(1, 999999)
    force_ortho = mode in ("recto", "recto_reaprendizaje", "testeador", "incidente")
    relearn_active = mode in ("reaprendizaje", "recto_reaprendizaje", "bordeador", "testeador", "incidente")

    if custom_vertices and len(custom_vertices) >= 3:
        base_label = custom_name or room_type.replace("saved:", "") or "Figura Personalizada"
        room = create_custom_room(
            custom_vertices,
            name=f"{base_label} (90° Normalizada)" if force_ortho else base_label,
            force_orthogonal_90=force_ortho,
        )
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
    elif room_type == "figura_743":
        room = create_figura_743_room()
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
        chords_data.append({
            "idx": c.idx,
            "x1": round(c.start.x, 4), "y1": round(c.start.y, 4),
            "x2": round(c.end.x, 4), "y2": round(c.end.y, 4),
            "length": round(c.length, 3),
            "duration_s": round(c.duration_s, 2),
            "start_frame": start_pose_idx // stride,
            "end_frame": max(start_pose_idx // stride + 1, end_pose_idx // stride),
        })

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
    area_error_pct = round(abs(final_poly_area - room.true_area_m2) / max(room.true_area_m2, 1e-6) * 100.0, 2)
    perim_error_pct = round(abs(final_poly_perim - room.true_perimeter_m) / max(room.true_perimeter_m, 1e-6) * 100.0, 2)
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


def normalize_shape(vertices: list, mode: str, name: str) -> dict:
    """Normaliza una figura dibujada (equivalente a POST /api/shapes)."""
    force_ortho = mode in ("recto", "recto_reaprendizaje", "testeador", "incidente")
    room_obj = create_custom_room(vertices, name=name, force_orthogonal_90=force_ortho)
    norm_verts = [[round(v.x, 3), round(v.y, 3)] for v in room_obj.vertices_ccw]
    return {"ok": True, "name": name, "normalized_vertices": norm_verts}


def handle(op: str, params_json: str) -> str:
    """Punto de entrada RPC para el worker de Pyodide. Devuelve JSON string."""
    params = json.loads(params_json or "{}")
    try:
        if op == "sim":
            return json.dumps(build_simulation_payload(**params))
        if op == "normalize_shape":
            return json.dumps(normalize_shape(
                params.get("vertices", []),
                params.get("mode", "classic"),
                params.get("name", "Figura_Personalizada"),
            ))
        return json.dumps({"error": f"Operación desconocida: {op}"})
    except Exception as exc:
        return json.dumps({"error": f"Simulación falló: {exc}"})
