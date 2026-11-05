# Script ejecutable de demostración de la simulación de Recorredor
import argparse
import json
from typing import Optional

from sim.config import RobotConfig
from sim.visualizer import export_svg_map, render_matplotlib_dashboard
from sim.world import (
    SimulationResult,
    create_pentagonal_room,
    create_rectangular_room,
    run_simulation,
)


def print_simulation_report(result: SimulationResult) -> None:
    room = result.room
    nav = result.navigator
    est = nav.last_estimates
    if est is None:
        print("Simulación sin datos de estimación.")
        return

    area_err_pct = abs(est.poly_area_m2 - room.true_area_m2) / room.true_area_m2 * 100.0
    perim_err_pct = (
        abs(est.poly_perimeter_m - room.true_perimeter_m) / room.true_perimeter_m * 100.0
    )
    ants_err_pct = (
        abs(est.ants_pairs_area_m2 - room.true_area_m2) / room.true_area_m2 * 100.0
    )

    print("=" * 78)
    print(f" REPORTE DE MISIÓN ROBÓTICA RECORREDOR — {room.name}")
    print("=" * 78)
    print(f" Estado final de la FSM        : {nav.state.value}")
    print(f" Tiempo total de misión        : {result.total_sim_time_s:.2f} s ({result.total_sim_steps} ciclos @ 50Hz)")
    print(f" Distancia total recorrida     : {nav.odom.total_distance_m:.2f} m")
    print(f" Tramos rectos (Cuerdas)       : {est.num_chords} (Longitud media: {est.mean_chord_length_m:.3f} m)")
    print(f" Cruces de rastro (Hormigas)   : {est.num_crossings} intersecciones interiores")
    print("-" * 78)
    print(" COMPARATIVA DE PERÍMETRO Y ÁREA (REAL vs. MAPA GEOMÉTRICO vs. HORMIGAS/CROFTON)")
    print("-" * 78)
    print(f" Perímetro Real de Habitación  : {room.true_perimeter_m:8.3f} m")
    print(f" Perímetro Polígono Estimado   : {est.poly_perimeter_m:8.3f} m  (Error: {perim_err_pct:.2f}%)")
    print(f" Perímetro Cauchy-Crofton      : {est.crofton_perimeter_from_poly_m:8.3f} m")
    print("-" * 78)
    print(f" Área Real de Habitación       : {room.true_area_m2:8.3f} m²")
    print(f" Área Polígono (Shoelace)      : {est.poly_area_m2:8.3f} m² (Error: {area_err_pct:.2f}%)")
    print(f" Área Método de las Hormigas   : {est.ants_pairs_area_m2:8.3f} m² (Error: {ants_err_pct:.2f}%)")
    print(f" Área Mallock (Split 2 Rastro) : {est.ants_mallock_split_area_m2:8.3f} m²")
    print("-" * 78)
    print(" VERIFICACIÓN DEL RECORRIDO PERIMETRAL FINAL")
    print("-" * 78)
    print(f" Vértices de esquina detectados: {len(nav.reconstructed_perimeter)}")
    print(f" Waypoints interiores seguros  : {len(nav.safe_perimeter_waypoints)}")
    print(f" Colisiones en vuelta final    : {result.perimeter_lap_collisions} (Objetivo: 0)")
    print(f" Distancia mínima a pared      : {result.min_perimeter_clearance_m * 100.0:.1f} cm de holgura libre")
    print("=" * 78)

    # Ejemplo de trama de telemetría JSON idéntica a la del ESP32
    telemetry_sample = {
        "t_ms": int(result.total_sim_time_s * 1000),
        "state": nav.state.value,
        "pose": {
            "x": round(nav.odom.pose.x, 3),
            "y": round(nav.odom.pose.y, 3),
            "theta": round(nav.odom.pose.theta, 3),
        },
        "chord_idx": est.num_chords,
        "crossings_total": est.num_crossings,
        "estimates": {
            "poly_area_m2": round(est.poly_area_m2, 3),
            "poly_perimeter_m": round(est.poly_perimeter_m, 3),
            "ants_area_m2": round(est.ants_pairs_area_m2, 3),
            "crofton_perimeter_m": round(est.crofton_perimeter_from_poly_m, 3),
            "converged": est.converged,
        },
    }
    print("\nTrama JSON de telemetría final (formato Serial ESP32):")
    print(json.dumps(telemetry_sample, ensure_ascii=False))


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Simulador de Mapeo Perimetral Recorredor")
    parser.add_argument(
        "--room",
        choices=["rect", "pentagon"],
        default="rect",
        help="Tipo de habitación cerrada para la simulación",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Ejecutar sin abrir ventana interactiva",
    )
    parser.add_argument(
        "--save-plot",
        default="sim_recorredor_mapa.png",
        help="Ruta para guardar la gráfica de resultados (PNG y SVG)",
    )
    args = parser.parse_args(argv)

    room = create_rectangular_room(4.0, 3.0) if args.room == "rect" else create_pentagonal_room()
    cfg = RobotConfig()
    result = run_simulation(room=room, config=cfg, seed=42)
    print_simulation_report(result)

    if args.save_plot:
        saved = render_matplotlib_dashboard(
            result, save_path=args.save_plot, show_window=not args.headless
        )
        svg_file = args.save_plot.rsplit(".", 1)[0] + ".svg"
        export_svg_map(result, svg_file)
        print(f"\n[OK] Mapa vectorial guardado en: {svg_file}")
        if saved and saved.endswith(".png"):
            print(f"[OK] Dashboard gráfico guardado en: {saved}")


if __name__ == "__main__":
    main()
