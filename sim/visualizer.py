# Visualizador 2D de resultados de simulación (Matplotlib si está instalado + SVG nativo siempre disponible)
from typing import Optional
from sim.world import SimulationResult


def export_svg_map(result: SimulationResult, output_path: str) -> None:
    """
    Exporta un mapa vectorial SVG autocontenido sin requerir librerías externas.
    Dibuja la habitación real, las cuerdas de exploración, los cruces (Hormigas),
    el perímetro reconstruido y el recorrido final libre de colisiones.
    """
    room = result.room
    nav = result.navigator
    est = nav.estimator

    all_x = [p.x for p in room.vertices_ccw] + [p.x for p in est.wall_points]
    all_y = [p.y for p in room.vertices_ccw] + [p.y for p in est.wall_points]
    min_x, max_x = min(all_x) - 0.5, max(all_x) + 0.5
    min_y, max_y = min(all_y) - 0.5, max(all_y) + 0.5

    width_px = 900
    height_px = int(width_px * (max_y - min_y) / max(max_x - min_x, 1e-3))

    def to_px(x: float, y: float) -> tuple[float, float]:
        px = (x - min_x) / (max_x - min_x) * width_px
        py = height_px - (y - min_y) / (max_y - min_y) * height_px
        return px, py

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width_px}" height="{height_px}" viewBox="0 0 {width_px} {height_px}">',
        '<rect width="100%" height="100%" fill="#0f172a"/>',
    ]

    # 1. Paredes reales de la habitación
    wall_pts_str = " ".join(f"{to_px(p.x, p.y)[0]:.1f},{to_px(p.x, p.y)[1]:.1f}" for p in room.vertices_ccw)
    lines.append(
        f'<polygon points="{wall_pts_str}" fill="#1e293b" stroke="#94a3b8" stroke-width="4"/>'
    )

    # 2. Cuerdas de exploración (rastro de las hormigas)
    for c in est.chords:
        x1, y1 = to_px(c.start.x, c.start.y)
        x2, y2 = to_px(c.end.x, c.end.y)
        lines.append(
            f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="#38bdf8" stroke-width="1.5" stroke-opacity="0.55"/>'
        )

    # 3. Intersecciones entre cuerdas (Cruces del Método de las Hormigas)
    for cr in est.crossings:
        cx, cy = to_px(cr.x, cr.y)
        lines.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="3.5" fill="#f59e0b"/>')

    # 4. Puntos de impacto en pared
    for wp in est.wall_points:
        wx, wy = to_px(wp.x, wp.y)
        lines.append(f'<circle cx="{wx:.1f}" cy="{wy:.1f}" r="5.5" fill="#ef4444"/>')

    # 5. Perímetro virtual reconstruido
    if nav.reconstructed_perimeter:
        poly_str = " ".join(
            f"{to_px(p.x, p.y)[0]:.1f},{to_px(p.y, p.y)[1]:.1f}" for p in nav.reconstructed_perimeter
        )
        poly_str = " ".join(
            f"{to_px(p.x, p.y)[0]:.1f},{to_px(p.x, p.y)[1]:.1f}"
            for p in nav.reconstructed_perimeter
        )
        lines.append(
            f'<polygon points="{poly_str}" fill="none" stroke="#ef4444" stroke-width="2.5" stroke-dasharray="8,5"/>'
        )

    # 6. Recorrido perimetral final sin chocar
    if nav.perimeter_trace:
        trace_str = " ".join(
            f"{to_px(p.x, p.y)[0]:.1f},{to_px(p.x, p.y)[1]:.1f}" for p in nav.perimeter_trace
        )
        lines.append(
            f'<polyline points="{trace_str}" fill="none" stroke="#22c55e" stroke-width="4"/>'
        )

    lines.append("</svg>")
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def render_matplotlib_dashboard(
    result: SimulationResult, save_path: Optional[str] = None, show_window: bool = False
) -> Optional[str]:
    """
    Genera el dashboard de 3 paneles con Matplotlib (Mapa 2D + Convergencia de Área + Convergencia de Perímetro).
    Si Matplotlib no está instalado, exporta automáticamente en formato SVG.
    """
    try:
        import matplotlib
        if not show_window:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        if save_path:
            svg_path = save_path.rsplit(".", 1)[0] + ".svg"
            export_svg_map(result, svg_path)
            return svg_path
        return None

    room = result.room
    nav = result.navigator
    est = nav.estimator

    fig, axes = plt.subplots(1, 3, figsize=(17, 5.5))
    ax_map, ax_area, ax_perim = axes

    # Panel 1: Mapa 2D de la habitación, cuerdas, cruces y recorrido perimetral
    wall_x = [p.x for p in room.vertices_ccw] + [room.vertices_ccw[0].x]
    wall_y = [p.y for p in room.vertices_ccw] + [room.vertices_ccw[0].y]
    ax_map.plot(wall_x, wall_y, color="#1e293b", linewidth=3.0, label="Pared Real")

    for i, c in enumerate(est.chords):
        ax_map.plot(
            [c.start.x, c.end.x],
            [c.start.y, c.end.y],
            color="#0284c7",
            alpha=0.45,
            linewidth=1.2,
            label="Rastro / Cuerdas" if i == 0 else None,
        )

    if est.crossings:
        ax_map.scatter(
            [p.x for p in est.crossings],
            [p.y for p in est.crossings],
            color="#f59e0b",
            s=18,
            zorder=4,
            label=f"Cruces Hormigas ({len(est.crossings)})",
        )

    if est.wall_points:
        ax_map.scatter(
            [p.x for p in est.wall_points],
            [p.y for p in est.wall_points],
            color="#dc2626",
            s=36,
            zorder=5,
            label=f"Impactos Pared ({len(est.wall_points)})",
        )

    if nav.reconstructed_perimeter:
        rx = [p.x for p in nav.reconstructed_perimeter] + [nav.reconstructed_perimeter[0].x]
        ry = [p.y for p in nav.reconstructed_perimeter] + [nav.reconstructed_perimeter[0].y]
        ax_map.plot(
            rx, ry, "--", color="#dc2626", linewidth=2.0, label="Perímetro Virtual Estimado"
        )

    if nav.perimeter_trace:
        tx = [p.x for p in nav.perimeter_trace]
        ty = [p.y for p in nav.perimeter_trace]
        ax_map.plot(
            tx,
            ty,
            color="#16a34a",
            linewidth=2.8,
            zorder=6,
            label="Recorrido Perimetral Final (Sin Chocar)",
        )

    ax_map.set_title(f"Mapa Virtual y Recorrido — {room.name}")
    ax_map.set_xlabel("X [m]")
    ax_map.set_ylabel("Y [m]")
    ax_map.set_aspect("equal")
    ax_map.grid(True, linestyle=":", alpha=0.5)
    ax_map.legend(loc="upper right", fontsize=8)

    # Panel 2: Evolución de la Estimación del Área (Polígono vs. Método de las Hormigas)
    steps_k = [h.num_chords for h in nav.estimates_history]
    poly_areas = [h.poly_area_m2 for h in nav.estimates_history]
    ants_areas = [h.ants_pairs_area_m2 for h in nav.estimates_history]

    ax_area.axhline(
        room.true_area_m2, color="#1e293b", linestyle="--", linewidth=2.0, label=f"Área Real ({room.true_area_m2:.2f} m²)"
    )
    ax_area.plot(steps_k, poly_areas, "-o", color="#dc2626", markersize=4, label="Área Polígono (Shoelace)")
    ax_area.plot(steps_k, ants_areas, "-s", color="#f59e0b", markersize=4, label="Área Método Hormigas (Cruces)")
    ax_area.set_title("Convergencia del Área Estimada")
    ax_area.set_xlabel("Número de Tramos (Cuerdas)")
    ax_area.set_ylabel("Área [m²]")
    ax_area.grid(True, linestyle=":", alpha=0.5)
    ax_area.legend(fontsize=8)

    # Panel 3: Evolución de la Estimación del Perímetro (Polígono vs. Cauchy-Crofton)
    poly_perims = [h.poly_perimeter_m for h in nav.estimates_history]
    crofton_perims = [h.crofton_perimeter_from_poly_m for h in nav.estimates_history]

    ax_perim.axhline(
        room.true_perimeter_m,
        color="#1e293b",
        linestyle="--",
        linewidth=2.0,
        label=f"Perímetro Real ({room.true_perimeter_m:.2f} m)",
    )
    ax_perim.plot(steps_k, poly_perims, "-o", color="#16a34a", markersize=4, label="Perímetro Poligonal")
    ax_perim.plot(steps_k, crofton_perims, "-^", color="#0284c7", markersize=4, label="Perímetro Cauchy-Crofton")
    ax_perim.set_title("Convergencia del Perímetro Estimado")
    ax_perim.set_xlabel("Número de Tramos (Cuerdas)")
    ax_perim.set_ylabel("Perímetro [m]")
    ax_perim.grid(True, linestyle=":", alpha=0.5)
    ax_perim.legend(fontsize=8)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150)
    if show_window:
        plt.show()
    plt.close(fig)
    return save_path
