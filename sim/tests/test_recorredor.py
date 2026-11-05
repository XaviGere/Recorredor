# Suite de pruebas unitarias y de misión completa para Recorredor
import math
import unittest

from sim.config import RobotConfig
from sim.core.estimators import PerimeterAndAntsEstimator
from sim.core.geometry import (
    ChordSegment,
    Point2D,
    build_anchored_perimeter_route,
    compute_inward_offset_polygon,
    convex_hull,
    point_to_segment_distance,
    polygon_area,
    polygon_perimeter,
    reconstruct_classic_polygon,
    reconstruct_orthogonal_polygon,
    rectilinearize_custom_polygon,
    segment_intersection,
    wrap_angle,
)
from sim.core.navigator import MissionState
from sim.world import (
    create_l_shaped_ortho_room,
    create_pentagonal_room,
    create_rectangular_room,
    run_simulation,
)


class TestRecorredorGeometryAndMath(unittest.TestCase):
    def test_wrap_angle_bounds(self) -> None:
        self.assertAlmostEqual(wrap_angle(0.0), 0.0)
        self.assertAlmostEqual(wrap_angle(2.0 * math.pi), 0.0)
        self.assertAlmostEqual(wrap_angle(3.0 * math.pi), math.pi)
        self.assertAlmostEqual(wrap_angle(-math.pi), math.pi)

    def test_segment_intersection_proper(self) -> None:
        # Dos diagonales de un cuadrado [0, 2] x [0, 2] se cruzan en (1, 1)
        p1, p2 = Point2D(0.0, 0.0), Point2D(2.0, 2.0)
        p3, p4 = Point2D(0.0, 2.0), Point2D(2.0, 0.0)
        hit = segment_intersection(p1, p2, p3, p4)
        self.assertIsNotNone(hit)
        assert hit is not None
        self.assertAlmostEqual(hit.x, 1.0, places=5)
        self.assertAlmostEqual(hit.y, 1.0, places=5)

        # Segmentos paralelos no se cruzan
        self.assertIsNone(
            segment_intersection(
                Point2D(0.0, 0.0), Point2D(2.0, 0.0), Point2D(0.0, 1.0), Point2D(2.0, 1.0)
            )
        )

    def test_convex_hull_and_shoelace_rectangle(self) -> None:
        # Puntos en el borde e interior de un rectángulo de 4m x 3m (Área=12, Perímetro=14)
        pts = [
            Point2D(-2.0, -1.5),
            Point2D(2.0, -1.5),
            Point2D(2.0, 1.5),
            Point2D(-2.0, 1.5),
            Point2D(0.0, -1.5),  # Punto colineal en pared inferior
            Point2D(0.4, 0.2),   # Punto interior
        ]
        hull = convex_hull(pts)
        self.assertAlmostEqual(polygon_area(hull), 12.0, places=5)
        self.assertAlmostEqual(polygon_perimeter(hull), 14.0, places=5)

    def test_inward_offset_polygon_rectangle(self) -> None:
        # Un rectángulo de 4m x 3m con margen interior de 0.25m debe dar un rectángulo de 3.5m x 2.5m
        rect = [
            Point2D(-2.0, -1.5),
            Point2D(2.0, -1.5),
            Point2D(2.0, 1.5),
            Point2D(-2.0, 1.5),
        ]
        inset = compute_inward_offset_polygon(rect, margin_m=0.25)
        self.assertEqual(len(inset), 4)
        self.assertAlmostEqual(polygon_area(inset), 3.5 * 2.5, places=4)
        self.assertAlmostEqual(polygon_perimeter(inset), 2.0 * (3.5 + 2.5), places=4)

    def test_ants_estimator_counts_non_adjacent_crossings(self) -> None:
        est = PerimeterAndAntsEstimator()
        # Cuerda 1: (-1, -1) -> (1, 1)
        c1 = ChordSegment(1, Point2D(-1.0, -1.0), Point2D(1.0, 1.0), 2.0, 1.414, 0.785)
        # Cuerda 2 (adyacente a 1): (1, 1) -> (1, -1)
        c2 = ChordSegment(2, Point2D(1.0, 1.0), Point2D(1.0, -1.0), 2.0, 1.0, -1.57)
        # Cuerda 3 (no adyacente a 1, cruza a 1 en (0,0)): (1, -1) -> (-1, 1)
        c3 = ChordSegment(3, Point2D(1.0, -1.0), Point2D(-1.0, 1.0), 2.0, 1.414, 2.356)

        est.add_chord(c1)
        est.add_chord(c2)
        crossings = est.add_chord(c3)
        self.assertEqual(len(crossings), 1)
        self.assertAlmostEqual(crossings[0].x, 0.0, places=4)
        self.assertAlmostEqual(crossings[0].y, 0.0, places=4)
        self.assertGreater(est.compute_ants_area_all_pairs(), 0.0)


class TestFullRoboticSimulation(unittest.TestCase):
    def test_rectangular_room_exploration_and_collision_free_perimeter(self) -> None:
        room = create_rectangular_room(4.0, 3.0)
        cfg = RobotConfig()
        result = run_simulation(room=room, config=cfg, seed=42)

        # 1. El robot debe completar toda la secuencia hasta COMPLETED
        self.assertEqual(result.navigator.state, MissionState.COMPLETED)

        est = result.navigator.last_estimates
        self.assertIsNotNone(est)
        assert est is not None

        # 2. El perímetro reconstruido debe estar dentro del 5% del perímetro real (14.0 m)
        perim_rel_err = abs(est.poly_perimeter_m - room.true_perimeter_m) / room.true_perimeter_m
        self.assertLess(perim_rel_err, 0.05)

        # 3. El área poligonal debe estar dentro del 8% del área real (12.0 m²)
        area_rel_err = abs(est.poly_area_m2 - room.true_area_m2) / room.true_area_m2
        self.assertLess(area_rel_err, 0.08)

        # 4. El método de las hormigas debe registrar múltiples cruces y una estimación positiva consistente
        self.assertGreater(est.num_crossings, 10)
        self.assertGreater(est.ants_pairs_area_m2, 6.0)
        self.assertLess(est.ants_pairs_area_m2, 18.0)

        # 5. Durante el recorrido perimetral final, debe haber CERO colisiones con las paredes
        # (con margen interior Radio * 1.2 = 14.4 cm, la holgura libre entre el borde del chasis y la pared es ~2.4 cm)
        self.assertEqual(result.perimeter_lap_collisions, 0)
        self.assertGreater(result.min_perimeter_clearance_m, 0.015)

    def test_pentagonal_room_exploration_and_collision_free_perimeter(self) -> None:
        room = create_pentagonal_room()
        cfg = RobotConfig()
        result = run_simulation(room=room, config=cfg, seed=7)

        self.assertEqual(result.navigator.state, MissionState.COMPLETED)
        self.assertEqual(result.perimeter_lap_collisions, 0)
        self.assertGreater(result.min_perimeter_clearance_m, 0.015)

        # Verificar que TODOS los puntos de choque estén sobre el perímetro reconstruido (ninguno adentro)
        poly = result.navigator.reconstructed_perimeter
        n = len(poly)
        for wp in result.navigator.estimator.wall_points:
            dist_to_perim = min(point_to_segment_distance(wp, poly[i], poly[(i + 1) % n]) for i in range(n))
            self.assertLessEqual(dist_to_perim, 0.025)

    def test_recto_mode_normalizes_and_reconstructs_orthogonal_90(self) -> None:
        # Probar normalización de figura personalizada con trazos diagonales a ángulos de 90°
        raw_custom = [(-1.8, -1.4), (1.9, -1.35), (1.85, 1.45), (-1.75, 1.42)]
        ortho = rectilinearize_custom_polygon(raw_custom)
        self.assertGreaterEqual(len(ortho), 4)
        for i in range(len(ortho)):
            x1, y1 = ortho[i]
            x2, y2 = ortho[(i + 1) % len(ortho)]
            # Cada segmento debe ser estrictamente horizontal (dy=0) o vertical (dx=0)
            self.assertTrue(abs(x1 - x2) < 1e-4 or abs(y1 - y2) < 1e-4)

    def test_relearning_mode_updates_map_on_concave_l_room(self) -> None:
        room = create_l_shaped_ortho_room()
        cfg = RobotConfig(min_chords_for_map=14, max_chords_explore=22)
        result = run_simulation(
            room=room,
            config=cfg,
            seed=19,
            mode="reaprendizaje",
            relearn_enabled=True,
        )
        self.assertEqual(result.navigator.state, MissionState.COMPLETED)
        self.assertEqual(result.perimeter_lap_collisions, 0)

    def test_exact_chord_count_and_no_self_intersecting_offset(self) -> None:
        # Probar en la figura escalonada que el simulador respete exactamente 40 cuerdas
        # y que el camino interior de navegación no tenga auto-intersecciones (lazos raros)
        room = create_l_shaped_ortho_room()
        cfg = RobotConfig(min_chords_for_map=40, max_chords_explore=40)
        result = run_simulation(room=room, config=cfg, seed=101, mode="recto")
        self.assertEqual(len(result.navigator.estimator.chords), 40)

        wps = result.navigator.safe_perimeter_waypoints[:-1]
        n = len(wps)
        for i in range(n):
            for j in range(i + 2, n):
                if i == 0 and (j + 1) % n == 0:
                    continue
                cross = segment_intersection(wps[i], wps[(i + 1) % n], wps[j], wps[(j + 1) % n], eps=1e-4)
                self.assertIsNone(cross)

    def test_recto_reaprendizaje_and_turn_modes_and_timings(self) -> None:
        room = create_l_shaped_ortho_room()
        cfg = RobotConfig(min_chords_for_map=25, max_chords_explore=25)
        result = run_simulation(
            room=room,
            config=cfg,
            seed=77,
            mode="recto_reaprendizaje",
            relearn_enabled=True,
            max_relearns=10,
            turn_mode="sequence",
            turn_sequence_deg=[105.0, 140.0, 160.0],
        )
        self.assertEqual(result.navigator.state, MissionState.COMPLETED)
        self.assertGreater(result.total_sim_time_s, 0.0)
        self.assertGreater(result.exploration_time_s, 0.0)
        self.assertGreater(result.final_lap_time_s, 0.0)
        self.assertLessEqual(result.navigator.relearn_count, 10)

    def test_anchored_route_prevents_diagonal_shortcut(self) -> None:
        # Si el robot termina la exploración a mitad de la pared superior (x=0.1, y=1.356),
        # el primer waypoint de la ruta anclada debe ser la proyección perpendicular (0.1, 1.356)
        # y NO la esquina lejana (1.856, 1.356), evitando cortar en diagonal al cerrar la vuelta.
        inset = [
            Point2D(-1.856, -1.356),
            Point2D(1.856, -1.356),
            Point2D(1.856, 1.356),
            Point2D(-1.856, 1.356),
        ]
        curr = Point2D(0.10, 1.32)
        route = build_anchored_perimeter_route(curr, inset)
        self.assertAlmostEqual(route[0].x, 0.10, places=3)
        self.assertAlmostEqual(route[0].y, 1.356, places=3)
        self.assertAlmostEqual(route[-1].x, 0.10, places=3)
        self.assertAlmostEqual(route[-1].y, 1.356, places=3)

    def test_bordeador_and_testeador_modes_complete_mission(self) -> None:
        room = create_l_shaped_ortho_room()
        cfg = RobotConfig(min_chords_for_map=20, max_chords_explore=20)
        res_bord = run_simulation(room=room, config=cfg, seed=42, mode="bordeador")
        self.assertEqual(res_bord.navigator.state, MissionState.COMPLETED)

        res_test = run_simulation(room=room, config=cfg, seed=42, mode="testeador")
        self.assertEqual(res_test.navigator.state, MissionState.COMPLETED)

    def test_incidente_mode_records_wall_orientation_and_completes(self) -> None:
        # El modo Incidente rebota igual que el clásico pero guarda la orientación de la
        # pared en cada impacto y reconstruye un perímetro estrictamente ortogonal.
        room = create_rectangular_room(4.0, 3.0)
        cfg = RobotConfig(min_chords_for_map=32, max_chords_explore=32)
        result = run_simulation(room=room, config=cfg, seed=711492, mode="incidente")

        self.assertEqual(result.navigator.state, MissionState.COMPLETED)
        self.assertEqual(result.perimeter_lap_collisions, 0)

        est = result.navigator.estimator
        # Cada punto de choque debe tener su orientación de pared registrada
        self.assertEqual(len(est.wall_dirs), len(est.wall_points))
        self.assertTrue(all(d is not None for d in est.wall_dirs))

        # El perímetro reconstruido debe ser 100% ortogonal y fiel al rectángulo 4x3
        poly = result.navigator.reconstructed_perimeter
        n = len(poly)
        self.assertEqual(n, 4)
        for i in range(n):
            a, b = poly[i], poly[(i + 1) % n]
            self.assertTrue(abs(a.x - b.x) < 0.02 or abs(a.y - b.y) < 0.02)
        area_err = abs(polygon_area(poly) - room.true_area_m2) / room.true_area_m2
        self.assertLess(area_err, 0.05)

        # La ruta interior y el trazo real de la vuelta no pueden tener diagonales
        wps = result.navigator.safe_perimeter_waypoints
        for i in range(len(wps) - 1):
            self.assertTrue(
                abs(wps[i + 1].x - wps[i].x) < 0.03 or abs(wps[i + 1].y - wps[i].y) < 0.03
            )
        poses = result.true_poses[result.perimeter_start_pose_idx :]
        for i in range(1, len(poses)):
            dx = abs(poses[i].x - poses[i - 1].x)
            dy = abs(poses[i].y - poses[i - 1].y)
            self.assertFalse(dx > 0.06 and dy > 0.06)

    def test_incidente_reconstructs_l_room_from_wall_dirs(self) -> None:
        # Reconstrucción pura: puntos de choque sobre cada pared de la L, con su
        # orientación de pared (0 = horizontal, pi/2 = vertical). Debe salir la L exacta.
        from sim.core.geometry import reconstruct_incidence_polygon

        room = create_l_shaped_ortho_room()
        verts = list(room.vertices_ccw)
        wall_points: List[Point2D] = []
        wall_dirs: List[float] = []
        n = len(verts)
        for i in range(n):
            a, b = verts[i], verts[(i + 1) % n]
            vert_wall = abs(b.x - a.x) < 1e-6
            d = math.pi / 2.0 if vert_wall else 0.0
            # Tres hits por arista (cuartos) para simular muestreo real
            for t in (0.25, 0.5, 0.75):
                wall_points.append(
                    Point2D(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y))
                )
                wall_dirs.append(d)

        poly = reconstruct_incidence_polygon(wall_points, wall_dirs)
        self.assertEqual(len(poly), n)
        area_err = abs(polygon_area(poly) - room.true_area_m2) / room.true_area_m2
        perim_err = abs(polygon_perimeter(poly) - room.true_perimeter_m) / room.true_perimeter_m
        self.assertLess(area_err, 0.03)
        self.assertLess(perim_err, 0.03)

        # Robustez ante un hit rasante mal clasificado: un punto en la pared inferior
        # con orientación votada "vertical" no debe crear paredes fantasma.
        wall_points.append(Point2D(0.0, -1.5))
        wall_dirs.append(math.pi / 2.0)
        poly2 = reconstruct_incidence_polygon(wall_points, wall_dirs)
        for i in range(len(poly2)):
            a, b = poly2[i], poly2[(i + 1) % len(poly2)]
            self.assertTrue(abs(a.x - b.x) < 0.02 or abs(a.y - b.y) < 0.02)

    def test_incidente_relearn_triggers_on_hidden_corner(self) -> None:
        # Con pocas cuerdas en una L, el modo Incidente debe poder reaprender al chocar
        # durante la vuelta perimetral (relearn_enabled activo por defecto del modo).
        room = create_l_shaped_ortho_room()
        cfg = RobotConfig(min_chords_for_map=12, max_chords_explore=12)
        result = run_simulation(
            room=room, config=cfg, seed=3, mode="incidente", max_relearns=15
        )
        self.assertEqual(result.navigator.state, MissionState.COMPLETED)
        self.assertTrue(result.navigator.relearn_enabled)
        self.assertLessEqual(result.navigator.relearn_count, 15)


if __name__ == "__main__":
    unittest.main()
