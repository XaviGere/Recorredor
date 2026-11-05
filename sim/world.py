# Mundo simulado 2D con habitaciones poligonales, ortogonales y personalizadas, métricas de tiempo y Reaprendizaje
from dataclasses import dataclass, field
import math
import random
from typing import List, Optional, Tuple

from sim.config import RobotConfig
from sim.core.geometry import (
    Point2D,
    Pose2D,
    polygon_area,
    polygon_perimeter,
    ray_segment_intersection_distance,
    rectilinearize_custom_polygon,
    wrap_angle,
)
from sim.core.navigator import MissionState, RecorredorNavigator


@dataclass(frozen=True)
class RoomEnvironment:
    name: str
    vertices_ccw: List[Point2D]

    @property
    def true_area_m2(self) -> float:
        return polygon_area(self.vertices_ccw)

    @property
    def true_perimeter_m(self) -> float:
        return polygon_perimeter(self.vertices_ccw)

    def wall_segments(self) -> List[Tuple[Point2D, Point2D]]:
        n = len(self.vertices_ccw)
        return [(self.vertices_ccw[i], self.vertices_ccw[(i + 1) % n]) for i in range(n)]

    def distance_to_wall_along_ray(self, pose: Pose2D, robot_radius_m: float) -> float:
        origin = pose.as_point()
        min_center_dist = float("inf")
        for a, b in self.wall_segments():
            d = ray_segment_intersection_distance(origin, pose.theta, a, b)
            if d is not None and d < min_center_dist:
                min_center_dist = d
        return max(0.0, min_center_dist - robot_radius_m)

    def closest_wall_point(self, pt: Point2D) -> Tuple[Point2D, float]:
        best_pt = self.vertices_ccw[0]
        best_dist = float("inf")
        for a, b in self.wall_segments():
            dx, dy = b.x - a.x, b.y - a.y
            len_sq = dx * dx + dy * dy
            if len_sq < 1e-12:
                proj = a
            else:
                t = max(0.0, min(1.0, ((pt.x - a.x) * dx + (pt.y - a.y) * dy) / len_sq))
                proj = Point2D(a.x + t * dx, a.y + t * dy)
            d = pt.distance_to(proj)
            if d < best_dist:
                best_dist = d
                best_pt = proj
        return best_pt, best_dist

    def min_clearance_from_walls(self, pt: Point2D, robot_radius_m: float) -> float:
        _, min_center_dist = self.closest_wall_point(pt)
        return min_center_dist - robot_radius_m


@dataclass
class SimulationResult:
    room: RoomEnvironment
    navigator: RecorredorNavigator
    true_poses: List[Pose2D]
    perimeter_lap_collisions: int
    total_relearn_collisions: int
    min_perimeter_clearance_m: float
    total_sim_steps: int
    total_sim_time_s: float
    exploration_time_s: float = 0.0
    relearn_time_s: float = 0.0
    final_lap_time_s: float = 0.0
    chord_pose_spans: List[Tuple[int, int]] = field(default_factory=list)
    perimeter_start_pose_idx: int = 0
    relearn_events_pose_indices: List[int] = field(default_factory=list)


def create_rectangular_room(width_m: float = 4.0, height_m: float = 3.0) -> RoomEnvironment:
    hw, hh = width_m / 2.0, height_m / 2.0
    return RoomEnvironment(
        name=f"Habitación Rectangular ({width_m:.1f}m x {height_m:.1f}m)",
        vertices_ccw=[
            Point2D(-hw, -hh),
            Point2D(hw, -hh),
            Point2D(hw, hh),
            Point2D(-hw, hh),
        ],
    )


def create_l_shaped_ortho_room() -> RoomEnvironment:
    return RoomEnvironment(
        name="Habitación Escalonada a 90° (Con Esquinas Entrantes)",
        vertices_ccw=[
            Point2D(-1.9, -1.5),
            Point2D(1.9, -1.5),
            Point2D(1.9, 0.2),
            Point2D(1.2, 0.2),
            Point2D(1.2, 1.5),
            Point2D(-1.9, 1.5),
        ],
    )


def create_pentagonal_room() -> RoomEnvironment:
    return RoomEnvironment(
        name="Salón Pentagonal Achaflanado",
        vertices_ccw=[
            Point2D(-2.2, -1.6),
            Point2D(2.0, -1.6),
            Point2D(2.4, 0.4),
            Point2D(0.8, 1.8),
            Point2D(-2.0, 1.5),
        ],
    )


def create_trapezoid_room() -> RoomEnvironment:
    return RoomEnvironment(
        name="Habitación Trapezoidal (Base 4.4m / Techo 2.6m)",
        vertices_ccw=[
            Point2D(-2.2, -1.5),
            Point2D(2.2, -1.5),
            Point2D(1.3, 1.5),
            Point2D(-1.3, 1.5),
        ],
    )


def create_hexagonal_room() -> RoomEnvironment:
    return RoomEnvironment(
        name="Aula Hexagonal (6 Paredes)",
        vertices_ccw=[
            Point2D(-1.4, -1.7),
            Point2D(1.4, -1.7),
            Point2D(2.3, -0.2),
            Point2D(1.3, 1.6),
            Point2D(-1.3, 1.6),
            Point2D(-2.3, -0.1),
        ],
    )


def create_diamond_room() -> RoomEnvironment:
    return RoomEnvironment(
        name="Salón Octogonal Achaflanado",
        vertices_ccw=[
            Point2D(-1.1, -1.7),
            Point2D(1.1, -1.7),
            Point2D(2.2, -0.7),
            Point2D(2.2, 0.7),
            Point2D(1.1, 1.7),
            Point2D(-1.1, 1.7),
            Point2D(-2.2, 0.7),
            Point2D(-2.2, -0.7),
        ],
    )


def create_figura_743_room() -> RoomEnvironment:
    return RoomEnvironment(
        name="Galpón Escalonado (14 Paredes — Figura_743)",
        vertices_ccw=[
            Point2D(-1.5, 0.5),
            Point2D(-1.0, 0.5),
            Point2D(-1.0, 0.0),
            Point2D(-0.5, 0.0),
            Point2D(-0.5, -0.5),
            Point2D(0.5, -0.5),
            Point2D(0.5, -1.0),
            Point2D(1.5, -1.0),
            Point2D(1.5, -0.5),
            Point2D(2.1, -0.5),
            Point2D(2.1, 0.5),
            Point2D(1.5, 0.5),
            Point2D(1.5, 1.5),
            Point2D(-1.5, 1.5),
        ],
    )


def create_custom_room(
    vertices: List[Tuple[float, float]],
    name: str = "Figura Personalizada",
    force_orthogonal_90: bool = False,
) -> RoomEnvironment:
    verts_in = rectilinearize_custom_polygon(vertices) if force_orthogonal_90 else vertices
    pts = [Point2D(float(x), float(y)) for x, y in verts_in]
    if len(pts) < 3:
        return create_rectangular_room()
    signed = 0.0
    n = len(pts)
    for i in range(n):
        p1, p2 = pts[i], pts[(i + 1) % n]
        signed += p1.x * p2.y - p2.x * p1.y
    if signed < 0.0:
        pts.reverse()
    return RoomEnvironment(name=name, vertices_ccw=pts)


def run_simulation(
    room: RoomEnvironment,
    config: RobotConfig = RobotConfig(),
    initial_pose: Optional[Pose2D] = None,
    seed: Optional[int] = None,
    mode: str = "classic",
    relearn_enabled: bool = False,
    max_relearns: int = 15,
    turn_mode: str = "random",
    fixed_turn_deg: float = 135.0,
    turn_sequence_deg: Optional[List[float]] = None,
    test_stride_m: float = 0.42,
    test_corner_pass_m: float = 0.12,
    early_stop_perimeter: bool = False,
    max_steps: int = 60000,
) -> SimulationResult:
    """
    Ejecuta la simulación física completa paso a paso midiendo tiempos desglosados:
      - `exploration_time_s`: Tiempo invertido en trazar las cuerdas iniciales.
      - `relearn_time_s`: Tiempo extra invertido en choques y recálculos de reaprendizaje.
      - `final_lap_time_s`: Tiempo de la vuelta perimetral final definitiva.
      - `total_sim_time_s`: Suma total de tiempo de misión.
    """
    actual_seed = seed if seed is not None else random.SystemRandom().randint(1, 999999)
    rng_init = random.Random(actual_seed)

    if initial_pose is None:
        rand_theta = rng_init.uniform(-math.pi, math.pi)
        initial_pose = Pose2D(0.0, 0.0, rand_theta)

    nav = RecorredorNavigator(
        config=config,
        initial_pose=initial_pose,
        seed=actual_seed,
        mode=mode,
        relearn_enabled=relearn_enabled,
        max_relearns=max_relearns,
        turn_mode=turn_mode,
        fixed_turn_deg=fixed_turn_deg,
        turn_sequence_deg=turn_sequence_deg,
        test_stride_m=test_stride_m,
        test_corner_pass_m=test_corner_pass_m,
        early_stop_perimeter=early_stop_perimeter,
        target_perimeter_m=room.true_perimeter_m,
    )
    nav.start_mission()

    true_pose = initial_pose
    true_poses: List[Pose2D] = [true_pose]
    perimeter_lap_collisions = 0
    total_relearn_collisions = 0
    min_perimeter_clearance_m = float("inf")

    chord_pose_spans: List[Tuple[int, int]] = []
    current_chord_start_idx: int = 0
    first_perimeter_entry_step: int = 0
    last_perimeter_entry_step: int = 0
    perimeter_start_pose_idx: int = 0
    relearn_events_pose_indices: List[int] = []
    in_collision_zone: bool = False

    steps = 0
    while nav.state != MissionState.COMPLETED and steps < max_steps:
        steps += 1
        prev_state = nav.state
        prev_num_chords = len(nav.estimator.chords)

        front_dist = room.distance_to_wall_along_ray(true_pose, config.robot_radius_m)
        bumper_hit = front_dist <= 0.002

        cmd = nav.step(
            front_distance_m=front_dist,
            bumper_triggered=bumper_hit,
            measured_dt_s=config.dt_s,
        )

        if len(nav.estimator.chords) > prev_num_chords:
            chord_pose_spans.append((current_chord_start_idx, len(true_poses) - 1))

        if prev_state == MissionState.TURN_HEADING and nav.state == MissionState.EXPLORE_FWD:
            current_chord_start_idx = len(true_poses) - 1
        elif prev_state != nav.state and nav.state in (MissionState.TESTEADOR_CRUISE, MissionState.TESTEADOR_PROBE):
            current_chord_start_idx = len(true_poses) - 1

        if prev_state != MissionState.FOLLOW_PERIMETER and nav.state == MissionState.FOLLOW_PERIMETER:
            if first_perimeter_entry_step == 0:
                first_perimeter_entry_step = steps
            last_perimeter_entry_step = steps
            perimeter_start_pose_idx = len(true_poses) - 1
            in_collision_zone = False
            if nav.relearn_enabled:
                perimeter_lap_collisions = 0
                min_perimeter_clearance_m = float("inf")

        # Registrar el inicio exacto de la vuelta verde cuando el robot llega al punto de anclaje perpendicular (idx == 1)
        if nav.state == MissionState.FOLLOW_PERIMETER and nav.current_waypoint_idx == 1 and len(nav.perimeter_trace) == 1:
            perimeter_start_pose_idx = len(true_poses) - 1

        if cmd.linear_speed_mps != 0.0 or cmd.angular_speed_radps != 0.0:
            ds = cmd.linear_speed_mps * config.dt_s
            dtheta = cmd.angular_speed_radps * config.dt_s
            mid_theta = true_pose.theta + 0.5 * dtheta
            nx = true_pose.x + ds * math.cos(mid_theta)
            ny = true_pose.y + ds * math.sin(mid_theta)
            ntheta = wrap_angle(true_pose.theta + dtheta)

            candidate_pose = Pose2D(nx, ny, ntheta)
            wall_pt_closest, dist_center = room.closest_wall_point(candidate_pose.as_point())
            clearance = dist_center - config.robot_radius_m

            # Empezar a medir holgura de la vuelta perimetral una vez superado el tramo inicial de acople perpendicular (idx >= 2)
            if nav.state == MissionState.FOLLOW_PERIMETER and nav.current_waypoint_idx >= 1:
                if nav.current_waypoint_idx >= 2:
                    min_perimeter_clearance_m = min(min_perimeter_clearance_m, clearance)
                if clearance <= -0.012:
                    if nav.relearn_enabled and nav.relearn_count < nav.MAX_RELEARNS:
                        total_relearn_collisions += 1
                        relearn_events_pose_indices.append(len(true_poses) - 1)
                        nav.trigger_relearning_from_collision(wall_pt_closest)
                        continue
                    elif nav.current_waypoint_idx >= 2:
                        if not in_collision_zone:
                            perimeter_lap_collisions += 1
                            in_collision_zone = True
                else:
                    in_collision_zone = False

            true_pose = candidate_pose
            nav.odom.integrate_step(
                cmd.linear_speed_mps, cmd.angular_speed_radps, config.dt_s
            )
            true_poses.append(true_pose)

    if min_perimeter_clearance_m == float("inf"):
        min_perimeter_clearance_m = 0.0
    if perimeter_start_pose_idx == 0:
        perimeter_start_pose_idx = len(true_poses) - 1
    if first_perimeter_entry_step == 0:
        first_perimeter_entry_step = steps
    if last_perimeter_entry_step == 0:
        last_perimeter_entry_step = first_perimeter_entry_step

    total_time_s = steps * config.dt_s
    exploration_time_s = first_perimeter_entry_step * config.dt_s
    relearn_time_s = max(0.0, (last_perimeter_entry_step - first_perimeter_entry_step) * config.dt_s)
    final_lap_time_s = max(0.0, (steps - last_perimeter_entry_step) * config.dt_s)

    return SimulationResult(
        room=room,
        navigator=nav,
        true_poses=true_poses,
        perimeter_lap_collisions=perimeter_lap_collisions,
        total_relearn_collisions=total_relearn_collisions + nav.relearn_count,
        min_perimeter_clearance_m=min_perimeter_clearance_m,
        total_sim_steps=steps,
        total_sim_time_s=total_time_s,
        exploration_time_s=exploration_time_s,
        relearn_time_s=relearn_time_s,
        final_lap_time_s=final_lap_time_s,
        chord_pose_spans=chord_pose_spans,
        perimeter_start_pose_idx=perimeter_start_pose_idx,
        relearn_events_pose_indices=relearn_events_pose_indices,
    )
