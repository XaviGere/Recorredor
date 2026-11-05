# Máquina de Estados Finita (FSM) con 6 Modos (Clásico, Recto, Reaprendizaje, Recto+Reaprendizaje,
# Bordeador y Testeador), Anclaje Perpendicular sin Diagonales y Control de Rumbo.
from dataclasses import dataclass
from enum import Enum
import math
import random
from typing import List, Optional

from sim.config import RobotConfig
from sim.core.estimators import MapEstimates, PerimeterAndAntsEstimator
from sim.core.geometry import (
    ChordSegment,
    Point2D,
    Pose2D,
    build_anchored_perimeter_route,
    compute_inward_offset_polygon,
    point_in_polygon,
    polygon_centroid,
    segment_intersection,
    wrap_angle,
)
from sim.core.odometry import OdometryEstimator


class MissionState(str, Enum):
    IDLE = "IDLE"
    EXPLORE_FWD = "EXPLORE_FWD"
    BRAKE_RECORD = "BRAKE_RECORD"
    EVALUATE_MAP = "EVALUATE_MAP"
    TURN_HEADING = "TURN_HEADING"
    # Estados específicos para Modo Bordeador (prueba ambos lados y elige el más prometedor)
    BORDEADOR_SCAN = "BORDEADOR_SCAN"
    # Estados específicos para Modo Testeador (sigue la pared y gira periódicamente hacia ella para tantearla)
    TESTEADOR_CRUISE = "TESTEADOR_CRUISE"
    TESTEADOR_PROBE = "TESTEADOR_PROBE"
    # Estados de construcción y recorrido limpio del perímetro
    BUILD_SAFE_PATH = "BUILD_SAFE_PATH"
    FOLLOW_PERIMETER = "FOLLOW_PERIMETER"
    COMPLETED = "COMPLETED"


@dataclass
class ActuatorCommand:
    linear_speed_mps: float
    angular_speed_radps: float
    servo_angle_rad: float  # Ángulo del servomotor de dirección/escaneo


class RecorredorNavigator:
    """
    Controlador principal del robot Recorredor.
    Soporta 6 modos de operación:
      1. 'classic': Exploración por cuerdas + perímetro poligonal general.
      2. 'recto': Exploración por cuerdas + perímetro ortogonal (90°).
      3. 'reaprendizaje': Clásico + recalcula el perímetro si choca en la vuelta final.
      4. 'recto_reaprendizaje': Recto 90° + recalcula el perímetro si choca en la vuelta final.
      5. 'bordeador': Al chocar, gira y avanza un corto tramo hacia ambos lados (izquierda y derecha),
         evalúa cuál de los dos caminos tiene mayor espacio libre/promesa y lo sigue hasta el próximo choque.
      6. 'testeador': Al chocar elige un lado (ej. dobla a la izquierda dejando la pared a su derecha),
         avanza paralelo a la pared y cada cierta distancia gira sobre su eje hacia la pared para testearla:
         - Si choca/detecta la pared ahí: vuelve a su rumbo paralelo y sigue avanzando.
         - Si NO choca (la pared terminó antes / esquina saliente): avanza un mínimo hacia ese hueco y
           gira nuevamente hacia la izquierda (porque por la derecha ya vino) para rodear la esquina
           hasta reencontrar la pared.
    """

    GOLDEN_ANGLE_RAD = math.pi * (3.0 - math.sqrt(5.0))

    def __init__(
        self,
        config: RobotConfig = RobotConfig(),
        initial_pose: Pose2D = Pose2D(0.0, 0.0, 0.0),
        seed: int = 42,
        mode: str = "classic",
        relearn_enabled: bool = False,
        max_relearns: int = 15,
        turn_mode: str = "random",
        fixed_turn_deg: float = 135.0,
        turn_sequence_deg: Optional[List[float]] = None,
        test_stride_m: float = 0.42,
        test_corner_pass_m: float = 0.12,
        early_stop_perimeter: bool = False,
        target_perimeter_m: float = 0.0,
        early_stop_tol: float = 0.05,
    ) -> None:
        self.cfg = config
        self.mode = mode
        self.early_stop_perimeter = early_stop_perimeter and target_perimeter_m > 0.0
        self.target_perimeter_m = float(target_perimeter_m)
        self.early_stop_tol = float(early_stop_tol)
        self.stopped_early: bool = False
        # Con corte por perímetro activo también se habilita el reaprendizaje:
        # si frenó estando "cerca" pero le faltó un rincón, la vuelta lo corrige.
        self.relearn_enabled = relearn_enabled or self.early_stop_perimeter or mode in (
            "reaprendizaje",
            "recto_reaprendizaje",
            "bordeador",
            "testeador",
            "incidente",
        )
        self.MAX_RELEARNS: int = max(0, int(max_relearns))
        self.turn_mode = turn_mode
        self.fixed_turn_deg = float(fixed_turn_deg)
        self.turn_sequence_deg = (
            [float(x) for x in turn_sequence_deg if float(x) != 0.0]
            if turn_sequence_deg
            else [110.0, 145.0, 125.0]
        )

        self.rng = random.Random(seed)
        self.odom = OdometryEstimator(initial_pose=initial_pose)

        geom_mode = (
            "recto"
            if mode in ("recto", "recto_reaprendizaje", "testeador")
            else ("incidente" if mode == "incidente" else "classic")
        )
        # En modos de habitaciones ortogonales la vuelta verde nunca debe tener tramos diagonales
        self.ortho_route: bool = geom_mode in ("recto", "incidente")
        self.estimator = PerimeterAndAntsEstimator(
            ants_boundary_factor=config.ants_boundary_factor,
            collinear_angle_tol_rad=config.collinear_angle_tol_rad,
            min_chords_for_map=config.min_chords_for_map,
            convergence_window=config.convergence_window,
            area_stability_tol=config.area_stability_tol,
            mode=geom_mode,
        )

        self.state: MissionState = MissionState.IDLE
        self.target_heading_rad: float = initial_pose.theta
        self.last_wall_distance_m: float = 0.0
        self.last_estimates: Optional[MapEstimates] = None
        self.estimates_history: List[MapEstimates] = []

        # Sub-máquina para Modo Bordeador (escaneo bilateral izquierdo y derecho)
        self.bordeador_subphase: int = 0
        self.bordeador_base_heading: float = 0.0
        self.bordeador_left_heading: float = 0.0
        self.bordeador_right_heading: float = 0.0
        self.bordeador_left_Clearance: float = 0.0
        self.bordeador_right_Clearance: float = 0.0
        self.bordeador_step_ticks: int = 0

        # Sub-máquina para Modo Testeador (seguimiento de pared + testeo periódico hacia la pared)
        self.testeador_wall_side: float = -1.0  # -1.0 = pared a la derecha (dobló a la izquierda)
        self.testeador_cruise_heading: float = 0.0
        self.testeador_probe_heading: float = 0.0
        self.testeador_subphase: int = 0
        self.testeador_step_dist_m: float = 0.0
        self.testeador_probe_dist_m: float = 0.0
        self.TEST_STRIDE_M: float = max(0.05, float(test_stride_m))        # Distancia que avanza antes de volver a testear la pared
        self.TEST_PROBE_MAX_M: float = 0.22                              # Avance de sondeo hacia la pared para ver si sigue ahí
        self.TEST_CORNER_PASS_M: float = max(0.04, float(test_corner_pass_m))  # Avance mínimo tras descubrir que la pared terminó

        # Ruta del recorrido perimetral final
        self.reconstructed_perimeter: List[Point2D] = []
        self.safe_perimeter_waypoints: List[Point2D] = []
        self.current_waypoint_idx: int = 0
        self.perimeter_trace: List[Point2D] = []
        self.relearn_count: int = 0
        self.relearn_backoff_ticks: int = 0
        self.wp_stuck_ticks: int = 0

    def _should_stop_exploration(self, est: Optional[MapEstimates]) -> bool:
        """
        Decide si la exploración debe terminar:
          - Criterio clásico: mapa convergido o se alcanzó `max_chords_explore`.
          - Criterio "corte por perímetro real" (toggle del usuario): si ya se recorrieron
            al menos 8 cuerdas y el perímetro estimado coincide con el real dentro de
            `early_stop_tol` (±5%), frena sin esperar a completar todas las cuerdas.
        """
        if est is None:
            return False
        if est.converged or est.num_chords >= self.cfg.max_chords_explore:
            return True
        if (
            self.early_stop_perimeter
            and not self.stopped_early
            and est.num_chords >= min(8, self.cfg.max_chords_explore)
            and self.target_perimeter_m > 1e-6
            and abs(est.poly_perimeter_m - self.target_perimeter_m)
            / self.target_perimeter_m
            <= self.early_stop_tol
        ):
            self.stopped_early = True
            return True
        return False

    def start_mission(self) -> None:
        self.state = MissionState.EXPLORE_FWD
        self.target_heading_rad = self.odom.pose.theta
        self.odom.reset_chord_origin()

    def _select_next_exploration_heading(self) -> float:
        k = len(self.estimator.chords)

        if self.turn_mode == "fixed":
            delta_rad = math.radians(self.fixed_turn_deg)
            return wrap_angle(self.odom.pose.theta + delta_rad)

        if self.turn_mode == "sequence" and self.turn_sequence_deg:
            deg_val = self.turn_sequence_deg[k % len(self.turn_sequence_deg)]
            return wrap_angle(self.odom.pose.theta + math.radians(deg_val))

        if len(self.estimator.wall_points) >= 3:
            center = polygon_centroid(self.estimator.wall_points)
            to_center = math.atan2(
                center.y - self.odom.pose.y, center.x - self.odom.pose.x
            )
            if k % 3 == 0 and len(self.estimator.wall_points) >= 5:
                angles = sorted(
                    math.atan2(p.y - center.y, p.x - center.x)
                    for p in self.estimator.wall_points
                )
                best_gap = -1.0
                target_polar = 0.0
                n_ang = len(angles)
                for i in range(n_ang):
                    a1 = angles[i]
                    a2 = angles[(i + 1) % n_ang]
                    gap = (a2 - a1) if i + 1 < n_ang else (a2 + 2.0 * math.pi - a1)
                    if gap > best_gap:
                        best_gap = gap
                        target_polar = wrap_angle(
                            a1 + self.rng.uniform(0.25, 0.75) * gap
                        )

                far_dist = 2.5
                goal_x = center.x + far_dist * math.cos(target_polar)
                goal_y = center.y + far_dist * math.sin(target_polar)
                candidate = math.atan2(goal_y - self.odom.pose.y, goal_x - self.odom.pose.x)
                diff_from_inward = wrap_angle(candidate - to_center)
                if abs(diff_from_inward) <= 1.25:
                    return wrap_angle(candidate + self.rng.uniform(-0.22, 0.22))

            return wrap_angle(to_center + self.rng.uniform(-1.25, 1.25))

        return wrap_angle(
            self.odom.pose.theta + math.pi + self.rng.uniform(-0.95, 0.95)
        )

    def _impact_wall_dir(self) -> float:
        """
        Orientación (tangente) de la pared en el punto de impacto, usada por el modo
        Incidente: el frente del robot apunta aproximadamente a la normal de la pared, así
        que la pared corre perpendicular al rumbo del choque. En habitaciones rectas se
        ajusta al eje más cercano (pared vertical si el rumbo era mayormente horizontal).
        """
        th = self.odom.pose.theta
        return math.pi / 2.0 if abs(math.cos(th)) >= abs(math.sin(th)) else 0.0

    def _record_current_chord_and_wall(self, wall_dist_m: float, dt: float) -> None:
        """Registra un punto de contacto con la pared y la cuerda recorrida hasta este instante."""
        wall_pt = self.odom.project_wall_contact(self.cfg.robot_radius_m, wall_dist_m)
        self.estimator.add_wall_point(wall_pt, wall_dir=self._impact_wall_dir())

        duration = max(self.odom.total_time_s - self.odom.chord_start_time_s, dt)
        end_pt = self.odom.pose.as_point()
        chord_len = self.odom.chord_start_point.distance_to(end_pt)

        if chord_len >= 0.02:
            chord = ChordSegment(
                idx=len(self.estimator.chords) + 1,
                start=self.odom.chord_start_point,
                end=end_pt,
                duration_s=duration,
                avg_speed_mps=chord_len / duration,
                heading_rad=self.odom.chord_heading_rad,
            )
            self.estimator.add_chord(chord)

        est = self.estimator.evaluate()
        self.last_estimates = est
        self.estimates_history.append(est)

    def trigger_relearning_from_collision(self, wall_contact_pt: Point2D) -> None:
        self.estimator.add_wall_point(wall_contact_pt, wall_dir=self._impact_wall_dir())
        est = self.estimator.evaluate()
        self.last_estimates = est
        self.estimates_history.append(est)
        self.relearn_count += 1
        self.relearn_backoff_ticks = 12
        self.wp_stuck_ticks = 0
        self.perimeter_trace.clear()
        self.state = MissionState.BUILD_SAFE_PATH

    def _pick_ortho_elbow(
        self, a: Point2D, b: Point2D, prev_wp: Optional[Point2D] = None
    ) -> Optional[Point2D]:
        """
        Devuelve el vértice de codo a 90° (L) válido entre `a` y `b` en modos ortogonales,
        exigiendo que ambas patas del recodo permanezcan dentro del perímetro reconstruido
        sin cruzar paredes.
        Si se conoce `prev_wp`, preferimos el codo cuyo SEGUNDO tramo (codo -> b) siga la
        dirección de la arista entrante del perímetro interior, para que el robot se
        reincorpore al carril correcto en vez de desplazarse a un carril interior erróneo.
        """
        boundary = self.reconstructed_perimeter
        if len(boundary) < 3:
            return None
        m = len(boundary)

        cands = [Point2D(b.x, a.y), Point2D(a.x, b.y)]
        if prev_wp is not None:
            incoming_horiz = abs(b.y - prev_wp.y) <= abs(b.x - prev_wp.x)
            # Arista entrante horizontal -> el segundo tramo debe ser horizontal: codo (a.x, b.y)
            # Arista entrante vertical   -> el segundo tramo debe ser vertical:   codo (b.x, a.y)
            cands = [cands[1], cands[0]] if incoming_horiz else cands

        for e in cands:
            ok = True
            for s, t in ((a, e), (e, b)):
                if not ok or s.distance_to(t) < 1e-9:
                    continue
                mid = Point2D((s.x + t.x) / 2.0, (s.y + t.y) / 2.0)
                if not point_in_polygon(mid, boundary):
                    ok = False
                    break
                for k in range(m):
                    hit = segment_intersection(
                        s, t, boundary[k], boundary[(k + 1) % m], eps=1e-4
                    )
                    if (
                        hit is not None
                        and hit.distance_to(a) > 0.01
                        and hit.distance_to(b) > 0.01
                        and hit.distance_to(e) > 0.01
                    ):
                        ok = False
                        break
                if not ok:
                    break
            if ok:
                return e
        return None

    def _align_to_heading_cmd(self, target_rad: float) -> Optional[ActuatorCommand]:
        """Si el error angular hacia `target_rad` supera ~1.8°, devuelve el comando de giro sobre el eje; si ya está alineado devuelve None."""
        err = wrap_angle(target_rad - self.odom.pose.theta)
        if abs(err) <= 0.032:
            return None
        omega = max(
            -self.cfg.max_angular_speed_radps,
            min(self.cfg.max_angular_speed_radps, self.cfg.heading_kp * err),
        )
        if abs(omega) < 0.40:
            omega = math.copysign(0.40, err)
        return ActuatorCommand(0.0, omega, max(-0.6, min(0.6, err)))

    def step(
        self, front_distance_m: float, bumper_triggered: bool, measured_dt_s: Optional[float] = None
    ) -> ActuatorCommand:
        dt = measured_dt_s if measured_dt_s is not None else self.cfg.dt_s

        if self.state == MissionState.IDLE or self.state == MissionState.COMPLETED:
            return ActuatorCommand(0.0, 0.0, 0.0)

        # =====================================================================
        # 1. ESTADO: EXPLORE_FWD (Avance hasta primer choque o choque de cuerda)
        # =====================================================================
        if self.state == MissionState.EXPLORE_FWD:
            if (bumper_triggered or front_distance_m <= self.cfg.stop_distance_m) and (
                self.odom.chord_distance_m >= 0.08 or front_distance_m <= self.cfg.stop_distance_m
            ):
                self.last_wall_distance_m = 0.0 if bumper_triggered else front_distance_m
                self.state = MissionState.BRAKE_RECORD
                return ActuatorCommand(0.0, 0.0, 0.0)

            err = wrap_angle(self.target_heading_rad - self.odom.pose.theta)
            omega = max(
                -self.cfg.max_angular_speed_radps,
                min(self.cfg.max_angular_speed_radps, self.cfg.heading_kp * err),
            )
            speed = 0.0 if abs(err) > 0.05 else self.cfg.cruise_speed_mps
            return ActuatorCommand(speed, omega, err)

        # =====================================================================
        # 2. ESTADO: BRAKE_RECORD (Registra impacto y decide transición según modo)
        # =====================================================================
        if self.state == MissionState.BRAKE_RECORD:
            self._record_current_chord_and_wall(self.last_wall_distance_m, dt)
            est = self.last_estimates

            if self._should_stop_exploration(est):
                self.state = MissionState.BUILD_SAFE_PATH
                return ActuatorCommand(0.0, 0.0, 0.0)

            # Si estamos en MODO BORDEADOR: al chocar prueba girar y avanzar hacia ambos lados
            if self.mode == "bordeador":
                self.bordeador_base_heading = self.odom.pose.theta
                # Probar candidatos a ~95°-115° a la izquierda y a la derecha (bordeando la pared)
                ang_probe = math.radians(102.0 + self.rng.uniform(-10.0, 12.0))
                self.bordeador_left_heading = wrap_angle(self.bordeador_base_heading + ang_probe)
                self.bordeador_right_heading = wrap_angle(self.bordeador_base_heading - ang_probe)
                self.bordeador_subphase = 0
                self.bordeador_step_ticks = 0
                self.state = MissionState.BORDEADOR_SCAN
                return ActuatorCommand(0.0, 0.0, 0.0)

            # Si estamos en MODO TESTEADOR: al chocar dobla a la izquierda (pared queda a su derecha)
            if self.mode == "testeador":
                # Alinear el rumbo al múltiplo de 90° más cercano para seguir la pared prolijamente
                snapped_wall_dir = round(self.odom.pose.theta / (0.5 * math.pi)) * (0.5 * math.pi)
                # Doblar +90° a la izquierda -> la pared que acabamos de chocar queda a -90° (derecha)
                self.testeador_cruise_heading = wrap_angle(snapped_wall_dir + 0.5 * math.pi)
                self.testeador_wall_side = -1.0  # Derecha
                self.testeador_subphase = 0
                self.testeador_step_dist_m = 0.0
                self.state = MissionState.TESTEADOR_CRUISE
                return ActuatorCommand(0.0, 0.0, 0.0)

            self.state = MissionState.EVALUATE_MAP
            return ActuatorCommand(0.0, 0.0, 0.0)

        # =====================================================================
        # 3. MODO BORDEADOR: Gira y avanza hacia ambos lados, evalúa y elige el mejor
        # =====================================================================
        if self.state == MissionState.BORDEADOR_SCAN:
            # Subfase 0: Girar hacia el lado Izquierdo
            if self.bordeador_subphase == 0:
                cmd = self._align_to_heading_cmd(self.bordeador_left_heading)
                if cmd is not None:
                    return cmd
                self.bordeador_subphase = 1
                self.bordeador_step_ticks = 0

            # Subfase 1: Avanzar un pequeño paso a la izquierda y medir cuán prometedor es
            if self.bordeador_subphase == 1:
                if self.bordeador_step_ticks < 6 and front_distance_m > self.cfg.stop_distance_m + 0.03:
                    self.bordeador_step_ticks += 1
                    return ActuatorCommand(0.22, 0.0, 0.0)
                # Medir distancia libre en dirección izquierda y penalizar si ya hay muchos puntos ahí
                pt_proj_l = Point2D(
                    self.odom.pose.x + front_distance_m * math.cos(self.odom.pose.theta),
                    self.odom.pose.y + front_distance_m * math.sin(self.odom.pose.theta),
                )
                min_dist_existing_l = min(
                    (pt_proj_l.distance_to(wp) for wp in self.estimator.wall_points),
                    default=2.0,
                )
                self.bordeador_left_Clearance = front_distance_m * (0.6 + 0.4 * min(min_dist_existing_l, 2.0))
                self.bordeador_subphase = 2
                return ActuatorCommand(0.0, 0.0, 0.0)

            # Subfase 2: Volver al punto de decisión y girar hacia el lado Derecho
            if self.bordeador_subphase == 2:
                if self.bordeador_step_ticks > 0:
                    self.bordeador_step_ticks -= 1
                    return ActuatorCommand(-0.22, 0.0, 0.0)
                cmd = self._align_to_heading_cmd(self.bordeador_right_heading)
                if cmd is not None:
                    return cmd
                self.bordeador_subphase = 3
                self.bordeador_step_ticks = 0

            # Subfase 3: Avanzar un pequeño paso a la derecha y medir cuán prometedor es
            if self.bordeador_subphase == 3:
                if self.bordeador_step_ticks < 6 and front_distance_m > self.cfg.stop_distance_m + 0.03:
                    self.bordeador_step_ticks += 1
                    return ActuatorCommand(0.22, 0.0, 0.0)
                pt_proj_r = Point2D(
                    self.odom.pose.x + front_distance_m * math.cos(self.odom.pose.theta),
                    self.odom.pose.y + front_distance_m * math.sin(self.odom.pose.theta),
                )
                min_dist_existing_r = min(
                    (pt_proj_r.distance_to(wp) for wp in self.estimator.wall_points),
                    default=2.0,
                )
                self.bordeador_right_Clearance = front_distance_m * (0.6 + 0.4 * min(min_dist_existing_r, 2.0))

                # Elegir el camino más prometedor entre Izquierda y Derecha
                if self.bordeador_left_Clearance > self.bordeador_right_Clearance and self.bordeador_left_Clearance > 0.18:
                    self.target_heading_rad = self.bordeador_left_heading
                elif self.bordeador_right_Clearance > 0.18:
                    self.target_heading_rad = self.bordeador_right_heading
                else:
                    # Si ambos lados están encerrados (esquina aguda), salir hacia el interior
                    self.target_heading_rad = self._select_next_exploration_heading()

                self.state = MissionState.TURN_HEADING
                return ActuatorCommand(0.0, 0.0, 0.0)

        # =====================================================================
        # 4. MODO TESTEADOR: Avanza paralelo a la pared y la tantea periódicamente
        # =====================================================================
        if self.state == MissionState.TESTEADOR_CRUISE:
            # Subfase 0: Alinearse con la dirección paralela a la pared (cruise_heading)
            cmd = self._align_to_heading_cmd(self.testeador_cruise_heading)
            if cmd is not None:
                return cmd

            # Iniciar nueva cuerda si recién terminamos de girar
            if self.testeador_step_dist_m == 0.0:
                self.odom.reset_chord_origin()

            # Si choca de frente contra una pared transversal (esquina cóncava / interior):
            # registra el choque y dobla a la izquierda (+90°) manteniendo la nueva pared a su derecha
            if bumper_triggered or front_distance_m <= self.cfg.stop_distance_m:
                self.last_wall_distance_m = 0.0 if bumper_triggered else front_distance_m
                self._record_current_chord_and_wall(self.last_wall_distance_m, dt)
                if self._should_stop_exploration(self.last_estimates):
                    self.state = MissionState.BUILD_SAFE_PATH
                    return ActuatorCommand(0.0, 0.0, 0.0)
                self.testeador_cruise_heading = wrap_angle(self.testeador_cruise_heading + 0.5 * math.pi)
                self.testeador_step_dist_m = 0.0
                return ActuatorCommand(0.0, 0.0, 0.0)

            # Si ya avanzó un tramo TEST_STRIDE_M, pasa a TESTEADOR_PROBE para girar hacia la pared y testearla
            if self.testeador_step_dist_m >= self.TEST_STRIDE_M:
                # La pared está a la derecha (wall_side = -1 -> -90°)
                self.testeador_probe_heading = wrap_angle(
                    self.testeador_cruise_heading + self.testeador_wall_side * (0.5 * math.pi)
                )
                self.testeador_subphase = 0
                self.testeador_probe_dist_m = 0.0
                self.state = MissionState.TESTEADOR_PROBE
                return ActuatorCommand(0.0, 0.0, 0.0)

            # Avanzar un paso por el camino paralelo
            self.testeador_step_dist_m += self.cfg.cruise_speed_mps * dt
            err = wrap_angle(self.testeador_cruise_heading - self.odom.pose.theta)
            omega = max(-self.cfg.max_angular_speed_radps, min(self.cfg.max_angular_speed_radps, self.cfg.heading_kp * err))
            return ActuatorCommand(self.cfg.cruise_speed_mps, omega, err)

        if self.state == MissionState.TESTEADOR_PROBE:
            # Subfase 0: Girar sobre su eje 90° hacia el lado donde estaba la pared
            if self.testeador_subphase == 0:
                cmd = self._align_to_heading_cmd(self.testeador_probe_heading)
                if cmd is not None:
                    return cmd
                self.odom.reset_chord_origin()
                self.testeador_subphase = 1
                self.testeador_probe_dist_m = 0.0

            # Subfase 1: Intentar avanzar hacia la pared para comprobar si sigue ahí
            if self.testeador_subphase == 1:
                if bumper_triggered or front_distance_m <= self.cfg.stop_distance_m + 0.02:
                    # ¡Chocó con la pared! La pared sigue estando ahí:
                    # Registra el punto de pared, retrocede a su carril y retoma su camino normal
                    self._record_current_chord_and_wall(0.0 if bumper_triggered else front_distance_m, dt)
                    if self._should_stop_exploration(self.last_estimates):
                        self.state = MissionState.BUILD_SAFE_PATH
                        return ActuatorCommand(0.0, 0.0, 0.0)
                    self.testeador_subphase = 2
                    return ActuatorCommand(0.0, 0.0, 0.0)

                if self.testeador_probe_dist_m < self.TEST_PROBE_MAX_M:
                    self.testeador_probe_dist_m += 0.28 * dt
                    return ActuatorCommand(0.28, 0.0, 0.0)

                # ¡NO chocó! Eso significa que la pared terminó antes (esquina saliente / abertura):
                # Avanza un mínimo más para librar la esquina y gira para rodearla hacia el nuevo borde
                self.testeador_subphase = 3
                return ActuatorCommand(0.0, 0.0, 0.0)

            # Subfase 2: Retroceder los pocos cm del tanteo tras haber tocado la pared y seguir recto
            if self.testeador_subphase == 2:
                if self.testeador_probe_dist_m > 0.01:
                    self.testeador_probe_dist_m -= 0.28 * dt
                    return ActuatorCommand(-0.28, 0.0, 0.0)
                self.testeador_step_dist_m = 0.0
                self.state = MissionState.TESTEADOR_CRUISE
                return ActuatorCommand(0.0, 0.0, 0.0)

            # Subfase 3: La pared terminó -> ya entró al hueco; adopta este rumbo como nuevo crucero
            # y en el próximo paso volverá a testear hacia el costado para reencontrar la pared doblada
            if self.testeador_subphase == 3:
                if self.testeador_probe_dist_m < self.TEST_PROBE_MAX_M + self.TEST_CORNER_PASS_M:
                    if bumper_triggered or front_distance_m <= self.cfg.stop_distance_m:
                        self._record_current_chord_and_wall(0.0 if bumper_triggered else front_distance_m, dt)
                        self.testeador_cruise_heading = wrap_angle(self.testeador_probe_heading + 0.5 * math.pi)
                        self.testeador_step_dist_m = 0.0
                        self.state = MissionState.TESTEADOR_CRUISE
                        return ActuatorCommand(0.0, 0.0, 0.0)
                    self.testeador_probe_dist_m += 0.28 * dt
                    return ActuatorCommand(0.28, 0.0, 0.0)

                # Ahora el nuevo rumbo principal es el del hueco por el que doblamos
                self.testeador_cruise_heading = self.testeador_probe_heading
                self.testeador_step_dist_m = self.TEST_STRIDE_M * 0.5
                self.state = MissionState.TESTEADOR_CRUISE
                return ActuatorCommand(0.0, 0.0, 0.0)

        # =====================================================================
        # 5. ESTADO: EVALUATE_MAP y TURN_HEADING (Modos por cuerdas)
        # =====================================================================
        if self.state == MissionState.EVALUATE_MAP:
            est = self.last_estimates or self.estimator.evaluate()
            if self._should_stop_exploration(est):
                self.state = MissionState.BUILD_SAFE_PATH
            else:
                self.target_heading_rad = self._select_next_exploration_heading()
                self.state = MissionState.TURN_HEADING
            return ActuatorCommand(0.0, 0.0, 0.0)

        if self.state == MissionState.TURN_HEADING:
            cmd = self._align_to_heading_cmd(self.target_heading_rad)
            if cmd is None:
                self.odom.reset_chord_origin()
                self.state = MissionState.EXPLORE_FWD
                return ActuatorCommand(0.0, 0.0, 0.0)
            return cmd

        # =====================================================================
        # 6. ESTADO: BUILD_SAFE_PATH (Construye ruta interior anclada SIN diagonales)
        # =====================================================================
        if self.state == MissionState.BUILD_SAFE_PATH:
            if self.relearn_backoff_ticks > 0:
                self.relearn_backoff_ticks -= 1
                center = polygon_centroid(self.estimator.wall_points)
                to_center = math.atan2(center.y - self.odom.pose.y, center.x - self.odom.pose.x)
                err = wrap_angle(to_center - self.odom.pose.theta)
                if abs(err) > 0.25:
                    return ActuatorCommand(0.0, math.copysign(2.2, err), 0.0)
                return ActuatorCommand(0.25, 0.0, 0.0)

            self.reconstructed_perimeter = self.estimator.reconstruct_perimeter_polygon()
            margin_m = self.cfg.robot_radius_m * 1.2
            inset_pts = compute_inward_offset_polygon(
                self.reconstructed_perimeter, margin_m, ortho=self.ortho_route
            )

            # Usar proyección perpendicular sobre la arista más cercana para eliminar el corte diagonal
            curr_pos = self.odom.pose.as_point()
            self.safe_perimeter_waypoints = build_anchored_perimeter_route(curr_pos, inset_pts)
            # Incrementar dinámicamente el límite de reaprendizaje si la figura es muy compleja
            if self.relearn_enabled and self.relearn_count >= self.MAX_RELEARNS - 1 and self.MAX_RELEARNS == 15:
                self.MAX_RELEARNS = 25

            self.current_waypoint_idx = 0
            self.wp_stuck_ticks = 0
            self.state = (
                MissionState.FOLLOW_PERIMETER
                if self.safe_perimeter_waypoints
                else MissionState.COMPLETED
            )
            return ActuatorCommand(0.0, 0.0, 0.0)

        # =====================================================================
        # 7. ESTADO: FOLLOW_PERIMETER (Recorrido perimetral final limpio)
        # =====================================================================
        if self.state == MissionState.FOLLOW_PERIMETER:
            # Solo grabar el rastro verde una vez que el robot ya se apoyó en el punto de inicio perpendicular (idx >= 1)
            if self.current_waypoint_idx >= 1:
                self.perimeter_trace.append(self.odom.pose.as_point())
            self.wp_stuck_ticks += 1

            if self.current_waypoint_idx >= len(self.safe_perimeter_waypoints):
                self.state = MissionState.COMPLETED
                return ActuatorCommand(0.0, 0.0, 0.0)

            target_wp = self.safe_perimeter_waypoints[self.current_waypoint_idx]
            curr_pos = self.odom.pose.as_point()
            dist_to_wp = curr_pos.distance_to(target_wp)

            # En modos ortogonales jamás navegar en diagonal: si el tramo hasta el waypoint
            # no es alineado a ejes, insertar un codo a 90° (L) como waypoint intermedio.
            if (
                self.ortho_route
                and dist_to_wp > self.cfg.waypoint_tolerance_m
                and abs(target_wp.x - curr_pos.x) > 0.03
                and abs(target_wp.y - curr_pos.y) > 0.03
            ):
                prev_wp = (
                    self.safe_perimeter_waypoints[self.current_waypoint_idx - 1]
                    if self.current_waypoint_idx > 0
                    else None
                )
                elbow = self._pick_ortho_elbow(curr_pos, target_wp, prev_wp)
                if elbow is not None:
                    self.safe_perimeter_waypoints.insert(self.current_waypoint_idx, elbow)
                    target_wp = elbow
                    dist_to_wp = curr_pos.distance_to(target_wp)

            if dist_to_wp <= self.cfg.waypoint_tolerance_m or self.wp_stuck_ticks > 350:
                if self.current_waypoint_idx == 0:
                    self.perimeter_trace.append(curr_pos)
                self.current_waypoint_idx += 1
                self.wp_stuck_ticks = 0
                if self.current_waypoint_idx >= len(self.safe_perimeter_waypoints):
                    self.state = MissionState.COMPLETED
                return ActuatorCommand(0.0, 0.0, 0.0)

            desired_theta = math.atan2(target_wp.y - curr_pos.y, target_wp.x - curr_pos.x)
            err = wrap_angle(desired_theta - self.odom.pose.theta)

            if abs(err) > math.radians(4.0):
                omega = max(
                    -self.cfg.max_angular_speed_radps,
                    min(self.cfg.max_angular_speed_radps, self.cfg.heading_kp * err),
                )
                if abs(omega) < 0.35:
                    omega = math.copysign(0.35, err)
                return ActuatorCommand(0.0, omega, max(-0.6, min(0.6, err)))

            if (bumper_triggered or front_distance_m <= 0.025) and self.relearn_enabled and self.relearn_count < self.MAX_RELEARNS:
                new_wall_pt = self.odom.project_wall_contact(
                    self.cfg.robot_radius_m, 0.0 if bumper_triggered else front_distance_m
                )
                self.trigger_relearning_from_collision(new_wall_pt)
                return ActuatorCommand(0.0, 0.0, 0.0)

            speed = min(self.cfg.perimeter_speed_mps, max(0.10, 2.2 * dist_to_wp))
            omega = max(
                -self.cfg.max_angular_speed_radps,
                min(self.cfg.max_angular_speed_radps, self.cfg.heading_kp * err),
            )
            return ActuatorCommand(speed, omega, max(-0.5, min(0.5, err)))

        return ActuatorCommand(0.0, 0.0, 0.0)
