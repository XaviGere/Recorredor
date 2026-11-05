# Parámetros físicos, cinemáticos y de calibración del robot Recorredor (Unidades SI)
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RobotConfig:
    # Geometría física del chasis (metros)
    robot_radius_m: float = 0.12          # Radio envolvente del robot (12 cm)
    wheel_radius_m: float = 0.0325        # Radio de rueda motriz (32.5 mm)
    wheelbase_m: float = 0.18             # Distancia entre ruedas / trocha (18 cm)
    ticks_per_rev: int = 720              # Pulsos de encoder por vuelta de rueda

    # Parámetros de sensado y seguridad (metros)
    stop_distance_m: float = 0.04         # Distancia frontal del sensor para frenar cerca de la pared
    backup_distance_m: float = 0.04       # Retroceso si se activó el bumper físico
    safe_perimeter_margin_m: float = 0.12 * 1.2  # Offset interior: Radio del círculo * 1.2 (0.144 m)

    # Cinemática y lazo de control
    dt_s: float = 0.02                    # Paso de integración (50 Hz = 20 ms)
    cruise_speed_mps: float = 0.45        # Velocidad lineal de exploración (m/s)
    perimeter_speed_mps: float = 0.35     # Velocidad lineal en recorrido perimetral final (m/s)
    max_angular_speed_radps: float = 2.5  # Velocidad angular máxima de giro (rad/s)
    heading_kp: float = 4.0               # Ganancia proporcional de control de rumbo
    waypoint_tolerance_m: float = 0.025   # Radio de llegada exacto a cada esquina del perímetro (2.5 cm)

    # Criterios de exploración y Método de las Hormigas / Cauchy-Crofton
    min_chords_for_map: int = 24          # Mínimo de tramos antes de evaluar convergencia
    max_chords_explore: int = 36          # Máximo de tramos de exploración antes de pasar a recorrido
    convergence_window: int = 4           # Ventana de rebotes para evaluar estabilidad del área
    area_stability_tol: float = 0.025     # 2.5% de variación relativa para declarar convergencia
    ants_boundary_factor: float = 1.18    # Factor de calibración geométrico de cuerdas de frontera
    collinear_angle_tol_rad: float = math.radians(8.0)  # Tolerancia para fusionar puntos en una misma pared
    rdp_epsilon_m: float = 0.06           # Tolerancia de simplificación poligonal (6 cm)

    # Ruido en simulación (calibración realista de hardware)
    odom_speed_noise_std: float = 0.002   # Desviación estándar de velocidad en m/s
    odom_gyro_noise_std: float = 0.001    # Desviación estándar angular en rad/s
    range_sensor_noise_std: float = 0.003 # Ruido del sensor ultrasónico/ToF en m
