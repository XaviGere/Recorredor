# Integrador cinemático de odometría (tiempo, velocidad lineal y velocidad angular -> coordenadas x, y, theta)
import math
from sim.core.geometry import Point2D, Pose2D, wrap_angle


class OdometryEstimator:
    """
    Estima la pose 2D (x, y, theta) del robot mediante integración de punto medio (Runge-Kutta 2º orden)
    a partir de intervalos de tiempo dt, velocidad lineal v y velocidad angular omega.
    """

    def __init__(self, initial_pose: Pose2D = Pose2D(0.0, 0.0, 0.0)) -> None:
        self.pose = initial_pose
        self.total_time_s: float = 0.0
        self.total_distance_m: float = 0.0

        # Variables acumuladoras para el tramo rectilíneo (cuerda) actual
        self.chord_start_point: Point2D = initial_pose.as_point()
        self.chord_start_time_s: float = 0.0
        self.chord_distance_m: float = 0.0
        self.chord_heading_rad: float = initial_pose.theta

    def reset_chord_origin(self) -> None:
        """Marca el inicio de un nuevo tramo rectilíneo tras haber girado."""
        self.chord_start_point = self.pose.as_point()
        self.chord_start_time_s = self.total_time_s
        self.chord_distance_m = 0.0
        self.chord_heading_rad = self.pose.theta

    def integrate_step(self, v_mps: float, omega_radps: float, dt_s: float) -> Pose2D:
        """
        Actualiza la coordenada estimada usando integración de punto medio:
            x_{k+1} = x_k + v * dt * cos(theta_k + omega * dt / 2)
            y_{k+1} = y_k + v * dt * sin(theta_k + omega * dt / 2)
            theta_{k+1} = wrap_angle(theta_k + omega * dt)
        """
        ds = v_mps * dt_s
        dtheta = omega_radps * dt_s
        mid_theta = self.pose.theta + 0.5 * dtheta

        nx = self.pose.x + ds * math.cos(mid_theta)
        ny = self.pose.y + ds * math.sin(mid_theta)
        ntheta = wrap_angle(self.pose.theta + dtheta)

        self.pose = Pose2D(nx, ny, ntheta)
        self.total_time_s += dt_s
        self.total_distance_m += abs(ds)
        self.chord_distance_m += abs(ds)
        return self.pose

    def project_wall_contact(self, robot_radius_m: float, sensor_distance_m: float) -> Point2D:
        """
        Calcula la coordenada exacta del punto de pared que detuvo al robot, sumando
        el radio del chasis y la distancia medida por el sensor en la dirección actual.
        """
        offset = robot_radius_m + max(0.0, sensor_distance_m)
        wx = self.pose.x + offset * math.cos(self.pose.theta)
        wy = self.pose.y + offset * math.sin(self.pose.theta)
        return Point2D(wx, wy)
