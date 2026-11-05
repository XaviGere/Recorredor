# Estimadores de Perímetro y Área: Método de las Hormigas (Mallock/Buffon), Cauchy-Crofton y Poligonal
from dataclasses import dataclass
import math
from typing import List, Tuple

from sim.core.geometry import (
    ChordSegment,
    Point2D,
    convex_hull,
    polygon_area,
    polygon_centroid,
    polygon_perimeter,
    reconstruct_walled_polygon,
    segment_intersection,
    simplify_polygon_collinear,
)


@dataclass(frozen=True)
class MapEstimates:
    num_chords: int
    num_wall_points: int
    num_crossings: int
    total_path_length_m: float
    mean_chord_length_m: float
    poly_area_m2: float
    poly_perimeter_m: float
    ants_pairs_area_m2: float
    ants_mallock_split_area_m2: float
    crofton_perimeter_from_ants_m: float
    crofton_perimeter_from_poly_m: float
    angular_sectors_covered: int
    converged: bool


class PerimeterAndAntsEstimator:
    """
    Mantiene el registro de impactos en pared, las cuerdas (trazos rectos del robot) y todos los
    cruces entre rastros para calcular en paralelo:
      1. El polígono envolvente del perímetro (Convex Hull simplificado + Shoelace).
      2. El Área por el Método de las Hormigas (Buffon-Mallock sobre pares de cuerdas y split-half).
      3. El Perímetro por el Teorema de Cauchy-Crofton (E[L] = pi * A / P).
    """

    def __init__(
        self,
        ants_boundary_factor: float = 1.18,
        collinear_angle_tol_rad: float = math.radians(8.0),
        min_chords_for_map: int = 16,
        convergence_window: int = 4,
        area_stability_tol: float = 0.025,
        mode: str = "classic",
    ) -> None:
        self.ants_boundary_factor = ants_boundary_factor
        self.collinear_angle_tol_rad = collinear_angle_tol_rad
        self.min_chords_for_map = min_chords_for_map
        self.convergence_window = convergence_window
        self.area_stability_tol = area_stability_tol
        self.mode = mode

        self.wall_points: List[Point2D] = []
        # Orientación de la pared en cada impacto (dirección de la tangente, rad mod pi).
        # La usa el modo Incidente para ajustar rectas de pared reales.
        self.wall_dirs: List[float] = []
        self.chords: List[ChordSegment] = []
        self.crossings: List[Point2D] = []
        self.crossing_pairs: List[Tuple[int, int]] = []
        self.area_history: List[float] = []

    def add_wall_point(self, pt: Point2D, wall_dir=None) -> None:
        self.wall_points.append(pt)
        self.wall_dirs.append(wall_dir)

    def add_chord(self, chord: ChordSegment) -> List[Point2D]:
        """
        Agrega un nuevo tramo recto recorrido por el robot y calcula todas sus intersecciones
        con las cuerdas anteriores no adyacentes (|i - j| > 1).
        """
        new_crossings: List[Point2D] = []
        # Excluir la cuerda inmediatamente anterior (comparten el vértice de rebote en la pared)
        for prev in self.chords[:-1]:
            if abs(chord.idx - prev.idx) <= 1:
                continue
            hit = segment_intersection(prev.start, prev.end, chord.start, chord.end)
            if hit is not None:
                self.crossings.append(hit)
                self.crossing_pairs.append((prev.idx, chord.idx))
                new_crossings.append(hit)

        self.chords.append(chord)
        current_poly = self.reconstruct_perimeter_polygon()
        self.area_history.append(polygon_area(current_poly))
        return new_crossings

    def reconstruct_perimeter_polygon(self) -> List[Point2D]:
        """Reconstruye los vértices del perímetro a partir de los puntos de impacto registrados."""
        if len(self.wall_points) < 3:
            return list(self.wall_points)
        return reconstruct_walled_polygon(
            self.wall_points,
            mode=self.mode,
            free_chords=self.chords,
            wall_dirs=self.wall_dirs,
        )

    def compute_ants_area_all_pairs(self) -> float:
        """
        Estimador generalizado de las Hormigas (Buffon-Laplace) sobre todos los pares de cuerdas
        no adyacentes:
            A_hat = (2 * eta / (pi * C)) * sum_{i < j-1} (l_i * l_j)
        """
        n = len(self.chords)
        c = len(self.crossings)
        if n < 3 or c == 0:
            return 0.0

        sum_length_products = 0.0
        for i in range(n):
            li = self.chords[i].length
            for j in range(i + 2, n):
                sum_length_products += li * self.chords[j].length

        return (2.0 * self.ants_boundary_factor * sum_length_products) / (math.pi * c)

    def compute_ants_area_mallock_two_pass(self) -> float:
        """
        Fórmula clásica de Mallock (Mugford, Mallock & Franks, 2000):
        Divide el recorrido en Primera Mitad (Rastro 1 de longitud L1) y Segunda Mitad (Rastro 2
        de longitud L2) y cuenta los cruces C_12 entre ambos rastros:
            A_mallock = (2 * eta * L1 * L2) / (pi * C_12)
        """
        n = len(self.chords)
        if n < 4:
            return 0.0

        mid = n // 2
        first_half = self.chords[:mid]
        second_half = self.chords[mid:]

        l1 = sum(c.length for c in first_half)
        l2 = sum(c.length for c in second_half)

        crossings_12 = 0
        for c1 in first_half:
            for c2 in second_half:
                if abs(c1.idx - c2.idx) <= 1:
                    continue
                if segment_intersection(c1.start, c1.end, c2.start, c2.end) is not None:
                    crossings_12 += 1

        if crossings_12 == 0:
            return 0.0
        return (2.0 * self.ants_boundary_factor * l1 * l2) / (math.pi * crossings_12)

    def count_covered_angular_sectors(self, num_sectors: int = 8) -> int:
        """Cuenta cuántos sectores angulares alrededor del centroide ya tienen puntos de pared."""
        if len(self.wall_points) < 3:
            return len(self.wall_points)
        center = polygon_centroid(self.wall_points)
        visited = set()
        for p in self.wall_points:
            ang = math.atan2(p.y - center.y, p.x - center.x) + math.pi  # [0, 2*pi]
            sector = int((ang / (2.0 * math.pi)) * num_sectors) % num_sectors
            visited.add(sector)
        return len(visited)

    def evaluate(self) -> MapEstimates:
        """Calcula todas las métricas geométricas y estocásticas y evalúa si el mapa convergió."""
        poly = self.reconstruct_perimeter_polygon()
        p_area = polygon_area(poly)
        p_perim = polygon_perimeter(poly)

        total_len = sum(c.length for c in self.chords)
        mean_chord = total_len / len(self.chords) if self.chords else 0.0

        ants_pairs_area = self.compute_ants_area_all_pairs()
        ants_mallock_area = self.compute_ants_area_mallock_two_pass()

        # Teorema de Cauchy-Crofton: E[l] = pi * A / P  =>  P = pi * A / E[l]
        crofton_perim_ants = (
            (math.pi * ants_pairs_area / mean_chord) if mean_chord > 1e-6 else 0.0
        )
        crofton_perim_poly = (
            (math.pi * p_area / mean_chord) if mean_chord > 1e-6 else 0.0
        )

        sectors = self.count_covered_angular_sectors(num_sectors=8)

        # Evaluar convergencia cuando tenemos suficientes cuerdas y cobertura de los 8 sectores
        converged = False
        if (
            len(self.chords) >= self.min_chords_for_map
            and sectors >= 8
            and len(self.area_history) >= self.convergence_window + 1
            and p_area > 0.1
        ):
            prev_area = self.area_history[-(self.convergence_window + 1)]
            rel_change = abs(p_area - prev_area) / p_area
            if rel_change <= self.area_stability_tol:
                converged = True

        return MapEstimates(
            num_chords=len(self.chords),
            num_wall_points=len(self.wall_points),
            num_crossings=len(self.crossings),
            total_path_length_m=total_len,
            mean_chord_length_m=mean_chord,
            poly_area_m2=p_area,
            poly_perimeter_m=p_perim,
            ants_pairs_area_m2=ants_pairs_area,
            ants_mallock_split_area_m2=ants_mallock_area,
            crofton_perimeter_from_ants_m=crofton_perim_ants,
            crofton_perimeter_from_poly_m=crofton_perim_poly,
            angular_sectors_covered=sectors,
            converged=converged,
        )
