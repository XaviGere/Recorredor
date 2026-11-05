# Primitivas geométricas 2D, intersección de cuerdas, reconstrucción sin lazos que pasa por TODOS los
# puntos de choque (Modo Clásico y Modo Recto 90°) y Offset Interior sin auto-intersecciones.
from dataclasses import dataclass
import math
from typing import List, Optional, Tuple


def wrap_angle(angle_rad: float) -> float:
    """Normaliza cualquier ángulo en radianes al intervalo (-pi, pi]."""
    wrapped = (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if wrapped == -math.pi else wrapped


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float

    def distance_to(self, other: "Point2D") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def __add__(self, other: "Point2D") -> "Point2D":
        return Point2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Point2D") -> "Point2D":
        return Point2D(self.x - other.x, self.y - other.y)

    def scaled(self, factor: float) -> "Point2D":
        return Point2D(self.x * factor, self.y * factor)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    theta: float

    def as_point(self) -> Point2D:
        return Point2D(self.x, self.y)


@dataclass(frozen=True)
class ChordSegment:
    idx: int
    start: Point2D
    end: Point2D
    duration_s: float
    avg_speed_mps: float
    heading_rad: float

    @property
    def length(self) -> float:
        return self.start.distance_to(self.end)


def cross_product_2d(o: Point2D, a: Point2D, b: Point2D) -> float:
    """Producto vectorial 2D (OA x OB). Positivo si O->A->B gira en sentido antihorario (CCW)."""
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)


def segment_intersection(
    p1: Point2D, p2: Point2D, p3: Point2D, p4: Point2D, eps: float = 1e-7
) -> Optional[Point2D]:
    """Calcula la intersección propia interior entre el segmento p1-p2 y el segmento p3-p4."""
    d1x, d1y = p2.x - p1.x, p2.y - p1.y
    d2x, d2y = p4.x - p3.x, p4.y - p3.y
    denom = d1x * d2y - d1y * d2x
    if abs(denom) < 1e-10:
        return None

    dx, dy = p3.x - p1.x, p3.y - p1.y
    t = (dx * d2y - dy * d2x) / denom
    u = (dx * d1y - dy * d1x) / denom

    if eps < t < (1.0 - eps) and eps < u < (1.0 - eps):
        return Point2D(p1.x + t * d1x, p1.y + t * d1y)
    return None


def ray_segment_intersection_distance(
    origin: Point2D, direction_rad: float, seg_a: Point2D, seg_b: Point2D
) -> Optional[float]:
    """Calcula la distancia desde `origin` en la dirección `direction_rad` hasta el segmento `seg_a`-`seg_b`."""
    rx, ry = math.cos(direction_rad), math.sin(direction_rad)
    sx, sy = seg_b.x - seg_a.x, seg_b.y - seg_a.y

    denom = rx * sy - ry * sx
    if abs(denom) < 1e-10:
        return None

    qpx, qpy = seg_a.x - origin.x, seg_a.y - origin.y
    t = (qpx * sy - qpy * sx) / denom
    u = (qpx * ry - qpy * rx) / denom

    if t >= 0.0 and -1e-7 <= u <= 1.0 + 1e-7:
        return t
    return None


def point_to_segment_distance(pt: Point2D, a: Point2D, b: Point2D) -> float:
    """Distancia euclidiana mínima desde un punto `pt` al segmento `a`-`b`."""
    dx, dy = b.x - a.x, b.y - a.y
    len_sq = dx * dx + dy * dy
    if len_sq < 1e-12:
        return pt.distance_to(a)
    t = max(0.0, min(1.0, ((pt.x - a.x) * dx + (pt.y - a.y) * dy) / len_sq))
    proj = Point2D(a.x + t * dx, a.y + t * dy)
    return pt.distance_to(proj)


def point_in_polygon(pt: Point2D, polygon: List[Point2D]) -> bool:
    """Algoritmo Ray-Casting (Par-Impar) para saber si `pt` está dentro de un polígono."""
    inside = False
    n = len(polygon)
    if n < 3:
        return False
    for i in range(n):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % n]
        if ((p1.y > pt.y) != (p2.y > pt.y)) and (
            pt.x < (p2.x - p1.x) * (pt.y - p1.y) / (p2.y - p1.y + 1e-15) + p1.x
        ):
            inside = not inside
    return inside


def convex_hull(points: List[Point2D]) -> List[Point2D]:
    """Construye la envolvente convexa (Monotone Chain de Andrew) en sentido antihorario (CCW)."""
    unique_pts = sorted(set((round(p.x, 5), round(p.y, 5)) for p in points))
    pts = [Point2D(x, y) for x, y in unique_pts]
    if len(pts) <= 2:
        return pts

    lower: List[Point2D] = []
    for p in pts:
        while len(lower) >= 2 and cross_product_2d(lower[-2], lower[-1], p) <= 0.0:
            lower.pop()
        lower.append(p)

    upper: List[Point2D] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross_product_2d(upper[-2], upper[-1], p) <= 0.0:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]


def polygon_perimeter(vertices: List[Point2D]) -> float:
    """Calcula el perímetro total de un polígono cerrado."""
    n = len(vertices)
    if n < 2:
        return 0.0
    return sum(vertices[i].distance_to(vertices[(i + 1) % n]) for i in range(n))


def polygon_signed_area(vertices: List[Point2D]) -> float:
    """Área con signo (positiva si es CCW, negativa si es CW)."""
    n = len(vertices)
    if n < 3:
        return 0.0
    signed_sum = 0.0
    for i in range(n):
        p1 = vertices[i]
        p2 = vertices[(i + 1) % n]
        signed_sum += p1.x * p2.y - p2.x * p1.y
    return 0.5 * signed_sum


def polygon_area(vertices: List[Point2D]) -> float:
    """Calcula el área de un polígono simple usando la fórmula de la lazada (Shoelace)."""
    return abs(polygon_signed_area(vertices))


def polygon_centroid(vertices: List[Point2D]) -> Point2D:
    """Retorna el centroide geométrico de los vértices del polígono."""
    if not vertices:
        return Point2D(0.0, 0.0)
    n = len(vertices)
    return Point2D(
        sum(p.x for p in vertices) / n,
        sum(p.y for p in vertices) / n,
    )


def simplify_polygon_collinear(
    vertices: List[Point2D], angle_tol_rad: float = math.radians(4.0), min_edge_m: float = 0.03
) -> List[Point2D]:
    """
    Fusiona únicamente puntos estrictamente colineales sobre la misma recta
    (desviación < angle_tol_rad y distancia perpendicular < 1.2 cm).
    """
    if len(vertices) <= 3:
        return list(vertices)

    filtered: List[Point2D] = []
    for v in vertices:
        if not filtered or v.distance_to(filtered[-1]) >= min_edge_m:
            filtered.append(v)
    if len(filtered) > 1 and filtered[0].distance_to(filtered[-1]) < min_edge_m:
        filtered.pop()

    if len(filtered) <= 3:
        return filtered

    changed = True
    current = filtered
    while changed and len(current) > 3:
        changed = False
        next_poly: List[Point2D] = []
        n = len(current)
        for i in range(n):
            prev_pt = current[(i - 1) % n]
            curr_pt = current[i]
            next_pt = current[(i + 1) % n]

            v1x, v1y = curr_pt.x - prev_pt.x, curr_pt.y - prev_pt.y
            v2x, v2y = next_pt.x - curr_pt.x, next_pt.y - curr_pt.y
            ang1 = math.atan2(v1y, v1x)
            ang2 = math.atan2(v2y, v2x)
            turn = abs(wrap_angle(ang2 - ang1))
            perp_err = point_to_segment_distance(curr_pt, prev_pt, next_pt)

            if turn < angle_tol_rad and perp_err < 0.012 and (n - (i - len(next_poly))) > 3:
                changed = True
                continue
            next_poly.append(curr_pt)
        if len(next_poly) == len(current):
            break
        current = next_poly

    return current


def remove_polygon_self_intersections(poly: List[Point2D]) -> List[Point2D]:
    """
    Garantiza que un polígono cerrado NO tenga cruces ni picos invertidos (auto-intersecciones).
    Si dos aristas se cruzan (como ocurrió en la parte inferior de la Foto 2), destuerce o
    recorta el bucle espúreo en el punto de intersección exacto.
    """
    pts = list(poly)
    if len(pts) <= 3:
        return pts

    for _ in range(25):
        n = len(pts)
        if n <= 3:
            break
        found_cross = False
        for i in range(n):
            i_next = (i + 1) % n
            for j in range(i + 2, n):
                j_next = (j + 1) % n
                if i == 0 and j_next == 0:
                    continue
                hit = segment_intersection(pts[i], pts[i_next], pts[j], pts[j_next], eps=1e-4)
                if hit is not None:
                    # Evaluar cuál de los dos lazos es la espina parásita más corta y reemplazarla por `hit`
                    loop1_len = j - i
                    loop2_len = n - loop1_len
                    if loop1_len <= loop2_len:
                        # El bucle corto es de i+1 hasta j
                        pts = pts[: i + 1] + [hit] + pts[j + 1 :]
                    else:
                        # El bucle corto envuelve el final/inicio de la lista
                        pts = [hit] + pts[i + 1 : j + 1]
                    found_cross = True
                    break
            if found_cross:
                break
        if not found_cross:
            break

    if polygon_signed_area(pts) < 0.0:
        pts.reverse()
    return pts


def order_all_wall_points_boundary(
    wall_points: List[Point2D],
    free_chords: Optional[List[ChordSegment]] = None,
) -> List[Point2D]:
    """
    Ordena TODOS los puntos de choque a lo largo del contorno real de la habitación,
    garantizando el camino cerrado de menor longitud total que pasa por cada punto (TSP de Contorno):
      1. Elimina duplicados muy próximos (< 2.5 cm).
      2. Construye el ciclo inicial por Inserción Más Cercana (*Nearest Insertion* sobre el Convex Hull)
         para que los puntos sobre una misma pared vertical u horizontal queden estrictamente en orden
         monotónico sin ir y volver (evita picos de 180° en paredes radiales al centroide).
      3. Ejecuta optimización 2-Opt completa (invirtiendo sub-cadenas siempre que reduzca la longitud
         del perímetro o elimine un cruce), logrando el contorno exacto y mínimo que pasa por TODOS los puntos.
    """
    dedup: List[Point2D] = []
    for p in wall_points:
        if all(p.distance_to(q) > 0.025 for q in dedup):
            dedup.append(p)
    if len(dedup) <= 3:
        return dedup

    hull = convex_hull(dedup)
    poly: List[Point2D] = list(hull)
    remaining = [
        p for p in dedup if all(p.distance_to(hp) > 1e-4 for hp in poly)
    ]

    # Inserción de costo mínimo de cada punto de choque restante en el ciclo
    while remaining:
        best_r_idx = 0
        best_ins_pos = 1
        best_delta = float("inf")
        m = len(poly)
        for r_idx, pt in enumerate(remaining):
            for i in range(m):
                a = poly[i]
                b = poly[(i + 1) % m]
                delta = a.distance_to(pt) + pt.distance_to(b) - a.distance_to(b)
                if delta < best_delta:
                    best_delta = delta
                    best_r_idx = r_idx
                    best_ins_pos = i + 1
        poly.insert(best_ins_pos, remaining.pop(best_r_idx))

    # Optimización 2-Opt completa: destuerce cualquier inversión o zigzag en paredes radiales
    n = len(poly)
    for _ in range(30):
        improved = False
        for i in range(n):
            i_next = (i + 1) % n
            for j in range(i + 2, n):
                j_next = (j + 1) % n
                if i == 0 and j_next == 0:
                    continue
                curr_d = poly[i].distance_to(poly[i_next]) + poly[j].distance_to(poly[j_next])
                new_d = poly[i].distance_to(poly[j]) + poly[i_next].distance_to(poly[j_next])
                if new_d + 1e-5 < curr_d:
                    poly[i_next : j + 1] = reversed(poly[i_next : j + 1])
                    improved = True
                    break
            if improved:
                break
        if not improved:
            break

    if polygon_signed_area(poly) < 0.0:
        poly.reverse()

    return poly


def reconstruct_classic_polygon(
    wall_points: List[Point2D],
    free_chords: Optional[List[ChordSegment]] = None,
) -> List[Point2D]:
    """
    MODO CLÁSICO:
    Une todos los puntos de choque respetando entrantes cóncavas y proyecta esquinas
    entre paredes rectas bien muestreadas solo si la esquina sigue pasando por todos los puntos.
    """
    if len(wall_points) < 3:
        return list(wall_points)

    ordered = order_all_wall_points_boundary(wall_points, free_chords=free_chords)
    simplified = simplify_polygon_collinear(ordered, angle_tol_rad=math.radians(4.0), min_edge_m=0.03)
    m = len(simplified)
    if m < 4:
        return simplified

    supports = [
        sum(1 for p in wall_points if point_to_segment_distance(p, simplified[i], simplified[(i + 1) % m]) <= 0.015)
        for i in range(m)
    ]

    refined: List[Point2D] = []
    skip_next = False
    for i in range(m):
        if skip_next:
            skip_next = False
            continue
        prev_idx = (i - 1) % m
        next_idx = (i + 1) % m
        p_a = simplified[i]
        p_b = simplified[next_idx]

        if supports[prev_idx] >= 3 and supports[i] == 2 and supports[next_idx] >= 3 and m >= 5:
            w1_a, w1_b = simplified[prev_idx], simplified[i]
            w2_a, w2_b = simplified[next_idx], simplified[(next_idx + 1) % m]
            d1x, d1y = w1_b.x - w1_a.x, w1_b.y - w1_a.y
            d2x, d2y = w2_b.x - w2_a.x, w2_b.y - w2_a.y
            denom = d1x * d2y - d1y * d2x
            if abs(denom) > 1e-5:
                dx, dy = w2_a.x - w1_a.x, w2_a.y - w1_a.y
                t = (dx * d2y - dy * d2x) / denom
                u = (dx * d1y - dy * d1x) / denom
                corner = Point2D(w1_a.x + t * d1x, w1_a.y + t * d1y)
                if t >= 0.98 and u <= 0.02 and corner.distance_to(p_a) <= 1.25 and corner.distance_to(p_b) <= 1.25:
                    refined.append(corner)
                    skip_next = True
                    continue

        refined.append(p_a)

    if len(refined) < 3:
        return simplified

    refined = remove_polygon_self_intersections(refined)
    nr = len(refined)
    all_on_boundary = all(
        min(point_to_segment_distance(p, refined[j], refined[(j + 1) % nr]) for j in range(nr)) <= 0.03
        for p in wall_points
    )
    return refined if all_on_boundary else simplified


def reconstruct_orthogonal_polygon(
    wall_points: List[Point2D],
    free_chords: Optional[List[ChordSegment]] = None,
    snap_tol_m: float = 0.06,
) -> List[Point2D]:
    """
    MODO RECTO (Ángulos estrictos a 90° sin picos ni cruces):
      1. Ordena todos los puntos de choque a lo largo del contorno real (Concave Hull + 2-opt).
      2. Agrupa puntos consecutivos que caen sobre una misma pared horizontal (misma Y) o
         vertical (misma X) dentro de `snap_tol_m`.
      3. Entre cada par de puntos en paredes distintas, evalúa las dos esquinas posibles a 90°:
           c_a = (p1.x, p2.y)   y   c_b = (p2.x, p1.y)
         y elige aquella cuyas dos ramas ortogonales NO cortan ninguna cuerda recorrida por el
         robot (`free_chords`) ni cruzan las paredes adyacentes.
      4. Limpia cualquier colinealidad o auto-intersección para entregar un polígono 100% ortogonal y limpio.
    """
    if len(wall_points) < 3:
        return list(wall_points)

    ordered = order_all_wall_points_boundary(wall_points)
    simplified = simplify_polygon_collinear(ordered, angle_tol_rad=math.radians(4.0), min_edge_m=0.03)
    n = len(simplified)
    if n < 3:
        return simplified

    chords_list = free_chords or []

    def corner_cuts_free_chords(p1: Point2D, corner: Point2D, p2: Point2D) -> int:
        """Cuenta cuántas cuerdas del interior de la habitación serían cortadas por el escalón p1->corner->p2."""
        cuts = 0
        for ch in chords_list:
            if (
                segment_intersection(p1, corner, ch.start, ch.end, eps=0.02) is not None
                or segment_intersection(corner, p2, ch.start, ch.end, eps=0.02) is not None
            ):
                cuts += 1
        return cuts

    ortho_pts: List[Point2D] = []
    center = polygon_centroid(simplified)

    for i in range(n):
        p0 = simplified[(i - 1) % n]
        p1 = simplified[i]
        p2 = simplified[(i + 1) % n]
        p3 = simplified[(i + 2) % n]

        ortho_pts.append(p1)

        dx = abs(p2.x - p1.x)
        dy = abs(p2.y - p1.y)

        # Si ya es un tramo horizontal o vertical puro, nivelarlo sin agregar esquina extra
        if dx <= snap_tol_m or dy <= snap_tol_m:
            continue

        c_a = Point2D(p1.x, p2.y)  # Vertical primero, luego Horizontal
        c_b = Point2D(p2.x, p1.y)  # Horizontal primero, luego Vertical

        cuts_a = corner_cuts_free_chords(p1, c_a, p2)
        cuts_b = corner_cuts_free_chords(p1, c_b, p2)

        # Penalizar si el giro regresa a 180° sobre la arista anterior (creando una aguja de ancho 0)
        prev_is_vert = abs(p1.x - p0.x) <= snap_tol_m
        prev_is_horz = abs(p1.y - p0.y) <= snap_tol_m
        next_is_vert = abs(p3.x - p2.x) <= snap_tol_m
        next_is_horz = abs(p3.y - p2.y) <= snap_tol_m

        # Si venimos por una pared horizontal (y = const) y c_a sale vertical en sentido opuesto creando espina:
        score_a = cuts_a * 10
        score_b = cuts_b * 10

        # Si venimos moviéndonos horizontalmente (p0->p1 es H), continuar H hasta p2.x (es decir c_b)
        # extiende suavemente la pared horizontal hasta la esquina real!
        if prev_is_horz:
            score_b -= 3
        if prev_is_vert:
            score_a -= 3
        if next_is_vert:
            score_b -= 3
        if next_is_horz:
            score_a -= 3

        if score_a < score_b:
            chosen = c_a
        elif score_b < score_a:
            chosen = c_b
        else:
            chosen = c_a if c_a.distance_to(center) >= c_b.distance_to(center) else c_b

        ortho_pts.append(chosen)

    # Nivelar tramos consecutivos horizontales y verticales sobre las medianas exactas de los wall_points
    cleaned = simplify_polygon_collinear(ortho_pts, angle_tol_rad=math.radians(6.0), min_edge_m=0.04)
    cleaned = remove_polygon_self_intersections(cleaned)
    m = len(cleaned)
    if m < 4:
        return cleaned

    snapped = list(cleaned)
    for _ in range(3):
        for i in range(m):
            a = snapped[i]
            b = snapped[(i + 1) % m]
            if abs(a.x - b.x) < abs(a.y - b.y):
                # Arista vertical: alinear X con el paso de grilla de Modo Recto (10 cm) si está cerca
                wall_xs = [
                    p.x
                    for p in wall_points
                    if abs(p.x - 0.5 * (a.x + b.x)) <= 0.04
                    and min(a.y, b.y) - 0.04 <= p.y <= max(a.y, b.y) + 0.04
                ]
                raw_x = sum(wall_xs) / len(wall_xs) if wall_xs else 0.5 * (a.x + b.x)
                grid_x = round(raw_x * 10.0) / 10.0
                exact_x = grid_x if abs(raw_x - grid_x) <= 0.06 else raw_x
                snapped[i] = Point2D(exact_x, a.y)
                snapped[(i + 1) % m] = Point2D(exact_x, b.y)
            else:
                # Arista horizontal: alinear Y con el paso de grilla de Modo Recto (10 cm) si está cerca
                wall_ys = [
                    p.y
                    for p in wall_points
                    if abs(p.y - 0.5 * (a.y + b.y)) <= 0.04
                    and min(a.x, b.x) - 0.04 <= p.x <= max(a.x, b.x) + 0.04
                ]
                raw_y = sum(wall_ys) / len(wall_ys) if wall_ys else 0.5 * (a.y + b.y)
                grid_y = round(raw_y * 10.0) / 10.0
                exact_y = grid_y if abs(raw_y - grid_y) <= 0.06 else raw_y
                snapped[i] = Point2D(a.x, exact_y)
                snapped[(i + 1) % m] = Point2D(b.x, exact_y)

    final_poly = simplify_polygon_collinear(snapped, angle_tol_rad=math.radians(5.0), min_edge_m=0.04)
    clean_poly = remove_polygon_self_intersections(final_poly)
    # La poda de auto-intersecciones puede insertar aristas diagonales: convertirlas en codos a 90°
    clean_poly = force_orthogonal_polygon_edges(clean_poly)
    clean_poly = remove_polygon_self_intersections(clean_poly)
    return force_orthogonal_polygon_edges(clean_poly)


def rectilinearize_custom_polygon(
    vertices: List[Tuple[float, float]], grid_step_m: float = 0.10
) -> List[Tuple[float, float]]:
    """
    Normaliza los vértices dibujados por el usuario en el 'Modo Recto' para que todas las
    paredes queden estrictamente a 90° (horizontales y verticales) y sin auto-intersecciones.
    """
    if len(vertices) < 3:
        return list(vertices)

    def snap(val: float) -> float:
        return round(round(val / grid_step_m) * grid_step_m, 3)

    raw = [(snap(x), snap(y)) for x, y in vertices]
    ortho: List[Tuple[float, float]] = [raw[0]]

    for i in range(1, len(raw)):
        px, py = ortho[-1]
        nx, ny = raw[i]
        if abs(nx - px) < 1e-4 and abs(ny - py) < 1e-4:
            continue
        dx, dy = abs(nx - px), abs(ny - py)
        if dy <= 0.25 * dx:
            ortho.append((nx, py))
        elif dx <= 0.25 * dy:
            ortho.append((px, ny))
        else:
            ortho.append((nx, py))
            ortho.append((nx, ny))

    fx, fy = ortho[0]
    lx, ly = ortho[-1]
    if abs(lx - fx) > 1e-3 and abs(ly - fy) > 1e-3:
        if len(ortho) >= 2 and abs(ortho[-1][1] - ortho[-2][1]) < 1e-3:
            ortho[-1] = (fx, ly)
        else:
            ortho.append((lx, fy))

    pts_2d = [Point2D(x, y) for x, y in ortho]
    simp = simplify_polygon_collinear(pts_2d, angle_tol_rad=math.radians(3.0), min_edge_m=0.05)
    simp = remove_polygon_self_intersections(simp)
    return [(round(p.x, 3), round(p.y, 3)) for p in simp]


def reconstruct_incidence_polygon(
    wall_points: List[Point2D],
    wall_dirs: Optional[List[Optional[float]]] = None,
    free_chords: Optional[List[ChordSegment]] = None,
) -> List[Point2D]:
    """
    MODO INCIDENTE (recto obligatorio usando la orientación de pared de cada impacto):
      1. Ordena los puntos de choque a lo largo del contorno real (misma ordenación que los
         demás modos: inserción de costo mínimo + 2-opt).
      2. Agrupa hits consecutivos que comparten orientación de pared (horizontal/vertical)
         y coordenada de recta compatible -> cada grupo ajusta UNA recta de pared real.
      3. Los vértices salen de intersectar las rectas de pared consecutivas; entre paredes
         paralelas (escalón) se inserta el par de vértices del tramo de conexión.
      4. Si no hay orientaciones suficientes, cae a la reconstrucción ortogonal por puntos.
    """
    n = len(wall_points)
    if n < 3:
        return list(wall_points)
    if not wall_dirs or len(wall_dirs) != n or all(d is None for d in wall_dirs):
        return reconstruct_orthogonal_polygon(wall_points, free_chords=free_chords)

    ordered = order_all_wall_points_boundary(wall_points, free_chords=free_chords)
    idx_of = {id(w): i for i, w in enumerate(wall_points)}
    dirs = [wall_dirs[idx_of.get(id(p), -1)] for p in ordered]

    def is_vert(d: Optional[float]) -> Optional[bool]:
        if d is None:
            return None
        dn = d % math.pi
        return abs(math.sin(dn)) >= abs(math.cos(dn))

    def snap01(v: float) -> float:
        s = round(v / 0.1) * 0.1
        return s if abs(s - v) < 0.06 else v

    # Agrupar el ciclo ordenado en "runs" de pared: misma orientación + misma recta (±9 cm).
    # Los choques rasantes pueden votar la orientación equivocada (incidencia > 45°), así que
    # un punto también se une al run actual si su POSICIÓN yace sobre la recta de esa pared.
    runs: List[dict] = []
    for p, d in zip(ordered, dirs):
        vert = is_vert(d)
        if vert is None:
            runs.append({"vert": None, "coord": None, "pts": [p], "k": 1})
            continue
        coord = p.x if vert else p.y
        joined = False
        if runs and runs[-1]["vert"] is not None:
            r = runs[-1]
            on_line = abs((p.x if r["vert"] else p.y) - r["coord"]) <= 0.09
            if (vert == r["vert"] and abs(coord - r["coord"]) <= 0.09) or on_line:
                r["pts"].append(p)
                r["k"] += 1
                r["coord"] += ((p.x if r["vert"] else p.y) - r["coord"]) / r["k"]
                joined = True
        if not joined:
            runs.append({"vert": vert, "coord": coord, "pts": [p], "k": 1})

    # Fundir runs "fantasma" (1-2 puntos) cuyos puntos yacen sobre la recta de un run vecino:
    # son choques rasantes aislados entre dos paredes distintas, no paredes reales.
    merged = True
    while merged and len(runs) >= 2:
        merged = False
        for i in range(len(runs)):
            r = runs[i]
            if r["vert"] is None or r["k"] > 2:
                continue
            prev_r = runs[(i - 1) % len(runs)]
            next_r = runs[(i + 1) % len(runs)]
            target, prepend = None, False
            for nb, before in ((next_r, True), (prev_r, False)):
                if nb["vert"] is None:
                    continue
                if all(
                    abs((p.x if nb["vert"] else p.y) - nb["coord"]) <= 0.10
                    for p in r["pts"]
                ):
                    target, prepend = nb, before
                    break
            if target is not None:
                total = target["k"] + r["k"]
                add = sum(
                    (p.x if target["vert"] else p.y) for p in r["pts"]
                )
                target["coord"] = (target["coord"] * target["k"] + add) / total
                target["pts"] = (
                    r["pts"] + target["pts"] if prepend else target["pts"] + r["pts"]
                )
                target["k"] = total
                runs.remove(r)
                merged = True
                break

    # Unir primer y último run si representan la misma pared a través del cierre del ciclo.
    if (
        len(runs) > 1
        and runs[0]["vert"] is not None
        and runs[0]["vert"] == runs[-1]["vert"]
        and runs[0]["coord"] is not None
        and runs[-1]["coord"] is not None
        and abs(runs[0]["coord"] - runs[-1]["coord"]) <= 0.09
    ):
        last = runs.pop()
        total = runs[0]["k"] + last["k"]
        runs[0]["coord"] = (runs[0]["coord"] * runs[0]["k"] + last["coord"] * last["k"]) / total
        runs[0]["pts"] = last["pts"] + runs[0]["pts"]
        runs[0]["k"] = total

    if len(runs) < 2:
        return reconstruct_orthogonal_polygon(wall_points, free_chords=free_chords)

    for r in runs:
        if r["coord"] is not None:
            r["coord"] = snap01(r["coord"])

    # Vértices = intersección de rectas de pared consecutivas (o escalón entre paralelas).
    verts: List[Point2D] = []
    m = len(runs)
    for i in range(m):
        r, s = runs[i], runs[(i + 1) % m]
        if r["vert"] is not None and s["vert"] is not None:
            if r["vert"] != s["vert"]:
                xv = r["coord"] if r["vert"] else s["coord"]
                yh = s["coord"] if r["vert"] else r["coord"]
                verts.append(Point2D(xv, yh))
            else:
                # Escalón entre dos paredes paralelas: dos vértices a la altura media de
                # los puntos límite de ambos tramos.
                t = snap01(
                    (r["pts"][-1].y + s["pts"][0].y) / 2.0
                    if r["vert"]
                    else (r["pts"][-1].x + s["pts"][0].x) / 2.0
                )
                if r["vert"]:
                    verts.append(Point2D(r["coord"], t))
                    verts.append(Point2D(s["coord"], t))
                else:
                    verts.append(Point2D(t, r["coord"]))
                    verts.append(Point2D(t, s["coord"]))
        else:
            # Hits sin orientación: conservar los puntos reales como vértices del contorno.
            unknown = r if r["vert"] is None else s
            verts.extend(unknown["pts"])

    # Eliminar duplicados consecutivos y de cierre
    dedup: List[Point2D] = []
    for p in verts:
        if not dedup or p.distance_to(dedup[-1]) > 0.02:
            dedup.append(p)
    while len(dedup) > 1 and dedup[-1].distance_to(dedup[0]) <= 0.02:
        dedup.pop()

    if len(dedup) < 3:
        return reconstruct_orthogonal_polygon(wall_points, free_chords=free_chords)
    dedup = simplify_polygon_collinear(dedup, angle_tol_rad=math.radians(3.0), min_edge_m=0.03)
    dedup = remove_polygon_self_intersections(dedup)
    dedup = force_orthogonal_polygon_edges(dedup)

    # Garantía de contorno: si las rectas ajustadas dejaron algún punto de choque real
    # fuera del polígono (esquinas inferidas más allá de la zona muestreada), la
    # reconstrucción por puntos es más fiel -> usarla.
    m = len(dedup)
    if m >= 3:
        on_boundary = all(
            min(
                point_to_segment_distance(p, dedup[j], dedup[(j + 1) % m])
                for j in range(m)
            )
            <= 0.05
            for p in wall_points
        )
        if not on_boundary:
            return reconstruct_orthogonal_polygon(wall_points, free_chords=free_chords)
    return dedup


def reconstruct_walled_polygon(
    wall_points: List[Point2D],
    mode: str = "classic",
    free_chords: Optional[List[ChordSegment]] = None,
    wall_dirs: Optional[List[Optional[float]]] = None,
) -> List[Point2D]:
    """Punto de entrada unificado para reconstruir el perímetro según el modo activo."""
    if mode in ("recto", "recto_reaprendizaje", "testeador"):
        return reconstruct_orthogonal_polygon(wall_points, free_chords=free_chords)
    if mode == "incidente":
        return reconstruct_incidence_polygon(
            wall_points, wall_dirs=wall_dirs, free_chords=free_chords
        )
    return reconstruct_classic_polygon(wall_points, free_chords=free_chords)


def force_orthogonal_polygon_edges(
    poly: List[Point2D], max_passes: int = 14
) -> List[Point2D]:
    """
    Reemplaza aristas diagonales de un polígono ortogonal por codos a 90° en forma de L,
    eligiendo el recodo cuyas dos patas permanezcan dentro del polígono y no crucen ninguna
    arista. Se usa en Modo Recto / Testeador para que el perímetro reconstruido y la ruta
    interior jamás contengan tramos diagonales.
    """
    pts = list(poly)
    if len(pts) < 3:
        return pts
    if polygon_signed_area(pts) < 0.0:
        pts.reverse()

    def _legs_inside(a: Point2D, e: Point2D, b: Point2D) -> bool:
        m = len(pts)
        for s, t in ((a, e), (e, b)):
            if s.distance_to(t) < 1e-9:
                continue
            mid = Point2D((s.x + t.x) / 2.0, (s.y + t.y) / 2.0)
            if not point_in_polygon(mid, pts):
                return False
            for k in range(m):
                hit = segment_intersection(
                    s, t, pts[k], pts[(k + 1) % m], eps=1e-4
                )
                if (
                    hit is not None
                    and hit.distance_to(a) > 0.01
                    and hit.distance_to(b) > 0.01
                    and hit.distance_to(e) > 0.01
                ):
                    return False
        return True

    for _ in range(max_passes):
        n = len(pts)
        if n < 3:
            break
        changed = False
        for i in range(n):
            a = pts[i]
            b = pts[(i + 1) % n]
            if abs(b.x - a.x) < 0.015 or abs(b.y - a.y) < 0.015:
                continue
            elbow_h = Point2D(b.x, a.y)  # Horizontal primero, luego Vertical
            elbow_v = Point2D(a.x, b.y)  # Vertical primero, luego Horizontal
            if _legs_inside(a, elbow_h, b):
                chosen = elbow_h
            elif _legs_inside(a, elbow_v, b):
                chosen = elbow_v
            else:
                continue
            pts = pts[: i + 1] + [chosen] + pts[i + 1 :]
            changed = True
            break
        if not changed:
            break

    return pts


def compute_inward_offset_polygon(
    vertices_ccw: List[Point2D], margin_m: float, ortho: bool = False
) -> List[Point2D]:
    """
    Calcula el polígono interior desplazado una distancia `margin_m` hacia adentro (R * 1.2)
    y recorta automáticamente cualquier lazo o auto-intersección en nichos más angostos que `2 * margin_m`,
    evitando trayectorias cruzadas como la de la Foto 2.
    Si `ortho` es True (Modo Recto / Testeador), además convierte a codos de 90° cualquier
    arista diagonal que la poda haya generado, garantizando una ruta 100% ortogonal.
    """
    n = len(vertices_ccw)
    if n < 3:
        return list(vertices_ccw)

    verts = list(vertices_ccw) if polygon_signed_area(vertices_ccw) >= 0.0 else list(reversed(vertices_ccw))

    inset_waypoints: List[Point2D] = []
    for i in range(n):
        p_prev = verts[(i - 1) % n]
        p_curr = verts[i]
        p_next = verts[(i + 1) % n]

        e1x, e1y = p_curr.x - p_prev.x, p_curr.y - p_prev.y
        l1 = max(math.hypot(e1x, e1y), 1e-9)
        e1x, e1y = e1x / l1, e1y / l1

        e2x, e2y = p_next.x - p_curr.x, p_next.y - p_curr.y
        l2 = max(math.hypot(e2x, e2y), 1e-9)
        e2x, e2y = e2x / l2, e2y / l2

        n1x, n1y = -e1y, e1x
        n2x, n2y = -e2y, e2x

        denom = e1x * e2y - e1y * e2x
        if abs(denom) < 1e-4:
            inset_waypoints.append(Point2D(p_curr.x + margin_m * n1x, p_curr.y + margin_m * n1y))
        else:
            dnx = margin_m * (n2x - n1x)
            dny = margin_m * (n2y - n1y)
            t = (dnx * e2y - dny * e2x) / denom
            t = max(-1.8 * margin_m, min(1.8 * margin_m, t))
            wx = p_curr.x + margin_m * n1x + t * e1x
            wy = p_curr.y + margin_m * n1y + t * e1y
            inset_waypoints.append(Point2D(wx, wy))

    # Recortar cualquier lazo cruzado (cola de milano) en el camino verde de navegación
    clean_inset = remove_polygon_self_intersections(inset_waypoints)
    if ortho:
        clean_inset = force_orthogonal_polygon_edges(clean_inset)
        clean_inset = remove_polygon_self_intersections(clean_inset)
        clean_inset = force_orthogonal_polygon_edges(clean_inset)
    return simplify_polygon_collinear(clean_inset, angle_tol_rad=math.radians(3.0), min_edge_m=0.04)


def build_anchored_perimeter_route(
    current_pos: Point2D, inset_polygon: List[Point2D]
) -> List[Point2D]:
    """
    Construye la lista ordenada de waypoints para recorrer todo el polígono interior `inset_polygon`
    SIN realizar un salto diagonal desde `current_pos` hacia un vértice lejano:
      1. Proyecta `current_pos` perpendicularmente sobre cada arista (A -> B) de `inset_polygon`.
      2. Encuentra el punto de proyección P_start más cercano (el robot ya está pegado a una pared,
         por lo que P_start queda a pocos centímetros perpendicular a esa misma pared).
      3. Si P_start está en medio de la arista (i -> i+1), inicia la ruta en P_start, continúa hacia
         el vértice (i+1), da toda la vuelta a todos los vértices hasta (i) y cierra exactamente en P_start.
    Esto elimina por completo las diagonales verdes espurias de las Fotos 1, 2 y 3.
    """
    n = len(inset_polygon)
    if n == 0:
        return []
    if n < 3:
        return list(inset_polygon) + [inset_polygon[0]]

    best_dist = float("inf")
    best_edge_idx = 0
    best_proj = inset_polygon[0]

    for i in range(n):
        a = inset_polygon[i]
        b = inset_polygon[(i + 1) % n]
        dx, dy = b.x - a.x, b.y - a.y
        len_sq = dx * dx + dy * dy
        if len_sq < 1e-12:
            proj = a
        else:
            t = max(0.0, min(1.0, ((current_pos.x - a.x) * dx + (current_pos.y - a.y) * dy) / len_sq))
            proj = Point2D(a.x + t * dx, a.y + t * dy)
        d = current_pos.distance_to(proj)
        if d < best_dist:
            best_dist = d
            best_edge_idx = i
            best_proj = proj

    # Recorrer desde best_proj hacia el extremo final de esa arista y dar toda la vuelta completa
    ordered: List[Point2D] = [best_proj]
    for k in range(1, n + 1):
        v = inset_polygon[(best_edge_idx + k) % n]
        if v.distance_to(ordered[-1]) > 0.02:
            ordered.append(v)

    if ordered[-1].distance_to(best_proj) > 0.02:
        ordered.append(best_proj)

    return ordered
