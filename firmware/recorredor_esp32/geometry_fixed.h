// Geometría 2D en memoria estática, Método de las Hormigas, Cauchy-Crofton, Convex Hull e Inward Offset
#ifndef RECORREDOR_GEOMETRY_FIXED_H
#define RECORREDOR_GEOMETRY_FIXED_H

#include <Arduino.h>
#include <math.h>
#include "config.h"

struct Point2D {
    float x;
    float y;
};

struct Pose2D {
    float x;
    float y;
    float theta;
};

struct ChordSegment {
    uint16_t idx;
    Point2D start;
    Point2D end;
    float length;
    float duration_s;
};

struct MapEstimates {
    uint16_t num_chords;
    uint16_t num_wall_points;
    uint16_t num_crossings;
    float total_path_length_m;
    float mean_chord_length_m;
    float poly_area_m2;
    float poly_perimeter_m;
    float ants_area_m2;
    float crofton_perimeter_m;
    uint8_t sectors_covered;
    bool converged;
};

inline float wrapAngle(float angle_rad) {
    while (angle_rad <= -PI) angle_rad += 2.0f * PI;
    while (angle_rad > PI)   angle_rad -= 2.0f * PI;
    return angle_rad;
}

inline float distance2D(const Point2D& a, const Point2D& b) {
    return hypotf(a.x - b.x, a.y - b.y);
}

inline float crossProduct2D(const Point2D& o, const Point2D& a, const Point2D& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

// Determina si dos segmentos p1-p2 y p3-p4 se cruzan propiamente en el interior
inline bool segmentsIntersect(const Point2D& p1, const Point2D& p2, const Point2D& p3, const Point2D& p4) {
    float d1x = p2.x - p1.x, d1y = p2.y - p1.y;
    float d2x = p4.x - p3.x, d2y = p4.y - p3.y;
    float denom = d1x * d2y - d1y * d2x;
    if (fabsf(denom) < 1e-7f) return false;

    float dx = p3.x - p1.x, dy = p3.y - p1.y;
    float t = (dx * d2y - dy * d2x) / denom;
    float u = (dx * d1y - dy * d1x) / denom;
    return (t > 1e-4f && t < 0.9999f && u > 1e-4f && u < 0.9999f);
}

// Fórmula de la lazada de Gauss (Shoelace) para área de polígono
inline float computePolygonArea(const Point2D* verts, uint16_t n) {
    if (n < 3) return 0.0f;
    float sum = 0.0f;
    for (uint16_t i = 0; i < n; ++i) {
        const Point2D& p1 = verts[i];
        const Point2D& p2 = verts[(i + 1) % n];
        sum += (p1.x * p2.y - p2.x * p1.y);
    }
    return 0.5f * fabsf(sum);
}

// Perímetro de un polígono cerrado
inline float computePolygonPerimeter(const Point2D* verts, uint16_t n) {
    if (n < 2) return 0.0f;
    float perim = 0.0f;
    for (uint16_t i = 0; i < n; ++i) {
        perim += distance2D(verts[i], verts[(i + 1) % n]);
    }
    return perim;
}

// Construcción de Convex Hull (Monotone Chain de Andrew) + simplificación de colineales en memoria estática
inline uint16_t buildSimplifiedHull(const Point2D* input_pts, uint16_t n_pts, Point2D* out_hull) {
    if (n_pts < 3) {
        for (uint16_t i = 0; i < n_pts; ++i) out_hull[i] = input_pts[i];
        return n_pts;
    }

    Point2D sorted[MAX_CHORDS];
    uint16_t count = min(n_pts, MAX_CHORDS);
    for (uint16_t i = 0; i < count; ++i) sorted[i] = input_pts[i];

    // Ordenamiento por inserción lexicográfico (X, luego Y) - muy rápido para N <= 128
    for (uint16_t i = 1; i < count; ++i) {
        Point2D key = sorted[i];
        int16_t j = i - 1;
        while (j >= 0 && (sorted[j].x > key.x || (fabsf(sorted[j].x - key.x) < 1e-5f && sorted[j].y > key.y))) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    Point2D hull[2 * MAX_HULL_VERTICES];
    uint16_t k = 0;

    // Cadena inferior
    for (uint16_t i = 0; i < count; ++i) {
        while (k >= 2 && crossProduct2D(hull[k - 2], hull[k - 1], sorted[i]) <= 0.0f) k--;
        if (k < 2 * MAX_HULL_VERTICES) hull[k++] = sorted[i];
    }
    // Cadena superior
    for (int16_t i = count - 2, t = k + 1; i >= 0; --i) {
        while (k >= t && crossProduct2D(hull[k - 2], hull[k - 1], sorted[i]) <= 0.0f) k--;
        if (k < 2 * MAX_HULL_VERTICES) hull[k++] = sorted[i];
    }
    if (k > 1) k--; // Eliminar punto duplicado de cierre

    // Simplificar vértices casi colineales (ángulos < 8 grados) sobre una misma pared
    uint16_t out_n = 0;
    const float angle_tol = 8.0f * DEG_TO_RAD;
    for (uint16_t i = 0; i < k && out_n < MAX_HULL_VERTICES; ++i) {
        const Point2D& prev = hull[(i + k - 1) % k];
        const Point2D& curr = hull[i];
        const Point2D& next = hull[(i + 1) % k];
        float a1 = atan2f(curr.y - prev.y, curr.x - prev.x);
        float a2 = atan2f(next.y - curr.y, next.x - curr.x);
        if (fabsf(wrapAngle(a2 - a1)) < angle_tol && k > 3) {
            continue;
        }
        out_hull[out_n++] = curr;
    }
    return out_n;
}

// Generación de Waypoints Interiores Seguros (Inward Polygon Offset por bisectrices)
inline uint16_t buildInwardOffsetPolygon(const Point2D* hull_ccw, uint16_t n, float margin_m, Point2D* out_waypoints) {
    if (n < 3) return 0;
    for (uint16_t i = 0; i < n; ++i) {
        const Point2D& p_prev = hull_ccw[(i + n - 1) % n];
        const Point2D& p_curr = hull_ccw[i];
        const Point2D& p_next = hull_ccw[(i + 1) % n];

        float e1x = p_curr.x - p_prev.x, e1y = p_curr.y - p_prev.y;
        float l1 = max(hypotf(e1x, e1y), 1e-6f);
        e1x /= l1; e1y /= l1;

        float e2x = p_next.x - p_curr.x, e2y = p_next.y - p_curr.y;
        float l2 = max(hypotf(e2x, e2y), 1e-6f);
        e2x /= l2; e2y /= l2;

        // Normales interiores (+90 grados en sentido antihorario)
        float n1x = -e1y, n1y = e1x;
        float n2x = -e2y, n2y = e2x;

        float bx = n1x + n2x, by = n1y + n2y;
        float blen = hypotf(bx, by);
        float cos_half = 1.0f;
        if (blen < 1e-5f) {
            bx = n1x; by = n1y;
        } else {
            bx /= blen; by /= blen;
            cos_half = max(n1x * bx + n1y * by, 0.35f);
        }
        float shift = margin_m / cos_half;
        out_waypoints[i] = {p_curr.x + bx * shift, p_curr.y + by * shift};
    }
    return n;
}

#endif // RECORREDOR_GEOMETRY_FIXED_H
