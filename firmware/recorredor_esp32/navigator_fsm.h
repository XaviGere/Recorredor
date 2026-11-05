// Máquina de Estados de Mapeo y Seguimiento Perimetral para ESP32
#ifndef RECORREDOR_NAVIGATOR_FSM_H
#define RECORREDOR_NAVIGATOR_FSM_H

#include <Arduino.h>
#include "config.h"
#include "geometry_fixed.h"

enum class MissionState : uint8_t {
    IDLE = 0,
    EXPLORE_FWD,
    BRAKE_RECORD,
    EVALUATE_MAP,
    TURN_HEADING,
    BUILD_SAFE_PATH,
    FOLLOW_PERIMETER,
    COMPLETED
};

class NavigatorFSM {
public:
    MissionState state = MissionState::IDLE;
    Pose2D pose = {0.0f, 0.0f, 0.0f};
    float total_time_s = 0.0f;
    float total_distance_m = 0.0f;

    // Origen del tramo rectilíneo (cuerda) actual
    Point2D chord_start_pt = {0.0f, 0.0f};
    float chord_start_time_s = 0.0f;
    float target_heading_rad = 0.0f;
    float last_wall_dist_m = 0.0f;

    // Arreglos estáticos de mapeo en SRAM
    Point2D wall_points[MAX_CHORDS];
    uint16_t num_wall_points = 0;

    ChordSegment chords[MAX_CHORDS];
    uint16_t num_chords = 0;
    uint16_t num_crossings = 0;
    float area_history[MAX_CHORDS];

    // Polígono reconstruido y ruta interior libre de colisiones
    Point2D hull_vertices[MAX_HULL_VERTICES];
    uint16_t num_hull_vertices = 0;

    Point2D safe_waypoints[MAX_HULL_VERTICES + 1];
    uint16_t num_safe_waypoints = 0;
    uint16_t current_wp_idx = 0;

    MapEstimates last_estimates = {};

    void startMission() {
        state = MissionState::EXPLORE_FWD;
        target_heading_rad = pose.theta;
        chord_start_pt = {pose.x, pose.y};
        chord_start_time_s = total_time_s;
    }

    // Actualiza la pose con integración de punto medio (RK2) usando d = v * dt (decisión confirmada)
    // Si hay MPU6050 conectado, combina el giro comandado con el giroscopio; si no, integra omega_cmd * dt.
    void updateOdometryVT(float v_cmd_mps, float omega_cmd_radps, float gyro_z_radps, bool mpu_ok, float dt_s) {
        float ds = (v_cmd_mps * LINEAR_SPEED_CALIB_GAIN) * dt_s;
        float dtheta_cmd = omega_cmd_radps * dt_s;
        float dtheta = mpu_ok
            ? (GYRO_COMPLEMENTARY_ALPHA * (gyro_z_radps * dt_s) + (1.0f - GYRO_COMPLEMENTARY_ALPHA) * dtheta_cmd)
            : dtheta_cmd;
        float mid_theta = pose.theta + 0.5f * dtheta;

        pose.x += ds * cosf(mid_theta);
        pose.y += ds * sinf(mid_theta);
        pose.theta = wrapAngle(pose.theta + dtheta);

        total_time_s += dt_s;
        total_distance_m += fabsf(ds);
    }

    // Variante opcional si en el futuro se habilitan encoders de rueda
    void updateOdometryEncoders(int32_t dl_ticks, int32_t dr_ticks, float gyro_z_radps, float dt_s) {
        float ds_l = dl_ticks * METERS_PER_TICK;
        float ds_r = dr_ticks * METERS_PER_TICK;
        float ds = 0.5f * (ds_l + ds_r);
        float dtheta_odom = (ds_r - ds_l) / WHEELBASE_M;
        float dtheta_gyro = gyro_z_radps * dt_s;

        float dtheta = GYRO_COMPLEMENTARY_ALPHA * dtheta_gyro + (1.0f - GYRO_COMPLEMENTARY_ALPHA) * dtheta_odom;
        float mid_theta = pose.theta + 0.5f * dtheta;

        pose.x += ds * cosf(mid_theta);
        pose.y += ds * sinf(mid_theta);
        pose.theta = wrapAngle(pose.theta + dtheta);

        total_time_s += dt_s;
        total_distance_m += fabsf(ds);
    }

    void step(float front_dist_m, bool bumper_hit, float dt_s, float& out_v_mps, float& out_omega_radps, float& out_servo_rad) {
        out_v_mps = 0.0f;
        out_omega_radps = 0.0f;
        out_servo_rad = 0.0f;

        switch (state) {
            case MissionState::IDLE:
            case MissionState::COMPLETED:
                return;

            case MissionState::EXPLORE_FWD: {
                if (bumper_hit || front_dist_m <= STOP_DISTANCE_M) {
                    last_wall_dist_m = bumper_hit ? 0.0f : front_dist_m;
                    state = MissionState::BRAKE_RECORD;
                    return;
                }
                float err = wrapAngle(target_heading_rad - pose.theta);
                out_omega_radps = constrain(HEADING_KP * err, -MAX_ANGULAR_SPEED_RADPS, MAX_ANGULAR_SPEED_RADPS);
                out_v_mps = CRUISE_SPEED_MPS;
                out_servo_rad = err;
                return;
            }

            case MissionState::BRAKE_RECORD: {
                float offset = ROBOT_RADIUS_M + max(0.0f, last_wall_dist_m);
                if (num_wall_points < MAX_CHORDS) {
                    wall_points[num_wall_points++] = {
                        pose.x + offset * cosf(pose.theta),
                        pose.y + offset * sinf(pose.theta)
                    };
                }

                Point2D end_pt = {pose.x, pose.y};
                float clen = distance2D(chord_start_pt, end_pt);
                float dur = max(total_time_s - chord_start_time_s, dt_s);

                if (clen >= 0.05f && num_chords < MAX_CHORDS) {
                    ChordSegment c = {(uint16_t)(num_chords + 1), chord_start_pt, end_pt, clen, dur};
                    // Contar cruces con todas las cuerdas previas no adyacentes (Método de las Hormigas)
                    for (uint16_t i = 0; i + 1 < num_chords; ++i) {
                        if (segmentsIntersect(chords[i].start, chords[i].end, c.start, c.end)) {
                            num_crossings++;
                        }
                    }
                    chords[num_chords] = c;
                    num_hull_vertices = buildSimplifiedHull(wall_points, num_wall_points, hull_vertices);
                    area_history[num_chords] = computePolygonArea(hull_vertices, num_hull_vertices);
                    num_chords++;
                }
                state = MissionState::EVALUATE_MAP;
                return;
            }

            case MissionState::EVALUATE_MAP: {
                evaluateEstimates();
                if (last_estimates.converged || num_chords >= MAX_CHORDS_EXPLORE) {
                    state = MissionState::BUILD_SAFE_PATH;
                } else {
                    target_heading_rad = selectNextHeading();
                    state = MissionState::TURN_HEADING;
                }
                return;
            }

            case MissionState::TURN_HEADING: {
                float err = wrapAngle(target_heading_rad - pose.theta);
                if (fabsf(err) <= 0.035f) {
                    chord_start_pt = {pose.x, pose.y};
                    chord_start_time_s = total_time_s;
                    state = MissionState::EXPLORE_FWD;
                    return;
                }
                out_omega_radps = constrain(HEADING_KP * err, -MAX_ANGULAR_SPEED_RADPS, MAX_ANGULAR_SPEED_RADPS);
                if (fabsf(out_omega_radps) < 0.45f) {
                    out_omega_radps = (err >= 0.0f) ? 0.45f : -0.45f;
                }
                out_servo_rad = err;
                return;
            }

            case MissionState::BUILD_SAFE_PATH: {
                num_hull_vertices = buildSimplifiedHull(wall_points, num_wall_points, hull_vertices);
                Point2D raw_wps[MAX_HULL_VERTICES];
                uint16_t nw = buildInwardOffsetPolygon(hull_vertices, num_hull_vertices, SAFE_PERIMETER_MARGIN_M, raw_wps);
                if (nw < 3) {
                    state = MissionState::COMPLETED;
                    return;
                }
                // Comenzar por el waypoint más cercano a la posición actual
                uint16_t closest = 0;
                Point2D curr = {pose.x, pose.y};
                float best_d = distance2D(curr, raw_wps[0]);
                for (uint16_t i = 1; i < nw; ++i) {
                    float d = distance2D(curr, raw_wps[i]);
                    if (d < best_d) { best_d = d; closest = i; }
                }
                for (uint16_t i = 0; i < nw; ++i) {
                    safe_waypoints[i] = raw_wps[(closest + i) % nw];
                }
                safe_waypoints[nw] = safe_waypoints[0]; // Cerrar el perímetro
                num_safe_waypoints = nw + 1;
                current_wp_idx = 0;
                state = MissionState::FOLLOW_PERIMETER;
                return;
            }

            case MissionState::FOLLOW_PERIMETER: {
                if (current_wp_idx >= num_safe_waypoints) {
                    state = MissionState::COMPLETED;
                    return;
                }
                Point2D target = safe_waypoints[current_wp_idx];
                Point2D curr = {pose.x, pose.y};
                float dist = distance2D(curr, target);
                if (dist <= WAYPOINT_TOLERANCE_M) {
                    current_wp_idx++;
                    if (current_wp_idx >= num_safe_waypoints) state = MissionState::COMPLETED;
                    return;
                }
                float desired_theta = atan2f(target.y - curr.y, target.x - curr.x);
                float err = wrapAngle(desired_theta - pose.theta);
                if (fabsf(err) > (12.0f * DEG_TO_RAD)) {
                    out_omega_radps = constrain(HEADING_KP * err, -MAX_ANGULAR_SPEED_RADPS, MAX_ANGULAR_SPEED_RADPS);
                    out_v_mps = 0.0f;
                    out_servo_rad = err;
                    return;
                }
                out_v_mps = min(PERIMETER_SPEED_MPS, max(0.12f, 1.8f * dist));
                out_omega_radps = constrain(HEADING_KP * err, -MAX_ANGULAR_SPEED_RADPS, MAX_ANGULAR_SPEED_RADPS);
                out_servo_rad = err;
                return;
            }
        }
    }

    size_t formatTelemetryJSON(char* buf, size_t buf_size) const {
        int written = snprintf(
            buf, buf_size,
            "{\"t_ms\":%lu,\"state\":%u,\"x\":%.3f,\"y\":%.3f,\"theta\":%.3f,"
            "\"chords\":%u,\"crossings\":%u,\"poly_area\":%.3f,\"poly_perim\":%.3f,"
            "\"ants_area\":%.3f,\"crofton_perim\":%.3f,\"converged\":%s}\n",
            (unsigned long)(total_time_s * 1000.0f),
            (unsigned int)state,
            pose.x, pose.y, pose.theta,
            last_estimates.num_chords,
            last_estimates.num_crossings,
            last_estimates.poly_area_m2,
            last_estimates.poly_perimeter_m,
            last_estimates.ants_area_m2,
            last_estimates.crofton_perimeter_m,
            last_estimates.converged ? "true" : "false"
        );
        return (written > 0) ? (size_t)written : 0;
    }

    void printTelemetryJSON(Stream& serial_out) const {
        char buf[320];
        formatTelemetryJSON(buf, sizeof(buf));
        serial_out.print(buf);
    }

private:
    float selectNextHeading() const {
        if (num_wall_points >= 3) {
            float cx = 0.0f, cy = 0.0f;
            for (uint16_t i = 0; i < num_wall_points; ++i) {
                cx += wall_points[i].x;
                cy += wall_points[i].y;
            }
            cx /= num_wall_points;
            cy /= num_wall_points;
            float to_center = atan2f(cy - pose.y, cx - pose.x);
            static const float offsets[9] = {-0.85f, 0.75f, -0.45f, 0.55f, 0.0f, -1.05f, 0.95f, -0.25f, 0.35f};
            return wrapAngle(to_center + offsets[num_chords % 9]);
        }
        return wrapAngle(pose.theta + PI - 0.55f + fmodf(num_chords * 2.39996f, 0.7f));
    }

    void evaluateEstimates() {
        float poly_area = computePolygonArea(hull_vertices, num_hull_vertices);
        float poly_perim = computePolygonPerimeter(hull_vertices, num_hull_vertices);

        float sum_len = 0.0f;
        for (uint16_t i = 0; i < num_chords; ++i) sum_len += chords[i].length;
        float mean_chord = (num_chords > 0) ? (sum_len / num_chords) : 0.0f;

        // Método de las Hormigas (Pares de cuerdas no adyacentes)
        float sum_prod = 0.0f;
        for (uint16_t i = 0; i < num_chords; ++i) {
            for (uint16_t j = i + 2; j < num_chords; ++j) {
                sum_prod += chords[i].length * chords[j].length;
            }
        }
        float ants_area = (num_crossings > 0)
            ? ((2.0f * ANTS_BOUNDARY_FACTOR * sum_prod) / (PI * num_crossings))
            : 0.0f;
        float crofton_perim = (mean_chord > 1e-4f) ? (PI * poly_area / mean_chord) : 0.0f;

        // Contar sectores angulares visitados (8 sectores de 45°)
        uint8_t sector_mask = 0;
        if (num_wall_points >= 3) {
            float cx = 0.0f, cy = 0.0f;
            for (uint16_t i = 0; i < num_wall_points; ++i) { cx += wall_points[i].x; cy += wall_points[i].y; }
            cx /= num_wall_points; cy /= num_wall_points;
            for (uint16_t i = 0; i < num_wall_points; ++i) {
                float ang = atan2f(wall_points[i].y - cy, wall_points[i].x - cx) + PI;
                uint8_t sec = ((uint8_t)((ang / (2.0f * PI)) * 8.0f)) & 0x07;
                sector_mask |= (1 << sec);
            }
        }
        uint8_t sectors = 0;
        for (uint8_t b = 0; b < 8; ++b) if (sector_mask & (1 << b)) sectors++;

        bool conv = false;
        if (num_chords >= MIN_CHORDS_FOR_MAP && sectors >= 8 && num_chords > 4 && poly_area > 0.1f) {
            float prev_a = area_history[num_chords - 5];
            if (fabsf(poly_area - prev_a) / poly_area <= AREA_STABILITY_TOL) {
                conv = true;
            }
        }

        last_estimates = {
            num_chords, num_wall_points, num_crossings,
            sum_len, mean_chord, poly_area, poly_perim,
            ants_area, crofton_perim, sectors, conv
        };
    }
};

#endif // RECORREDOR_NAVIGATOR_FSM_H
