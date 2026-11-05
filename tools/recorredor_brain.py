"""RECORREDOR — Cerebro en PC (modo Recto + Reaprendizaje).

Compañero del firmware "tonto" `firmware/recorredor_thin_client/`.
El Arduino/Nano/ESP32 solo mueve motores, lee los 2 HC-SR04 y el bumper y
transmite telemetría; TODA la inteligencia (FSM, odometría v*t, mapa,
reconstrucción ortogonal, ruta interior, reaprendizaje) corre acá.

ENLACES SOPORTADOS:
  --udp 5005            ESP32 con USE_WIFI_UDP=1 (WiFi real del robot).
  --serial COM5         UNO/Nano por USB (o el link que exponga el adaptador).

PROTOCOLO (el mismo que entiende el thin client):
  PC -> robot : "DRV <v> <w>"  tracción+giro  |  "STR <deg>" servo  |  "STP"
  robot -> PC : {"d1":..,"d2":..,"bump":0|1,"v":..,"w":..}

Uso:
    python tools/recorredor_brain.py --udp 5005
    python tools/recorredor_brain.py --serial COM5 --baud 115200
    python tools/recorredor_brain.py --chords 20 --relearns 8
"""

import argparse
import json
import math
import socket
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional

sys.path.insert(0, ".")  # importar geometría del simulador
from sim.core.geometry import (
    Point2D,
    Pose2D,
    ChordSegment,
    wrap_angle,
    reconstruct_walled_polygon,
    compute_inward_offset_polygon,
    build_anchored_perimeter_route,
    polygon_area,
    polygon_perimeter,
    point_to_segment_distance,
)

# ------------------------------------------------------------------ enlace
class UDPLink:
    """Escucha telemetría UDP y responde comandos a la última IP conocida."""

    def __init__(self, port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.peer: Optional[tuple] = None

    def recv_lines(self) -> List[str]:
        out = []
        while True:
            try:
                data, addr = self.sock.recvfrom(512)
            except BlockingIOError:
                break
            self.peer = addr
            out.append(data.decode(errors="ignore").strip())
        return out

    def send(self, line: str):
        if self.peer:
            self.sock.sendto(line.encode(), self.peer)


class SerialLink:
    """Serial USB para UNO/Nano (o enlace serial de cualquier adaptador)."""

    def __init__(self, port: str, baud: int):
        import serial  # pyserial

        self.ser = serial.Serial(port, baud, timeout=0)

    def recv_lines(self) -> List[str]:
        out = []
        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line:
                out.append(line)
        return out

    def send(self, line: str):
        self.ser.write((line + "\n").encode())


# ------------------------------------------------------------------ config
@dataclass
class Cfg:
    robot_radius_m: float = 0.12
    stop_distance_m: float = 0.05
    wheelbase_m: float = 0.18
    cruise_mps: float = 0.30
    perimeter_mps: float = 0.24
    heading_kp: float = 2.6
    waypoint_tol_m: float = 0.09
    chords_explore: int = 16
    max_relearns: int = 8
    differential_steer: bool = True   # True: DRV v w | False: DRV v 0 + STR deg
    tick_hz: float = 20.0

    @property
    def safe_margin_m(self) -> float:
        return self.robot_radius_m * 1.2


# ----------------------------------------------------------------- cerebro
class Brain:
    def __init__(self, cfg: Cfg, link):
        self.cfg = cfg
        self.link = link
        self.state = "EXPLORE"
        self.pose = Pose2D(0.0, 0.0, 0.0)
        self.target_heading = 0.0
        self.wall_points: List[Point2D] = []
        self.chords: List[ChordSegment] = []
        self.chord_start = Point2D(0.0, 0.0)
        self.polygon: List[Point2D] = []
        self.waypoints: List[Point2D] = []
        self.wp_idx = 0
        self.relearns = 0
        self.last_telemetry = {"d1": 2.0, "d2": 2.0, "bump": 0}
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0
        self.last_steer_deg = 0.0

    # --- comandos hacia el robot ---
    def drive(self, v: float, w: float, steer_deg: float = 0.0):
        self.last_cmd_v, self.last_cmd_w = v, w
        if self.cfg.differential_steer:
            self.link.send(f"DRV {v:.3f} {w:.3f}")
        else:
            self.link.send(f"DRV {v:.3f} 0.000")
            if abs(steer_deg - self.last_steer_deg) > 2.0:
                self.link.send(f"STR {steer_deg:.1f}")
                self.last_steer_deg = steer_deg

    def stop(self):
        self.drive(0.0, 0.0)

    # --- odometría abierta v*t (igual que en el firmware autónomo) ---
    def update_odom(self, dt: float):
        cfg = self.cfg
        if cfg.differential_steer:
            dth = self.last_cmd_w * dt
        else:
            steer = math.radians(self.last_steer_deg)
            dth = (self.last_cmd_v * math.tan(steer) / cfg.wheelbase_m) * dt
        ds = self.last_cmd_v * dt
        mid = self.pose.theta + 0.5 * dth
        self.pose = Pose2D(
            self.pose.x + ds * math.cos(mid),
            self.pose.y + ds * math.sin(mid),
            wrap_angle(self.pose.theta + dth),
        )

    # --- registro ---
    def record_wall_hit(self, d_wall: float):
        off = self.cfg.robot_radius_m + max(0.0, d_wall)
        pt = Point2D(
            self.pose.x + off * math.cos(self.pose.theta),
            self.pose.y + off * math.sin(self.pose.theta),
        )
        # Dedup: si ya hay un punto de pared a <3 cm, no acumular ruido ni
        # dejar crecer la lista cuando el robot queda pegado a una esquina.
        if not any(math.hypot(pt.x - p.x, pt.y - p.y) < 0.03
                   for p in self.wall_points):
            self.wall_points.append(pt)
        now = Point2D(self.pose.x, self.pose.y)
        l = math.hypot(now.x - self.chord_start.x, now.y - self.chord_start.y)
        if l >= 0.05:
            self.chords.append(ChordSegment(
                idx=len(self.chords), start=self.chord_start, end=now,
                duration_s=0.0, avg_speed_mps=0.0, heading_rad=self.pose.theta))

    def pick_next_heading(self) -> float:
        """Elige el rumbo con MAYOR despeje estimado: proyecta cada candidato
        contra los puntos de pared ya mapeados (cono de ±40°) y toma el que
        más espacio libre tiene. Evita el bucle de rebotar contra la misma
        pared cuando el robot quedó pegado a una esquina."""
        px, py = self.pose.x, self.pose.y
        candidates = []
        if len(self.wall_points) >= 3:
            cx = sum(p.x for p in self.wall_points) / len(self.wall_points)
            cy = sum(p.y for p in self.wall_points) / len(self.wall_points)
            to_c = math.atan2(cy - py, cx - px)
            for off in (0.0, 0.5, -0.5, 1.0, -1.0, 0.35, -0.35, 0.8):
                candidates.append(wrap_angle(to_c + off))
        for off in (math.pi, 1.57, -1.57, math.pi - 0.5, -(math.pi - 0.5),
                    0.5, -0.5, 1.0, -1.0, 0.0):
            candidates.append(wrap_angle(self.pose.theta + off))

        best_h, best_clear = candidates[0], -1.0
        for h in candidates:
            ch, sh = math.cos(h), math.sin(h)
            clear = float("inf")
            for p in self.wall_points:
                dx, dy = p.x - px, p.y - py
                if abs(wrap_angle(math.atan2(dy, dx) - h)) < 0.7:  # cono ±40°
                    proj = dx * ch + dy * sh
                    if proj > 0:
                        clear = min(clear, proj)
            if clear > best_clear:
                best_clear, best_h = clear, h
        return best_h

    def build_safe_route(self):
        """Reconstrucción ORTOGONAL completa del simulador + offset R*1.2 + anclaje."""
        poly = reconstruct_walled_polygon(
            self.wall_points, mode="recto", free_chords=self.chords)
        if len(poly) < 3:
            return False
        self.polygon = poly
        inset = compute_inward_offset_polygon(
            poly, self.cfg.safe_margin_m, ortho=True)
        self.waypoints = build_anchored_perimeter_route(
            Point2D(self.pose.x, self.pose.y), inset)
        self.wp_idx = 0
        # Gracia de ~1.2 s: al entrar a FOLLOW el robot está pegado a la pared
        # que acaba de chocar; sin esto cada rebuild dispararía un relearn
        # fantasma en el mismo punto hasta agotar MAX_RELEARNS.
        self._follow_grace_ticks = int(1.2 * self.cfg.tick_hz)
        return len(self.waypoints) >= 2

    # ------------------------------- FSM -----------------------------------
    def step(self, front_m: float, bump: bool):
        cfg = self.cfg
        st = self.state

        if st in ("IDLE", "DONE"):
            self.stop(); return

        if st == "EXPLORE":
            if bump or front_m <= cfg.stop_distance_m:
                self.record_wall_hit(0.0 if bump else front_m)
                self.state = "BRAKE"
                return
            err = wrap_angle(self.target_heading - self.pose.theta)
            om = max(-2.2, min(2.2, cfg.heading_kp * err))
            self.drive(cfg.cruise_mps, om, steer_deg=math.degrees(err) * 1.4)
            return

        if st == "BRAKE":
            self.stop()
            self.state = "EVALUATE"; return

        if st == "EVALUATE":
            if len(self.chords) >= cfg.chords_explore:
                self.state = "BUILD_PATH"
            else:
                self.target_heading = self.pick_next_heading()
                self.state = "TURN"
            return

        if st == "TURN":
            err = wrap_angle(self.target_heading - self.pose.theta)
            if abs(err) <= 0.05:
                self.chord_start = Point2D(self.pose.x, self.pose.y)
                self.state = "EXPLORE"; return
            if cfg.differential_steer:
                om = max(-2.0, min(2.0, cfg.heading_kp * err))
                if abs(om) < 0.4:
                    om = 0.4 if err >= 0 else -0.4
                self.drive(0.0, om)
            else:
                sgn = 45.0 if err >= 0 else -45.0
                self.drive(0.12, 0.0, steer_deg=sgn)
            return

        if st == "BUILD_PATH":
            self.state = "FOLLOW" if self.build_safe_route() else "DONE"
            return

        if st == "FOLLOW":
            if self.wp_idx >= len(self.waypoints):
                self.state = "DONE"; return
            # REAPRENDIZAJE: choque en la vuelta -> punto nuevo -> rebuild.
            # Solo cuenta si el punto de impacto es NUEVO (<5cm de ninguno
            # ya mapeado): rozar una pared conocida al doblar una esquina es
            # odometría drift, no un mapa equivocado — reaprenderla en bucle
            # agotaría los relearns sin aportar información.
            if self._follow_grace_ticks > 0:
                self._follow_grace_ticks -= 1
            elif bump or front_m <= cfg.stop_distance_m * 0.8:
                off = cfg.robot_radius_m + max(0.0, 0.0 if bump else front_m)
                hit = Point2D(self.pose.x + off * math.cos(self.pose.theta),
                              self.pose.y + off * math.sin(self.pose.theta))
                # ¿Está el impacto sobre el perímetro YA reconstruido?
                # Rozar una arista conocida al doblar una esquina es drift de
                # odometría, no un mapa equivocado: se ignora y se sigue.
                known = len(self.polygon) >= 3 and any(
                    point_to_segment_distance(hit,
                        self.polygon[i], self.polygon[(i + 1) % len(self.polygon)])
                    < 0.06
                    for i in range(len(self.polygon)))
                if not known:
                    self.wall_points.append(hit)
                    self.relearns += 1
                    print(f"[RELEARN #{self.relearns}] pared nueva en "
                          f"({self.pose.x:.2f},{self.pose.y:.2f})")
                    self.state = ("BUILD_PATH" if self.relearns < cfg.max_relearns
                                  else "DONE")
                return
            tgt = self.waypoints[self.wp_idx]
            d = math.hypot(tgt.x - self.pose.x, tgt.y - self.pose.y)
            if d <= cfg.waypoint_tol_m:
                self.wp_idx += 1
                if self.wp_idx >= len(self.waypoints):
                    self.state = "DONE"; return
                tgt = self.waypoints[self.wp_idx]
            des = math.atan2(tgt.y - self.pose.y, tgt.x - self.pose.x)
            err = wrap_angle(des - self.pose.theta)
            om = max(-2.2, min(2.2, cfg.heading_kp * err))
            v = min(cfg.perimeter_mps, max(0.10, 1.6 * d))
            self.drive(v, om, steer_deg=math.degrees(err) * 1.4)
            return

    # ------------------------------ lazo -----------------------------------
    def run(self):
        dt = 1.0 / self.cfg.tick_hz
        print(f"[BRAIN] modo=recto+reaprendizaje enlazando... (Ctrl+C para salir)")
        last = time.monotonic()
        try:
            while True:
                for line in self.link.recv_lines():
                    try:
                        self.last_telemetry.update(json.loads(line))
                    except json.JSONDecodeError:
                        pass
                now = time.monotonic()
                dt = min(now - last, 0.2)
                last = now
                front = min(float(self.last_telemetry.get("d1", 2.0)),
                            float(self.last_telemetry.get("d2", 2.0)))
                bump = bool(self.last_telemetry.get("bump", 0))
                prev = self.state
                self.update_odom(dt)
                self.step(front, bump)
                if self.state != prev:
                    print(f"[{self.state}] x={self.pose.x:.2f} y={self.pose.y:.2f} "
                          f"th={math.degrees(self.pose.theta):.0f}° "
                          f"chords={len(self.chords)} walls={len(self.wall_points)}")
                if self.state == "DONE":
                    print(f"\n=== COMPLETADO ===")
                    print(f"Área estimada     : {polygon_area(self.polygon):.3f} m²")
                    print(f"Perímetro estimado: {polygon_perimeter(self.polygon):.3f} m")
                    print(f"Cuerdas: {len(self.chords)}  Puntos de pared: {len(self.wall_points)}"
                          f"  Reaprendizajes: {self.relearns}")
                    self.stop()
                    return
                time.sleep(1.0 / self.cfg.tick_hz)
        except KeyboardInterrupt:
            self.stop()
            print("\n[BRAIN] detenido por el usuario — motores frenados.")


def main():
    ap = argparse.ArgumentParser(description="Recorredor — cerebro PC (thin client)")
    ap.add_argument("--udp", type=int, default=0, help="Puerto UDP del ESP32 (ej. 5005)")
    ap.add_argument("--serial", type=str, default="", help="Puerto serie del UNO/Nano (ej. COM5)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--chords", type=int, default=16, help="Cuerdas de exploración")
    ap.add_argument("--relearns", type=int, default=8, help="Máx. reaprendizajes en la vuelta")
    args = ap.parse_args()

    if args.serial:
        link = SerialLink(args.serial, args.baud)
        print(f"[BRAIN] enlace serial {args.serial} @{args.baud}")
    else:
        port = args.udp or 5005
        link = UDPLink(port)
        print(f"[BRAIN] enlace UDP :{port} (esperando telemetría del robot)")

    cfg = Cfg(chords_explore=args.chords, max_relearns=args.relearns)
    Brain(cfg, link).run()


if __name__ == "__main__":
    main()
