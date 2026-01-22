#!/usr/bin/env python3
import math
import time
import threading

import rclpy
from rclpy.node import Node
import serial


class Mission1mTurnBack(Node):
    """
    Commande Arduino via Serial:
      - Envoie: "V <lin_m_s> <ang_rad_s>\n"
      - Reçoit (optionnel): "R <dl_m> <dr_m>\r\n"

    Mission:
      1) Avancer ~distance_m
      2) Demi-tour 180°
      3) Avancer ~distance_m (retour)

    Obstacle:
      - Stop si obstacle < obstacle_stop_m
      - Reprend si obstacle > obstacle_resume_m (hystérésis)
      - Pendant stop: on PAUSE le chrono de la phase
    """

    def __init__(self):
        super().__init__('autopilot_mission')

        # -------- Params --------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        self.declare_parameter('speed', 0.20)         # m/s
        self.declare_parameter('distance_m', 1.0)     # m (approx)
        self.declare_parameter('ang_speed', 1.0)      # rad/s for turn in place

        # Obstacle: "un peu proche"
        self.declare_parameter('obstacle_stop_m', 0.18)    # stop si < 18 cm
        self.declare_parameter('obstacle_resume_m', 0.23)  # reprend si > 23 cm
        self.declare_parameter('use_min_lr', True)         # min(dl,dr) vs moyenne

        # Petites pauses visibles (optionnel)
        self.declare_parameter('phase_brake_s', 0.20)  # stop entre phases

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)

        self.v = float(self.get_parameter('speed').value)
        self.dist = float(self.get_parameter('distance_m').value)
        self.w = float(self.get_parameter('ang_speed').value)

        self.obs_stop = float(self.get_parameter('obstacle_stop_m').value)
        self.obs_resume = float(self.get_parameter('obstacle_resume_m').value)
        self.use_min_lr = bool(self.get_parameter('use_min_lr').value)

        self.brake_s = float(self.get_parameter('phase_brake_s').value)

        # Durées approx (time-based)
        self.t_forward = abs(self.dist / max(abs(self.v), 1e-6))
        self.t_turn = abs(math.pi / max(abs(self.w), 1e-6))  # 180°

        # -------- Serial --------
        self.ser = serial.Serial(self.port, self.baud, timeout=0.0)
        self.get_logger().info(f"Serial opened: {self.port} @ {self.baud}")

        # RX
        self._rx_stop = False
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # Latest ranges
        self.dl = -1.0
        self.dr = -1.0
        self.obstacle = False
        self.blocked = False

        # State machine
        self.FWD1 = 'FWD1'
        self.TURN = 'TURN'
        self.FWD2 = 'FWD2'
        self.DONE = 'DONE'

        self.phase = self.FWD1
        self.remaining = self.t_forward

        self.last_tick = time.monotonic()

        # 20 Hz pour garder le watchdog Arduino OK (500ms)
        self.timer = self.create_timer(0.05, self._tick)

        self._send_cmd(0.0, 0.0)
        self.get_logger().info(
            f"Mission: forward {self.dist}m (~{self.t_forward:.2f}s) -> turn 180° (~{self.t_turn:.2f}s) -> forward {self.dist}m"
        )
        self.get_logger().info(
            f"Obstacle: stop<{self.obs_stop:.2f}m, resume>{self.obs_resume:.2f}m"
        )

    def destroy_node(self):
        try:
            self._send_cmd(0.0, 0.0)
        except Exception:
            pass
        self._rx_stop = True
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()

    # ---------- Serial RX ----------
    def _rx_loop(self):
        buf = b""
        while not self._rx_stop:
            try:
                data = self.ser.read(256)
                if data:
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self._handle_line(line.decode(errors='ignore').strip())
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.05)

    def _handle_line(self, line: str):
        # Ignore ACK; use only R dl dr
        if line.startswith('R '):
            parts = line.split()
            if len(parts) >= 3:
                try:
                    self.dl = float(parts[1])
                    self.dr = float(parts[2])
                    self._update_obstacle()
                except ValueError:
                    pass

    def _update_obstacle(self):
        vals = []
        if self.dl > 0:
            vals.append(self.dl)
        if self.dr > 0:
            vals.append(self.dr)

        if not vals:
            self.obstacle = False
            return

        front = min(vals) if self.use_min_lr else sum(vals) / len(vals)

        # Hystérésis: stop proche, reprise plus loin
        if not self.obstacle and front < self.obs_stop:
            self.obstacle = True
        elif self.obstacle and front > self.obs_resume:
            self.obstacle = False

    # ---------- Control ----------
    def _send_cmd(self, lin: float, ang: float):
        msg = f"V {lin:.3f} {ang:.3f}\n"
        self.ser.write(msg.encode('ascii'))

    def _switch_phase(self, new_phase: str, new_remaining: float, log: str):
        self.phase = new_phase
        self.remaining = new_remaining
        self.get_logger().info(log)

        # petite pause stop entre phases (optionnel)
        if self.brake_s > 0:
            self._send_cmd(0.0, 0.0)
            time.sleep(self.brake_s)

    def _tick(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now

        # Obstacle -> STOP + pause chrono
        if self.obstacle:
            if not self.blocked:
                self.blocked = True
                self.get_logger().warn("Obstacle proche -> STOP (pause mission)")
            self._send_cmd(0.0, 0.0)
            return
        else:
            if self.blocked:
                self.blocked = False
                self.get_logger().info("Obstacle disparu -> reprise mission")

        if self.phase == self.DONE:
            self._send_cmd(0.0, 0.0)
            return

        # Décrémenter chrono phase
        self.remaining -= dt
        if self.remaining <= 0.0:
            if self.phase == self.FWD1:
                self._switch_phase(self.TURN, self.t_turn, "Phase -> TURN (180°)")
            elif self.phase == self.TURN:
                self._switch_phase(self.FWD2, self.t_forward, "Phase -> FWD2 (return)")
            elif self.phase == self.FWD2:
                self._switch_phase(self.DONE, 0.0, "DONE -> STOP")
                self._send_cmd(0.0, 0.0)
                return

        # Commande selon phase
        if self.phase in (self.FWD1, self.FWD2):
            self._send_cmd(self.v, 0.0)
        elif self.phase == self.TURN:
            self._send_cmd(0.0, self.w)


def main(args=None):
    rclpy.init(args=args)
    node = Mission1mTurnBack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
