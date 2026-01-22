#!/usr/bin/env python3
import math
import time
import threading
import rclpy
from rclpy.node import Node

import serial


class SerialAutoPilot(Node):
    """
    Parle directement avec ton Arduino via Serial:
      - Envoie:  "V <lin_m_s> <ang_rad_s>\n"
      - Reçoit:  "R <dist_left_m> <dist_right_m>\r\n" (optionnel)
                "ACK V ..." (ignoré)
    """

    def __init__(self):
        super().__init__('serial_autopilot')

        # -------- Params --------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('speed', 0.20)        # m/s
        self.declare_parameter('distance', 1.0)      # m (approx)
        self.declare_parameter('ang_speed', 1.0)     # rad/s (turn in place)
        self.declare_parameter('obstacle_m', 0.25)   # seuil m (stop si <)
        self.declare_parameter('use_min_lr', True)   # obstacle = min(dl,dr)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.v = float(self.get_parameter('speed').value)
        self.dist = float(self.get_parameter('distance').value)
        self.w = float(self.get_parameter('ang_speed').value)
        self.obs_th = float(self.get_parameter('obstacle_m').value)
        self.use_min_lr = bool(self.get_parameter('use_min_lr').value)

        # Durées (approx)
        self.t_forward = abs(self.dist / max(abs(self.v), 1e-6))
        self.t_turn = abs(math.pi / max(abs(self.w), 1e-6))

        # -------- Serial --------
        self.ser = serial.Serial(self.port, self.baud, timeout=0.0)
        self.get_logger().info(f"Serial opened: {self.port} @ {self.baud}")

        # Latest ranges
        self.dl = -1.0
        self.dr = -1.0
        self.obstacle = False

        # State machine
        self.P_FORWARD_1 = 'FORWARD_1'
        self.P_TURN = 'TURN'
        self.P_FORWARD_2 = 'FORWARD_2'
        self.P_DONE = 'DONE'

        self.phase = self.P_FORWARD_1
        self.remaining = self.t_forward
        self.blocked = False

        self.last_tick = time.monotonic()

        # Serial reader thread
        self._rx_stop = False
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # Control loop timer (20 Hz => ok avec watchdog Arduino 500ms)
        self.timer = self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f"Plan: forward {self.dist}m (~{self.t_forward:.2f}s) -> turn 180deg (~{self.t_turn:.2f}s) -> forward {self.dist}m"
        )
        self._send_cmd(0.0, 0.0)

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

    # ------------- Serial RX -------------
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
                    time.sleep(0.005)
            except Exception:
                time.sleep(0.02)

    def _handle_line(self, line: str):
        # Attend "R dl dr"
        if line.startswith('R '):
            parts = line.split()
            if len(parts) >= 3:
                try:
                    dl = float(parts[1])
                    dr = float(parts[2])
                    self.dl, self.dr = dl, dr
                    self._update_obstacle()
                except ValueError:
                    pass
        # ACK ignored

    def _update_obstacle(self):
        # dl/dr = -1 => invalide
        vals = []
        if self.dl > 0:
            vals.append(self.dl)
        if self.dr > 0:
            vals.append(self.dr)

        if not vals:
            self.obstacle = False
            return

        front = min(vals) if self.use_min_lr else sum(vals)/len(vals)
        self.obstacle = (front < self.obs_th)

    # ------------- Control -------------
    def _send_cmd(self, lin: float, ang: float):
        msg = f"V {lin:.3f} {ang:.3f}\n"
        self.ser.write(msg.encode('ascii'))

    def _tick(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now

        # Si obstacle => stop + pause chrono
        if self.obstacle:
            if not self.blocked:
                self.get_logger().warn("Obstacle détecté -> STOP (pause)")
                self.blocked = True
            self._send_cmd(0.0, 0.0)
            return
        else:
            if self.blocked:
                self.get_logger().info("Obstacle disparu -> reprise")
                self.blocked = False

        # Si fini
        if self.phase == self.P_DONE:
            self._send_cmd(0.0, 0.0)
            return

        # Décrémenter le temps restant
        self.remaining -= dt
        if self.remaining <= 0.0:
            # Passer à la phase suivante
            if self.phase == self.P_FORWARD_1:
                self.phase = self.P_TURN
                self.remaining = self.t_turn
                self.get_logger().info("Phase -> TURN (180deg)")
            elif self.phase == self.P_TURN:
                self.phase = self.P_FORWARD_2
                self.remaining = self.t_forward
                self.get_logger().info("Phase -> FORWARD_2 (return)")
            elif self.phase == self.P_FORWARD_2:
                self.phase = self.P_DONE
                self.get_logger().info("DONE -> STOP")
                self._send_cmd(0.0, 0.0)
                return

        # Commander selon phase
        if self.phase in (self.P_FORWARD_1, self.P_FORWARD_2):
            self._send_cmd(self.v, 0.0)
        elif self.phase == self.P_TURN:
            # demi-tour sur place (choisis le signe si tu veux tourner à gauche/droite)
            self._send_cmd(0.0, self.w)

def main(args=None):
    rclpy.init(args=args)
    node = SerialAutoPilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Always try to stop motors + close serial cleanly
        try:
            node.destroy_node()
        except Exception:
            pass

        # Avoid double shutdown (SIGINT handler may have already done it)
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass



if __name__ == '__main__':
    main()
