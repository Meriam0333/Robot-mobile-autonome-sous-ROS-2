#!/usr/bin/env python3
import time
import threading

import rclpy
from rclpy.node import Node
import serial


class ForwardOnceWithObstacle(Node):
    """
    - Avance une distance approx (time-based): distance_m / speed
    - Pause si obstacle détecté (ultrason gauche OU droite) et reprend quand l'obstacle disparaît
    - Stop définitif à la fin de la distance
    """

    def __init__(self):
        super().__init__('autopilot_forward_once')

        # ----- Params -----
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        self.declare_parameter('speed', 0.20)          # m/s
        self.declare_parameter('distance_m', 0.40)     # m (approx)

        # obstacle proche (STOP) puis reprise (hystérésis)
        self.declare_parameter('obstacle_stop_m', 0.20)
        self.declare_parameter('obstacle_resume_m', 0.25)
        self.declare_parameter('use_min_lr', True)     # min(dl,dr) => gauche OU droite

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)

        self.v = float(self.get_parameter('speed').value)
        self.distance_m = float(self.get_parameter('distance_m').value)

        self.obs_stop = float(self.get_parameter('obstacle_stop_m').value)
        self.obs_resume = float(self.get_parameter('obstacle_resume_m').value)
        self.use_min_lr = bool(self.get_parameter('use_min_lr').value)

        # durée approx
        self.t_forward = abs(self.distance_m / max(abs(self.v), 1e-6))
        self.remaining = self.t_forward

        # ----- Serial -----
        self.ser = serial.Serial(self.port, self.baud, timeout=0.0)
        self.get_logger().info(f"Serial opened: {self.port} @ {self.baud}")
        self.get_logger().info(f"Goal: forward {self.distance_m}m (~{self.t_forward:.2f}s) then STOP forever")
        self.get_logger().info(f"Obstacle: stop<{self.obs_stop:.2f}m resume>{self.obs_resume:.2f}m (either sensor)")

        # ranges
        self.dl = -1.0
        self.dr = -1.0
        self.obstacle = False
        self.done = False
        self.blocked = False

        # RX thread (read R lines)
        self._rx_stop = False
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.last_tick = time.monotonic()
        self.timer = self.create_timer(0.05, self._tick)  # 20 Hz

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

    def _send_cmd(self, lin: float, ang: float):
        self.ser.write(f"V {lin:.3f} {ang:.3f}\n".encode('ascii'))

    # ----- Serial RX -----
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
        if line.startswith("R "):
            parts = line.split()
            if len(parts) >= 3:
                try:
                    self.dl = float(parts[1])
                    self.dr = float(parts[2])
                    self._update_obstacle()
                except ValueError:
                    pass
        # ignore ACK

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

        # hystérésis
        if (not self.obstacle) and front < self.obs_stop:
            self.obstacle = True
        elif self.obstacle and front > self.obs_resume:
            self.obstacle = False

    # ----- Control -----
    def _tick(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now

        if self.done:
            self._send_cmd(0.0, 0.0)
            return

        # obstacle -> stop + pause chrono
        if self.obstacle:
            if not self.blocked:
                self.blocked = True
                self.get_logger().warn("Obstacle -> STOP (pause)")
            self._send_cmd(0.0, 0.0)
            return
        else:
            if self.blocked:
                self.blocked = False
                self.get_logger().info("Obstacle removed -> resume")

        # avancer + décrémenter le chrono
        self.remaining -= dt
        if self.remaining <= 0.0:
            self.done = True
            self.get_logger().info("Reached 0.4m (time-based) -> STOP forever")
            self._send_cmd(0.0, 0.0)
            return

        self._send_cmd(self.v, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardOnceWithObstacle()
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
