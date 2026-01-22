#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def normalize_angle(a: float) -> float:
    """Ramène un angle dans [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class SimpleDelivery(Node):
    def __init__(self):
        super().__init__('simple_delivery')

        self.get_logger().info("SimpleDelivery node started (1m forward/back, 5 cycles, ultrasonic).")

        # ---------- Paramètres comportement ----------
        self.forward_distance = 1.0     # distance vers le point désiré (m)
        self.pos_tolerance = 0.03       # tolérance sur la position cible (m)
        self.home_tolerance = 0.03      # tolérance pour considérer qu'on est revenu à la base (m)
        self.yaw_tolerance = 0.03       # tolérance sur l'orientation (rad ~ 1.7°)

        # ---------- Gains de translation / orientation ----------
        # Contrôle couplé (aller/retour)
        self.linear_speed = 0.25        # m/s
        self.angular_speed_max = 1.2    # rad/s (vitesse angulaire max pour le suivi de point)
        self.k_ang = 2.0                # gain pour correction orientation (go_to_point)
        self.k_lin = 0.5                # gain pour vitesse linéaire

        # Rotation pure (demi-tours) : plus agressive
        self.k_ang_turn = 3.5           # gain rotation pure
        self.angular_speed_turn_max = 2.0
        self.angular_speed_turn_min = 0.25  # min pour que le robot tourne franchement

        self.obstacle_stop_dist = 0.5   # m : arrêt si obstacle devant

        # ---------- Cycles ----------
        self.cycle_limit = 5            # nombre de cycles à faire
        self.cycle_count = 0

        # ---------- Données ULTRASONS ----------
        self.front_left_dist = math.inf
        self.front_right_dist = math.inf
        self.front_dist = math.inf  # min des deux

        # ---------- Données ODOM ----------
        self.home_x = None
        self.home_y = None
        self.home_yaw = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Cibles de position
        self.target_forward_x = None
        self.target_forward_y = None

        self.target_x = None
        self.target_y = None
        self.target_yaw = None

        # ---------- Machine à états ----------
        # WAIT_ODOM -> FORWARD -> TURN_AT_TARGET -> GO_HOME -> TURN_AT_HOME -> ...
        self.state = 'WAIT_ODOM'
        self.state_start_time = None

        # Souscriptions
        self.create_subscription(Range, '/ultrasonic_left', self.ultrasonic_left_cb, 10)
        self.create_subscription(Range, '/ultrasonic_right', self.ultrasonic_right_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Publication cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    # ----------- Utils temps / distance -----------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _dist(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def set_state(self, new_state: str):
        self.state = new_state
        self.state_start_time = self._now()
        # reset cibles d'orientation pour le nouvel état
        self.target_yaw = None
        self.get_logger().info(
            f"--> NEW STATE: {self.state} (cycle {self.cycle_count}/{self.cycle_limit})"
        )

    # ----------- Mise à jour distance avant à partir des 2 ultrasons -----------
    def _update_front_dist(self):
        self.front_dist = min(self.front_left_dist, self.front_right_dist)

    def ultrasonic_left_cb(self, msg: Range):
        if math.isfinite(msg.range) and msg.range > 0.0:
            self.front_left_dist = msg.range
        else:
            self.front_left_dist = math.inf
        self._update_front_dist()

    def ultrasonic_right_cb(self, msg: Range):
        if math.isfinite(msg.range) and msg.range > 0.0:
            self.front_right_dist = msg.range
        else:
            self.front_right_dist = math.inf
        self._update_front_dist()

    # ----------- ODOM : position + orientation ----------- 
    def odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        # yaw (rotation autour de Z) à partir du quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Initialisation de la pose "home" une seule fois
        if self.home_x is None:
            self.home_x = self.current_x
            self.home_y = self.current_y
            self.home_yaw = self.current_yaw

            # On fixe directement la cible avant à 1 m dans la direction de home_yaw
            self.target_forward_x = self.home_x + self.forward_distance * math.cos(self.home_yaw)
            self.target_forward_y = self.home_y + self.forward_distance * math.sin(self.home_yaw)

            self.get_logger().info(
                f"Home pose set at x={self.home_x:.2f}, y={self.home_y:.2f}, yaw={self.home_yaw:.2f}"
            )
            self.get_logger().info(
                f"Forward target fixed at x={self.target_forward_x:.2f}, y={self.target_forward_y:.2f}"
            )

            if self.state == 'WAIT_ODOM':
                self.set_state('FORWARD')

    # ----------- Contrôle vers un point (x,y) -----------
    def _go_to_point(self, target_x, target_y):
        """
        Contrôle de position simple : aller vers (target_x, target_y).
        Retourne (v, w, dist) = (vitesse linéaire, vitesse angulaire, distance restante).
        """
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 1e-4:
            return 0.0, 0.0, dist

        desired_yaw = math.atan2(dy, dx)
        err_yaw = normalize_angle(desired_yaw - self.current_yaw)

        # contrôle orientation proportionnel
        w = self.k_ang * err_yaw
        w = max(min(w, self.angular_speed_max), -self.angular_speed_max)

        # si l'erreur d'angle est grande, on tourne d'abord
        if abs(err_yaw) > 0.6:
            v = 0.0
        else:
            # près de la cible, on ralentit pour éviter les overshoots
            if dist < 0.20:
                v_cmd = self.k_lin * dist
                v_cmd *= 0.6           # ralentir dans les 20 derniers cm
            else:
                v_cmd = self.k_lin * dist

            v = min(self.linear_speed, v_cmd)

        # v est toujours >= 0 : le robot ne recule jamais,
        # il avance toujours vers la cible.
        return v, w, dist

    # ----------- Contrôle de rotation vers un yaw cible -----------
    def _turn_to_yaw(self, target_yaw):
        err = normalize_angle(target_yaw - self.current_yaw)
        err_abs = abs(err)

        # contrôle proportionnel plus agressif pour la rotation pure
        w = self.k_ang_turn * err

        # saturation
        if w > self.angular_speed_turn_max:
            w = self.angular_speed_turn_max
        elif w < -self.angular_speed_turn_max:
            w = -self.angular_speed_turn_max

        # imposer une vitesse min tant qu'on n'est pas dans la tolérance
        if err_abs > self.yaw_tolerance and err_abs > 0.05:
            if 0.0 < abs(w) < self.angular_speed_turn_min:
                w = self.angular_speed_turn_min * (1.0 if w >= 0.0 else -1.0)

        return w, err_abs

    # ----------- Boucle principale ----------- 
    def control_loop(self):
        now = self._now()
        if self.state_start_time is None:
            self.state_start_time = now

        cmd = Twist()

        # Si on n'a pas encore d'odom, on ne bouge pas
        if self.state == 'WAIT_ODOM':
            self.cmd_pub.publish(cmd)
            return

        # Si on a fini tous les cycles -> arrêt définitif
        if self.state == 'FINISHED':
            self.cmd_pub.publish(cmd)
            return

        # --- Sécurité obstacle (à l'aller et au retour) ---
        if self.state in ('FORWARD', 'GO_HOME'):
            if self.front_dist < self.obstacle_stop_dist:
                self.get_logger().warn(
                    f"Obstacle à {self.front_dist:.2f} m -> arrêt temporaire (state={self.state})."
                )
                self.cmd_pub.publish(Twist())
                return

        # 1) Avancer de 1 m vers la cible avant
        if self.state == 'FORWARD':
            v, w, dist = self._go_to_point(self.target_forward_x, self.target_forward_y)
            cmd.linear.x = v
            cmd.angular.z = w

            self.get_logger().info(f"[FORWARD] dist = {dist:.3f} m")

            if dist <= self.pos_tolerance:
                # arrivé au point désiré -> demi-tour (pour regarder vers la maison)
                self.set_state('TURN_AT_TARGET')

        # 2) Demi-tour à la cible (regarder vers la maison)
        elif self.state == 'TURN_AT_TARGET':
            if self.target_yaw is None:
                # orientation = home_yaw + pi → direction "vers la maison"
                self.target_yaw = normalize_angle(self.home_yaw + math.pi)
                self.get_logger().info(
                    f"Target yaw at destination = {self.target_yaw:.2f}"
                )

            w, err_abs = self._turn_to_yaw(self.target_yaw)
            cmd.angular.z = w
            cmd.linear.x = 0.0

            self.get_logger().info(f"[TURN_AT_TARGET] err_yaw = {err_abs:.3f} rad")

            if err_abs <= self.yaw_tolerance:
                self.set_state('GO_HOME')

        # 3) Revenir à la position initiale (en avançant, jamais en marche arrière)
        elif self.state == 'GO_HOME':
            v, w, dist = self._go_to_point(self.home_x, self.home_y)
            cmd.linear.x = v
            cmd.angular.z = w

            self.get_logger().info(f"[GO_HOME] dist_home = {dist:.3f} m")

            if dist <= self.home_tolerance:
                # arrivé près de la base -> tourner pour retrouver l'orientation initiale
                self.set_state('TURN_AT_HOME')

        # 4) Demi-tour à la maison pour retrouver orientation initiale
        elif self.state == 'TURN_AT_HOME':
            if self.target_yaw is None:
                self.target_yaw = self.home_yaw
                self.get_logger().info(
                    f"Target yaw at home = {self.target_yaw:.2f}"
                )

            w, err_abs = self._turn_to_yaw(self.target_yaw)
            cmd.angular.z = w
            cmd.linear.x = 0.0

            self.get_logger().info(f"[TURN_AT_HOME] err_yaw = {err_abs:.3f} rad")

            if err_abs <= self.yaw_tolerance:
                self.cycle_count += 1
                self.get_logger().info(f"Cycle {self.cycle_count} terminé.")

                if self.cycle_count >= self.cycle_limit:
                    self.get_logger().info(
                        "Tous les cycles sont terminés. Robot arrêté à la position initiale."
                    )
                    self.state = 'FINISHED'
                    self.state_start_time = self._now()
                else:
                    # on enchaîne directement sur un nouveau FORWARD
                    self.set_state('FORWARD')

        else:
            self.get_logger().warn(f"État inconnu : {self.state}, retour à FORWARD")
            self.set_state('FORWARD')

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDelivery()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
