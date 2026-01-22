#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self.get_logger().info('ObstacleAvoider node started (LaserScan version)')

        # Distance de sécurité (m)
        self.safe_dist = 0.5

        # Dernières mesures dans 3 secteurs
        self.front_dist = math.inf
        self.left_dist = math.inf
        self.right_dist = math.inf

        # Subscriber /scan (LIDAR 2D)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Publisher cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer de contrôle (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    # --- Utils pour extraire des distances par secteur angulaire ---
    def _min_in_sector(self, msg: LaserScan, angle_min: float, angle_max: float) -> float:
        """
        Retourne la distance minimale dans le secteur [angle_min, angle_max] en radians,
        en ignorant les valeurs invalides (0, inf, etc.).
        """
        a0 = msg.angle_min
        inc = msg.angle_increment
        n = len(msg.ranges)

        # indices approximatifs correspondant aux angles demandés
        start_idx = int((angle_min - a0) / inc)
        end_idx = int((angle_max - a0) / inc)

        # clamp dans [0, n-1]
        start_idx = max(0, min(n - 1, start_idx))
        end_idx = max(0, min(n - 1, end_idx))
        if end_idx < start_idx:
            start_idx, end_idx = end_idx, start_idx

        vals = []
        for i in range(start_idx, end_idx + 1):
            r = msg.ranges[i]
            # on ignore 0, inf, NaN, et tout ce qui est aberrant
            if math.isinf(r) or math.isnan(r):
                continue
            if r <= 0.0:
                continue
            # seuil max "raisonnable" (par ex. 10 m)
            if r > 10.0:
                continue
            vals.append(r)

        if not vals:
            return math.inf
        return min(vals)

    # --- Callback LIDAR ---
    def scan_cb(self, msg: LaserScan):
        # Secteurs (en radians)
        # Devant : [-30°, 30°]
        # Gauche : [30°, 90°]
        # Droite : [-90°, -30°]
        deg = math.pi / 180.0

        self.front_dist = self._min_in_sector(msg, -30 * deg, 30 * deg)
        self.left_dist = self._min_in_sector(msg, 30 * deg, 90 * deg)
        self.right_dist = self._min_in_sector(msg, -90 * deg, -30 * deg)

        # LOG IMPORTANT POUR VOIR CE QUE VOIT LE ROBOT
        self.get_logger().info(
            f"[SCAN] front={self.front_dist:.2f} m, left={self.left_dist:.2f} m, right={self.right_dist:.2f} m"
        )

    # --- Boucle de contrôle ---
    def control_loop(self):
        cmd = Twist()

        # Cas 1 : obstacle devant → tourner du côté le plus libre
        if self.front_dist < self.safe_dist:
            self.get_logger().info(
                f"Obstacle devant ({self.front_dist:.2f} m) -> je tourne"
            )
            if self.left_dist > self.right_dist:
                cmd.angular.z = 0.6   # tourne à gauche
            else:
                cmd.angular.z = -0.6  # tourne à droite

        # Cas 2 : pas d’obstacle devant → avancer
        else:
            cmd.linear.x = 0.18  # avance
            # légère correction latérale en fonction des obstacles côté gauche/droite
            if self.left_dist < self.safe_dist and self.right_dist > self.left_dist:
                cmd.angular.z = -0.3   # éloigne-toi du mur à gauche
            elif self.right_dist < self.safe_dist and self.left_dist > self.right_dist:
                cmd.angular.z = 0.3    # éloigne-toi du mur à droite

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
