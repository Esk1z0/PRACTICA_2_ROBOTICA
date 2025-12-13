#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Bug2ControllerNode(Node):
    """
    Bug2 (estilo minimal.py) integrado en ROS2:
      SUB: /robot/pose (PoseStamped), /goal (PoseStamped), /robot/sonar_i (Range)
      PUB: /cmd_vel (Twist)  -> (v,w) estándar
    """

    MOTION_TO_GOAL = 0
    WALL_FOLLOWING = 1

    def __init__(self):
        super().__init__('bug2_controller_node')

        # Parámetros (alineados con tu minimal.py)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_linear_speed', 0.6)     # m/s (empieza conservador)
        self.declare_parameter('max_angular_speed', 1.2)    # rad/s
        self.declare_parameter('goal_reached_tolerance', 0.8)
        self.declare_parameter('m_line_tolerance', 0.3)
        self.declare_parameter('obstacle_threshold', 0.4)   # equivalente a bug2smoke/minimal
        self.declare_parameter('target_wall_distance', 0.5)
        self.declare_parameter('max_range', 5.0)

        self.control_rate = float(self.get_parameter('control_rate').value)
        self.v_max = float(self.get_parameter('max_linear_speed').value)
        self.w_max = float(self.get_parameter('max_angular_speed').value)
        self.goal_tol = float(self.get_parameter('goal_reached_tolerance').value)
        self.m_line_tol = float(self.get_parameter('m_line_tolerance').value)
        self.obs_th = float(self.get_parameter('obstacle_threshold').value)
        self.wall_dist = float(self.get_parameter('target_wall_distance').value)
        self.max_range = float(self.get_parameter('max_range').value)

        # Estado
        self.current_pose: PoseStamped | None = None
        self.goal_pose: PoseStamped | None = None
        self.sonar_readings: dict[int, Range] = {}

        self.state = self.MOTION_TO_GOAL
        self.start_point = None
        self.hit_point = None
        self.hit_distance_to_goal = None
        self.wall_following_side = 'right'

        # Mapeo de ángulos igual que tu minimal.py
        self.sensor_angles = {
            1: -90, 2: -50, 3: -30, 4: -10, 5: 10, 6: 30, 7: 50, 8: 90,
            9: 90, 10: 130, 11: 150, 12: 170, 13: -170, 14: -150, 15: -130, 16: -90
        }

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(PoseStamped, '/robot/pose', self.pose_cb, 10)
        self.create_subscription(PoseStamped, '/goal', self.goal_cb, 10)

        for i in range(1, 17):
            self.create_subscription(
                Range,
                f'/robot/sonar_{i}',
                lambda msg, sid=i: self.sonar_cb(msg, sid),
                10
            )

        self.create_timer(1.0 / self.control_rate, self.control_loop)
        self.get_logger().info('Bug2ControllerNode (minimal-style) listo.')

    # ----------------- Callbacks -----------------

    def pose_cb(self, msg: PoseStamped):
        first = self.current_pose is None
        self.current_pose = msg
        if first and self.start_point is None:
            self.start_point = [msg.pose.position.x, msg.pose.position.y]
            self.get_logger().info(f'Start point: ({self.start_point[0]:.2f}, {self.start_point[1]:.2f})')

    def goal_cb(self, msg: PoseStamped):
        self.goal_pose = msg
        if self.current_pose is not None:
            self.start_point = [self.current_pose.pose.position.x, self.current_pose.pose.position.y]
        self.state = self.MOTION_TO_GOAL
        self.hit_point = None
        self.hit_distance_to_goal = None
        self.get_logger().info(f'Nueva meta: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def sonar_cb(self, msg: Range, sensor_id: int):
        self.sonar_readings[sensor_id] = msg

    # ----------------- Helpers -----------------

    def yaw(self) -> float:
        q = self.current_pose.pose.orientation
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def dist_to_goal(self) -> float:
        dx = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
        return math.hypot(dx, dy)

    def angle_to_goal(self) -> float:
        dx = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
        goal_ang = math.atan2(dy, dx)
        return wrap_pi(goal_ang - self.yaw())

    def valid_range(self, sid: int) -> float | None:
        r = self.sonar_readings.get(sid)
        if r is None:
            return None
        if math.isfinite(r.range) and r.min_range <= r.range <= r.max_range and r.range <= self.max_range:
            return float(r.range)
        return None

    def get_clearance_in_direction(self, direction: str) -> float:
        if direction == 'left':
            relevant = [1, 2, 3, 4]
        else:
            relevant = [5, 6, 7, 8]
        vals = [self.valid_range(s) for s in relevant]
        vals = [v for v in vals if v is not None]
        return sum(vals) / len(vals) if vals else 0.0

    def is_on_m_line(self) -> bool:
        if self.start_point is None:
            return False
        x1, y1 = self.start_point
        x2, y2 = self.goal_pose.pose.position.x, self.goal_pose.pose.position.y
        x0, y0 = self.current_pose.pose.position.x, self.current_pose.pose.position.y

        line_len = math.hypot(x2 - x1, y2 - y1)
        if line_len < 0.1:
            return True

        perp = abs((y2 - y1)*x0 - (x2 - x1)*y0 + x2*y1 - y2*x1) / line_len
        return perp <= self.m_line_tol

    def distance_along_m_line_to_goal(self) -> float:
        if self.start_point is None:
            return float('inf')

        x1, y1 = self.start_point
        x2, y2 = self.goal_pose.pose.position.x, self.goal_pose.pose.position.y
        x0, y0 = self.current_pose.pose.position.x, self.current_pose.pose.position.y

        line_vec = [x2 - x1, y2 - y1]
        pt_vec = [x0 - x1, y0 - y1]
        line_len = math.hypot(line_vec[0], line_vec[1])
        if line_len < 0.1:
            return self.dist_to_goal()

        proj = (pt_vec[0]*line_vec[0] + pt_vec[1]*line_vec[1]) / (line_len**2)
        proj_x = x1 + proj * line_vec[0]
        proj_y = y1 + proj * line_vec[1]
        return math.hypot(x2 - proj_x, y2 - proj_y)

    def is_path_to_goal_clear(self, angle_to_goal: float) -> bool:
        # como minimal.py: usa front 4-5 y sensores hacia la meta
        front = [4, 5]
        for sid in front:
            d = self.valid_range(sid)
            if d is not None and d < self.obs_th:
                return False

        for sid, r in self.sonar_readings.items():
            d = self.valid_range(sid)
            if d is None:
                continue
            sensor_angle = math.radians(self.sensor_angles.get(sid, 0))
            angle_diff = abs(wrap_pi(sensor_angle - angle_to_goal))
            if angle_diff < math.pi/4:  # ±45°
                if d < self.obs_th:
                    return False
        return True

    # ----------------- Behaviours (minimal-style) -----------------

    def transition_to_wall_following(self):
        self.state = self.WALL_FOLLOWING
        self.hit_point = [self.current_pose.pose.position.x, self.current_pose.pose.position.y]
        self.hit_distance_to_goal = self.dist_to_goal()

        left_clear = self.get_clearance_in_direction('left')
        right_clear = self.get_clearance_in_direction('right')
        self.wall_following_side = 'left' if left_clear > right_clear else 'right'

        self.get_logger().info(
            f'Obstacle -> WALL_FOLLOWING ({self.wall_following_side}) | '
            f'hit_dist={self.hit_distance_to_goal:.2f}'
        )

    def can_leave_wall_following(self) -> bool:
        if self.hit_distance_to_goal is None:
            return False
        if not self.is_on_m_line():
            return False
        current = self.distance_along_m_line_to_goal()
        if current >= self.hit_distance_to_goal:
            return False
        ang = self.angle_to_goal()
        if not self.is_path_to_goal_clear(ang):
            return False
        return True

    def motion_to_goal(self) -> tuple[float, float]:
        ang = self.angle_to_goal()
        if self.is_path_to_goal_clear(ang):
            v = self.v_max * 0.8
            w = max(-self.w_max, min(self.w_max, ang * 1.5))
            return v, w

        self.transition_to_wall_following()
        return self.wall_following()

    def wall_following(self) -> tuple[float, float]:
        if self.can_leave_wall_following():
            self.state = self.MOTION_TO_GOAL
            self.get_logger().info('Leave wall -> MOTION_TO_GOAL')
            return self.motion_to_goal()

        # Control como minimal.py (base_speed + reglas)
        if self.wall_following_side == 'right':
            wall_sensors = [6, 7, 8]
            front_sensors = [4, 5]
        else:
            wall_sensors = [1, 2, 3]
            front_sensors = [4, 5]

        wall_vals = [self.valid_range(s) for s in wall_sensors]
        wall_vals = [v for v in wall_vals if v is not None]
        front_vals = [self.valid_range(s) for s in front_sensors]
        front_vals = [v for v in front_vals if v is not None]

        avg_wall = sum(wall_vals)/len(wall_vals) if wall_vals else 1.5
        min_front = min(front_vals) if front_vals else 1.5

        base = self.v_max * 0.35  # equivalente a "base_speed" de tu minimal (pero en m/s)

        # obstáculo frontal -> girar fuerte
        if min_front < self.obs_th:
            v = 0.0
            w = +self.w_max if self.wall_following_side == 'right' else -self.w_max
            return v, w

        # control lateral simple (mismo espíritu que minimal)
        err = avg_wall - self.wall_dist
        k = 1.8  # ganancia giro
        w = -k * err if self.wall_following_side == 'right' else +k * err
        w = max(-self.w_max, min(self.w_max, w))

        return base, w

    # ----------------- Loop -----------------

    def control_loop(self):
        if self.current_pose is None or self.goal_pose is None:
            self.publish_cmd(0.0, 0.0)
            return

        d = self.dist_to_goal()
        if d < self.goal_tol:
            self.publish_cmd(0.0, 0.0)
            return

        if self.state == self.MOTION_TO_GOAL:
            v, w = self.motion_to_goal()
        else:
            v, w = self.wall_following()

        self.publish_cmd(v, w)

    def publish_cmd(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Bug2ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
