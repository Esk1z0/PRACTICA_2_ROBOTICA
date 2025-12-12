#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range, LaserScan
from std_msgs.msg import Header

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class CoppeliaInterfaceNode(Node):
    def __init__(self):
        super().__init__('coppelia_interface_node')

        self.declare_parameter('robot_name', 'Pioneer_p3dx')
        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('max_speed', 2.0)

        self.declare_parameter('scan_frame', 'laser')
        self.declare_parameter('angle_min', -120.0 * math.pi / 180.0)
        self.declare_parameter('angle_max', 120.0 * math.pi / 180.0)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 5.0)

        self.robot_name = self.get_parameter('robot_name').value
        self.update_rate = float(self.get_parameter('update_rate').value)
        self.max_speed = float(self.get_parameter('max_speed').value)

        self.scan_frame = self.get_parameter('scan_frame').value
        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max = float(self.get_parameter('angle_max').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        self.client = None
        self.sim = None
        self.robot_handle = None
        self.left_motor = None
        self.right_motor = None
        self.robot_script = None

        self.pose_pub = self.create_publisher(PoseStamped, '/robot/pose', 10)

        self.sonar_pubs = {}
        for i in range(1, 17):
            self.sonar_pubs[i] = self.create_publisher(Range, f'/robot/sonar_{i}', 10)

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.connect_to_coppelia()
        self.create_timer(1.0 / self.update_rate, self.update_callback)

    def connect_to_coppelia(self):
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')

            self.robot_handle = self.sim.getObject(f'/{self.robot_name}')
            self.left_motor = self.sim.getObject(f'/{self.robot_name}_leftMotor')
            self.right_motor = self.sim.getObject(f'/{self.robot_name}_rightMotor')
            self.robot_script = self.sim.getScript(self.sim.scripttype_childscript, self.robot_handle)

            self.get_logger().info('Conectado a CoppeliaSim')
            return True
        except Exception as e:
            self.sim = None
            self.get_logger().error(f'No conecta con CoppeliaSim: {e}')
            return False

    def update_callback(self):
        if self.sim is None:
            return
        try:
            self.publish_robot_pose()
            self.publish_sonar_readings()
            self.publish_lidar_scan()
        except Exception as e:
            self.get_logger().warn(f'Update falló: {e}')

    def publish_robot_pose(self):
        pos = self.sim.getObjectPosition(self.robot_handle, -1)
        ori = self.sim.getObjectOrientation(self.robot_handle, -1)

        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])

        yaw = float(ori[2])
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.pose_pub.publish(msg)

    def publish_sonar_readings(self):
        readings = self.sim.callScriptFunction('getAllSonars', self.robot_script)
        now = self.get_clock().now().to_msg()

        for i, d in enumerate(readings, 1):
            if i not in self.sonar_pubs:
                break
            msg = Range()
            msg.header = Header()
            msg.header.stamp = now
            msg.header.frame_id = f'sonar_{i}'
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.26
            msg.min_range = 0.0
            msg.max_range = 5.0
            msg.range = float(d)
            self.sonar_pubs[i].publish(msg)

    def publish_lidar_scan(self):
        ranges = self.sim.callScriptFunction('getLaserScan', self.robot_script)
        now = self.get_clock().now().to_msg()

        n = len(ranges)
        if n <= 1:
            return

        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = now
        msg.header.frame_id = self.scan_frame

        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = (self.angle_max - self.angle_min) / float(n - 1)
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.update_rate
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = [float(x) for x in ranges]

        self.scan_pub.publish(msg)

    def cmd_vel_callback(self, msg: Twist):
        if self.sim is None:
            return
        try:
            v = float(msg.linear.x)
            w = float(msg.angular.z)

            left = max(-self.max_speed, min(self.max_speed, v - w))
            right = max(-self.max_speed, min(self.max_speed, v + w))

            self.sim.setJointTargetVelocity(self.left_motor, left)
            self.sim.setJointTargetVelocity(self.right_motor, right)
        except Exception as e:
            self.get_logger().warn(f'cmd_vel falló: {e}')

    def shutdown(self):
        try:
            if self.sim is not None:
                self.sim.setJointTargetVelocity(self.left_motor, 0.0)
                self.sim.setJointTargetVelocity(self.right_motor, 0.0)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
