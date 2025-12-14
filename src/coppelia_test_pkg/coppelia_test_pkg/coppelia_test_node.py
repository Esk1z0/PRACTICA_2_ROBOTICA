import math
import sys
import numpy as np
import random
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion, Point # Importamos Point explícitamente
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class PioneerP3dxNode(Node):
    def __init__(self):
        super().__init__('pioneer_p3dx_node')
        
        # Conexión
        self.declare_parameter('coppelia_host', 'host.docker.internal')
        host = self.get_parameter('coppelia_host').get_parameter_value().string_value

        self.get_logger().info(f'Conectando a {host}...')
        try:
            self._client = RemoteAPIClient(host=host)
            self._sim = self._client.require('sim')
            self.get_logger().info('¡Conectado!')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            sys.exit(1)

        # Handles
        try:
            self._robot_handle = self._sim.getObject('/PioneerP3DX')
            self._left_motor = self._sim.getObject('/PioneerP3DX/leftMotor')
            self._right_motor = self._sim.getObject('/PioneerP3DX/rightMotor')
        except:
            self.get_logger().error("No encuentro el robot '/PioneerP3DX'. Revisa la escena.")
            sys.exit(1)
        
        # Sonares
        self._sonar_handles = []
        for i in range(16):
            try:
                h = self._sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]')
                self._sonar_handles.append(h)
            except:
                try:
                    h = self._sim.getObject(f'/PioneerP3DX/visible/ultrasonicSensor[{i}]')
                    self._sonar_handles.append(h)
                except:
                    pass
        
        self.get_logger().info(f'Sonares encontrados: {len(self._sonar_handles)}/16')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Variables control
        self.timer = self.create_timer(0.1, self.control_loop)
        self.turn_timer = 0 

    def control_loop(self):
        # --- 1. LECTURA SENSORES ---
        min_dist_front = 999.0
        for i in range(1, 7): 
            if i < len(self._sonar_handles):
                res, dist, _, _, _ = self._sim.readProximitySensor(self._sonar_handles[i])
                if res > 0:
                    if dist < min_dist_front: min_dist_front = dist

        # --- 2. MOVIMIENTO ---
        v_lin = 0.5 
        v_ang = 0.0

        if self.turn_timer > 0:
            v_lin = 0.0
            v_ang = 0.5
            self.turn_timer -= 1
        else:
            if min_dist_front < 0.5:
                self.get_logger().info(f'¡OBSTÁCULO A {min_dist_front:.2f}m! GIRANDO.')
                self.turn_timer = 20 
                v_lin = -0.1
                v_ang = 0.5
            else:
                v_lin = 0.5
                if random.random() < 0.1: v_ang = random.uniform(-0.3, 0.3)

        self._sim.setJointTargetVelocity(self._left_motor, v_lin - v_ang)
        self._sim.setJointTargetVelocity(self._right_motor, v_lin + v_ang)

        # --- 3. PUBLICAR ODOM & TF ---
        current_time = self.get_clock().now()
        pos = self._sim.getObjectPosition(self._robot_handle, -1)
        ori = self._sim.getObjectOrientation(self._robot_handle, -1)
        
        # TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion_from_euler(0, 0, ori[2])
        self.tf_broadcaster.sendTransform(t)

        # Odom (AQUI ESTABA EL ERROR)
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # CORRECCIÓN: Asignamos campo a campo para evitar error de tipos (Vector3 vs Point)
        odom.pose.pose.position.x = t.transform.translation.x
        odom.pose.pose.position.y = t.transform.translation.y
        odom.pose.pose.position.z = t.transform.translation.z
        
        odom.pose.pose.orientation = t.transform.rotation
        self.odom_pub.publish(odom)

        # --- 4. PUBLICAR LÁSER ---
        scan = LaserScan()
        scan.header.stamp = current_time.to_msg()
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi / 2
        scan.angle_max = math.pi / 2
        scan.angle_increment = math.pi / 16
        scan.range_min = 0.1
        scan.range_max = 5.0
        scan.ranges = [5.0] * 16
        
        for i, s in enumerate(self._sonar_handles):
            res, dist, _, _, _ = self._sim.readProximitySensor(s)
            if res > 0:
                scan.ranges[15-i] = dist
        
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = PioneerP3dxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()