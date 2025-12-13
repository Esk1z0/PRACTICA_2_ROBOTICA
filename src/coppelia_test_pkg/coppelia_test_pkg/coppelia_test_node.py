import math
import sys
import numpy as np
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
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
        self.declare_parameter('coppelia_port', 23000)
        host = self.get_parameter('coppelia_host').get_parameter_value().string_value
        port = self.get_parameter('coppelia_port').get_parameter_value().integer_value

        self.get_logger().info(f'Conectando a {host}:{port}...')
        try:
            self._client = RemoteAPIClient(host, port)
            self._sim = self._client.require('sim')
            self.get_logger().info('¡Conectado!')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            sys.exit(1)

        # Handles
        self._left_motor = self._sim.getObject('/Pioneer_p3dx_leftMotor')
        self._right_motor = self._sim.getObject('/Pioneer_p3dx_rightMotor')
        self._robot_handle = self._sim.getObject('/Pioneer_p3dx')
        
        # Sonares (16 unidades)
        self._sonar_handles = [self._sim.getObject(f'/Pioneer_p3dx_ultrasonicSensor{i}') for i in range(1, 17)]

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Variables
        self.timer = self.create_timer(0.1, self.control_loop)
        self.turn_timer = 0 # Contador para girar un rato si hay obstáculo

    def control_loop(self):
        # --- 1. LECTURA DE SENSORES ---
        # El Pioneer tiene 8 sonares frontales (1-8) y 8 traseros (9-16).
        # Los índices en la lista son 0-7 (frente) y 8-15 (atrás).
        # Los centrales frontales son los índices 3 y 4.
        
        min_dist_front = 999.0
        
        # Leemos solo los sensores frontales (índices 1 a 6 para tener un cono)
        for i in range(1, 7): 
            res, dist, _, _, _ = self._sim.readProximitySensor(self._sonar_handles[i])
            if res > 0:
                if dist < min_dist_front:
                    min_dist_front = dist

        # --- 2. LÓGICA DE MOVIMIENTO (EXPLORACIÓN) ---
        v_lin = 2.0  # Velocidad crucero alta
        v_ang = 0.0

        # Si estamos girando por un obstáculo anterior, seguimos girando un poco más
        if self.turn_timer > 0:
            v_lin = 0.0
            v_ang = 1.0 # Girar derecha
            self.turn_timer -= 1
        else:
            # Si detectamos algo muy cerca (menos de 0.5 metros)
            if min_dist_front < 0.5:
                self.get_logger().info(f'¡OBSTÁCULO A {min_dist_front:.2f}m! INICIANDO GIRO.')
                self.turn_timer = 15 # Girar durante 1.5 segundos (15 ciclos)
                v_lin = 0.0
                v_ang = 1.0
            else:
                # Camino libre -> AVANZAR
                v_lin = 2.0
                v_ang = 0.0
                # A veces girar un pelín aleatoriamente para no ir en línea recta perfecta
                if random.random() < 0.05: 
                    v_ang = random.uniform(-0.5, 0.5)

        # Enviar a motores
        self._sim.setJointTargetVelocity(self._left_motor, v_lin - v_ang)
        self._sim.setJointTargetVelocity(self._right_motor, v_lin + v_ang)

        # --- 3. PUBLICAR ODOMETRÍA Y TF (NECESARIO PARA EL MAPA) ---
        current_time = self.get_clock().now()
        pos = self._sim.getObjectPosition(self._robot_handle, -1)
        ori = self._sim.getObjectOrientation(self._robot_handle, -1)
        
        # TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion_from_euler(0, 0, ori[2])
        self.tf_broadcaster.sendTransform(t)

        # Odom
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.orientation = t.transform.rotation
        self.odom_pub.publish(odom)

        # --- 4. PUBLICAR LÁSER VIRTUAL (SIMULADO) ---
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
                idx = 15 - i
                if 0 <= idx < 16:
                    scan.ranges[idx] = dist
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