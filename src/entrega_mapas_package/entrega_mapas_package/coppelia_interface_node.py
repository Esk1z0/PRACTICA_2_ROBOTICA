#!/usr/bin/env python3
"""
Nodo de interfaz con CoppeliaSim
Responsabilidades:
- Conectar con CoppeliaSim via ZMQ
- Leer sensores y publicar en topics ROS2
- Escuchar comandos de velocidad y aplicarlos a los motores
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Pose
from sensor_msgs.msg import Range
from std_msgs.msg import Header
import time
import math

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:
    print("ERROR: No se puede importar coppeliasim_zmqremoteapi_client")
    print("Instala con: pip install coppeliasim-zmqremoteapi-client")

# Mensaje custom para array de sonares (simplificado con Range)
# Para producción podrías crear un .msg custom

class CoppeliaInterfaceNode(Node):
    """
    Nodo que hace de puente entre CoppeliaSim y ROS2
    
    PUBLICA:
        - /robot/pose (PoseStamped): Posición y orientación del robot
        - /robot/sonar_* (Range): Un topic por cada sensor sonar
        
    SUSCRIBE:
        - /cmd_vel (Twist): Comandos de velocidad para el robot
    """
    
    def __init__(self):
        super().__init__('coppelia_interface_node')
        
        # Parámetros configurables
        self.declare_parameter('robot_name', 'Pioneer_p3dx')
        self.declare_parameter('update_rate', 20.0)  # Hz
        self.declare_parameter('max_speed', 2.0)
        
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('max_speed').value
        update_rate = self.get_parameter('update_rate').value
        
        # Variables de CoppeliaSim
        self.client = None
        self.sim = None
        self.robot_handle = None
        self.left_motor = None
        self.right_motor = None
        self.script_handle = None
        
        # Mapeo de ángulos de sensores (del script original)
        self.sensor_angles = {
            1: -90, 2: -50, 3: -30, 4: -10, 
            5: 10, 6: 30, 7: 50, 8: 90,
            9: 90, 10: 130, 11: 150, 12: 170, 
            13: -170, 14: -150, 15: -130, 16: -90
        }
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/robot/pose', 10)
        
        # Crear publisher para cada sonar
        self.sonar_pubs = {}
        for i in range(1, 17):  # 16 sensores
            topic_name = f'/robot/sonar_{i}'
            self.sonar_pubs[i] = self.create_publisher(Range, topic_name, 10)
        
        # Subscriber para comandos de velocidad
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Timer para actualizar sensores y pose
        self.timer = self.create_timer(1.0 / update_rate, self.update_callback)
        
        # Conectar con CoppeliaSim
        self.connect_to_coppelia()
        
        self.get_logger().info('Nodo CoppeliaSim Interface iniciado')
    
    def connect_to_coppelia(self):
        """Establece conexión con CoppeliaSim"""
        try:
            self.get_logger().info('Conectando con CoppeliaSim...')
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            
            # Obtener handles del robot
            self.robot_handle = self.sim.getObject(f'/{self.robot_name}')
            self.left_motor = self.sim.getObject(f'/{self.robot_name}_leftMotor')
            self.right_motor = self.sim.getObject(f'/{self.robot_name}_rightMotor')
            self.script_handle = self.sim.getScript(
                self.sim.scripttype_childscript, 
                self.robot_handle
            )
            
            self.get_logger().info('✓ Conectado a CoppeliaSim correctamente')
            return True
            
        except Exception as e:
            self.get_logger().error(f'✗ Error conectando a CoppeliaSim: {e}')
            self.get_logger().error('Asegúrate de que CoppeliaSim está ejecutándose')
            return False
    
    def update_callback(self):
        """
        Callback del timer: actualiza y publica pose y sensores
        Se ejecuta a la frecuencia definida (por defecto 20 Hz)
        """
        if self.sim is None:
            return
        
        try:
            # Publicar pose del robot
            self.publish_robot_pose()
            
            # Publicar lecturas de sensores
            self.publish_sonar_readings()
            
        except Exception as e:
            self.get_logger().warn(f'Error en update: {e}')
    
    def publish_robot_pose(self):
        """Obtiene pose del robot de CoppeliaSim y la publica"""
        try:
            # Obtener posición (x, y, z)
            position = self.sim.getObjectPosition(self.robot_handle, -1)
            
            # Obtener orientación (roll, pitch, yaw)
            orientation = self.sim.getObjectOrientation(self.robot_handle, -1)
            
            # Crear mensaje PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header = Header()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'world'
            
            # Posición
            pose_msg.pose.position.x = position[0]
            pose_msg.pose.position.y = position[1]
            pose_msg.pose.position.z = position[2]
            
            # Orientación (convertir de Euler a Quaternion)
            # Para simplificar, solo usamos yaw (rotación en Z)
            yaw = orientation[2]
            pose_msg.pose.orientation.z = math.sin(yaw / 2.0)
            pose_msg.pose.orientation.w = math.cos(yaw / 2.0)
            
            # Publicar
            self.pose_pub.publish(pose_msg)
            
        except Exception as e:
            self.get_logger().warn(f'Error publicando pose: {e}')
    
    def publish_sonar_readings(self):
        """Obtiene lecturas de sonares y las publica"""
        try:
            # Llamar función del script de CoppeliaSim
            readings_array = self.sim.callScriptFunction(
                'getAllSonars', 
                self.script_handle
            )
            
            # Publicar cada sensor individualmente
            for i, distance in enumerate(readings_array, 1):
                range_msg = Range()
                range_msg.header = Header()
                range_msg.header.stamp = self.get_clock().now().to_msg()
                range_msg.header.frame_id = f'sonar_{i}'
                
                # Configuración del sensor
                range_msg.radiation_type = Range.ULTRASOUND
                range_msg.field_of_view = 0.26  # ~15 grados en radianes
                range_msg.min_range = 0.0
                range_msg.max_range = 5.0
                range_msg.range = distance
                
                # Publicar
                self.sonar_pubs[i].publish(range_msg)
                
        except Exception as e:
            self.get_logger().warn(f'Error publicando sonares: {e}')
    
    def cmd_vel_callback(self, msg):
        """
        Callback cuando recibe comandos de velocidad
        Convierte Twist (linear.x, angular.z) a velocidades de ruedas
        
        Args:
            msg (Twist): linear.x = velocidad lineal (m/s)
                        angular.z = velocidad angular (rad/s)
        """
        try:
            # Extraer velocidades del mensaje Twist
            linear_vel = msg.linear.x
            angular_vel = msg.angular.z
            
            # Modelo diferencial: convertir a velocidades de ruedas
            # Para un robot diferencial:
            # v_left = v - (angular * wheelbase / 2)
            # v_right = v + (angular * wheelbase / 2)
            
            # Simplificado (ajusta según tu robot)
            left_speed = linear_vel - angular_vel
            right_speed = linear_vel + angular_vel
            
            # Limitar velocidades
            left_speed = max(-self.max_speed, min(self.max_speed, left_speed))
            right_speed = max(-self.max_speed, min(self.max_speed, right_speed))
            
            # Aplicar a motores
            self.sim.setJointTargetVelocity(self.left_motor, left_speed)
            self.sim.setJointTargetVelocity(self.right_motor, right_speed)
            
        except Exception as e:
            self.get_logger().warn(f'Error aplicando velocidades: {e}')
    
    def shutdown(self):
        """Limpieza al cerrar el nodo"""
        self.get_logger().info('Deteniendo motores...')
        try:
            if self.sim is not None:
                self.sim.setJointTargetVelocity(self.left_motor, 0.0)
                self.sim.setJointTargetVelocity(self.right_motor, 0.0)
        except:
            pass
        self.get_logger().info('Nodo CoppeliaSim Interface cerrado')


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