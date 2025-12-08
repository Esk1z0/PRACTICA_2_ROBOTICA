#!/usr/bin/env python3
"""
Nodo controlador Bug2
Responsabilidades:
- Implementar la lógica del algoritmo Bug2
- Leer pose y sensores desde topics
- Publicar comandos de velocidad
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
import math


class Bug2ControllerNode(Node):
    """
    Implementación del algoritmo Bug2 en ROS2
    
    SUSCRIBE:
        - /robot/pose (PoseStamped): Posición actual del robot
        - /robot/sonar_* (Range): Lecturas de sensores
        - /goal (PoseStamped): Meta a alcanzar
        
    PUBLICA:
        - /cmd_vel (Twist): Comandos de velocidad
    """
    
    def __init__(self):
        super().__init__('bug2_controller_node')
        
        # Parámetros configurables
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('max_linear_speed', 1.5)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('goal_tolerance', 1.5)
        self.declare_parameter('m_line_tolerance', 0.3)
        self.declare_parameter('obstacle_threshold', 0.4)
        self.declare_parameter('wall_distance', 0.5)
        
        # Obtener parámetros
        self.control_rate = self.get_parameter('control_rate').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.m_line_tolerance = self.get_parameter('m_line_tolerance').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.target_wall_distance = self.get_parameter('wall_distance').value
        
        # Estado del robot
        self.current_pose = None
        self.sonar_readings = {}  # {sensor_id: Range}
        self.goal_pose = None
        
        # Estados Bug2
        self.MOTION_TO_GOAL = 0
        self.WALL_FOLLOWING = 1
        self.current_state = self.MOTION_TO_GOAL
        
        # Variables Bug2
        self.start_point = None
        self.hit_point = None
        self.hit_distance_to_goal = None
        self.wall_following_side = 'right'
        
        # Mapeo de ángulos de sensores (del script original)
        self.sensor_angles = {
            1: -90, 2: -50, 3: -30, 4: -10,
            5: 10, 6: 30, 7: 50, 8: 90,
            9: 90, 10: 130, 11: 150, 12: 170,
            13: -170, 14: -150, 15: -130, 16: -90
        }
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot/pose',
            self.pose_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal',
            self.goal_callback,
            10
        )
        
        # Suscribirse a todos los sonares
        for i in range(1, 17):
            self.create_subscription(
                Range,
                f'/robot/sonar_{i}',
                lambda msg, sensor_id=i: self.sonar_callback(msg, sensor_id),
                10
            )
        
        # Timer para ejecutar el control
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )
        
        self.get_logger().info('Nodo Bug2 Controller iniciado')
        self.get_logger().info('Esperando pose del robot y meta...')
    
    def pose_callback(self, msg):
        """Actualiza la pose actual del robot"""
        self.current_pose = msg
        
        # Establecer punto inicial si no existe
        if self.start_point is None:
            self.start_point = [msg.pose.position.x, msg.pose.position.y]
            self.get_logger().info(
                f'Punto inicial establecido: ({self.start_point[0]:.2f}, {self.start_point[1]:.2f})'
            )
    
    def sonar_callback(self, msg, sensor_id):
        """Almacena lectura de un sensor sonar"""
        self.sonar_readings[sensor_id] = msg
    
    def goal_callback(self, msg):
        """Actualiza la meta"""
        self.goal_pose = msg
        
        # Resetear estado Bug2 cuando hay nueva meta
        if self.current_pose is not None:
            self.start_point = [
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y
            ]
            self.current_state = self.MOTION_TO_GOAL
            self.hit_point = None
            self.hit_distance_to_goal = None
            
            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y
            self.get_logger().info(f'Nueva meta recibida: ({goal_x:.2f}, {goal_y:.2f})')
    
    def control_loop(self):
        """
        Loop principal de control
        Se ejecuta a la frecuencia definida (por defecto 20 Hz)
        """
        # Verificar que tenemos datos necesarios
        if self.current_pose is None:
            return
        
        if self.goal_pose is None:
            # Sin meta, no hacer nada
            self.publish_velocity(0.0, 0.0)
            return
        
        # Verificar si llegamos a la meta
        distance_to_goal = self.distance_to_goal()
        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info(
                f'¡Meta alcanzada! Distancia: {distance_to_goal:.2f}m'
            )
            self.publish_velocity(0.0, 0.0)
            # Aquí podrías publicar en un topic de "goal_reached" 
            # para que goal_manager genere nueva meta
            return
        
        # Ejecutar comportamiento según estado Bug2
        if self.current_state == self.MOTION_TO_GOAL:
            linear_vel, angular_vel = self.motion_to_goal_behavior()
        else:  # WALL_FOLLOWING
            linear_vel, angular_vel = self.wall_following_behavior()
        
        # Publicar velocidades
        self.publish_velocity(linear_vel, angular_vel)
    
    def motion_to_goal_behavior(self):
        """
        Comportamiento Motion-to-Goal de Bug2
        Retorna: (linear_velocity, angular_velocity)
        """
        # Calcular ángulo hacia la meta
        angle_to_goal = self.angle_to_goal()
        
        # Verificar si el camino está libre
        if self.is_path_clear(angle_to_goal):
            # Moverse hacia la meta
            forward_speed = self.max_linear_speed * 0.8
            angular_speed = angle_to_goal * 1.5
            
            # Limitar velocidad angular
            angular_speed = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, angular_speed)
            )
            
            return forward_speed, angular_speed
        
        else:
            # Obstáculo detectado - cambiar a wall following
            self.transition_to_wall_following()
            return self.wall_following_behavior()
    
    def transition_to_wall_following(self):
        """Transición a estado de seguimiento de pared"""
        self.current_state = self.WALL_FOLLOWING
        
        # Guardar hit point
        self.hit_point = [
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y
        ]
        self.hit_distance_to_goal = self.distance_to_goal()
        
        # Determinar lado de seguimiento
        left_clear = self.get_clearance_in_direction('left')
        right_clear = self.get_clearance_in_direction('right')
        self.wall_following_side = 'left' if left_clear > right_clear else 'right'
        
        self.get_logger().info(
            f'Bug2: Obstáculo detectado - Wall following ({self.wall_following_side})'
        )
        self.get_logger().info(
            f'Hit point: ({self.hit_point[0]:.2f}, {self.hit_point[1]:.2f}), '
            f'distancia: {self.hit_distance_to_goal:.2f}m'
        )
    
    def wall_following_behavior(self):
        """
        Comportamiento Wall-Following de Bug2
        Retorna: (linear_velocity, angular_velocity)
        """
        # Verificar condiciones de salida Bug2
        if self.can_leave_wall_following():
            self.current_state = self.MOTION_TO_GOAL
            self.get_logger().info('Bug2: Volviendo a motion-to-goal (en M-line y más cerca)')
            return self.motion_to_goal_behavior()
        
        # Continuar siguiendo la pared
        return self.wall_following_control()
    
    def can_leave_wall_following(self):
        """
        Verifica las condiciones Bug2 para dejar wall following
        1. Estar en la M-line
        2. Estar más cerca de la meta que en el hit point
        3. Camino directo a meta libre
        """
        if self.hit_point is None:
            return False
        
        # Condición 1: Estar en M-line
        if not self.is_on_m_line():
            return False
        
        # Condición 2: Más cerca que en hit point
        current_distance = self.distance_along_m_line_to_goal()
        if current_distance >= self.hit_distance_to_goal:
            return False
        
        # Condición 3: Camino libre
        angle_to_goal = self.angle_to_goal()
        if not self.is_path_clear(angle_to_goal):
            return False
        
        return True
    
    def wall_following_control(self):
        """
        Control básico de seguimiento de pared
        Retorna: (linear_velocity, angular_velocity)
        """
        # Seleccionar sensores según lado
        if self.wall_following_side == 'right':
            wall_sensors = [6, 7, 8]  # Sensores derechos
            front_sensors = [4, 5]
        else:
            wall_sensors = [1, 2, 3]  # Sensores izquierdos
            front_sensors = [4, 5]
        
        # Obtener distancias
        wall_distances = [
            self.sonar_readings[s].range 
            for s in wall_sensors 
            if s in self.sonar_readings and self.sonar_readings[s].range < 5.0
        ]
        
        front_distances = [
            self.sonar_readings[s].range
            for s in front_sensors
            if s in self.sonar_readings and self.sonar_readings[s].range < 5.0
        ]
        
        avg_wall_dist = sum(wall_distances) / len(wall_distances) if wall_distances else 1.5
        min_front_dist = min(front_distances) if front_distances else 1.5
        
        # Velocidad base
        base_speed = 1.2
        
        # Obstáculo frontal - girar
        if min_front_dist < self.obstacle_threshold:
            angular_vel = 1.0 if self.wall_following_side == 'right' else -1.0
            return 0.3, angular_vel
        
        # Control lateral proporcional
        error = avg_wall_dist - self.target_wall_distance
        kp = 2.0  # Ganancia proporcional
        
        if self.wall_following_side == 'right':
            angular_vel = -kp * error  # Negativo para corregir hacia la derecha
        else:
            angular_vel = kp * error   # Positivo para corregir hacia la izquierda
        
        # Limitar velocidad angular
        angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, angular_vel))
        
        return base_speed, angular_vel
    
    # ========== FUNCIONES AUXILIARES ==========
    
    def distance_to_goal(self):
        """Calcula distancia euclidiana a la meta"""
        if self.goal_pose is None or self.current_pose is None:
            return float('inf')
        
        dx = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def angle_to_goal(self):
        """Calcula ángulo hacia la meta (relativo a orientación actual)"""
        if self.goal_pose is None or self.current_pose is None:
            return 0.0
        
        # Ángulo hacia la meta en coordenadas globales
        dx = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
        goal_angle = math.atan2(dy, dx)
        
        # Orientación actual del robot (de quaternion a yaw)
        q = self.current_pose.pose.orientation
        robot_yaw = 2 * math.atan2(q.z, q.w)
        
        # Diferencia de ángulo
        angle_diff = goal_angle - robot_yaw
        
        # Normalizar a [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        return angle_diff
    
    def is_on_m_line(self):
        """Verifica si la posición actual está en la M-line"""
        if self.start_point is None or self.goal_pose is None or self.current_pose is None:
            return False
        
        x1, y1 = self.start_point
        x2 = self.goal_pose.pose.position.x
        y2 = self.goal_pose.pose.position.y
        x0 = self.current_pose.pose.position.x
        y0 = self.current_pose.pose.position.y
        
        # Longitud de la línea
        line_length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        if line_length < 0.1:
            return True
        
        # Distancia perpendicular a la M-line
        perp_dist = abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1) / line_length
        
        return perp_dist <= self.m_line_tolerance
    
    def distance_along_m_line_to_goal(self):
        """Distancia a lo largo de la M-line hacia la meta"""
        if self.start_point is None or self.goal_pose is None or self.current_pose is None:
            return float('inf')
        
        x1, y1 = self.start_point
        x2 = self.goal_pose.pose.position.x
        y2 = self.goal_pose.pose.position.y
        x0 = self.current_pose.pose.position.x
        y0 = self.current_pose.pose.position.y
        
        # Vector de la línea
        line_vec = [x2 - x1, y2 - y1]
        point_vec = [x0 - x1, y0 - y1]
        
        line_length = math.sqrt(line_vec[0]**2 + line_vec[1]**2)
        
        if line_length < 0.1:
            return self.distance_to_goal()
        
        # Proyección
        projection = (point_vec[0]*line_vec[0] + point_vec[1]*line_vec[1]) / (line_length**2)
        
        # Punto proyectado
        proj_x = x1 + projection * line_vec[0]
        proj_y = y1 + projection * line_vec[1]
        
        # Distancia del punto proyectado a la meta
        return math.sqrt((x2 - proj_x)**2 + (y2 - proj_y)**2)
    
    def is_path_clear(self, target_angle):
        """Verifica si el camino en una dirección está libre"""
        # Sensores frontales
        front_sensors = [4, 5]
        
        for sensor_id in front_sensors:
            if sensor_id in self.sonar_readings:
                distance = self.sonar_readings[sensor_id].range
                if distance < self.obstacle_threshold:
                    return False
        
        # Verificar sensores en dirección del objetivo
        for sensor_id, reading in self.sonar_readings.items():
            if reading.range >= 5.0:  # Fuera de rango
                continue
            
            sensor_angle_deg = self.sensor_angles.get(sensor_id, 0)
            sensor_angle = math.radians(sensor_angle_deg)
            
            angle_diff = abs(sensor_angle - target_angle)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            # Si el sensor apunta hacia el objetivo (±45°)
            if angle_diff < math.pi/4:
                if reading.range < self.obstacle_threshold:
                    return False
        
        return True
    
    def get_clearance_in_direction(self, direction):
        """Calcula el espacio libre en una dirección (left/right)"""
        if direction == 'left':
            sensors = [1, 2, 3, 4]
        else:
            sensors = [5, 6, 7, 8]
        
        distances = []
        for sensor_id in sensors:
            if sensor_id in self.sonar_readings:
                dist = self.sonar_readings[sensor_id].range
                if dist < 5.0:
                    distances.append(dist)
        
        return sum(distances) / len(distances) if distances else 0.0
    
    def publish_velocity(self, linear, angular):
        """Publica comando de velocidad"""
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)
    
    def shutdown(self):
        """Limpieza al cerrar"""
        self.get_logger().info('Deteniendo robot...')
        self.publish_velocity(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = Bug2ControllerNode()
    
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