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
        
        # Parámetros configurables (ajustados según bug2smoke.py)
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('goal_tolerance', 0.3)  
        self.declare_parameter('m_line_tolerance', 0.2)  
        self.declare_parameter('obstacle_threshold', 0.2)  
        self.declare_parameter('front_obstacle_threshold', 0.2) 
        self.declare_parameter('wall_distance', 0.2) 
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('wheel_separation', 0.4)

        
        # Obtener parámetros
        self.control_rate = self.get_parameter('control_rate').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.m_line_tolerance = self.get_parameter('m_line_tolerance').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.front_obstacle_threshold = self.get_parameter('front_obstacle_threshold').value
        self.target_wall_distance = self.get_parameter('wall_distance').value
        self.max_range = self.get_parameter('max_range').value
        self.wheel_separation = self.get_parameter('wheel_separation').value

        
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
        q = self.current_pose.pose.orientation
        robot_yaw = self._yaw_from_quat(q)
        angle_to_goal = self.angle_to_goal()

        # Grupos de sensores (ajusta si tu mapeo real difiere)
        front_ids = [3, 4, 5, 6]       # cono frontal
        left_ids  = [1, 2, 3, 4]       # lateral-izq + frontal-izq
        right_ids = [5, 6, 7, 8]       # lateral-der + frontal-der

        d_front = self._min_of(front_ids, default=float('inf'))
        d_left  = self._min_of(left_ids,  default=float('inf'))
        d_right = self._min_of(right_ids, default=float('inf'))

        # 1) Condición Bug2 clásica: obstáculo frontal -> wall follow
        if d_front < self.front_obstacle_threshold:
            self.transition_to_wall_following()
            return self.wall_following_behavior()

        # 2) Control de velocidad lineal según clearance (incluye laterales)
        #    - si vas "encajonado", baja el lineal para no rozar.
        clearance = min(d_front, d_left, d_right)
        # Rampa simple (tú defines números; estos funcionan bien como punto de partida)
        if clearance < 0.35:
            forward_speed = self.max_linear_speed * 0.25
        elif clearance < 0.60:
            forward_speed = self.max_linear_speed * 0.45
        else:
            forward_speed = self.max_linear_speed * 0.75

        # 3) Componente de giro hacia la meta
        k_goal = 1.5
        angular_speed = k_goal * angle_to_goal

        # 4) Componente reactivo lateral (repulsión)
        #    Si un lado está muy cerca, gira para abrirte.
        side_threshold = 0.30
        k_avoid = 0.9

        avoid = 0.0
        if d_left < side_threshold:
            # obstáculo a la izquierda -> gira a la derecha (angular negativo o positivo según convención)
            # En ROS, angular.z > 0 suele ser giro CCW (izquierda). Ajustamos para girar a la derecha:
            avoid -= k_avoid * (1.0/max(d_left, 1e-3) - 1.0/side_threshold)
        if d_right < side_threshold:
            # obstáculo a la derecha -> gira a la izquierda
            avoid += k_avoid * (1.0/max(d_right, 1e-3) - 1.0/side_threshold)

        angular_speed += avoid

        # 5) Saturaciones
        angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))

        return forward_speed, angular_speed

    
    def transition_to_wall_following(self):
        """Transición a estado de seguimiento de pared (mejorado según bug2smoke.py)"""
        self.current_state = self.WALL_FOLLOWING
        
        # Guardar hit point
        self.hit_point = [
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y
        ]
        self.hit_distance_to_goal = self.distance_to_goal()
        
        # Determinar lado de seguimiento (algoritmo mejorado)
        left_distances = []
        right_distances = []
        
        for sensor_id, reading in self.sonar_readings.items():
            if reading.range < self.max_range:  # Válido
                angle = self.sensor_angles.get(sensor_id, 0)
                if -90 <= angle < 0:
                    left_distances.append(reading.range)
                elif 0 < angle <= 90:
                    right_distances.append(reading.range)
        
        avg_left = sum(left_distances) / len(left_distances) if left_distances else 0
        avg_right = sum(right_distances) / len(right_distances) if right_distances else 0
        
        # Elegir el lado con MÁS espacio libre
        self.wall_following_side = 'right' if avg_right > avg_left else 'left'
        
        self.get_logger().info(
            f'Bug2: Obstáculo detectado - Wall following ({self.wall_following_side})'
        )
        self.get_logger().info(
            f'Espacios: Izq={avg_left:.2f}m, Der={avg_right:.2f}m'
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
        Verifica las condiciones Bug2 para dejar wall following (mejorado)
        1. Estar en la M-line
        2. Estar más cerca de la meta que en el hit point (con margen)
        3. Camino directo a meta libre
        """
        if self.hit_point is None:
            return False
        
        # Condición 1: Estar en M-line
        if not self.is_on_m_line():
            return False
        
        # Condición 2: Más cerca que en hit point (con margen de 0.1m)
        current_distance = self.distance_along_m_line_to_goal()
        if current_distance >= self.hit_distance_to_goal - 0.1:
            return False
        
        # Condición 3: Camino libre usando detección mejorada
        position = [self.current_pose.pose.position.x, self.current_pose.pose.position.y]
        q = self.current_pose.pose.orientation
        orientation = 2 * math.atan2(q.z, q.w)
        angle_to_goal = self.angle_to_goal()
        
        # Verificar sensores frontales
        front_sensors = [2, 3, 4, 5, 6, 7]
        for sensor_id in front_sensors:
            if sensor_id in self.sonar_readings:
                reading = self.sonar_readings[sensor_id]
                if reading.range < self.max_range:
                    sensor_angle_deg = self.sensor_angles.get(sensor_id, 0)
                    sensor_angle = math.radians(sensor_angle_deg)
                    angle_diff = abs(sensor_angle - angle_to_goal)
                    
                    if angle_diff > math.pi:
                        angle_diff = 2 * math.pi - angle_diff
                    
                    # Si el sensor apunta hacia la meta
                    if angle_diff < math.pi/6:  # ±30 grados
                        if reading.range < self.obstacle_threshold:
                            return False
        
        return True
    
    def wall_following_control(self):
        """
        Control de seguimiento de pared basado en bug2smoke.py:
        - Primero calcula velocidades de rueda (left_speed, right_speed)
        - Luego las convierte a (linear, angular) para /cmd_vel
        """
        # Seleccionar sensores según lado
        if self.wall_following_side == 'right':
            wall_sensors = [6, 7, 8]
            front_sensors = [4, 5, 6]   # añade esquina derecha
        else:
            wall_sensors = [1, 2, 3]
            front_sensors = [3, 4, 5]   # añade esquina izquierda
        
        # Obtener distancias
        wall_distances = [self._valid_range(s) for s in wall_sensors]
        wall_distances = [d for d in wall_distances if d is not None]

        front_distances = [self._valid_range(s) for s in front_sensors]
        front_distances = [d for d in front_distances if d is not None]

        
        avg_wall_dist = sum(wall_distances) / len(wall_distances) if wall_distances else 1.5
        min_front_dist = min(front_distances) if front_distances else 1.5
        
        # Velocidad base (igual que bug2smoke)
        base_speed = 1.3
        
        # --- Cálculo de velocidades de rueda, como en el script original ---
        if min_front_dist < 0.5:
            # Obstáculo frontal: giro fuerte
            if self.wall_following_side == 'right':
                # En bug2smoke: return -0.5, base_speed
                left_speed = -0.5
                right_speed = base_speed
            else:
                # En bug2smoke: return base_speed, -0.5
                left_speed = base_speed
                right_speed = -0.5
        
        elif avg_wall_dist < self.target_wall_distance - 0.15:
            # Muy cerca de la pared: alejarse
            if self.wall_following_side == 'right':
                # return base_speed * 0.6, base_speed * 1.3
                left_speed = base_speed * 0.6
                right_speed = base_speed * 1.3
            else:
                # return base_speed * 1.3, base_speed * 0.6
                left_speed = base_speed * 1.3
                right_speed = base_speed * 0.6
        
        elif avg_wall_dist > self.target_wall_distance + 0.15:
            # Muy lejos de la pared: acercarse
            if self.wall_following_side == 'right':
                # return base_speed * 1.3, base_speed * 0.6
                left_speed = base_speed * 1.3
                right_speed = base_speed * 0.6
            else:
                # return base_speed * 0.6, base_speed * 1.3
                left_speed = base_speed * 0.6
                right_speed = base_speed * 1.3
        
        else:
            # Distancia correcta: avanzar recto
            left_speed = base_speed
            right_speed = base_speed
        
        # --- Conversión de velocidades de rueda a (linear, angular) ---
        linear = 0.5 * (left_speed + right_speed)
        angular = (right_speed - left_speed) / self.wheel_separation
        
        # Limitar velocidad angular a max_angular_speed para evitar oscilaciones locas
        if angular > self.max_angular_speed:
            angular = self.max_angular_speed
        elif angular < -self.max_angular_speed:
            angular = -self.max_angular_speed
        
        return linear, angular

    
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


    def _yaw_from_quat(self, q):
        # Fórmula estándar yaw de quaternion (robusta aunque x,y no sean 0)
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def _valid_range(self, sensor_id):
        """Devuelve distancia válida (float) o None."""
        r = self.sonar_readings.get(sensor_id)
        if r is None:
            return None
        # Validación por contrato del mensaje Range
        if math.isfinite(r.range) and r.min_range <= r.range <= r.max_range:
            return r.range
        return None

    def _min_of(self, sensor_ids, default=float('inf')):
        vals = []
        for sid in sensor_ids:
            d = self._valid_range(sid)
            if d is not None:
                vals.append(d)
        return min(vals) if vals else default



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