#!/usr/bin/env python3
"""
Nodo gestor de metas
Responsabilidades:
- Generar metas válidas (dentro del mapa, fuera de obstáculos)
- Publicar metas en /goal
- Detectar cuando se alcanza una meta y generar nueva
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import random
import math


class GoalManagerNode(Node):
    """
    Gestor de metas para el robot
    
    PUBLICA:
        - /goal (PoseStamped): Meta actual del robot
    
    SUSCRIBE:
        - /robot/pose (PoseStamped): Para detectar cuando se alcanza meta
    """
    
    def __init__(self):
        super().__init__('goal_manager_node')
        
        # Parámetros configurables
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('min_goal_distance', 2.0)
        self.declare_parameter('map_min_x', -4.0)
        self.declare_parameter('map_max_x', 4.0)
        self.declare_parameter('map_min_y', -4.0)
        self.declare_parameter('map_max_y', 4.0)
        self.declare_parameter('map_buffer', 0.8)
        self.declare_parameter('auto_generate', True)  # Generar nuevas metas automáticamente
        
        # Obtener parámetros
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.map_limits = {
            'min_x': self.get_parameter('map_min_x').value,
            'max_x': self.get_parameter('map_max_x').value,
            'min_y': self.get_parameter('map_min_y').value,
            'max_y': self.get_parameter('map_max_y').value,
            'buffer': self.get_parameter('map_buffer').value
        }
        self.auto_generate = self.get_parameter('auto_generate').value
        
        # Estado
        self.current_pose = None
        self.current_goal = None
        self.goal_reached = False

        # Timer para nueva meta (simulación de one-shot)
        self.new_goal_timer = None
        
        # Obstáculos conocidos (placeholder)
        self.obstacles = self.load_obstacles()
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal', 10)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot/pose',
            self.pose_callback,
            10
        )
        
        # Timer para verificar si se alcanzó la meta
        self.check_timer = self.create_timer(1.0, self.check_goal_reached)
        
        self.get_logger().info('Nodo Goal Manager iniciado')
        self.get_logger().info('Esperando pose del robot para generar primera meta...')
    
    def load_obstacles(self):
        """
        Carga información de obstáculos.
        En una versión más avanzada, esto vendría de un topic o servicio.
        """
        # Por ahora retornamos una lista vacía y confiamos en los sensores
        return []
    
    def pose_callback(self, msg):
        """Actualiza pose actual del robot"""
        was_none = self.current_pose is None
        self.current_pose = msg
        
        # Generar primera meta cuando recibimos primera pose
        if was_none and self.auto_generate:
            self.get_logger().info('Primera pose recibida, generando meta inicial...')
            self.generate_and_publish_goal()
    
    def check_goal_reached(self):
        """Verifica si el robot alcanzó la meta actual"""
        if self.current_pose is None or self.current_goal is None:
            return
        
        # Calcular distancia a la meta
        distance = self.distance_to_goal()
        
        if distance < self.goal_tolerance and not self.goal_reached:
            self.goal_reached = True
            self.get_logger().info(
                f'¡Meta alcanzada! Distancia: {distance:.2f}m'
            )
            
            # Generar nueva meta si está en modo automático
            if self.auto_generate:
                self.get_logger().info('Generando nueva meta en 2 segundos...')
                
                # Crear timer solo si no hay uno ya pendiente
                if self.new_goal_timer is None:
                    self.new_goal_timer = self.create_timer(
                        2.0,
                        self.generate_new_goal_callback
                    )
    
    def generate_new_goal_callback(self):
        """Callback para generar nueva meta (usado con timer one-shot simulado)"""
        # Resetear flag de meta alcanzada
        self.goal_reached = False

        # Generar y publicar nueva meta
        self.generate_and_publish_goal()

        # Cancelar el timer para que sea one-shot
        if self.new_goal_timer is not None:
            self.new_goal_timer.cancel()
            self.new_goal_timer = None
    
    def generate_and_publish_goal(self):
        """Genera una meta válida y la publica"""
        if self.current_pose is None:
            self.get_logger().warn('No hay pose del robot, esperando...')
            return
        
        robot_pos = [
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y
        ]
        
        goal = self.generate_valid_goal(robot_pos)
        
        if goal is not None:
            # Crear mensaje PoseStamped
            goal_msg = PoseStamped()
            goal_msg.header = Header()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'world'
            
            goal_msg.pose.position.x = goal[0]
            goal_msg.pose.position.y = goal[1]
            goal_msg.pose.position.z = 0.5  # Altura para visualización
            
            # Orientación no importa para la meta
            goal_msg.pose.orientation.w = 1.0
            
            self.current_goal = goal_msg
            self.goal_pub.publish(goal_msg)
            
            self.get_logger().info(
                f'Meta publicada: ({goal[0]:.2f}, {goal[1]:.2f})'
            )
        else:
            self.get_logger().error('No se pudo generar meta válida')
    
    def generate_valid_goal(self, current_position, max_attempts=100):
        """
        Genera una meta válida:
        - Dentro de los límites del mapa
        - Fuera de obstáculos
        - A distancia mínima del robot
        """
        for _ in range(max_attempts):
            # Generar posición aleatoria
            goal_x = random.uniform(
                self.map_limits['min_x'] + self.map_limits['buffer'],
                self.map_limits['max_x'] - self.map_limits['buffer']
            )
            goal_y = random.uniform(
                self.map_limits['min_y'] + self.map_limits['buffer'],
                self.map_limits['max_y'] - self.map_limits['buffer']
            )
            
            # Verificar distancia mínima al robot
            distance_to_robot = math.sqrt(
                (goal_x - current_position[0])**2 +
                (goal_y - current_position[1])**2
            )
            
            if distance_to_robot < self.min_goal_distance:
                continue
            
            # Verificar que no está en obstáculo (si tuviéramos la info)
            if self.is_in_obstacle([goal_x, goal_y]):
                continue
            
            # Meta válida encontrada
            return [goal_x, goal_y]
        
        # Si no encontramos meta válida, usar una por defecto
        self.get_logger().warn('No se pudo generar meta aleatoria, usando (0, 0)')
        return [0.0, 0.0]
    
    def is_in_obstacle(self, position):
        """Verifica si una posición está dentro de un obstáculo"""
        for obstacle in self.obstacles:
            obs_x, obs_y = obstacle['position']
            half_size = obstacle['size'] / 2.0
            
            if (obs_x - half_size <= position[0] <= obs_x + half_size and
                obs_y - half_size <= position[1] <= obs_y + half_size):
                return True
        
        return False
    
    def distance_to_goal(self):
        """Calcula distancia a la meta actual"""
        if self.current_pose is None or self.current_goal is None:
            return float('inf')
        
        dx = self.current_goal.pose.position.x - self.current_pose.pose.position.x
        dy = self.current_goal.pose.position.y - self.current_pose.pose.position.y
        return math.sqrt(dx*dx + dy*dy)


def main(args=None):
    rclpy.init(args=args)
    node = GoalManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
