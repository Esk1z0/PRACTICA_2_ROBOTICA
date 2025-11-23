import math

import rclpy
from rclpy.node import Node
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class PioneerP3dxNode(Node):
    def __init__(self):
        super().__init__('pioneer_p3dx_node')

        # Parámetros configurables
        self.declare_parameter('coppelia_host', '127.0.0.1')
        self.declare_parameter('coppelia_port', 23000)

        host = self.get_parameter('coppelia_host').get_parameter_value().string_value
        port = self.get_parameter('coppelia_port').get_parameter_value().integer_value

        self.get_logger().info(f'Conectando a CoppeliaSim en {host}:{port}...')

        # Cliente ZeroMQ Remote API
        self._client = RemoteAPIClient(host, port)
        self._sim = self._client.require('sim')

        # Motores del Pioneer
        self._left_motor = self._sim.getObject('/Pioneer_p3dx_leftMotor')
        self._right_motor = self._sim.getObject('/Pioneer_p3dx_rightMotor')

        # Sonar: 16 sensores alrededor del robot
        sensor_names = [
            f'/Pioneer_p3dx_ultrasonicSensor{i}' for i in range(1, 17)
        ]
        self._sensors = [self._sim.getObject(name) for name in sensor_names]

        self.get_logger().info('Handles de motores y sonares obtenidos correctamente.')

        # Parámetros de comportamiento
        self._v_forward = 2.0     # velocidad lineal "crucero"
        self._v_turn = 1.5        # velocidad angular para girar
        self._safe_dist = 0.7     # a partir de aquí empieza a preocuparse
        self._stop_dist = 0.3     # muy cerca → frena fuerte / casi para

        # Bucle de control cada 100 ms
        self._timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        # Leemos todos los sensores y calculamos la distancia mínima detectada
        min_distance = math.inf
        detected_any = False

        for s in self._sensors:
            detection_state, detected_point, _, _ = self._sim.readProximitySensor(s)

            if detection_state:
                distance = math.sqrt(
                    detected_point[0] ** 2 +
                    detected_point[1] ** 2 +
                    detected_point[2] ** 2
                )
                detected_any = True
                if distance < min_distance:
                    min_distance = distance

        # Lógica muy sencilla de evasión de obstáculos
        if not detected_any:
            # Nada cerca → avanzar recto
            v_left = self._v_forward
            v_right = self._v_forward
            self.get_logger().info('Sin obstáculos: avanzando.')
        else:
            self.get_logger().info(f'Obstáculo detectado a {min_distance:.3f} m')

            if min_distance < self._stop_dist:
                # Muy cerca → giro fuerte sobre el sitio
                v_left = -self._v_turn
                v_right = self._v_turn
                self.get_logger().info('Obstáculo MUY cerca: giro fuerte.')
            elif min_distance < self._safe_dist:
                # Zona de seguridad → reducir velocidad y girar suave
                v_left = 0.5 * self._v_forward
                v_right = self._v_forward
                self.get_logger().info('Obstáculo cerca: giro suave.')
            else:
                # Algo detectado pero suficientemente lejos → seguir recto
                v_left = self._v_forward
                v_right = self._v_forward
                self.get_logger().info('Obstáculo lejos: avanzo normal.')

        # Mandamos velocidades a los motores
        self._sim.setJointTargetVelocity(self._left_motor, v_left)
        self._sim.setJointTargetVelocity(self._right_motor, v_right)


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
