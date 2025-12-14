import sys
import rclpy
from rclpy.node import Node
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class PioneerUniversal(Node):
    def __init__(self):
        super().__init__('pioneer_universal_node')
        
        # 1. CONEXIÓN
        self.get_logger().info('Conectando con CoppeliaSim...')
        try:
            # Usamos host.docker.internal para Mac
            self.client = RemoteAPIClient(host='host.docker.internal')
            self.sim = self.client.require('sim')
            self.get_logger().info('¡Conectado!')
        except Exception as e:
            self.get_logger().error(f'No se pudo conectar: {e}')
            sys.exit(1)

        # 2. BUSCADOR INTELIGENTE DE MOTORES
        self.left_motor = self.find_object(['/PioneerP3DX/leftMotor', '/Pioneer_p3dx_leftMotor', 'leftMotor'])
        self.right_motor = self.find_object(['/PioneerP3DX/rightMotor', '/Pioneer_p3dx_rightMotor', 'rightMotor'])
        
        if not self.left_motor or not self.right_motor:
            self.get_logger().error("❌ ERROR CRÍTICO: No encuentro los motores. Revisa la escena.")
            sys.exit(1)
            
        self.get_logger().info("✅ Motores encontrados y vinculados.")

        # 3. TEST DE MOVIMIENTO (Girar 3 segundos)
        self.get_logger().info("Probando motores... ¡GIRANDO!")
        self.sim.setJointTargetVelocity(self.left_motor, 0.5)
        self.sim.setJointTargetVelocity(self.right_motor, -0.5)

    def find_object(self, possible_names):
        """Prueba una lista de nombres hasta encontrar uno que exista"""
        for name in possible_names:
            try:
                handle = self.sim.getObject(name)
                self.get_logger().info(f'Objeto encontrado: {name}')
                return handle
            except:
                continue
        return None

def main(args=None):
    rclpy.init(args=args)
    node = PioneerUniversal()
    # Mantenemos el nodo vivo un momento para que gire
    try:
        import time
        time.sleep(3) 
    finally:
        # Paramos motores al salir
        if hasattr(node, 'sim') and node.left_motor:
            node.sim.setJointTargetVelocity(node.left_motor, 0.0)
            node.sim.setJointTargetVelocity(node.right_motor, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()