# En coppelia_interface_node
- Poner en la linea 95 la IP de nuestro host
# Comandos 1 a 1
- docker compose build
- docker compose up -d
- docker compose exec --user root ros2 bash -c "chown -R rosuser:rosuser /ros2_ws"
- docker compose exec ros2 bash
- source /opt/ros/humble/setup.bash
- colcon build --symlink-install
- source /ros2_ws/install/setup.bash
- ros2 launch entrega_mapas_package bug2_system.launch.py

# En otra terminal 
- docker compose exec ros2 bash
- rviz2

Cuando se muestre rviz2, añadir map a partir del topic /map para observar los cambios.
