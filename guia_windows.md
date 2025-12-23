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

# Guia para usar A* en cualquier png
1. Crear directorio maps y añadir png a maps (IMPORTANTE: el png debe estar en blanco y negro o colores claros y oscuros para diferenciar bien los obstáculos)
2. Modificar a_star_node.py incluyendo en la linea 144 el nuevo escenario: se debe indicar el nombre del escenario, el nombre del png, la posición del robot y la posición de la meta
3. Ejecutar los "comandos 1 a 1" de arriba excepto el último
4. Ejecutar ros2 run entrega_mapas_package a_star_node
5. Debería aparecer el resultado en /maps, si no aparece es porque o bien la meta o el inicio están en un obstáculo, ve probando valores random hasta que te aparezca