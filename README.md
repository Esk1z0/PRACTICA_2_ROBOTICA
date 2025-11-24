# 🤖 Proyecto ROS2 - Construcción de Mapas

Guía completa para trabajar con el proyecto usando Docker y ROS2.

---

## 📋 Requisitos Previos

- **Docker**: [Instalar Docker](https://docs.docker.com/get-docker/)
- **Docker Compose**: [Instalar Docker Compose](https://docs.docker.com/compose/install/)
- **Git**: Para clonar el repositorio

### Verificar instalación

```bash
docker --version
docker-compose --version
```

---

## 🚀 Instalación y Primera Configuración

### 1. Clonar el repositorio

```bash
git clone <URL_DEL_REPO>
cd <NOMBRE_DEL_PROYECTO>
```

### 2. Construir la imagen Docker

```bash
docker-compose build
```

**Nota**: Este proceso puede tardar 5-10 minutos la primera vez.

### 3. Verificar que todo funciona

```bash
# Iniciar el contenedor
docker-compose up -d

# Entrar al contenedor
docker-compose exec ros2 bash

# Dentro del contenedor, verificar instalación
python3 -c "from coppeliasim_zmqremoteapi_client import RemoteAPIClient; print('✓ CoppeliaSim ZMQ OK')"
python3 -c "import numpy; print('✓ NumPy OK')"
python3 -c "import rclpy; print('✓ ROS2 OK')"

# Salir del contenedor
exit
```

Si ves los tres ✓, ¡todo está listo!

---

## 🐳 Comandos Docker Compose Esenciales

### Gestión del contenedor

```bash
# Construir/reconstruir la imagen
docker-compose build

# Iniciar el contenedor en segundo plano
docker-compose up -d

# Detener el contenedor
docker-compose down

# Ver logs del contenedor
docker-compose logs

# Ver logs en tiempo real
docker-compose logs -f

# Reiniciar el contenedor
docker-compose restart
```

### Trabajar dentro del contenedor

```bash
# Entrar al contenedor (sesión principal)
docker-compose exec ros2 bash

# Abrir una segunda terminal en el mismo contenedor
docker-compose exec ros2 bash

# Ejecutar un comando sin entrar al contenedor
docker-compose exec ros2 <comando>

# Ejemplo: compilar sin entrar
docker-compose exec ros2 bash -c "source install/setup.bash && colcon build"
```

### Limpieza y mantenimiento

```bash
# Detener y eliminar el contenedor (los datos en src/ se mantienen)
docker-compose down

# Eliminar también los volúmenes (¡cuidado! borra build/install/log)
docker-compose down -v

# Ver contenedores activos
docker ps

# Ver todas las imágenes
docker images

# Eliminar imagen antigua
docker rmi mi_proyecto_ros2:latest

# Limpiar todo Docker (¡cuidado! afecta otros proyectos)
docker system prune -a
```

---

## 🔧 Comandos ROS2 Fundamentales

### Compilación del workspace

```bash
# Compilar todos los paquetes
colcon build

# Compilar con enlaces simbólicos (hot-reload para Python)
colcon build --symlink-install

# Compilar solo un paquete específico
colcon build --packages-select mi_paquete

# Compilar con output detallado
colcon build --event-handlers console_direct+

# Limpiar build anterior
rm -rf build install log
colcon build --symlink-install
```

### Cargar el entorno

```bash
# Cargar ROS2 base (ya está en .bashrc, pero por si acaso)
source /opt/ros/jazzy/setup.bash

# Cargar tu workspace (SIEMPRE después de compilar)
source install/setup.bash

# Ver variables de entorno ROS
env | grep ROS
```

### Ejecutar nodos

```bash
# Formato general
ros2 run <nombre_paquete> <nombre_nodo>

# Ejemplo
ros2 run mi_paquete test_node

# Con argumentos
ros2 run mi_paquete mi_nodo --ros-args -p param:=value

# Con remapping de topics
ros2 run mi_paquete mi_nodo --ros-args -r topic_viejo:=topic_nuevo
```

### Información del sistema

```bash
# Listar todos los nodos activos
ros2 node list

# Información detallada de un nodo
ros2 node info /nombre_nodo

# Listar topics activos
ros2 topic list

# Ver mensajes de un topic en tiempo real
ros2 topic echo /nombre_topic

# Información de un topic
ros2 topic info /nombre_topic

# Frecuencia de publicación de un topic
ros2 topic hz /nombre_topic

# Listar servicios
ros2 service list

# Listar parámetros de un nodo
ros2 param list /nombre_nodo
```

### Launch files

```bash
# Ejecutar un launch file
ros2 launch mi_paquete mi_launch.py

# Con argumentos
ros2 launch mi_paquete mi_launch.py param:=value
```

### Debugging

```bash
# Ver todos los logs
ros2 run rqt_console rqt_console

# Ver grafo de nodos (requiere X11)
rqt_graph

# Publicar manualmente en un topic
ros2 topic pub /topic_name std_msgs/msg/String "data: 'Hello'"

# Llamar a un servicio
ros2 service call /service_name std_srvs/srv/Empty
```

---

## 📂 Estructura del Proyecto

```
.
├── Dockerfile                  # Definición de la imagen Docker
├── docker-compose.yml          # Configuración de servicios
├── .dockerignore              # Archivos ignorados por Docker
├── README.md                  # Este archivo
└── src/                       # Tu código ROS2
    └── mi_paquete/
        ├── package.xml        # Metadatos y dependencias
        ├── setup.py           # Configuración Python
        ├── resource/          # Recursos del paquete
        ├── mi_paquete/        # Código fuente
        │   ├── __init__.py
        │   ├── controlador.py
        │   └── mapper.py
        └── launch/            # Launch files
            └── sistema.launch.py
```

---

## 🔄 Workflow Típico de Desarrollo

### Sesión de trabajo normal

```bash
# 1. Iniciar contenedor
docker-compose up -d

# 2. Entrar al contenedor
docker-compose exec ros2 bash

# 3. Compilar (primera vez o tras cambios en setup.py/package.xml)
colcon build --symlink-install
source install/setup.bash

# 4. Ejecutar tu nodo
ros2 run mi_paquete mi_nodo

# 5. Al terminar (desde fuera del contenedor)
docker-compose down
```

### Desarrollo con múltiples terminales

**Terminal 1** (simulador/nodo principal):
```bash
docker-compose exec ros2 bash
source install/setup.bash
ros2 run mi_paquete controlador
```

**Terminal 2** (monitoring):
```bash
docker-compose exec ros2 bash
ros2 topic list
ros2 topic echo /scan
```

**Terminal 3** (visualización):
```bash
docker-compose exec ros2 bash
ros2 run rviz2 rviz2
```

---

## 🐛 Solución de Problemas Comunes

### "ModuleNotFoundError" al ejecutar un nodo

```bash
# Asegúrate de haber compilado y cargado el entorno
colcon build --symlink-install
source install/setup.bash
```

### Los cambios en Python no se reflejan

```bash
# Si usaste --symlink-install, no hace falta recompilar
# Solo recarga el módulo o reinicia el nodo

# Si el problema persiste:
colcon build --symlink-install --packages-select mi_paquete
```

### "Package not found"

```bash
# Verifica que el paquete está en src/
ls src/

# Verifica package.xml y setup.py
cat src/mi_paquete/package.xml
cat src/mi_paquete/setup.py

# Recompila desde cero
rm -rf build install log
colcon build --symlink-install
```

### El contenedor no arranca

```bash
# Ver logs de error
docker-compose logs

# Reconstruir desde cero
docker-compose down -v
docker-compose build --no-cache
docker-compose up -d
```

### Error "cannot connect to X server" (GUI)

```bash
# En tu máquina host, permitir conexiones X11:
xhost +local:docker

# Verificar que DISPLAY está configurado en docker-compose.yml:
environment:
  - DISPLAY=${DISPLAY}
volumes:
  - /tmp/.X11-unix:/tmp/.X11-unix:rw
```

---

## 📚 Recursos Útiles

- [Documentación ROS2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Tutoriales ROS2](https://docs.ros.org/en/jazzy/Tutorials.html)
- [CoppeliaSim Documentation](https://www.coppeliarobotics.com/helpFiles/)
- [Docker Compose Reference](https://docs.docker.com/compose/compose-file/)

---

## 👥 Colaboración

### Añadir dependencias nuevas

1. Editar `Dockerfile` (sección de dependencias)
2. Reconstruir imagen:
   ```bash
   docker-compose down
   docker-compose build
   docker-compose up -d
   ```

### Compartir código

```bash
# El directorio src/ está montado como volumen
# Los cambios se sincronizan automáticamente entre host y contenedor
# Usa git normalmente desde tu host:

git add src/
git commit -m "Added new feature"
git push
```

### Trabajar en diferentes ramas

```bash
# Cambiar de rama (desde el host)
git checkout feature-branch

# No hace falta reconstruir el contenedor
# Solo recompila dentro del contenedor si hay cambios en setup.py/package.xml
docker-compose exec ros2 bash
colcon build --symlink-install
```

---

## 🎯 Checklist de Verificación Rápida

Usa esto para verificar que todo funciona:

```bash
# ✓ Docker instalado
docker --version

# ✓ Imagen construida
docker images | grep mi_proyecto

# ✓ Contenedor corriendo
docker ps | grep ros2_dev

# ✓ Entrar y verificar ROS2
docker-compose exec ros2 bash -c "source /opt/ros/jazzy/setup.bash && ros2 --version"

# ✓ Verificar Python packages
docker-compose exec ros2 python3 -c "import rclpy, numpy, coppeliasim_zmqremoteapi_client; print('OK')"

# ✓ Workspace compilado
docker-compose exec ros2 bash -c "ls /ros2_ws/install"
```

---

## 📝 Notas Finales

- **Los cambios en `src/` se mantienen** incluso si destruyes el contenedor
- **Los directorios `build/`, `install/`, `log/` se almacenan en volúmenes Docker** y persisten entre sesiones
- **Para una limpieza completa** usa `docker-compose down -v` (¡cuidado! borra build/install/log)
- **El código Python con `--symlink-install` no requiere recompilación** tras cada cambio

---

¿Dudas? Abre un issue o contacta con el equipo.

**¡Happy coding!** 🚀
