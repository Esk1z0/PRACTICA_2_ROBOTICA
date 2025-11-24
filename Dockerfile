FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# ============================================
# DEPENDENCIAS DEL SISTEMA (apt)
# ============================================
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-opencv \
    python3-skimage \
    python3-networkx \
    && rm -rf /var/lib/apt/lists/*

# ============================================
# DEPENDENCIAS PYTHON (pip) - Solo las que NO están en apt
# ============================================
RUN python3 -m pip install --no-cache-dir --break-system-packages \
    coppeliasim-zmqremoteapi-client

# ============================================
# WORKSPACE ROS2
# ============================================
WORKDIR /ros2_ws

# Configurar entorno automático
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' >> /root/.bashrc && \
    echo 'cd /ros2_ws' >> /root/.bashrc

CMD ["/bin/bash"]FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Instalar dependencias del sistema
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ============================================
# DEPENDENCIAS PYTHON - Añade las que necesites aquí
# ============================================
RUN python3 -m pip install --no-cache-dir --break-system-packages \
    coppeliasim-zmqremoteapi-client \
    numpy \
    scipy \
    scikit-image \
    matplotlib \
    opencv-python \
    networkx

# ============================================
# WORKSPACE ROS2
# ============================================
WORKDIR /ros2_ws

# Configurar entorno automático
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' >> /root/.bashrc && \
    echo 'cd /ros2_ws' >> /root/.bashrc

CMD ["/bin/bash"]