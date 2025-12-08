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
    sudo \
    tree \
    vim \
    nano \
    && rm -rf /var/lib/apt/lists/*

# ============================================
# DEPENDENCIAS PYTHON (pip)
# ============================================
RUN python3 -m pip install --no-cache-dir --break-system-packages \
    coppeliasim-zmqremoteapi-client

# ============================================
# CREAR USUARIO NO-ROOT
# ============================================
ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=rosuser

# Crear usuario manejando casos donde el grupo/usuario ya existe
RUN set -e; \
    # Verificar si el GID ya existe
    if getent group $USER_GID > /dev/null 2>&1; then \
        EXISTING_GROUP=$(getent group $USER_GID | cut -d: -f1); \
        echo "Grupo $EXISTING_GROUP con GID $USER_GID ya existe, lo usaremos"; \
    else \
        groupadd --gid $USER_GID $USERNAME; \
        EXISTING_GROUP=$USERNAME; \
    fi; \
    # Crear usuario o modificar existente
    if id -u $USER_UID > /dev/null 2>&1; then \
        EXISTING_USER=$(id -un $USER_UID); \
        echo "Usuario $EXISTING_USER con UID $USER_UID ya existe"; \
        usermod -l $USERNAME -d /home/$USERNAME -m $EXISTING_USER 2>/dev/null || true; \
        usermod -g $USER_GID $USERNAME 2>/dev/null || true; \
    else \
        useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME; \
    fi; \
    # Dar permisos sudo
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME; \
    chmod 0440 /etc/sudoers.d/$USERNAME; \
    # Crear home directory si no existe
    mkdir -p /home/$USERNAME; \
    chown -R $USER_UID:$USER_GID /home/$USERNAME

# ============================================
# WORKSPACE ROS2
# ============================================
WORKDIR /ros2_ws

# Dar permisos al workspace
RUN chown -R $USER_UID:$USER_GID /ros2_ws

# ============================================
# CONFIGURAR ENTORNO PARA EL USUARIO
# ============================================
USER $USERNAME

# Configurar bashrc
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/$USERNAME/.bashrc && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' >> /home/$USERNAME/.bashrc && \
    echo 'cd /ros2_ws' >> /home/$USERNAME/.bashrc && \
    echo 'alias fix-perms="sudo chown -R rosuser:rosuser /ros2_ws"' >> /home/$USERNAME/.bashrc && \
    echo 'alias build="colcon build --symlink-install"' >> /home/$USERNAME/.bashrc && \
    echo 'alias source-ws="source install/setup.bash"' >> /home/$USERNAME/.bashrc

CMD ["/bin/bash"]