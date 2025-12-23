FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# ============================================
# DEPENDENCIAS DEL SISTEMA (apt)
# ============================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-opencv \
    python3-skimage \
    python3-networkx \
    python3-requests \
    python3-pil \
    sudo \
    tree \
    vim \
    nano \
    jq \
    ros-humble-slam-toolbox \
    ros-humble-nav2-map-server \
    && rm -rf /var/lib/apt/lists/*

# ============================================
# DEPENDENCIAS PYTHON (pip)
# ============================================
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir \
    coppeliasim-zmqremoteapi-client \
    openai

# ============================================
# CREAR USUARIO NO-ROOT (OPCIONAL: UID/GID)
# Nota Windows: en bind mounts desde NTFS, chown/uid no siempre aplica.
# ============================================
ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=rosuser

RUN set -e; \
    if ! getent group "${USER_GID}" >/dev/null 2>&1; then \
    groupadd --gid "${USER_GID}" "${USERNAME}"; \
    fi; \
    if ! id -u "${USERNAME}" >/dev/null 2>&1; then \
    useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}"; \
    fi; \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}; \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# ============================================
# WORKSPACE ROS2
# OJO: si montas ./src como bind mount, esto queda “tapado” por el mount.
# ============================================
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src /ros2_ws/maps /ros2_ws/build /ros2_ws/install /ros2_ws/log \
    && chown -R "${USER_UID}:${USER_GID}" /ros2_ws

# ============================================
# CONFIGURAR ENTORNO DE SHELL
# ============================================
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' >> /home/${USERNAME}/.bashrc && \
    echo 'cd /ros2_ws' >> /home/${USERNAME}/.bashrc && \
    echo 'alias build="colcon build --symlink-install"' >> /home/${USERNAME}/.bashrc && \
    echo 'alias source-ws="source install/setup.bash"' >> /home/${USERNAME}/.bashrc

# Mantén entrypoint ROS (inicializa entorno) y abre bash por defecto
ENTRYPOINT ["/ros_entrypoint.sh"]
USER rosuser
CMD ["bash"]
