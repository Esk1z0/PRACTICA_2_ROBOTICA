FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# 1. Paquetes base
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 2. Cliente Python ZeroMQ para CoppeliaSim
#    (instalado en el Python del sistema con --break-system-packages)
RUN python3 -m pip install --no-cache-dir \
    --break-system-packages coppeliasim-zmqremoteapi-client

# 3. Workspace ROS 2
WORKDIR /ros2_ws

# 4. Copiar código
COPY ./src ./src

# 5. Build condicional
RUN bash -lc "source /opt/ros/jazzy/setup.bash && \
              if [ -d src ] && [ \"$(ls -A src)\" ]; then colcon build --symlink-install; fi"

# 6. Entorno por defecto
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'cd /ros2_ws' >> /root/.bashrc && \
    echo 'if [ -f install/setup.bash ]; then source install/setup.bash; fi' >> /root/.bashrc

CMD [\"/bin/bash\"]
