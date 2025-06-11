FROM ros:kilted

RUN apt-get update && apt-get install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        libx11-dev \
        libxtst-dev \
        xauth \
        x11-apps && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages pynput

WORKDIR /work

CMD ["bash", "-c", "source /opt/ros/kilted/setup.bash && bash"]