FROM ubuntu:noble

# Настраиваем локаль
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Устанавливаем базовые зависимости для добавления репозитория ROS
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    software-properties-common

# Добавляем ключ ROS 2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Добавляем репозиторий ROS 2 Kilted
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# Обновляем список пакетов и устанавливаем
RUN apt-get update && \
    apt-get install -y \
		ros-kilted-desktop\
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        libx11-dev \
        libxtst-dev \
        xauth \
        x11-apps && \
    rm -rf /var/lib/apt/lists/*

# Инициализируем rosdep
RUN rosdep init

# Устанавливаем pynput
RUN pip3 install --break-system-packages pynput

# Настраиваем рабочую директорию
RUN mkdir -p /work/src
WORKDIR /work

# Автоподгрузка ROS при запуске
RUN echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc

# Команда по умолчанию
CMD ["bash", "-c", "source /opt/ros/kilted/setup.bash && bash"]