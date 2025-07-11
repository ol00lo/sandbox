FROM ros:kilted

RUN apt-get update && apt-get install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        libx11-dev \
        libxtst-dev \
        xauth \
        x11-apps \
        mc \
        gdb \
        valgrind \
        python3-pudb \
        postgresql \
        postgresql-client \
        libpqxx-dev \
        postgresql-server-dev-all  && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages pynput

RUN printf '\n\
# run mc remembering last mc directory on exit\n\
function mcc() {\n\
    MC_TMP_FILE=~/.mclast.txt\n\
    /usr/bin/mc --printwd="$MC_TMP_FILE"\n\
    if [ $? -eq 0 ] && [ -f "$MC_TMP_FILE" ]; then\n\
        NEW_DIR=$(cat "$MC_TMP_FILE" 2>/dev/null)\n\
        if [ -d "$NEW_DIR" ] && [ "$NEW_DIR" != "$ORIG_DIR" ]; then\n\
            echo $NEW_DIR\n\
            cd "$NEW_DIR" || return\n\
        fi\n\
    fi\n\
    rm -f "$MC_TMP_FILE"\n\
}\n' >> ~/.bashrc

WORKDIR /work

CMD ["bash", "-c", "source /opt/ros/kilted/setup.bash && (tail -f /dev/null &) && bash"]