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
        python3-pudb && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages pynput

RUN echo $'\n\
# run mc remembering last mc directory on exit
function mcc() {
    MC_TMP_FILE=~/.mclast.txt
    /usr/bin/mc --printwd="$MC_TMP_FILE"
    if [ $? -eq 0 ] && [ -f "$MC_TMP_FILE" ]; then
        NEW_DIR=$(cat "$MC_TMP_FILE" 2>/dev/null)
        if [ -d "$NEW_DIR" ] && [ "$NEW_DIR" != "$ORIG_DIR" ]; then
            echo $NEW_DIR
            cd "$NEW_DIR" || return
        fi
    fi
    rm -f "$MC_TMP_FILE"
}' >> ~/.bashrc

WORKDIR /work

CMD ["bash", "-c", "source /opt/ros/kilted/setup.bash && bash"]