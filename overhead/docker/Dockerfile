# ROS 2 Lyrical image for the overhead group. Multi-arch: the same file builds on the PC
# (amd64) and on the Raspberry Pi 5 (arm64).
FROM ros:lyrical-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DOMAIN_ID=0 \
    WORKSPACE=/ws

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-opencv \
        python3-pytest \
        v4l-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY entrypoint.sh /entrypoint.sh
COPY colcon-build.sh /usr/local/bin/colcon-build
RUN chmod +x /entrypoint.sh /usr/local/bin/colcon-build

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
