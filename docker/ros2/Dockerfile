ARG ROS2_DISTRO=foxy

FROM ros:${ROS2_DISTRO}

ARG ROS2_DISTRO

COPY . /ws

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
