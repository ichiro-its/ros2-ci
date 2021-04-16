ARG ROS2_DISTRO=foxy

FROM ros:${ROS2_DISTRO}

ARG ROS2_DISTRO
ENV ROS2_DISTRO=${ROS2_DISTRO}

COPY ws /ws

COPY entrypoint.bash /bin/entrypoint.bash
RUN chmod +x /bin/entrypoint.bash

RUN apt-get update && apt-get install -y git python3-pip

ENTRYPOINT ["entrypoint.bash"]
