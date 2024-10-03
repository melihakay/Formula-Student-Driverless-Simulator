FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
RUN adduser --quiet --disabled-password --gecos '' --uid ${UID:=1000} --uid ${GID:=1000} ros

RUN apt update -y &&\
    apt install ros-humble-foxglove-bridge ros-humble-ackermann-msgs -y

COPY ./src /ros-ws/src
WORKDIR /ros-ws
RUN source /opt/ros/humble/setup.bash &&\
    colcon build

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT /entrypoint.sh

