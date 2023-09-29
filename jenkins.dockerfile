FROM osrf/ros:noetic-desktop-full-focal

RUN apt-get update && apt-get install -y \
  gazebo11 \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-joint-state-publisher \
  ros-noetic-joint-state-controller \
  ros-noetic-robot-state-publisher \
  ros-noetic-xacro \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-tools \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src

COPY ./tortoisebot_gazebo /catkin_ws/src/tortoisebot_gazebo
COPY ./tortoisebot_description /catkin_ws/src/tortoisebot_description
COPY ./tortoisebot_waypoints /catkin_ws/src/tortoisebot_waypoints

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /catkin_ws && catkin_make"

RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN sed -i 's|source "/opt/ros/\$ROS_DISTRO/setup.bash"|source "/catkin_ws/devel/setup.bash"|g' /ros_entrypoint.sh

CMD ["bash"]