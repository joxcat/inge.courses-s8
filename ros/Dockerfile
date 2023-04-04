FROM ros:melodic

ARG UID 1000
ARG GID 1000

RUN apt update -y && \
  apt upgrade -y && \
  apt install -y \
  python3-pip \
  vim

RUN groupadd --gid $GID notroot && \
  useradd --uid $UID --gid $GID --shell /bin/bash --create-home notroot && \
  cp /root/.bashrc /home/notroot && \
  chown $UID:$GID /home/notroot/.bashrc && \
  echo "notroot ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN sed -i "s/#force_color/force_color/" /home/notroot/.bashrc && \
  echo "source /opt/ros/melodic/setup.bash" >> /home/notroot/.bashrc

RUN python3 -m pip install \
  catkin_pkg \
  pyaml \
  empy

RUN apt install -y \
  ros-melodic-turtlesim \
  ros-melodic-rqt \
  ros-melodic-rqt-common-plugins \
  ros-melodic-rospy-tutorials

RUN echo 'PS1="\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\n\$ "' >> /home/notroot/.bashrc

USER notroot
