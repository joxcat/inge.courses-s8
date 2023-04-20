FROM ros:melodic

ARG UID 1000
ARG GID 1000

RUN groupadd --gid $GID notroot && \
  useradd --uid $UID --gid $GID --shell /bin/bash --create-home notroot && \
  cp /root/.bashrc /home/notroot && \
  chown $UID:$GID /home/notroot/.bashrc && \
  echo "notroot ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN sed -i "s/#force_color/force_color/" /home/notroot/.bashrc && \
  echo "source /opt/ros/melodic/setup.bash" >> /home/notroot/.bashrc

RUN apt update -y && \
  apt upgrade -y && \
  apt install -y \
  python3-pip \
  vim \
  wget \
  libjpeg-dev

RUN python3 -m pip install \
  catkin_pkg \
  pyaml \
  empy \
  bagpy

RUN apt install -y ros-melodic-turtlesim
RUN apt install -y ros-melodic-rqt ros-melodic-rqt-common-plugins
RUN apt install -y ros-melodic-rospy-tutorials
RUN apt install -y ros-melodic-arbotix-python
RUN apt install -y ros-melodic-moveit-core
RUN apt install -y ros-melodic-moveit-ros-planning-interface
RUN apt install -y ros-melodic-rviz


RUN mkdir -p /home/notroot/bin && \
  cd /home/notroot/bin && \
  wget https://raw.githubusercontent.com/ElectricRCAircraftGuy/eRCaGuy_dotfiles/master/useful_scripts/ros_readbagfile.py && \
  chmod +x ros_readbagfile.py && \
  echo 'export PATH="$PATH:/home/notroot/bin"' >> /home/notroot/.bashrc

RUN echo 'alias rostup="source ./devel/setup.bash"' >> /home/notroot/.bashrc

RUN echo 'PS1="\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\n\$ "' >> /home/notroot/.bashrc

USER notroot
