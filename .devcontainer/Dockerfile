FROM osrf/ros:jazzy-desktop-full
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libgdal-dev \
    libgeos-dev \
    libtinyxml2-dev \
    python3-pip \
    python3-matplotlib \
    python3-tk \
    bash-completion \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-turtlebot3-navigation2 \
    ros-jazzy-turtlebot3-simulations \
    && rm -rf /var/lib/apt/lists/*
RUN echo '' >> /home/$USERNAME/.bashrc \
    && echo '# Enable bash completion' >> /home/$USERNAME/.bashrc \
    && echo 'if [ -f /usr/share/bash-completion/bash_completion ]; then' >> /home/$USERNAME/.bashrc \
    && echo '  . /usr/share/bash-completion/bash_completion' >> /home/$USERNAME/.bashrc \
    && echo 'fi' >> /home/$USERNAME/.bashrc \
    && echo '' >> /home/$USERNAME/.bashrc \
    && echo '# ROS/TurtleBot3 defaults for this workspace' >> /home/$USERNAME/.bashrc \
    && echo 'source /opt/ros/jazzy/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'if [ -f /home/ws/install/setup.bash ]; then source /home/ws/install/setup.bash; fi' >> /home/$USERNAME/.bashrc \
    && echo 'export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}' >> /home/$USERNAME/.bashrc \
    && chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
