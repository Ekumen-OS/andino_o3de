################################################################################
# Base Image
################################################################################

ARG BASE_IMAGE=osrf/ros:rolling-desktop

FROM ${BASE_IMAGE} as base

USER root

################################################################################
# Install requirements
################################################################################

# Copy requirement files and install dependencies
COPY docker/requirements.txt .
RUN apt-get update && apt-get install --no-install-recommends -y $(cat requirements.txt)
RUN rm requirements.txt

################################################################################
# Development with a user
################################################################################

ARG USERNAME=ros_user
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
ARG BASHRC_APPEND

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -s "/bin/bash" -m $USERNAME && \
    apt-get install -y sudo && \
    echo "${USERNAME} ALL=NOPASSWD: ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}
RUN echo "export PS1='\[\033[01;36m\](docker)\[\033[00m\] \[\033[01;32m\]\u@${NAME}\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /home/${USERNAME}/.bashrc && \
    echo "alias ll='ls --color=auto -alFNh'" >> /home/${USERNAME}/.bashrc && \
    echo "alias ls='ls --color=auto -Nh'" >> /home/${USERNAME}/.bashrc

# Check if the directory /opt/ros/ exists, and if it does, append the command to .bashrc
RUN if [ -d /opt/ros/ ]; then \
    echo "source /opt/ros/$(ls /opt/ros/)/setup.bash" >> /home/${USERNAME}/.bashrc; \
    fi

# Check if /usr/share/colcon_argcomplete/ is installed, and if it does, append the arg complete command to .bashrc
RUN if [ -d /usr/share/colcon_argcomplete/ ]; then \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/${USERNAME}/.bashrc; \
    fi


# If BASHRC_APPEND is set, append it to .bashrc
RUN if [ -n "${BASHRC_APPEND}" ]; then \
    echo "${BASHRC_APPEND}" >> /home/${USERNAME}/.bashrc; \
    fi

FROM base as binaries

USER ${USERNAME}

################################################################################
# o3de installation
################################################################################

WORKDIR /home/${USERNAME}/o3de_deb
RUN wget https://o3debinaries.org/main/Latest/Linux/o3de_2310_2.deb
RUN sudo apt-get install -y ./o3de_2310_2.deb
RUN echo "export O3DE_ROOT=/opt/O3DE/23.10.2" >> /home/${USERNAME}/.bashrc

################################################################################
# o3de sources
################################################################################

WORKDIR /home/${USERNAME}/o3de_sources
COPY docker/o3de.repos .
RUN vcs import < o3de.repos

################################################################################
# register gems and stuff
################################################################################

WORKDIR /opt/O3DE/23.10.2/
RUN  \
     # Register sdk engine
     scripts/o3de.sh register --this-engine && \
     # o3de-extras
     scripts/o3de.sh register --all-gems-path /home/${USERNAME}/o3de_sources/o3de-extras/Gems && \
     scripts/o3de.sh register --all-templates-path /home/${USERNAME}/o3de_sources/o3de-extras/Templates && \
     scripts/o3de.sh register --all-projects-path /home/${USERNAME}/o3de_sources/o3de-extras/Projects && \
     # o3de-rgl-gem
     scripts/o3de.sh register --gem-path /home/${USERNAME}/o3de_sources/o3de-rgl-gem && \
     # RobotVacuumSample
     scripts/o3de.sh register --project-path /home/${USERNAME}/o3de_sources/RobotVacuumSample --force && \
     # ROSConDemo
     scripts/o3de.sh register --project-path /home/${USERNAME}/o3de_sources/ROSConDemo/Project

WORKDIR /home/${USERNAME}

CMD ["/bin/bash"]

FROM binaries as demo

USER ${USERNAME}

ENV ANDINO_O3DE_PATH=/home/${USERNAME}/andino_o3de
ENV O3DE_VERSION=23.10.2

################################################################################
# Copy andino_o3de to the container
################################################################################

WORKDIR ${ANDINO_O3DE_PATH}
COPY andino_o3de .

# Register the andino_o3de project
RUN /opt/O3DE/$O3DE_VERSION/scripts/o3de.sh register --project-path $ANDINO_O3DE_PATH

WORKDIR /home/${USERNAME}/
RUN mkdir o3de_packages

################################################################################
# Build project
################################################################################

WORKDIR ${ANDINO_O3DE_PATH}

ENV ROS_DISTRO=humble
RUN \
  . /opt/ros/${ROS_DISTRO}/setup.sh &&\
  cmake -B build/linux -S . -G "Ninja Multi-Config" -DLY_3RDPARTY_PATH=/home/${USERNAME}/o3de-packages &&\
  cmake --build build/linux --target andino_o3de.GameLauncher Editor --config profile -j 8

RUN echo "export O3DE_VERSION=${O3DE_VERSION}" >> /home/${USERNAME}/.bashrc
RUN echo "export ANDINO_O3DE_PATH=${ANDINO_O3DE_PATH}" >> /home/${USERNAME}/.bashrc

CMD ["/bin/bash", "-c", "/opt/O3DE/${O3DE_VERSION}/bin/Linux/profile/Default/Editor --project-path ~/andino_o3de"]
