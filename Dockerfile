FROM ros:humble-perception

COPY scripts/requirements.txt /tmp/requirements.txt

# Install extra ROS 2 packages
RUN apt update && apt install -y \
    python3-pip \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    ros-humble-rviz2 \
    ros-humble-foxglove-bridge \
    && apt clean

# Accept user and group IDs as build args, with defaults
ARG LOCAL_USER_ID=1000
ARG LOCAL_GROUP_ID=1000

# Create a user that matches the host UID/GID
RUN groupadd -g ${LOCAL_GROUP_ID} rosuser && \
    useradd -u ${LOCAL_USER_ID} -g rosuser -m rosuser

# install sudo and give the rosuser passwordless sudo access
RUN apt update && apt install -y sudo && \
    usermod -aG sudo rosuser && \
    echo "rosuser ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && apt clean

# Switch to the new user
USER rosuser

# Set working directory inside container
WORKDIR /carpi

# Install Python dependencies
RUN pip install --user -r /tmp/requirements.txt
