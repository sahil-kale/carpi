FROM ros:humble-perception

# Install extra ROS 2 packages
RUN apt update && apt install -y \
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

# Switch to the new user
USER rosuser

# Set working directory inside container
WORKDIR /carpi
