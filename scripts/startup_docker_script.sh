# Allow local Docker containers to access the X11 display (only needed if using GUI apps)
xhost +local:docker  # Only if using GUI on Pi desktop

# Remove any existing container named 'ros-dev' to avoid name conflicts
docker rm ros-dev

# Start the Docker container
docker run -it \                                # Run interactively with a TTY (for bash shell)
  --name ros-dev \                              # Name the container 'ros-dev'
  --network host \                              # Share the host's network stack (important for ROS 2 discovery)
  -e DISPLAY=$DISPLAY \                         # Pass the host display environment to the container (for GUI apps)
  -v /tmp/.X11-unix:/tmp/.X11-unix \            # Mount X11 Unix socket for GUI display support
  -v ~/Desktop/carpi:/carpi \                   # Mount local project directory into the container at /carpi
  --device=/dev/rrc \                           # Give the container access to the physical /dev/rrc device
  --group-add=$(getent group dialout | cut -d: -f3) \  # Add the 'dialout' group (or whatever owns /dev/rrc) for permission
  ros-dev-custom                                # Use the 'ros-dev-custom' image as the container base
