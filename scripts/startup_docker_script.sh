xhost +local:docker  # Only if using GUI on Pi desktop
docker rm ros-dev
docker run -it \
  --name ros-dev \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/Desktop/carpi:/carpi \
  ros-dev-custom