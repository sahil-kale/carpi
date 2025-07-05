docker build \
  --build-arg LOCAL_USER_ID=$(id -u) \
  --build-arg LOCAL_GROUP_ID=$(id -g) \
  -t ros-dev-custom .
