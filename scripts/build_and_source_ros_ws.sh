cd ros_ws
# --symlink-install allows for easier development by creating symlinks instead of copying files
colcon build --symlink-install
source install/setup.bash
cd ..
echo "ROS workspace built and sourced successfully."