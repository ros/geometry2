# expanded from tf2_ros/env_hook/console_bridge_libs.sh

# This is necessary due to console bridge installing to a non-standard location
# https://github.com/ros2/ros2/issues/195
PROCESSOR=`uname -p`
ament_prepend_unique_value LD_LIBRARY_PATH "$AMENT_CURRENT_PREFIX/lib/$PROCESSOR-linux-gnu"
