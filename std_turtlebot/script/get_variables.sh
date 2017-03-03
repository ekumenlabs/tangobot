ARGS=$*
MY_HOSTNAME=$(hostname -I)
set -- $MY_HOSTNAME
export ROS_HOSTNAME=$1
echo "ROS Hostname: $ROS_HOSTNAME"
export TURTLEBOT_3D_SENSOR=kinect
echo "Turtlebot sensor: $TURTLEBOT_3D_SENSOR"
NAVSTACK_TOOLS_PATH=`rospack find navstack_tools`
export TURTLEBOT_EMPTY_MAP_FILE=$NAVSTACK_TOOLS_PATH/map/empty.yaml
echo "Turtlebot empty map file: $TURTLEBOT_EMPTY_MAP_FILE"
set -- $ARGS
