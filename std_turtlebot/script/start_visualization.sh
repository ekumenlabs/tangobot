if (( $# > 1)); then
    echo "Wrong number of arguments. Usage: ./start_turtlebot_baseline [-model / -nav]"
fi

STD_TURTLEBOT_PATH=`rospack find std_turtlebot`
source $STD_TURTLEBOT_PATH/script/get_variables.sh

if (( $# < 1)) || [[ $1 = "-model" ]]; then
    echo "Running model viewer"
    roslaunch turtlebot_rviz_launchers view_model.launch --screen
elif [[ $1 = "-nav" ]]; then 
    echo "Running navigation viewer"
    roslaunch std_turtlebot view_navigation.launch
fi
