#! /usr/bin/env bash
pushd
echo -e "\e[1;91mWelcome to the IRIS kinect obstacle detection demo! :)\e[0m"
sleep 0.5
echo -e "\e[1;91m--- setting up environment\e[0m"
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
echo -e "\e[1;91m--- ros hydro initialization\e[0m"
source /opt/ros/hydro/setup.bash
echo -e "\e[1;91m--- go to project folder\e[0m"
cd ~/wIRIS
echo -e "\e[1;91m--- compile everything\e[0m"
catkin_make
echo -e "\e[1;91m--- project initialization\e[0m"
source devel/setup.bash
echo -e "\e[1;91m--- launch the core\e[0m"
popd
roscore &
KILL_PID1=$!
echo -e "\e[1;91m--- launch the package\e[0m"
rosrun obstacle_detection computer_display &
KILL_PID2=$!
echo -e "\e[1;91m--- set up ctrl-c trap\e[0m"
trap "kill $KILL_PID1 && kill $KILL_PID2" EXIT
wait $KILL_PID2
echo -e "\e[1;91m--- kill dangling roscore\e[0m"
trap - EXIT
kill $KILL_PID1
wait $KILL_PID1
echo -e "\e[1;91m--- done :)\e[0m"


