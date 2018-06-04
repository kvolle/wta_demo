#!/bin/bash

source ~/.bashrc
source ~/REEF/catkin_ws/devel/setup.bash

roscore &

sleep 2

# Launch 2 virtual targets
roslaunch virtual_target virtual_target.launch ns:=goal0 x:=-1.0 y:=-2.0 &
roslaunch virtual_target virtual_target.launch ns:=goal1 x:=1.0 y:=2.0 &

# Launch 2 (virtual, for now) agents
roslaunch wta_demo wta_demo.launch name:=/robot0 &
roslaunch wta_demo wta_demo.launch name:=/robot1 &

# Wait and clean up on exit
read -p "Press enter to exit"
kill -- -$$
