#!/bin/bash

# Launch 1 turtlebot
roslaunch turtlebot_bringup pid_gps.launch name:=/$1&
roslaunch bot_pid_kobuki bot_pid_kobuki_min.launch name:=/$1 &
roslaunch wta_demo wta_demo.launch name:=/$1 &


