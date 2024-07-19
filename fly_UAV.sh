#!/bin/sh

#启动mpc控制器
gnome-terminal --tab -- bash -c "source ~/fly/devel/setup.bash &&
roslaunch uav_control control.launch"
sleep 3
s
#启动规划节点
gnome-terminal --tab -- bash -c "source ~/fly/devel/setup.bash &&
roslaunch ego_planner simple_run.launch"
sleep 3

#启动状态机
gnome-terminal --tab -- bash -c "source ~/fly/devel/setup.bash &&
roslaunch state_machine state_machine.launch"
sleep 2

#启动圆环识别
#source ~/fly/devel/setup.bash
#roslaunch circle_det circle_det.launch


#订阅定位信息
#启动状态机
gnome-terminal --tab -- bash -c "rostopic echo /mavros/local_position/odom"





