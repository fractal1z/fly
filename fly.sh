#!/bin/sh
#启动仿真世界
roslaunch px4 livox.launch

#启动fast_lio并转换到px4坐标系
roslaunch fast_lio mapping_mid360.launch

#启动mpc控制器
source ~/fly/devel/setup.bash
roslaunch uav_control control.launch

#启动规划节点
source ~/fly/devel/setup.bash
roslaunch ego_planner simple_run.launch

#启动状态机
source ~/fly/devel/setup.bash
roslaunch state_machine state_machine.launch

#启动圆环识别
source ~/fly/devel/setup.bash
roslaunch circle_det circle_det.launch

#移动障碍物
python3 control_move.py

#订阅定位信息
rostopic echo /mavros/local_position/odom




