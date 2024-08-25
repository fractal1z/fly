#! /usr/bin/env python3
#coding=utf-8

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State,PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool,Int32


class Controller:
    def __init__(self) -> None:
        rospy.init_node("setpoint")

        self.load_params()
        self.sleep_time = 0.05
        self.sleep_time_time = 0.2
        self.center_pos =[0,0,0]
        self.circle_center_pos =[0,0,0]
        self.center_pos_end =[0,0,0]
        self.det_flag = False
        rospy.Subscriber("~odom", Odometry, self.odom_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/aruco_det/aruco_id', Int32, self.detected_callback)
        rospy.Subscriber("/center_position", PoseStamped, self.pose_callback)
        
        self.current_state = State()
        rospy.Subscriber("mavros/state", State, self.state_cb)
        self.pub_point = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arm_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.rate = rospy.Rate(20)
        while(not rospy.is_shutdown() and not self.current_state.connected):
            self.rate.sleep()
        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'

        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True

        while(not rospy.is_shutdown()):
            if(self.current_state.mode != "OFFBOARD" ):
                if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")
                else:
                    rospy.loginfo("No OFFBOARD enabled")
            else:
                if(not self.current_state.armed ):
                    if(self.arm_client.call(self.arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")
                    else:
                        rospy.loginfo("No Vehicle armed")
            self.rate.sleep()
        

    def state_cb(self,msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.center_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def get_center_pos(self):
        self.circle_center_pos = [self.center_pos[0], self.center_pos[1], self.center_pos[2]]
        print(f"Calculated circle_center_pos: {self.circle_center_pos}")
        self.center_pos_end = [self.center_pos[0] + 0.2, self.center_pos[1]-0.2, self.center_pos[2]]
        print(f"Calculated center_pos_end: {self.center_pos_end}")
    def load_params(self):
        self.state = 0
        self.takeoff_point = rospy.get_param('~takeoff')
        self.first_square_point_pre = rospy.get_param('~1st_square_pre')
        self.first_square_point = rospy.get_param('~1st_square')
        self.first_square_point_end = rospy.get_param('~1st_square_end')
        self.second_square_point_pre = rospy.get_param('~2nd_square_pre')
        self.second_square_point_end = rospy.get_param('~2nd_square_end')
        self.car_up_point = rospy.get_param('~car_up')
        self.car_point = rospy.get_param('~car')
        self.land_point = rospy.get_param('~land')
        self.land_point_end = rospy.get_param('~land_end')
        print(type(self.land_point))

    def pub(self, point: list):
        target_pose = PositionTarget()
        target_pose.type_mask = (PositionTarget.IGNORE_YAW_RATE |  
                            PositionTarget.IGNORE_AFX |  
                            PositionTarget.IGNORE_AFY |  
                            PositionTarget.IGNORE_AFZ |
                            PositionTarget.IGNORE_YAW)
        target_pose.position.x = point[0]
        target_pose.position.y = point[1]
        target_pose.position.z = point[2]
        if(self.state == 0 ):
            target_pose.velocity.x = 0
            target_pose.velocity.y = 0
            target_pose.velocity.z = 1.0
        
        elif( self.state == 1):
            target_pose.velocity.x = 1.0
            target_pose.velocity.y = 0
            target_pose.velocity.z = 0
        elif( self.state == 2):
            target_pose.velocity.x = 1.0
            target_pose.velocity.y = 0
            target_pose.velocity.z = 0
        elif(self.state == 3 ):
            target_pose.velocity.x = 0.8
            target_pose.velocity.y = 0
            target_pose.velocity.z = 0
        elif(self.state == 5):
            target_pose.velocity.x = 0.8
            target_pose.velocity.y = 0
            target_pose.velocity.z = 0
        elif( self.state == 9):
            target_pose.velocity.x = 0
            target_pose.velocity.y = 0
            target_pose.velocity.z = 0
        else:
            target_pose.velocity.x = 1.0
            target_pose.velocity.y = 0
            target_pose.velocity.z = 0
        target_pose.coordinate_frame = target_pose.FRAME_LOCAL_NED
        self.pub_point.publish(target_pose)

    def is_close(self, odom: Odometry, point: list):
        if (self.state == 0):
            if (odom.pose.pose.position.z - point[2])**2 < 0.08:
                return True
            return False 
        elif (self.state == 9):
            if (odom.pose.pose.position.x - point[0])**2 + \
                (odom.pose.pose.position.y - point[1])**2 < 0.4:
                return True
            return False 
        elif (self.state == 1 or self.state == 2 or self.state == 3 or self.state == 5 ):
            if (odom.pose.pose.position.x - point[0]> -0.2):
                return True
            return False 
        else:
            if (odom.pose.pose.position.x - point[0])**2 + \
                (odom.pose.pose.position.y - point[1])**2 + \
                (odom.pose.pose.position.z - point[2])**2 < 0.15:
                return True
            return False 

    def detected_callback(self, msg):
        self.det_flag = True

    def odom_cb(self, msg: Odometry):
        # print(self.state)
        # print(f"Position:{msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z}")
        # print(f"Velocity:{msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z}")
        if self.state == 0:
            if self.is_close(msg, self.takeoff_point):
                rospy.sleep(self.sleep_time)
                self.state = 1
                pass
            else:
                self.pub(self.takeoff_point)
                # print(f"STATE:{self.state},pub_point:{self.takeoff_point}")
        if self.state == 1:
            if self.is_close(msg, self.first_square_point_pre):
                rospy.sleep(self.sleep_time_time)
                self.state = 2
                self.get_center_pos()
            else:
                self.pub(self.first_square_point_pre)
                #  print(f"STATE:{self.state},pub_point:{self.first_square_point_pre}")
        if self.state == 2:
            if self.is_close(msg, self.circle_center_pos):
                rospy.sleep(self.sleep_time)
                self.state = 3
                # self.state = 4
            else:
                self.pub(self.circle_center_pos)
                # print(f"STATE:{self.state},pub_point:{self.circle_center_pos}")
        if self.state == 3:
            if self.is_close(msg, self.first_square_point_end):
                rospy.sleep(self.sleep_time_time)
                self.state = 5
                self.get_center_pos()
            else:
                self.pub(self.first_square_point_end)
                # print(f"STATE:{self.state},pub_point:{self.first_square_point_end}")
        if self.state == 4:
            if self.is_close(msg, self.circle_center_pos):
                rospy.sleep(self.sleep_time)
                self.state = 5
            else:
                self.pub(self.circle_center_pos)
        if self.state == 5:
            if self.is_close(msg, self.center_pos_end):
                rospy.sleep(self.sleep_time)
                # self.state = 6
                self.state = 9
                pass
            else:
                self.pub(self.center_pos_end)
                # print(f"STATE:{self.state},pub_point:{self.center_pos_end}")
        if self.state == 6:
            if self.is_close(msg, self.car_up_point):
                rospy.sleep(self.sleep_time)
                self.state = 7
                pass
            else:
                self.pub(self.car_up_point)
        if self.state == 7:
            if self.det_flag:
                rospy.sleep(self.sleep_time)
                self.state = 8
                pass
            else:
                self.pub(self.car_point)
        if self.state == 8:
            if self.is_close(msg, self.land_point):
                rospy.sleep(self.sleep_time)
                self.state = 9
                pass
            else:
                self.pub(self.land_point)
        if self.state == 9:
            if self.is_close(msg, self.land_point_end):
                rospy.sleep(self.sleep_time)
                self.state = 10
                pass
            else:
                self.pub(self.land_point_end)
                # print(f"STATE:{self.state},pub_point:{self.land_point_end}")
        if self.state == 10:
            self.set_mode_client(0,'AUTO.LAND')
            rospy.sleep(self.sleep_time)
            self.arm_client(False)
            pass
            
        
controller = Controller()

rospy.spin()


# #! /usr/bin/env python3
# #coding=utf-8

# import rospy
# import numpy as np
# from geometry_msgs.msg import PoseStamped
# from mavros_msgs.msg import State,PositionTarget
# from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Bool,Int32

# class Controller:
#     def __init__(self) -> None:
#         self.load_params()
#         self.pub_point = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
#         rospy.wait_for_service("/mavros/set_mode")
#         self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
#         rospy.wait_for_service("/mavros/cmd/arming")
#         self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
#         self.sleep_time = 0.05
#         self.sleep_time_time = 0.2
#         self.center_pos = [0, 0, 0]
#         self.circle_center_pos = [0, 0, 0]
#         # self.center_pos_end = [0, 0, 0]
#         self.origin = [0, 0, 0]
#         self.det_flag = False
#         self.points = None
#         self.nextpoint = [0, 0, 0]
#         self.cur_odom = [0, 0, 0]
#         self.pub_flag = False
#         self.current_state = State()
#         self.state_sub = rospy.Subscriber("mavros/state", State,self.state_cb)
#         rospy.Subscriber("~odom", Odometry, self.odom_cb, queue_size=1, tcp_nodelay=True)
#         rospy.Subscriber('/aruco_det/aruco_id', Int32, self.detected_callback)
#         rospy.Subscriber("/center_position", PoseStamped, self.pose_callback)
#         rospy.Timer(rospy.Duration(0.1), self.timer_callback)
#         self.rate = rospy.Rate(20)

#         # Wait for Flight Controller connection
#         while(not rospy.is_shutdown() and not self.current_state.connected):
#             self.rate.sleep()

#     def pose_callback(self, msg):
#         self.center_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

#     def get_center_pos(self):
#         self.circle_center_pos = [self.center_pos[0], self.center_pos[1], self.center_pos[2]]
#         print(f"Calculated circle_center_pos: {self.circle_center_pos}")

#     def state_cb(self,msg):
#         self.current_state = msg

#     def load_params(self):
#         self.state = 0
#         self.index = 0
#         self.takeoff_point = np.array(rospy.get_param('~takeoff'))
#         self.first_square_point_pre = np.array(rospy.get_param('~1st_square_pre'))
#         self.first_square_point = np.array(rospy.get_param('~1st_square'))
#         self.first_square_point_end = np.array(rospy.get_param('~1st_square_end'))
#         self.second_square_point_pre = np.array(rospy.get_param('~2nd_square_pre'))
#         self.second_square_point_end = np.array(rospy.get_param('~2nd_square_end'))
#         self.car_up_point = np.array(rospy.get_param('~car_up'))
#         self.car_point = np.array(rospy.get_param('~car'))
#         self.land_point = np.array(rospy.get_param('~land'))
#         self.land_point_end = np.array(rospy.get_param('~land_end'))
#         print(type(self.land_point))

#     def line_between_points(self, p1, p2, num_points = 100):
#         x_values = np.linspace(p1[0], p2[0], num_points)
#         y_values = np.linspace(p1[1], p2[1], num_points)
#         z_values = np.linspace(p1[2], p2[2], num_points)
#         return np.column_stack((x_values, y_values, z_values))

#     def timer_callback(self, event):
#         if self.points is not None and self.index < len(self.points) and self.pub_flag:
#             self.pub(self.points[self.index], self.nextpoint)
#             self.index += 1
#         print(f"sdads")
#         rospy.loginfo("aka")

#     def pub(self, point, point1):
#         target_pose = PositionTarget()
#         target_pose.type_mask = (PositionTarget.IGNORE_YAW_RATE |  
#                             PositionTarget.IGNORE_AFX |  
#                             PositionTarget.IGNORE_AFY |  
#                             PositionTarget.IGNORE_AFZ |
#                             PositionTarget.IGNORE_YAW)
#         target_pose.position.x = point[0]
#         target_pose.position.y = point[1]
#         target_pose.position.z = point[2]
#         target_pose.velocity.x = 1.2
#         target_pose.velocity.y = 0
#         target_pose.velocity.z = 0
#         target_pose.coordinate_frame = target_pose.FRAME_LOCAL_NED
         
#         current_pos = np.array(self.cur_odom)
#         dist = np.array(point) - current_pos
#         dist_norm = np.linalg.norm(dist)

#         if np.allclose(point, point1):
#             target_pose.velocity.x = 0
#             target_pose.velocity.y = 0
#             target_pose.velocity.z = 0
#         else:
#             target_pose.velocity.x = 1.2 * dist[0] / dist_norm
#             target_pose.velocity.y = 1.2 * dist[1] / dist_norm
#             target_pose.velocity.z = 1.2 * dist[2] / dist_norm

#         self.pub_point.publish(target_pose)

#     def is_close(self, odom, point):
#         if (odom.pose.pose.position.x - point[0])**2 + \
#            (odom.pose.pose.position.y - point[1])**2 + \
#            (odom.pose.pose.position.z - point[2])**2 < 0.0004:
#             return True
#         return False
    
#     def detected_callback(self, msg):
#         self.det_flag = True

#     def odom_cb(self, msg: Odometry):
#         print(self.state)
#         self.cur_odom = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
#         if self.state == 0:
#             self.nextpoint = self.takeoff_point
#             if self.is_close(msg, self.takeoff_point):
#                 self.state = 1
#                 self.index = 0
#             else:
#                 self.num = int(np.linalg.norm(self.takeoff_point - self.origin) / 0.15)
#                 self.points = self.line_between_points(self.origin, self.takeoff_point, self.num)
#         if self.state == 1:
#             self.nextpoint = self.first_square_point_pre
#             if self.is_close(msg, self.first_square_point_pre):
#                 self.state = 2
#                 self.index = 0
#                 self.get_center_pos()
#             else:
#                 self.num = int(np.linalg.norm(self.first_square_point_pre - self.takeoff_point) / 0.15)
#                 self.points = self.line_between_points(self.takeoff_point, self.first_square_point_pre, self.num)
#         if self.state == 2:
#             self.nextpoint = self.circle_center_pos
#             if self.is_close(msg, self.circle_center_pos):
#                 self.state = 3
#                 self.index = 0
#             else:
#                 self.num = int(np.linalg.norm(self.circle_center_pos - self.first_square_point_pre) / 0.15)
#                 self.points = self.line_between_points(self.first_square_point_pre, self.circle_center_pos, self.num)
#         if self.state == 3:
#             self.nextpoint = self.first_square_point_end
#             if self.is_close(msg, self.first_square_point_end):
#                 self.state = 4
#                 self.index = 0
#                 self.get_center_pos()
#             else:
#                 self.num = int(np.linalg.norm(self.first_square_point_end - self.circle_center_pos) / 0.15)
#                 self.points = self.line_between_points(self.circle_center_pos, self.first_square_point_end, self.num)
#         if self.state == 4:
#             self.nextpoint = self.circle_center_pos
#             if self.is_close(msg, self.circle_center_pos):
#                 self.state = 5
#                 self.index = 0
#             else:
#                 self.num = int(np.linalg.norm(self.circle_center_pos-self.first_square_point_end) / 0.15)
#                 self.points = self.line_between_points(self.first_square_point_end, self.circle_center_pos, self.num)   
#         if self.state == 5:
#             self.nextpoint = self.land_point_end
#             if self.is_close(msg, self.land_point_end):
#                 # rospy.sleep(self.sleep_time)
#                 self.state = 6
#                 self.index = 0
#                 pass
#             else:
#                 self.num = int(np.linalg.norm(self.land_point_end-self.circle_center_pos) / 0.15)
#                 self.points = self.line_between_points(self.circle_center_pos, self.land_point_end, self.num)
#                 # self.pub(self.land_point_end)
#         if self.state == 6:
#             self.set_mode_client(0,'AUTO.LAND')
#             rospy.sleep(self.sleep_time)
#             self.arm_client(False)
#             pass



# if __name__ == "__main__":
#     # state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

#     # local_pos_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
#     # rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
#     # rospy.wait_for_service("/mavros/cmd/arming")
#     # arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

#     # rospy.wait_for_service("/mavros/set_mode")
#     # set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


#     # Setpoint publishing MUST be faster than 2Hz
#     # rate = rospy.Rate(20)

#     # # Wait for Flight Controller connection
#     # while(not rospy.is_shutdown() and not current_state.connected):
#     #     rate.sleep()
#     control = Controller()
#     pose = PositionTarget()
#     pose.type_mask = (PositionTarget.IGNORE_YAW_RATE |  
#                             PositionTarget.IGNORE_AFX |  
#                             PositionTarget.IGNORE_AFY |  
#                             PositionTarget.IGNORE_AFZ |
#                             PositionTarget.IGNORE_YAW)
#     pose.position.x = 0
#     pose.position.y = 0
#     pose.position.z = 0.2
#     pose.velocity.x = 1
#     pose.velocity.y = 0
#     pose.velocity.z = 0.5
#     pose.coordinate_frame = pose.FRAME_LOCAL_NED
#     # Send a few setpoints before starting
#     for i in range(100):
#         if(rospy.is_shutdown()):
#             break

#         control.pub_point.publish(pose)
        
#         control.rate.sleep()

#     offb_set_mode = SetModeRequest()
#     offb_set_mode.custom_mode = 'OFFBOARD'

#     arm_cmd = CommandBoolRequest()
#     arm_cmd.value = True

#     last_req = rospy.Time.now()

#     while(not rospy.is_shutdown()):
#         if(control.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
#             if(control.set_mode_client.call(offb_set_mode).mode_sent == True):
#                 rospy.loginfo("OFFBOARD enabled")

#             last_req = rospy.Time.now()
#         else:
#             if(not control.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
#                 if(control.arming_client.call(arm_cmd).success == True):
#                     rospy.loginfo("Vehicle armed")
#                     control.pub_flag = True

#                 last_req = rospy.Time.now()
                
#         control.rate.sleep()
