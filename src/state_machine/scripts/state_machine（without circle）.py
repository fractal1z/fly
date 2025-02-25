#! /usr/bin/env python3
#coding=utf-8

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry, Path
import time
from std_msgs.msg import Int8
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import apriltag
import subprocess
from mavros_msgs.msg import AttitudeTarget
from circle_det.msg import circles

class Algorithm:
    def __init__(self):

        rospy.init_node("arcuo_det")
        
        self.odom = Odometry()
        self.mavros_state = State()
        self.target_point = PoseStamped()
        self.bridge = CvBridge()
        self.detector = apriltag.Detector()
        self.point_count = 0
        self.stay_time = 0.2

        # 静态点数量
        self.point_num = 10

        # 跳跃到第几个点
        self.debug_jump_to = 11

        # 储存的静态点
        self.point = [None] * 15  

        # 当前状态机状态
        self.state = 0

        # 降落标志
        self.land_flag = 0

        self.odom_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_callback)
        self.mavros_state_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        self.image_sub = rospy.Subscriber("/iris/camera/image_raw", Image, self.image_callback)
        self.circle_det_sub = rospy.Subscriber("/circle", circles, self.circle_det_callback)

        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        self.target_point_pub = rospy.Publisher("/waypoint_generator/waypoints", Path, queue_size=10)
        self.state_machine_state_pub = rospy.Publisher("/state_machine", Int8, queue_size=10)
        self.vel_cmd_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        self.setpoint_raw_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1, tcp_nodelay=True)
        
        self.load_params()
        
    def odom_callback(self, msg):
        self.odom = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg

    def image_callback(self, data):
        if self.state == 6:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                rospy.logerr(e)
                return

            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            detections = self.detector.detect(gray_image)
            
            if detections:
                print("AprilTag detected!")
                self.land_flag = 1
            else:
                print("AprilTag undetected!")
                self.land_flag = 0

    def kill_node(self,node_name):
        try:
            subprocess.call(['rosnode', 'kill', node_name])
            rospy.loginfo(f"Node {node_name} killed successfully")
        except Exception as e:
            rospy.logerr(f"Failed to kill node {node_name}: {e}")
    
    def is_close(self, odom: Odometry, pose: list, z):
        # print(abs(odom.pose.pose.position.x - pose[0]), abs(odom.pose.pose.position.y - pose[1]), abs(odom.pose.pose.position.z - pose[2]))
        if abs(odom.pose.pose.position.x - pose[0]) < 0.05 and abs(odom.pose.pose.position.y - pose[1]) < 0.05 and abs(odom.pose.pose.position.z - pose[2]) < z:
            return True
        return False
    
    def go_to(self, pose: list):
        path_to_pub = Path()
        path_to_pub.poses.clear()
        self.target_point.pose.position.x = pose[0]
        self.target_point.pose.position.y = pose[1]
        self.target_point.pose.position.z = pose[2]
        # self.target_point.pose.orientation.x = pose[3]
        # self.target_point.pose.orientation.y = pose[4]
        # self.target_point.pose.orientation.z = pose[5]
        # self.target_point.pose.orientation.w = pose[6]

        path_to_pub.poses.append(self.target_point)
        # print(path_to_pub)
        self.target_point_pub.publish(path_to_pub)

    def run(self):
        # publish state_now
        print('state: ', self.state)
        state_now = Int8()
        state_now.data = self.state
        self.state_machine_state_pub.publish(state_now)

        # Arming
        if self.state == 0:
            if self.mavros_state.mode == "OFFBOARD" :
                if self.mavros_state.armed == True :
                    self.state = 1
                    return
                self.arming_client.call(True)
                return
            else:
                print('waiting offboard')
                return
        
        # Taking off
        elif self.state == 1:
            if  self.is_close(self.odom, self.takeoff_point,0.05):
                time.sleep(self.stay_time)
                self.state = 2
                return
            else:
                self.go_to(self.takeoff_point)
                #self.state = 3
                return 
        
        # static point
        elif self.state == 2:
            if self.debug_jump_to > self.point_count or self.is_close(self.odom, self.point[self.point_count],0.5):
                if self.point_count == self.point_num-1:
                    self.state = 3
                    return
                time.sleep(self.stay_time)
                self.state = 2
                self.point_count+=1
                print(self.point_count)
                return
            else:
                self.go_to(self.point[self.point_count])
                #print(self.point[self.point_count])
                return

        # static circle
        elif self.state == 3:
            if self.debug_jump_to > 11 or self.is_close(self.odom, self.static_circle,0.15):
                time.sleep(self.stay_time)
                self.state = 4
                return
            else:
                self.go_to(self.static_circle)
                return
            
        # dymatic circle
        elif self.state == 4:
            if self.debug_jump_to > 12 or self.is_close(self.odom, self.dymatic_circle,0.15):
                time.sleep(self.stay_time)
                self.state = 5
                return
            else:
                self.go_to(self.dymatic_circle)

                return
        # land point
        elif self.state == 5:
            if  self.is_close(self.odom, self.land_point,0.4):
                self.state = 6
                return
            else:
                self.go_to(self.land_point)
                return

        # disarming
        elif self.state == 6:
            if  self.land_flag == 1:
                #self.set_mode_client(0,'Failsafe')
                self.set_mode_client(0,'AUTO.LAND')
                self.arming_client(False)
                self.state = 7
                self.land_flag = 0
                return
            else:
                return

        # finish
        elif self.state == 7:
            self.kill_node("track_mpc")
            u = AttitudeTarget()
            u.type_mask = AttitudeTarget.IGNORE_ATTITUDE
            u.body_rate.x = 0
            u.body_rate.y = 0
            u.body_rate.z = 0
            u.thrust = 0
            self.setpoint_raw_pub.publish(u)
            return
        
    def load_params(self):

        self.debug = rospy.get_param('~is_debug')
        self.takeoff_point = rospy.get_param('~takeoff')
        self.point[0] = rospy.get_param('~point1')
        self.point[1] = rospy.get_param('~point2')
        self.point[2] = rospy.get_param('~point3')
        self.point[3] = rospy.get_param('~point4')
        self.point[4] = rospy.get_param('~point5')
        self.point[5] = rospy.get_param('~point6')
        self.point[6] = rospy.get_param('~point7')
        self.point[7] = rospy.get_param('~point8')
        self.point[8] = rospy.get_param('~point9')
        self.point[9] = rospy.get_param('~point10')
        self.static_circle = rospy.get_param('~circle1')
        self.dymatic_circle = rospy.get_param('~circle2')
        self.land_point = rospy.get_param('~land')

if __name__ == "__main__":
    main_algorithm = Algorithm()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        main_algorithm.run()
        #print(main_algorithm.point)
        rate.sleep()
