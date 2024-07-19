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
from std_msgs.msg import Bool
from collections import deque
from datetime import datetime, timedelta

class Algorithm:
    def __init__(self):

        rospy.init_node("arcuo_det")
        
        self.odom = Odometry()
        self.mavros_state = State()
        self.target_point = PoseStamped()
        self.bridge = CvBridge()
        self.detector = apriltag.Detector()
        self.point_count = 0
        self.circle_det_count = 0
        self.stay_time = 0.2
        self.circles_msg = circles()
        self.wait_dymatic = True
        self.msg_queue = deque()  # 队列用于保存消息和时间戳

        # 静态点数量
        self.point_num = 10

        # 跳跃到第几个点
        self.debug_jump_to = 0

        # 储存的静态点
        self.point = [None] * 15  

        # 当前状态机状态
        self.state = 0

        # 降落标志
        self.land_flag = 0

        # 穿越动环指令
        self.go = False

        self.odom_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_callback)
        self.mavros_state_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.circle_det_sub = rospy.Subscriber("/circle", circles, self.circle_det_callback)

        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        self.target_point_pub = rospy.Publisher("/waypoint_generator/waypoints", Path, queue_size=10)
        self.state_machine_state_pub = rospy.Publisher("/state_machine", Int8, queue_size=10)
        self.vel_cmd_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        self.setpoint_raw_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1, tcp_nodelay=True)
        self.state_machine_state_pub = rospy.Publisher("/state_machine", Int8, queue_size=10)
        self.fly_dynamic_pub = rospy.Publisher("/fly_dynamic",Bool,queue_size=10)
        self.load_params()
        
    def odom_callback(self, msg):
        self.odom = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg

    def image_callback(self, data):
        if self.state == 7:
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

    def circle_det_callback(self, msg):
        current_time = datetime.now()

        # 清理超过1.0秒的旧消息
        while self.msg_queue and (current_time - self.msg_queue[0][0]) > timedelta(seconds=1.0):
            self.msg_queue.popleft()

        # 将当前消息和时间戳添加到队列
        self.msg_queue.append((current_time, msg))

        # 存储当前消息
        self.circles_msg = msg
        #print("now message:", self.circles_msg.pos[0].y)

        # 访问0.5秒前的消息
        if len(self.msg_queue) > 1:
            self.time_1s_ago, self.msg_1s_ago = self.msg_queue[0]
            #print("1.0 seconds ago message:", self.msg_1s_ago.pos[0].y)
        

    def kill_node(self,node_name):
        try:
            subprocess.call(['rosnode', 'kill', node_name])
            rospy.loginfo(f"Node {node_name} killed successfully")
        except Exception as e:
            rospy.logerr(f"Failed to kill node {node_name}: {e}")
    
    def is_close(self, odom: Odometry, pose: list, z):
        # print(abs(odom.pose.pose.position.x - pose[0]), abs(odom.pose.pose.position.y - pose[1]), abs(odom.pose.pose.position.z - pose[2]))
        if abs(odom.pose.pose.position.x - pose[0]) < 0.1 and abs(odom.pose.pose.position.y - pose[1]) < 0.1 and abs(odom.pose.pose.position.z - pose[2]) < z:
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
                # 取消ego的地图
                if self.fly_dynamic == True:
                    bool_msg = Bool()
                    bool_msg.data = self.fly_dynamic
                    self.fly_dynamic_pub.publish(bool_msg)
                    print('clean part of grid map of ego-planner')
                print('waiting offboard')
                return
        
        # Taking off
        elif self.state == 1:
            if  self.is_close(self.odom, self.takeoff_point,0.10):
                time.sleep(self.stay_time)
                self.state = 2
                return
            else:
                self.go_to(self.takeoff_point)
                #self.state = 3
                return 
        
        # static point
        elif self.state == 2:
            if self.debug_jump_to-1 > self.point_count or self.is_close(self.odom, self.point[self.point_count],0.1):
                if self.point_count == self.point_num-1:
                    self.state = 3
                    #print(self.point_count)
                    return
                time.sleep(self.stay_time)
                self.state = 2
                self.point_count+=1
                return
            else:
                self.go_to(self.point[self.point_count])
                print(f'飞往第{self.point_count+1}个点')
                #print(self.point[self.point_count])
                return

        # static circle
        elif self.state == 3:
            # 调试或到达目标点附近
            if self.debug_jump_to > 11 or self.is_close(self.odom, self.static_circle,0.20):
                time.sleep(self.stay_time)
                self.state = 4
                return
            
            # 标点通过静态圆环
            elif self.fly_static ==False:
                self.go_to(self.static_circle)
                print(f'标点通过静圆:{self.static_circle}')

            # 识别通过静态圆环
            elif len(self.circles_msg.pos) > 0:
                if self.circle_det_count == 0:
                    self.static_circle[1]=0
                    self.static_circle[0]=0
                    self.circle_det_count+=1
                elif self.circle_det_count < 11:
                    time.sleep(self.stay_time)
                    self.static_circle[1]+=self.circles_msg.pos[0].y
                    self.static_circle[0]+=self.circles_msg.pos[0].x
                    self.circle_det_count+=1
                    return
                elif self.circle_det_count == 11:
                    self.static_circle[1]/=10
                    self.static_circle[0]/=10
                    self.static_circle[0]+=0.1
                    self.circle_det_count+=1
                    return
                else:
                    self.go_to(self.static_circle)
                    print(f'识别通过静圆: {self.static_circle}')
                    return
            else:
                return
        
        # led to next circle
        elif self.state == 4:
            if  self.debug_jump_to > 12 or self.is_close(self.odom, self.led_point,0.20):
                time.sleep(self.stay_time)
                self.state = 5
                time.sleep(self.stay_time*6) #必要
                return
            else:
                self.go_to(self.led_point)
                return 
        
        # dymatic circle
        elif self.state == 5:
            # 调试或到达目标点附近
            if self.debug_jump_to > 13 or self.is_close(self.odom, self.dymatic_circle,0.15):
                time.sleep(self.stay_time)
                self.state = 6
                return
            
            # 不通过动圆
            elif self.fly_dynamic == False:
                self.go_to(self.dymatic_circle)
                print(f'不通过动圆:{self.dymatic_circle}')

            # 识别通过动圆
            elif len(self.circles_msg.pos) > 0:
                # 判断是否通过动圆
                # 动圆移动范围（-13.8——-11.2）
                self.left_range = -13.8+0.2        ###????
                self.right_range = -11.2-0.2
                self.center = (self.left_range+self.right_range)/2.0   ###
                self.time = 0.40
                # 圆环向左移动，并且圆心在中轴线右边
                if (self.circles_msg.pos[0].y-self.odom.pose.pose.position.y > self.time and
                    self.circles_msg.pos[0].y-self.odom.pose.pose.position.y < self.time + 0.05
                    and self.wait_dymatic):
                    if (self.msg_1s_ago.pos[0].y-self.odom.pose.pose.position.y > self.time+ 0.05):
                        self.go = True
                        print("圆环向左移动且可通过")
                # 圆环向右移动，并且圆心在中轴线左边
                elif (self.odom.pose.pose.position.y-self.circles_msg.pos[0].y > self.time and
                      self.odom.pose.pose.position.y-self.circles_msg.pos[0].y < self.time + 0.05
                    and self.wait_dymatic):
                    if (self.odom.pose.pose.position.y-self.msg_1s_ago.pos[0].y > self.time+ 0.05):
                        self.go = True
                        print("圆环向右移动且可通过")

                if  (self.go): 
                    self.go_to(self.dymatic_circle)
                    print(f'识别通过动圆:{self.dymatic_circle}')
                    self.wait_dymatic = False
                elif self.wait_dymatic:
                    print("wait for dymatic circle")
                    self.go_to(self.led_point)
                    return
                else:
                    return
            else:
                return
        
        # land point
        elif self.state == 6:
            if  self.is_close(self.odom, self.land_point,0.15):
                self.state = 7
                return
            else:
                self.go_to(self.land_point)
                return

        # disarming
        elif self.state == 7:
            if  self.land_flag == 1:
                #self.set_mode_client(0,'Failsafe')
                self.set_mode_client(0,'AUTO.LAND')
                self.arming_client(False)
                self.state = 8
                self.land_flag = 0
                return
            else:
                return

        # finish
        elif self.state == 8:
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
        self.led_point = rospy.get_param('~led_point')
        self.dymatic_circle = rospy.get_param('~circle2')
        self.land_point = rospy.get_param('~land')
        self.fly_static =rospy.get_param('~fly_static')
        self.fly_dynamic =rospy.get_param('~fly_dynamic')
        self.range_time =rospy.get_param('~range_time')

if __name__ == "__main__":
    main_algorithm = Algorithm()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        main_algorithm.run()
        #print(main_algorithm.point)
        rate.sleep()
