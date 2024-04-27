#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import time

class Follower:
  def __init__(self):
    self.node_name = "rosbot_line_follower"
    rospy.init_node(self.node_name)
    self.bridge = cv_bridge.CvBridge()
    
    self.image_sub = rospy.Subscriber(
       '/camera/color/image_raw', 
      Image, self.image_callback)
    self.rplidar_sub = rospy.Subscriber(
        "/scan",
        LaserScan, self.lidar_callback)
    self.cmd_vel_pub = rospy.Publisher(
       '/cmd_vel',
       Twist,
       queue_size=1)
    self.speed = Twist()
    self.last_seen_angular_speed = 0
    self.front_object = False
    self.object_detection = False


  def lidar_callback(self, msg):
    ranges_len = len(msg.ranges)
    left_laser = msg.ranges[int(ranges_len*0.10):int(ranges_len*0.15)]
    right_laser = msg.ranges[int(ranges_len*0.85):int(ranges_len*0.90)]
    middle_laser1 = msg.ranges[int(ranges_len*0):int(ranges_len*0.05)]
    middle_laser2 = msg.ranges[int(ranges_len*0.95):int(ranges_len)]

    right_laser_min = min(right_laser)
    left_laser_min = min(left_laser)
    middle_laser1_min = min(middle_laser1)
    middle_laser2_min = min(middle_laser2)


    if left_laser_min < 0.18:
      self.speed.angular.z = -0.3
      self.speed.linear.x = 0
      self.cmd_vel_pub.publish(self.speed)
      print("left", left_laser_min)
      self.object_detection = True
    elif right_laser_min < 0.18:
      self.speed.angular.z = 0.3
      self.speed.linear.x = 0
      print("right", right_laser_min)
      self.cmd_vel_pub.publish(self.speed)
      self.object_detection = True
    elif middle_laser1_min < 0.16 or middle_laser2_min < 0.16:
      self.speed.angular.z = 0.3
      self.speed.linear.x = 0
      print("middle", right_laser_min, left_laser_min)
      self.cmd_vel_pub.publish(self.speed)
      self.front_object = True
      self.object_detection = True
    else:
      self.object_detection = False
      self.front_object = False


  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    cv2.imwrite("original.jpg", image)
    time.sleep(10)
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #cv2.imwrite("hsv_image.jpg", hsv_image)
    lower_white = np.array([0, 0, 180], dtype=np.uint8)
    upper_white = np.array([255, 50, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv_image, lower_white, upper_white)
    #cv2.imwrite("masked.jpg", mask)

    res = cv2.bitwise_and(image, image, mask=mask)
    #cv2.imwrite("bit.jpg", res)

    res_gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    height, width = res_gray.shape
    res_gray[:int(height*0.6), :] = 0 # Cut x % of the top image
    cv2.imwrite("res_grey.jpg", res_gray)
    res_edges = cv2.Canny(res_gray, 70, 150)
    #cv2.imwrite("edges.jpg", res_edges)

    left_half = res_gray[:, :int(width*0.60)]
    right_half = res_gray[:, int(width*0.40):]
    left_white_pixels = cv2.countNonZero(left_half)
    right_white_pixels = cv2.countNonZero(right_half)
    total_white_pixels = left_white_pixels + right_white_pixels

    if total_white_pixels > 0:
      turn_ratio = ((left_white_pixels - right_white_pixels) / total_white_pixels)/2

      # Adjust turn speed based on the ratio
      max_turn_speed = 0.5  # Maximum angular speed
      turn_speed = max_turn_speed * turn_ratio
    
    if not self.object_detection:
      if not self.front_object:
        if total_white_pixels > 2000:
          self.speed.angular.z = turn_speed
          self.last_seen_angular_speed = turn_speed
          self.speed.linear.x=0.1
        else:
          if self.last_seen_angular_speed > 0:
            self.speed.angular.z = 0.3
          else:
            self.speed.angular.z = -0.3
          self.speed.linear.x = 0
      else:
        while total_white_pixels < 2000:
          self.speed.angular.z = 0.3
          self.speed.linear.x = 0
          self.cmd_vel_pub.publish(self.speed)

    # print(self.speed)
    self.cmd_vel_pub.publish(self.speed)

follower = Follower()
rospy.spin()