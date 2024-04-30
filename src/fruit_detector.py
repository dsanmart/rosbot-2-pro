#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv_bridge
import cv2
from ultralytics import YOLO
import rospkg
import os
from rosbot_2_pro.msg import Fruits
import numpy as np


class FruitDetector:
    def __init__(self, model_path):
        self.node_name = "rosbot_fruit_detector"
        rospy.init_node(self.node_name)
        self.bridge = cv_bridge.CvBridge()
        self.subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.detection_callback, queue_size=1)
        self.publisher = rospy.Publisher("/fruit_detector", Fruits, queue_size=1)
        self.model = YOLO(model_path)
        self.fruit_detected = False

    def detection_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #cv2.imwrite("detection.jpg", image)
        # Create the sharpening kerne
  
        # image = cv2.imread(os.path.join(rospkg.RosPack().get_path('rosbot_2_pro'), 'src/image_captures', 'prd.jpeg'))
        # coord = None
        # cy, cx = [i//2 for i in image.shape[:-1]] if coord is None else coord[::-1]
        # rot_mat = cv2.getRotationMatrix2D((cx, cy), 0, 1.5)
        # image = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
        # #image = cv2.addWeighted(image, 2, image, 0, 5)
        # kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]]) 
        
        # Sharpen the image 
        #image = cv2.filter2D(image, -1, kernel) 
        # image = cv2.convertScaleAbs(image, alpha=1, beta=10)
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # lower_brown = np.array([0,30,20])
        # upper_brown = np.array([30,130,150])
        # brown_mask = cv2.inRange(hsv, lower_brown, upper_brown)
        # cv2.imwrite("Transformed_image_1.jpeg", brown_mask)
        # kernel = np.ones((5,5), np.uint8)
        # brown_mask = cv2.morphologyEx(brown_mask, cv2.MORPH_OPEN, kernel=kernel)
        # res = cv2.bitwise_and(image, image, mask=brown_mask)
        # cv2.imwrite("Transformed_image.jpeg", image)
        #cv2.imshow('YOLO Object Detection', image)
        #cv2.waitKey(0)

        preds = self.model.predict(image)
        detected_objects = []
        for box in preds[0].boxes.data:
           detected_objects.append(preds[0].names.get(box[-1].item())) # Get name of detected objects
        #print("PREDS", detected_objects)
        #print(preds)
        self.fruit_detected = False
        for pred in preds:
            probs = pred.boxes.conf.numpy()
            if len(probs) > 0:
                print("PROBS", probs)
                print(detected_objects)

                for prob in range(len(probs)):
                    if probs[prob] >= 0.9:
                        # print("FOUND SOMETHING", prob)
                        self.fruit_detected = True
                if self.fruit_detected:
                    #print("PREDS", detected_objects[0])
                    self.publisher.publish(detected_objects[0], probs[0])
                    #cv2.imwrite("prd.jpg", pred.plot())
                    # plot_img = pred.plot()
                    # cv2.imshow('YOLO Object Detection', plot_img)
                    # cv2.waitKey(0)
                else:
                    self.publisher.publish("None", 0.0)
            else:
                self.publisher.publish("None", 0.0)
 

if __name__ == '__main__':
    model_path = os.path.join(rospkg.RosPack().get_path('rosbot_2_pro'), 'src/models', 'yolo_object_detection.pt')
    image_processor = FruitDetector(model_path)
    #rospy.init_node(image_processor.node_name)
    rospy.spin()