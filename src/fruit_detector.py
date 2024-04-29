#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv_bridge
import cv2
from ultralytics import YOLO
import rospkg
import os
from rosbot_2_pro.msg import Fruits


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
        #image = cv2.imread(os.path.join(rospkg.RosPack().get_path('rosbot_2_pro'), 'src/image_captures', 'original.jpeg'))
        #cv2.imshow('YOLO Object Detection', image)
        #cv2.waitKey(0)

        preds = self.model.predict(image)
        detected_objects = []
        for box in preds[0].boxes.data:
           detected_objects.append(preds[0].names.get(box[-1].item())) # Get name of detected objects
        print("PREDS", detected_objects)
        # print(preds)
        self.fruit_detected = False
        for pred in preds:
            probs = pred.boxes.conf.numpy()
            if len(probs) > 0:
                # print("PROBS", probs)
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