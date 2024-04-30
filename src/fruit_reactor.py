#!/usr/bin/env python3

import rospy
from rosbot_2_pro.msg import Fruits
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import rospkg
import os

class Reaction:
    def __init__(self):
        self.node_name = "fruit_reactor"
        rospy.init_node(self.node_name)
        self.subscriber = rospy.Subscriber("/fruit_detector", Fruits, callback=self.reaction, queue_size=1)

    def reaction(self, msg):
        if msg.fruit != "None":
            fruit = msg.fruit
            prob = msg.probs
            fruits = {'Fresh Apple': "good", 
                    'Fresh Banana':"good", 
                    'Fresh Guava':"good", 
                    'Fresh Orange':"good", 
                    'Fresh Pomegranate':"good", 
                    'Rotten Apple':"good", 
                    'Rotten Banana':"bad", 
                    'Rotten Guava':"good", 
                    'Rotten Orange':"good", 
                    'Rotten Pomegranate':"good", 
                    'Stale Apple':"good", 
                    'Stale Banana':"good", 
                    'Stale Guava':"good", 
                    'Stale Orange':"good", 
                    'Stale Pomegranate':"good"}
            print(fruit, prob)
            print(fruits[msg.fruit])
            """ImageAddress = os.path.join(rospkg.RosPack().get_path('rosbot_2_pro'), 'src/image_captures', 'warning.jpg')
            ImageItself = Image.open(ImageAddress)
            ImageNumpyFormat = np.asarray(ImageItself)
            plt.imshow(ImageNumpyFormat)
            plt.draw()
            plt.pause(5) # pause how many seconds
            plt.close()"""

 

if __name__ == '__main__':
    reactor = Reaction()
    #rospy.init_node(image_processor.node_name)
    rospy.spin()