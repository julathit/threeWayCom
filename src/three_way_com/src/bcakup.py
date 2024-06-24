#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

import pygame
import numpy as np
import sys


# Initialize Pygame
pygame.init()

# Set up the display
width, height = 640, 480
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption('Camera Feed')


class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_received = False
        self.cv_image = None
        # Publisher
        self.publisher = rospy.Publisher('chatter2', String, queue_size=10)

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_received = True
        except CvBridgeError as e:
            rospy.logerr(e)

    def subscribe(self):
        rospy.init_node('image_subscriber', anonymous=True)
        rospy.Subscriber('camera/image', Image, self.callback)

    def get_image(self):
        return self.cv_image
    
    def publish_message(self, message):
        rospy.loginfo("Publishing: %s", message)
        self.publisher.publish(message)

def main():
    image_subscriber = ImageSubscriber()
    image_subscriber.subscribe()

    while not rospy.is_shutdown():
        
        #image handeling
        if image_subscriber.image_received:
            cv_image = image_subscriber.get_image()
            cv_image = cv2.resize(cv_image,(width,height))
            # Convert the frame to a Pygame surface and blit it to the screen
            pygame_frame = pygame.image.frombuffer(cv_image.tostring(), cv_image.shape[1::-1], "BGR")
            screen.blit(pygame_frame, (0, 0))
            # Update the display
            pygame.display.flip()
            image_subscriber.image_received = False

        #control handeling
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:  # Example: press ESC to exit
                    pygame.quit()
                    sys.exit()
                else:
                    key_name = pygame.key.name(event.key)
                    print(f"Key pressed: {key_name}")
                    message = String()
                    message.data = key_name
                    image_subscriber.publish_message(message)

if __name__ == '__main__':
    main()
