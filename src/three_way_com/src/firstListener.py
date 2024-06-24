#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import pygame
import sys

pygame.init()
pygame.display.set_mode((100, 100))  # Create a small window (but we won't use it)

class ROSListenerPublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('listener_publisher', anonymous=True)

        # Subscriber
        rospy.Subscriber('chatter', String, self.callback)

        # Publisher
        self.publisher = rospy.Publisher('chatter2', String, queue_size=10)

        rospy.loginfo("ROS Listener and Publisher initialized.")

    def callback(self, data):
        rospy.loginfo("I heard: %s", data.data)

    def publish_message(self, message):
        rospy.loginfo("Publishing: %s", message)
        self.publisher.publish(message)

def main():
    listener_publisher = ROSListenerPublisher()

    while not rospy.is_shutdown():
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
                    listener_publisher.publish_message(message)

if __name__ == '__main__':
    main()
