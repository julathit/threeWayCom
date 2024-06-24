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
screen = pygame.display.set_mode((width * 2, height + 50))
pygame.display.set_caption('Camera Feeds')

class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image1_received = False
        self.image2_received = False
        self.cv_image1 = None
        self.cv_image2 = None
        # Publisher
        self.publisher = rospy.Publisher('chatter2', String, queue_size=10)

    def callback1(self, data):
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image1_received = True
        except CvBridgeError as e:
            rospy.logerr(e)

    def callback2(self, data):
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image2_received = True
        except CvBridgeError as e:
            rospy.logerr(e)

    def subscribe(self):
        rospy.init_node('image_subscriber', anonymous=True)
        rospy.Subscriber('camera/image', Image, self.callback1)
        rospy.Subscriber('camera/image2', Image, self.callback2)

    def get_image1(self):
        return self.cv_image1

    def get_image2(self):
        return self.cv_image2
    
    def publish_message(self, message):
        rospy.loginfo("Publishing: %s", message)
        self.publisher.publish(message)

# Function to draw a button
def draw_button(surface, text, x, y, w, h, color):
    pygame.draw.rect(surface, color, (x, y, w, h))
    font = pygame.font.Font(None, 36)
    text_surface = font.render(text, True, (255, 255, 255))
    surface.blit(text_surface, (x + (w - text_surface.get_width()) // 2, y + (h - text_surface.get_height()) // 2))

def main():
    image_subscriber = ImageSubscriber()
    image_subscriber.subscribe()

    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rospy.signal_shutdown("Pygame Quit")
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    rospy.signal_shutdown("ESC pressed")
                    pygame.quit()
                    sys.exit()
                else:
                    key_name = pygame.key.name(event.key)
                    print(f"Key pressed: {key_name}")
                    message = String()
                    message.data = key_name
                    image_subscriber.publish_message(message)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse = pygame.mouse.get_pos()
                if 50 <= mouse[0] <= 150 and height + 10 <= mouse[1] <= height + 40:
                    msg = String()
                    msg.data = "p"
                    image_subscriber.publish_message(msg)


        if image_subscriber.image1_received:
            cv_image1 = image_subscriber.get_image1()
            cv_image1 = cv2.resize(cv_image1, (width, height))
            pygame_frame1 = pygame.image.frombuffer(cv_image1.tobytes(), cv_image1.shape[1::-1], "BGR")
            screen.blit(pygame_frame1, (0, 0))
            image_subscriber.image1_received = False
        
        if image_subscriber.image2_received:
            cv_image2 = image_subscriber.get_image2()
            cv_image2 = cv2.resize(cv_image2, (width, height))
            pygame_frame2 = pygame.image.frombuffer(cv_image2.tobytes(), cv_image2.shape[1::-1], "BGR")
            screen.blit(pygame_frame2, (width, 0))
            image_subscriber.image2_received = False

        bar_height = 50             

        # Draw black bars under the camera feeds
        pygame.draw.rect(screen, (128, 128, 128), (0, height, 2*width, bar_height))

        # Draw the Quit button
        draw_button(screen, "Terminate", 50, height + 10, 120, 30, (255, 0, 0))

        pygame.display.flip()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        pygame.quit()
