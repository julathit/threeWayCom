import pygame
import cv2
import numpy as np

# Initialize Pygame
pygame.init()

# Set the width and height of the Pygame window
frame_width, frame_height = 640, 480
bar_height = 50
window_width, window_height = 2 * frame_width, frame_height + bar_height
window = pygame.display.set_mode((window_width, window_height))
pygame.display.set_caption("Dual Camera Feed")

# Open two camera streams
cam1 = cv2.VideoCapture(0)
cam2 = cv2.VideoCapture(2)

# Check if cameras opened successfully
if not cam1.isOpened() or not cam2.isOpened():
    print("Error: Could not open camera.")
    exit()

# Function to convert OpenCV image to Pygame image
def cv2_to_pygame(cv2_img):
    cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
    cv2_img = np.rot90(cv2_img)
    pygame_img = pygame.surfarray.make_surface(cv2_img)
    return pygame_img

# Function to draw a button
def draw_button(surface, text, x, y, w, h, color):
    pygame.draw.rect(surface, color, (x, y, w, h))
    font = pygame.font.Font(None, 36)
    text_surface = font.render(text, True, (255, 255, 255))
    surface.blit(text_surface, (x + (w - text_surface.get_width()) // 2, y + (h - text_surface.get_height()) // 2))

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            mouse = pygame.mouse.get_pos()
            if 50 <= mouse[0] <= 150 and frame_height + bar_height - 50 <= mouse[1] <= frame_height + bar_height:
                running = False

    # Capture frame-by-frame
    ret1, frame1 = cam1.read()
    ret2, frame2 = cam2.read()

    if not ret1 or not ret2:
        print("Error: Could not read frame.")
        break

    # Convert frames to Pygame surfaces
    frame1_surface = cv2_to_pygame(frame1)
    frame2_surface = cv2_to_pygame(frame2)

    # Clear the screen
    window.fill((0, 0, 0))

    # Display the frames
    window.blit(frame1_surface, (0, 0))
    window.blit(frame2_surface, (frame_width, 0))

    # Draw black bars under the camera feeds
    pygame.draw.rect(window, (128, 128, 128), (0, frame_height, 2*frame_width, bar_height))
    # pygame.draw.rect(window, (128, 128, 128), (frame_width, frame_height, frame_width, bar_height))

    # Draw the Quit button
    draw_button(window, "Quit", 50, frame_height + 10, 100, 30, (255, 0, 0))

    # Update the Pygame display
    pygame.display.flip()

# Release the cameras and close Pygame
cam1.release()
cam2.release()
pygame.quit()
