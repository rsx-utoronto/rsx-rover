import rospy
from sensor_msgs.msg import NavSatFix
import pygame
import os

# Pygame setup
pygame.init()
width, height = 800, 800  # Set dimensions for the Pygame window
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Rover Path Visualization")

# Load your image (replace 'your_image.png' with the actual image path)
image_dir = os.path.join(os.path.dirname(__file__), '..', 'images')  # Navigate one level up to find images
image_path = os.path.join(image_dir, 'Utah_Testing.png')  # Update this to your image file
background_image = pygame.image.load(image_path)
background_image = pygame.transform.scale(background_image, (width, height))  # Scale image to fit the window

# Initialize path points
path_points = []

def gps_callback(msg):
    # Convert GPS coordinates to pixels on the image
    lat = msg.latitude
    lon = msg.longitude
    
    # Example conversion function (this needs to be customized based on your image)
    x, y = convert_gps_to_pixels(lat, lon)

    path_points.append((x, y))

def convert_gps_to_pixels(latitude, longitude):
    # Convert GPS coordinates to pixel coordinates
    # Replace with your conversion logic based on your image's dimensions and GPS coordinates
    x = int((longitude + 110.7918) * (width / 0.003))  # Adjusting based on your actual coordinates
    y = int((38.4063 - latitude) * (height / 0.003))  # Adjusting based on your actual coordinates
    return x, y

def listener():
    rospy.init_node('gps_data_listener', anonymous=True)
    rospy.Subscriber("/gps/fix", NavSatFix, gps_callback)

    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
        
        # Draw the background image
        screen.blit(background_image, (0, 0))
        
        # Draw the path
        if len(path_points) > 1:
            pygame.draw.lines(screen, (255, 0, 0), False, path_points, 3)  # Draw the path in red

        pygame.display.flip()

if __name__ == '__main__':
    listener()
