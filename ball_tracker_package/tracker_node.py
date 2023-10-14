#!/usr/bin/env python3

## IMPORTS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


## SETUP
# Loading calibration data
calibration_data = np.load("calibration_data.npz")
CAMERA_MATRIX = calibration_data["mtx"] # Webcam's camera matrix values
DIST_COEFFS = calibration_data["dist"] # Webcam's distortion coefficients
BALL_RADIUS = 0.5  # Radius of the ball in meters
LOWER_COLOUR = (0, 100, 100)  # Lower bound for red color in HSV
UPPER_COLOUR = (10, 255, 255)  # Upper bound for red color in HSV


## HELPER FUNCTIONS
def undistort_image(image, camera_matrix, dist_coeffs):
    '''
    Undistorting the image using calibration data
    '''
    return cv2.undistort(image, camera_matrix, dist_coeffs)

def hsv_image(image):
    '''
    Converting the image from BGR to HSV
    '''
    return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

def mask_image(image, lower_colour, upper_colour):
    '''
    Creating a mask for the red color
    '''
    return cv2.inRange(image, lower_colour, upper_colour)

def find_contours(image):
    '''
    Finding contours in the mask
    '''
    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def calculate_depth(camera_matrix, detected_radius, real_radius):
    '''
    Calculating depth using calibration data, estimated radius and real radius of ball
    '''
    depth = (real_radius * camera_matrix[0][0]) / (detected_radius) # camera_matrix[0][0] is the focal length
    return depth

def get_ball_coordinates(contours):
    '''
    Finding the largest contour and calculating its centroid
    '''
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        (cx, cy), r = cv2.minEnclosingCircle(largest_contour)
        cz = calculate_depth(CAMERA_MATRIX, r, BALL_RADIUS)
        return (cx, cy, cz), r
    else:
        return None, None

def draw_ball(image, ball_coordinates, radius):
    '''
    Drawing a circle around the ball
    '''
    if ball_coordinates:
        cv2.circle(image, ball_coordinates, radius, (0, 255, 0), -1)
    return image

def display_image(image):
    '''
    Displaying the image
    '''
    cv2.imshow("Ball Tracking", image)
    cv2.waitKey(1)


## NODE CLASS
class BallTrackingNode(Node):
    def __init__(self):
        super().__init__('ball_tracking_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        '''
        Callback function for the image subscriber
        '''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
            return

        # Undistort the image
        undistorted_image = undistort_image(cv_image, CAMERA_MATRIX, DIST_COEFFS)

        # Convert the BGR image to HSV
        hsv_image = hsv_image(undistorted_image)

        # Create a mask for red color
        mask = mask_image(hsv_image, LOWER_COLOUR, UPPER_COLOUR)

        # Find contours in the mask
        contours = find_contours(mask)

        # Initialize variables for ball coordinates (centre of the ball)
        cx, cy, cz = None, None, None
        (cx, cy, cz), r = get_ball_coordinates(contours)
        
        # Draw a circle around the ball
        marked_ball = draw_ball(cv_image, (int(cx), int(cy)), int(r))

        # Display the processed image (for visualization)
        display_image(marked_ball)

        # Log the ball coordinates (Can publish to a topic later if necessary)
        self.get_logger().info(f"Ball coordinates: ({cx}, {cy}, {cz})" if cx and cy and cz else "No ball detected")


## MAIN FUNCTION
def main(args=None):
    rclpy.init(args=args)
    node = BallTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


## MAIN CALL
if __name__ == '__main__':
    main()