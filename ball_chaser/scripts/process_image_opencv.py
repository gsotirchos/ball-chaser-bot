#!/usr/bin/env python
import rospy
from ball_chaser.srv import DriveToTarget
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from numpy import round, floor
import cv2
from cv_bridge import CvBridge


class CommandRobotClient:
    def __init__(self):
        # Define a _client service capable of requesting services from command_robot
        rospy.wait_for_service("/ball_chaser/command_robot")
        self._client = rospy.ServiceProxy("/ball_chaser/command_robot", DriveToTarget)

        # Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
        self._pub1 = rospy.Subscriber("/camera/rgb/image_raw", Image, self._process_image_callback)

        # Object to convert ROS image message to OpenCV image
        self._bridge = CvBridge()

        # Store last issued velocities
        self._current_lin_x = -1
        self._current_ang_z = -1

    # This method calls the command_robot service to drive the robot in the specified direction
    def _drive_robot(self, lin_x, ang_z):
        # Call the command_robot service and pass the requested velocities
        try:
            resp = self._client(lin_x, ang_z)
            return resp.msg_feedback
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    # This method processes the image, detects, and returns the location of a ball (circle)
    def _detect_ball(self, img):
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        # adjust alpha and beta
        alpha=2
        beta=0
        adj = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)

        # sobel
        x_sobel = cv2.Sobel(adj, ddepth=cv2.CV_32F, dx=1, dy=0, ksize=3)
        y_sobel = cv2.Sobel(adj, ddepth=cv2.CV_32F, dx=0, dy=1, ksize=3)

        x_sobel = cv2.convertScaleAbs(x_sobel)
        y_sobel = cv2.convertScaleAbs(y_sobel)

        combined = cv2.addWeighted(x_sobel, 0.5, y_sobel, 0.5, 0)

        # detect circle in the image
        dp = 1
        minDist = 700
        param1 = 1100
        param2 = 20
        minRadius = 10
        maxRadius = 90

        circle = cv2.HoughCircles(
            combined,
            method=cv2.HOUGH_GRADIENT,
            dp=dp,
            minDist=minDist,
            param1=param1,
            param2=param2,
            minRadius=minRadius,
            maxRadius=maxRadius)

        if circle is not None:
            circle_data = dict(
                zip({"x", "y", "r"},
                    round(circle[0, 0]).astype("int")))

            return circle_data

    # This callback method continuously executes and reads the image data
    def _process_image_callback(self, data):
        # convert ROS image message to OpenCV image
        img = self._bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

        # detect the ball location in the image
        ball = self._detect_ball(img)
        if ball is not None:
            ball_pos = int(floor(ball["x"] / (img.shape[1] / 3)))
            msg = ("Ball (x, y, r): ("
                + str(ball["x"]) + ", "
                + str(ball["y"]) + ", "
                + str(ball["r"]) + ")")

            # Set desired velocities according to ball position, if found
            if ball_pos == 0:  # left
                lin_x = 0.5
                ang_z = 5
            elif ball_pos == 1:  # center
                lin_x = 0.5
                ang_z = 0
            elif ball_pos == 2:  # right
                lin_x = 0.5
                ang_z = -5
        else:  # zero velocities
            msg = "Ball position: NONE"
            lin_x = 0
            ang_z = 0

        if (lin_x != self._current_lin_x or ang_z != self._current_ang_z):
            rospy.loginfo(msg)
            self._drive_robot(lin_x, ang_z)
            self._current_lin_x = lin_x
            self._current_ang_z = ang_z

def main():
    # Initialize the process_image node and create a handle to it
    rospy.init_node("process_image_opencv", anonymous=True)

    # Create a an object of class CommandRobotClient
    commandRobotClient = CommandRobotClient()

    # Handle ROS communication events
    rospy.spin()

if __name__ == "__main__":
    main()
