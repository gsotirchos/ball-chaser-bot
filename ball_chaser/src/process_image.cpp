#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>


// Single class containing ROS entities and relevant state variables, inspired by Martin Peris' answer:
// https://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/?answer=59738#post-id-59738
class CommandRobotClient {
  public:
    CommandRobotClient()
      : current_lin_x_{-1},
        current_ang_z_{-1}
    {
        // Define a client_ service capable of requesting services from command_robot
        client_ = n_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

        // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
        sub_ = n_.subscribe(
            "/camera/rgb/image_raw",
            10,
            &CommandRobotClient::process_image_callback,
            this
        );
    }

    // This callback method continuously executes and reads the image data
    void process_image_callback(sensor_msgs::Image const & img) {
        int cnt = 0;
        bool ball_seen = false;
        int ball_pos;
        float lin_x;
        float ang_z;

        // Loop through each pixel in the image and check if it's white
        for (int i = 0; i < img.height * img.width; i++) {
            if (is_white_pixel_(img, i) == true) {
                cnt++;

                // when 3 consecutive white pixels are found determine the ball's position
                if (cnt == 3) {
                    ball_pos = (i % img.width) / (img.width / 3);
                    ball_seen = true;
                }

                // check if a big part of the image is covered by the ball
                if (cnt > 0.2 * img.height * img.width) {
                    ball_seen = false;
                    break;
                }
            }
        }

        // Set desired velocities according to ball position, if found
        if (ball_seen == true) {
            switch (ball_pos) {
                case 0:  // left
                    lin_x = 0.5;
                    ang_z = 5;
                    break;
                case 1:  // center
                    lin_x = 0.5;
                    ang_z = 0;
                    break;
                case 2:  // right
                    lin_x = 0.5;
                    ang_z = -5;
                    break;
            }
        } else {  // zero velocities
            lin_x = 0;
            ang_z = 0;
        }

        // If new velocities differ from existing then call the drive_robot service to set velocities
        if (lin_x != current_lin_x_ || ang_z != current_ang_z_) {
            drive_robot_(lin_x, ang_z);
            current_lin_x_ = lin_x;
            current_ang_z_ = ang_z;
        }
    }

  private:
    ros::NodeHandle n_;
    ros::ServiceClient client_;
    ros::Subscriber sub_;

    float current_lin_x_;
    float current_ang_z_;

    // This method calls the command_robot service to drive the robot in the specified direction
    void drive_robot_(float lin_x, float ang_z) {
        // Request desired vlelocities
        ball_chaser::DriveToTarget srv;
        srv.request.linear_x = lin_x;
        srv.request.angular_z = ang_z;

        // Call the command_robot service and pass the requested velocities
        if (!client_.call(srv)) {
            ROS_ERROR("Failed to call service command_robot");
        }
    }

    // This method checks whether the pixel at the specified location of a sensor_msgs/Image message data is white (i.e. if all its RGB values are 255)
    bool is_white_pixel_(sensor_msgs::Image const & img, int i) {
        for (int n = 0; n < 3; n++) {
            if (img.data[3 * i + n] != 255) {
                return false;
            }
        }

        return true;
    }
};  // class CommandRobotClient


int main(int argc, char * * argv) {
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");

    // Create a an object of class CommandRobotClient
    CommandRobotClient CRCObject;

    // Handle ROS communication events
    ros::spin();

    return 0;
}
