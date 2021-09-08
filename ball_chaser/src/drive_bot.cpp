#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"


// Single class containing ROS entities and relevant state variables, inspired by Martin Peris' answer:
// https://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/?answer=59738#post-id-59738
class CommandRobotServer {
  public:
    CommandRobotServer() {
        // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
        motor_command_publisher_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
        service_ = n_.advertiseService("/ball_chaser/command_robot", &CommandRobotServer::handle_drive_request, this);
    }

    // This method publishes the requested linear x and angular velocities to the robot wheel joints
    bool handle_drive_request(
        ball_chaser::DriveToTarget::Request& req,
        ball_chaser::DriveToTarget::Response& res
    ) {
        // Create a motor_command object of type geometry_msgs::Twist
        geometry_msgs::Twist motor_command;

        // Set wheel velocities
        motor_command.linear.x = req.linear_x;
        motor_command.angular.z = req.angular_z;

        // Publish velocities to drive the robot
        motor_command_publisher_.publish(motor_command);

        // Return a response message
        res.msg_feedback = "Velocities set - linear x: " + std::to_string(req.linear_x) + " , angular z: " + std::to_string(req.angular_z);
        ROS_INFO_STREAM(res.msg_feedback);

        return true;
    }

  private:
    ros::NodeHandle n_;
    ros::ServiceServer service_;
    ros::Publisher motor_command_publisher_;
};  // class CommandRobotServer


int main(int argc, char * * argv) {
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Createa an object of class CommandRobotServer
    CommandRobotServer CRSObject;

    ROS_INFO("Ready to send commands");

    // Handle ROS communication events
    ros::spin();

    return 0;
}
