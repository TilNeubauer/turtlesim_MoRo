#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <cmath>

using namespace std::chrono_literals;

class TurtleSimAutomata : public rclcpp::Node
{
public:
    TurtleSimAutomata() : Node("turtlesim_automata_node"), at_edge_(false), rotating_(false), moving_backward_(false), target_angle_(0.0), initial_orientation_set_(false)
    {
        // Create a publisher to send velocity commands to the turtle
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        
        // Create a subscription to receive the pose updates of the turtle
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleSimAutomata::poseCallback, this, std::placeholders::_1));
        
        // Create a timer to control the turtle's movement
        timer_ = this->create_wall_timer(500ms, std::bind(&TurtleSimAutomata::moveTurtle, this));

        // Seed the random number generator and set a random initial velocity
        std::srand(std::time(0));
        setRandomVelocity();
    }

private:
    // Callback function to handle updates to the turtle's pose
    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;

        // Set random initial orientation once at the beginning
        if (!initial_orientation_set_)
        {
            setRandomOrientation();
            initial_orientation_set_ = true;
        }

        // Detect if the turtle has reached the edge and handle accordingly
        if ((msg->x <= 1.0 || msg->x >= 10.0 || msg->y <= 1.0 || msg->y >= 10.0) && !rotating_ && !moving_backward_)
        {
            if (!at_edge_)
            {
                RCLCPP_INFO(this->get_logger(), "Edge detected!");
                at_edge_ = true;
                stopTurtle();
                moveBackward();
            }
        }
        else if (!(msg->x <= 0.0 || msg->x >= 11.0 || msg->y <= 0.0 || msg->y >= 11.0))
        {
            at_edge_ = false;
        }
    }

    // Function to move the turtle forward
    void moveTurtle()
    {
        if (!rotating_ && !moving_backward_)
        {
            publisher_->publish(velocity_);
        }
    }

    // Function to set a random initial orientation for the turtle
    void setRandomOrientation()
    {
        // Generate a random orientation angle between 0 and 2 * M_PI
        double random_orientation = static_cast<double>(std::rand()) / RAND_MAX * 2 * M_PI;
        geometry_msgs::msg::Twist twist;
        twist.angular.z = random_orientation;
        publisher_->publish(twist);
    }

    // Function to set a random initial linear velocity for the turtle
    void setRandomVelocity()
    {
        velocity_.linear.x = (std::rand() % 2) + 1.0;
        velocity_.angular.z = 0.0;
    }

    // Function to stop the turtle
    void stopTurtle()
    {
        velocity_.linear.x = 0.0;
        velocity_.angular.z = 0.0;
        publisher_->publish(velocity_);
    }

    // Function to move the turtle backward for a short duration
    void moveBackward()
    {
        moving_backward_ = true;
        velocity_.linear.x = -1.0;
        publisher_->publish(velocity_);
        
        // Move backward for one second
        backward_timer_ = this->create_wall_timer(1s, std::bind(&TurtleSimAutomata::stopMovingBackward, this));
    }

    // Function to stop moving backward and start rotating
    void stopMovingBackward()
    {
        velocity_.linear.x = 0.0;
        publisher_->publish(velocity_);
        moving_backward_ = false;
        rotateTurtle();  // Rotate after moving backward
        backward_timer_->cancel();
    }

    // Function to rotate the turtle 90 degrees clockwise
    void rotateTurtle()
    {
        rotating_ = true;
        initial_theta_ = current_pose_.theta;
        target_angle_ = fmod(initial_theta_ - M_PI_2, 2 * M_PI); // Rotate 90 degrees clockwise

        velocity_.angular.z = -0.5;  // Reduced rotation speed for more precision

        rotate_timer_ = this->create_wall_timer(10ms, std::bind(&TurtleSimAutomata::rotate, this));
    }

    // Function to handle the rotation of the turtle
    void rotate()
    {
        double angle_difference = target_angle_ - current_pose_.theta;

        // Ensure the angle difference is within the correct range
        if (angle_difference < -M_PI)
        {
            angle_difference += 2 * M_PI;
        }
        else if (angle_difference > M_PI)
        {
            angle_difference -= 2 * M_PI;
        }

        // Stop rotating if the target angle is reached
        if (fabs(angle_difference) < 0.01)
        {
            velocity_.angular.z = 0.0;
            publisher_->publish(velocity_);
            rotating_ = false;
            rotate_timer_->cancel();
            setRandomVelocity();
            publisher_->publish(velocity_);
        }
        else
        {
            publisher_->publish(velocity_);
        }
    }

    // ROS 2 components for publishing, subscribing, and timing
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr rotate_timer_;
    rclcpp::TimerBase::SharedPtr backward_timer_;

    // Variables to store the turtle's state and movement parameters
    geometry_msgs::msg::Twist velocity_;
    turtlesim::msg::Pose current_pose_;
    bool at_edge_;
    bool rotating_;
    bool moving_backward_;
    bool initial_orientation_set_;
    double target_angle_;
    double initial_theta_;
};

int main(int argc, char **argv)
{
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);
    // Create and spin the node
    rclcpp::spin(std::make_shared<TurtleSimAutomata>());
    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}

