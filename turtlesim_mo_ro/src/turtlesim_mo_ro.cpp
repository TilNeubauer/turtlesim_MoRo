
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <cmath>


//use namespace 
using namespace std::chrono_literals;


//class: move Turtle, inherit from rclcpp::Node
class Turtlesim_auto_move : public rclcpp::Node{
public:
    Turtlesim_auto_move() : Node("turtlesim_auto_move_node"), at_edge_(false), rotating_(false), moving_backward_(false), target_angle_(0.0), initial_orientation_set_(false){
        // init publisher for velocity to turtle
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        
        // init subscription to get pose from turtle
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&Turtlesim_auto_move::poseCallback, this, std::placeholders::_1));
        
        // int timer (fuer turltle mewegung)
        timer_ = this->create_wall_timer(500ms, std::bind(&Turtlesim_auto_move::moveTurtle, this));

        //rand number generator
        std::srand(std::time(0));

        //rand start velocity
        setRandomVelocity();
    }

private:
    // get pose
    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg){
        current_pose_ = *msg;

        // rand orientation
        if (!initial_orientation_set_){ //once at start 
            setRandomOrientation();
            initial_orientation_set_ = true;
        }

        // is edge??? -> react
        if ((msg->x <= 1.0 || msg->x >= 10.0 || msg->y <= 1.0 || msg->y >= 10.0) && !rotating_ && !moving_backward_){
            if (!at_edge_){
                RCLCPP_INFO(this->get_logger(), "Edge detected!");
                at_edge_ = true;
                stopTurtle(); //trutle stop
                moveBackward(); //turtle move back
            }
        }
        else if (!(msg->x <= 0.0 || msg->x >= 11.0 || msg->y <= 0.0 || msg->y >= 11.0)){ //not edge
            at_edge_ = false;
        }
    }

    // move forward
    void moveTurtle(){
        if (!rotating_ && !moving_backward_){ // check if doing something 
            publisher_->publish(velocity_);
        }
    }

    // set rand orientation
    void setRandomOrientation(){
        double random_orientation = static_cast<double>(std::rand()) / RAND_MAX * 2 * M_PI; //rand 0-2pi
        geometry_msgs::msg::Twist twist;
        twist.angular.z = random_orientation;
        publisher_->publish(twist);
    }

    // set rand velocity linear
    void setRandomVelocity(){
        velocity_.linear.x = (std::rand() % 2) + 1.0;
        velocity_.angular.z = 0.0;
    }

    // turtle stop
    void stopTurtle(){
        velocity_.linear.x = 0.0;
        velocity_.angular.z = 0.0;
        publisher_->publish(velocity_);
    }


    //turtle move back
    void moveBackward(){
        moving_backward_ = true;
        velocity_.linear.x = -1.0;
        publisher_->publish(velocity_);
        backward_timer_ = this->create_wall_timer(1s, std::bind(&Turtlesim_auto_move::stopMovingBackward, this)); // stop after 1s 
    }

    // stop moving backward + start rotating
    void stopMovingBackward(){
        velocity_.linear.x = 0.0; // stop
        publisher_->publish(velocity_); // publish
        moving_backward_ = false;


        rotateTurtle();  // start rotation
        backward_timer_->cancel();
    }

    //rotate 90 degrees clockwise
    void rotateTurtle(){
        rotating_ = true;
        initial_theta_ = current_pose_.theta; //get direction 
        target_angle_ = fmod(initial_theta_ - M_PI_2, 2 * M_PI); // Rotate 90 degrees clockwise

        velocity_.angular.z = -0.5;  // Reduced rotation speed --> more precision

        rotate_timer_ = this->create_wall_timer(10ms, std::bind(&Turtlesim_auto_move::rotate, this)); //start rotte
    }

    //rotation turtle
    void rotate(){
        double angle_difference = target_angle_ - current_pose_.theta;

        //check angle in correct range 
        if (angle_difference < -M_PI){
            angle_difference += 2 * M_PI;
        }
        else if (angle_difference > M_PI){
            angle_difference -= 2 * M_PI;
        }


        // Stop rotating when target angle 
        if (fabs(angle_difference) < 0.01){
            velocity_.angular.z = 0.0;
            publisher_->publish(velocity_);
            rotating_ = false;
            rotate_timer_->cancel();
            setRandomVelocity();
            publisher_->publish(velocity_);
        }
        else{
            publisher_->publish(velocity_);
        }
    }



    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // publisher_
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_; // subscription_
    rclcpp::TimerBase::SharedPtr timer_; // timing
    rclcpp::TimerBase::SharedPtr rotate_timer_; // timing
    rclcpp::TimerBase::SharedPtr backward_timer_; // timing


    // var state and movement param
    geometry_msgs::msg::Twist velocity_; // move velovitiy 
    turtlesim::msg::Pose current_pose_; // pos
    bool at_edge_;  
    bool rotating_;
    bool moving_backward_;
    bool initial_orientation_set_;
    double target_angle_;
    double initial_theta_;
};

int main(int argc, char **argv){
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);
    // Create and spin the node
    rclcpp::spin(std::make_shared<Turtlesim_auto_move>());
    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}

