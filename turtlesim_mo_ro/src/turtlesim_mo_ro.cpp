
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
    Turtlesim_auto_move() : Node("turtlesim_auto_move_node"), is_edge(false), is_rotating(false), is_moving_back(false), target_angle_(0.0), is_init_orient_set(false){
        // init publisher for velocity to turtle
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        
        // init subscription to get pose from turtle
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&Turtlesim_auto_move::get_pose, this, std::placeholders::_1));
        
        // int timer (fuer turltle mewegung)
        timer = this->create_wall_timer(500ms, std::bind(&Turtlesim_auto_move::move_turtle, this));

        //rand number generator
        std::srand(std::time(0));

        //rand start velocity
        set_rand_v();
    }

private:
    //ros 2 
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // publisher
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_; // subscription
    

    rclcpp::TimerBase::SharedPtr timer; // timing
    rclcpp::TimerBase::SharedPtr rot_timer; // timing movement rotation 
    rclcpp::TimerBase::SharedPtr back_timer; // timing movment backwords 


    // var state and movement param
    geometry_msgs::msg::Twist v; // move velovitiy 
    turtlesim::msg::Pose pos; // position of turtle
    
    //state var
    bool is_init_orient_set;
    bool is_edge; 
    bool is_rotating;
    bool is_moving_back;

    double initial_angle;
    double target_angle_;
    


    // get pose
    void get_pose(const turtlesim::msg::Pose::SharedPtr msg){
        pos = *msg;

        // rand orientation
        if (!is_init_orient_set){ //once at start 
            set_rand_orientation();
            is_init_orient_set = true;
        }

        // is edge??? -> react
        if ((msg->x <= 1.0 || msg->x >= 10.0 || msg->y <= 1.0 || msg->y >= 10.0) && !is_rotating && !is_moving_back){
            if (!is_edge){
                RCLCPP_INFO(this->get_logger(), "Edge detected!");
                is_edge = true;
                stopTurtle(); //trutle stop
                move_back(); //turtle move back
            }
        }
        else if (!(msg->x <= 0.0 || msg->x >= 11.0 || msg->y <= 0.0 || msg->y >= 11.0)){ //not edge
            is_edge = false;
        }
    }

    // move forward
    void move_turtle(){
        if (!is_rotating && !is_moving_back){ // check if doing something 
            publisher_->publish(v);
        }
    }

    // set rand orientation
    void set_rand_orientation(){
        double random_orientation = static_cast<double>(std::rand()) / RAND_MAX * 2 * M_PI; //rand 0-2pi
        geometry_msgs::msg::Twist twist;
        twist.angular.z = random_orientation;
        publisher_->publish(twist);
    }

    // set rand velocity linear
    void set_rand_v(){
        v.linear.x = (std::rand() % 2) + 1.0;
        v.angular.z = 0.0;
    }

    // turtle stop
    void stopTurtle(){
        v.linear.x = 0.0;
        v.angular.z = 0.0;
        publisher_->publish(v);
    }


    //turtle move back
    void move_back(){
        is_moving_back = true;
        v.linear.x = -1.0;
        publisher_->publish(v);
        back_timer = this->create_wall_timer(1s, std::bind(&Turtlesim_auto_move::stop_move_back, this)); // stop after 1s 
    }

    // stop moving backward + start rotating
    void stop_move_back(){
        v.linear.x = 0.0; // stop
        publisher_->publish(v); // publish
        is_moving_back = false;


        rotate_tutle_fkt();  // start rotation
        back_timer->cancel();
    }

    //rotate 90 degrees clockwise
    void rotate_tutle_fkt(){
        is_rotating = true;
        initial_angle = pos.theta; //get direction 
        target_angle_ = fmod(initial_angle - M_PI_2, 2 * M_PI); // Rotate 90 degrees clockwise

        v.angular.z = -0.5;  // Reduced rotation speed --> more precision

        rot_timer = this->create_wall_timer(10ms, std::bind(&Turtlesim_auto_move::rotate, this)); //start rotte
    }

    //rotation turtle
    void rotate(){
        double angle_difference = target_angle_ - pos.theta;

        //check angle in correct range 
        if (angle_difference < -M_PI){
            angle_difference += 2 * M_PI;
        }
        else if (angle_difference > M_PI){
            angle_difference -= 2 * M_PI;
        }


        // Stop rotating when target angle 
        if (fabs(angle_difference) < 0.01){
            v.angular.z = 0.0;
            publisher_->publish(v);
            is_rotating = false;
            rot_timer->cancel();
            set_rand_v();
            publisher_->publish(v);
        }
        else{
            publisher_->publish(v);
        }
    }

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

