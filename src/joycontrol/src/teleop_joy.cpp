#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class Teleop {

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    int linear_, angular_, boost_;
    double l_scale_, a_scale_, minboost_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;

public:
    Teleop();
};


Teleop::Teleop() :
        linear_(4),
        angular_(0),
        boost_(5) {

    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("axis_boost", boost_, boost_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    nh_.param("minboost", minboost_, minboost_);


    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    joy_sub_ = nh_.subscribe("joy", 10, &Teleop::joyCallback, this);
}

void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_ * joy->axes[angular_];

    double boost = (joy->axes[boost_] - 1) * (minboost_ - 1) / 2;

    twist.linear.x = l_scale_ * joy->axes[linear_] * (minboost_ + boost);
    vel_pub_.publish(twist);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "teleop_joy");
    Teleop teleop;

    ros::spin();
}
