#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

float wheelbase;
std::string frame_id;
ros::Publisher pub;

inline float convert_trans_rot_vel_to_steering_angle(float v, float omega, float wheelbase) {
    return (omega == 0 || v == 0) ? 0 : std::atan(wheelbase * omega / v);
}

void cmd_callback(const geometry_msgs::Twist& data) {
    float v = (float) data.linear.x;
    float steering = convert_trans_rot_vel_to_steering_angle(v, (float) data.angular.z, wheelbase);

    ackermann_msgs::AckermannDriveStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.drive.steering_angle = steering;
    msg.drive.speed = v;

    pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ackermann_cpp_rewrite_node");
    ros::NodeHandle n;

    std::string twist_cmd_topic, ackermann_cmd_topic;

    if (!n.getParam("twist_cmd_topic", twist_cmd_topic)) {
        twist_cmd_topic = "/cmd_vel";
    }

    if (!n.getParam("ackermann_cmd_topic", ackermann_cmd_topic)) {
        ackermann_cmd_topic = "/ackermann_cmd";
    }

    if (!n.getParam("wheelbase", wheelbase)) {
        wheelbase = 1;
    }

    if (!n.getParam("frame_id", frame_id)) {
        frame_id = "odom";
    }

    ROS_INFO("%s\n%s\n%lf\n%s\n",
             twist_cmd_topic.c_str(),
             ackermann_cmd_topic.c_str(),
             wheelbase,
             frame_id.c_str());

    n.subscribe(twist_cmd_topic, 1, cmd_callback);
    pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(ackermann_cmd_topic, 1);

    ROS_INFO(
            "Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %lf",
            twist_cmd_topic.c_str(),
            ackermann_cmd_topic.c_str(), frame_id.c_str(), wheelbase);

    ros::spin();
    return 0;
}
