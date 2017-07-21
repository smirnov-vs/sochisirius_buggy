#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

typedef boost::function<void(const nav_msgs::Odometry&)> OdometryCallback;

int main(int argc, char* argv[]){
    ros::init(argc, argv, "odom_broadcaster");
    ros::NodeHandle n("~");
    tf::TransformBroadcaster broadcaster;

    const tf::Transform map_odom(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.0));
    tf::Transform transform;
    tf::Quaternion quaternion;

    OdometryCallback callback = [&] (const nav_msgs::Odometry& msg) {
        tf::quaternionMsgToTF(msg.pose.pose.orientation, quaternion);
        const auto& position = msg.pose.pose.position;
        transform.setOrigin(tf::Vector3(position.x, position.y, position.z));
        transform.setRotation(quaternion);

        const auto now = ros::Time::now();
        broadcaster.sendTransform(tf::StampedTransform(map_odom, now, "map", "odom"));
        broadcaster.sendTransform(tf::StampedTransform(transform, now, "odom", "base_link"));
    };

    std::string topic;
    n.param<std::string>("odom_topic", topic, "/odom");
    const auto subscriber = n.subscribe<nav_msgs::Odometry>(topic, 1, callback);
    ROS_INFO("odom_broadcaster: subscribed to topic %s", topic.c_str());

    ros::spin();
    return 0;
}
