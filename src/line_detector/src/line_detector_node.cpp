#include <ros/ros.h>

#include "line_detector.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_detector");
    LineDetector ic;
    ros::spin();
    return 0;
}
