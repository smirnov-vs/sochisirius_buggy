#include "line_detector.h"

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <queue>

LineDetector::LineDetector() : it_(nh_)
{
    nh_.param("/line_detector/image_topic", image_topic, image_topic);
    nh_.param("/line_detector/grid_topic", grid_topic, grid_topic);
    nh_.param("/line_detector/odometry_topic", odometry_topic, odometry_topic);
    nh_.param("/line_detector/output_topic", output_topic, output_topic);
    nh_.param("/line_detector/image_out_topic", image_out_topic, image_out_topic);
    nh_.param("/line_detector/image_width", width, width);
    nh_.param("/line_detector/image_height", height, height);
    nh_.param("/line_detector/real_width", real_width, real_width);
    nh_.param("/line_detector/real_height", real_height, real_height);
    nh_.param("/line_detector/frequency", frequency, frequency);
    nh_.param("/line_detector/alpha", alpha, alpha);
    nh_.param("/line_detector/beta", beta, beta);
    nh_.param("/line_detector/gamma", gamma, gamma);
    nh_.param("/line_detector/white", white, white);

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(image_topic, 1,
        &LineDetector::imageCb, this);

    ROS_INFO("Subscribed to %s", image_topic.c_str());

    grid_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(grid_topic, 1,
        &LineDetector::occupancyCb, this);

    odometry_sub_ = nh_.subscribe<nav_msgs::Odometry>(odometry_topic, 1,
        &LineDetector::odometryCb, this);

    grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(output_topic, 1);

    image_pub_ = it_.advertise(image_out_topic, 1);

    src.emplace_back(std::round(((width * (1 - top_width)) / 2)),
        std::round(height * (1 - src_height)));

    src.emplace_back(std::round((width * (1 - top_width)) / 2 + width * top_width),
        std::round(height * (1 - src_height)));

    src.emplace_back(std::round((width * (1 - bottom_width)) / 2 + width * bottom_width),
        height - car_hood);

    src.emplace_back(std::round((width * (1 - bottom_width)) / 2),
        height - car_hood);

    ROS_INFO("[0]: (%f; %f)", src[0].x, src[0].y);
    ROS_INFO("[1]: (%f; %f)", src[1].x, src[1].y);
    ROS_INFO("[2]: (%f; %f)", src[2].x, src[2].y);
    ROS_INFO("[3]: (%f; %f)", src[3].x, src[3].y);

    dst.emplace_back(src[0].x, 0);

    dst.emplace_back(src[1].x, 0);
    
    dst.emplace_back(src[1].x, height);
    
    dst.emplace_back(src[0].x, height);

    ROS_INFO("[0]: (%f; %f)", dst[0].x, dst[0].y);
    ROS_INFO("[1]: (%f; %f)", dst[1].x, dst[1].y);
    ROS_INFO("[2]: (%f; %f)", dst[2].x, dst[2].y);
    ROS_INFO("[3]: (%f; %f)", dst[3].x, dst[3].y);

    M = cv::findHomography(src, dst);

    std::thread th(&LineDetector::process, this);
    th.detach();

    #ifdef DEBUG
    cv::namedWindow(SOURCE_WINDOW);
    cv::namedWindow(TEST_WINDOW);
    cv::namedWindow(OPENCV_WINDOW);
    #endif
}

LineDetector::~LineDetector()
{
    #ifdef DEBUG
    cv::destroyWindow(SOURCE_WINDOW);
    cv::destroyWindow(TEST_WINDOW);
    cv::destroyWindow(OPENCV_WINDOW);
    #endif
}

void LineDetector::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    auto start = Time::now();

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        if (enc::isColor(msg->encoding))
            cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        else
            cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    #ifdef DEBUG
    cv::Mat source;

    cv_ptr->image.copyTo(source);

    cv::line(source, cv::Point(src[3].x, src[3].y), cv::Point(src[0].x, src[0].y), CV_RGB(0, 0, 255), 2);
    cv::line(source, cv::Point(src[0].x, src[0].y), cv::Point(src[1].x, src[1].y), CV_RGB(0, 0, 255), 2);
    cv::line(source, cv::Point(src[1].x, src[1].y), cv::Point(src[2].x, src[2].y), CV_RGB(0, 0, 255), 2);
    cv::line(source, cv::Point(src[2].x, src[2].y), cv::Point(src[3].x, src[3].y), CV_RGB(0, 0, 255), 2);

    #endif

    cv::Mat img_out;

    cv::warpPerspective(cv_ptr->image, img_out, M, cv_ptr->image.size());

    cv::Mat img_cb;

    img_out.convertTo(img_cb, -1, alpha, beta);

    cv::Mat threshold;
    cv::inRange(img_cb, cv::Scalar(200, 200, 200), cv::Scalar(255, 255, 255), threshold);

    point_vector points;

    cv::Mat grayscale;

    cv::cvtColor(img_cb, grayscale, cv::COLOR_RGB2GRAY);

    cv::Mat equalized;

    cv::equalizeHist(grayscale, equalized);

    cv::Mat gammaMat;

    gammaCorrection(grayscale, gammaMat, gamma);

    cv::Mat grayThreshold;

    if(white){
        cv::inRange(gammaMat, 200, 255, grayThreshold);
    } else {
        cv::inRange(gammaMat, 0, 50, grayThreshold);
    }

    cv::goodFeaturesToTrack(grayThreshold, points, 1200, 0.002, 10);

    cv::Mat edgimg;

    img_cb.copyTo(edgimg);

    for (const auto& p : points){
        cv::circle(edgimg, p, 3, CV_RGB(0, 255, 0), 2);
    }

    { // Lock image queue
        std::lock_guard<std::mutex> lock(image_mutex);
        point_queue.emplace(std::move(points));

        if(point_queue.size() > max_point)
            point_queue.pop();
    } // --

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", edgimg).toImageMsg();
    
    image_pub_.publish(img_msg);

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(Time::now() - start);
    std::cout << "Frame took " << duration.count() << "ms" << std::endl;

    #ifdef DEBUG
    // Update GUI Windows
    cv::imshow(SOURCE_WINDOW, source);

    cv::imshow(TEST_WINDOW, grayThreshold);

    cv::imshow(OPENCV_WINDOW, edgimg);

    cv::waitKey(1);

    #endif
}

void LineDetector::occupancyCb(const nav_msgs::OccupancyGrid::ConstPtr& grid){
    std::lock_guard<std::mutex> lock(grid_mutex);
    grid_queue.push(grid);

    if(grid_queue.size() > max_grid)
        grid_queue.pop();
}

void LineDetector::odometryCb(const nav_msgs::Odometry::ConstPtr& odometry_){
    odometry = odometry_;
}

void LineDetector::process(){
    ros::Rate rate(frequency);

    point_vector point_vec;
    grid_ptr grid;

    while(nh_.ok()){
        rate.sleep();

        auto start = Time::now();

        int point_size = 0;
        int grid_size = 0;

        { //Lock both queues
            std::lock_guard<std::mutex> image_lock(image_mutex);
            std::lock_guard<std::mutex> grid_lock(grid_mutex);

            if(point_queue.empty() || grid_queue.empty()) {
                continue;
            }

            point_size = point_queue.size();
            grid_size = grid_queue.size();

            point_vec = point_queue.front();
            grid = grid_queue.front();
            point_queue.pop();
            grid_queue.pop();
        } // --

        nav_msgs::OccupancyGrid msg;

        const int gw = grid->info.width;
        const int gh = grid->info.height;
        const float resolution = grid->info.resolution;

        msg.info.height = gh;
        msg.info.width = gw;
        msg.info.resolution = resolution;
        msg.info.origin = grid->info.origin;
        msg.header = grid->header;
        msg.data = grid->data;

        for(const auto &point : point_vec){
            cv::Point2f position = imgToLocalSpace(point);

            double yaw = tf::getYaw(odometry->pose.pose.orientation);

            position = rotateVector(position, yaw);

            position += cv::Point2f(odometry->pose.pose.position.x, odometry->pose.pose.position.y);

            position += cv::Point2f(msg.info.origin.position.x, msg.info.origin.position.y);

            int py = static_cast<int>(std::round(position.y / resolution));

            int px = static_cast<int>(std::round(position.x / resolution));

            if (py < 0 || py >= gw || px < 0 || px >= gh)
                continue;

            msg.data[py + px * gw] = 100;
        }

        grid_pub_.publish(msg);

        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(Time::now() - start);
        std::cout << "Async frame took " << duration.count() / 1000 << "ms; " << point_size << "in point queue, " << grid_size << "in grid queue" << std::endl;
    }
}

cv::Point2f LineDetector::imgToLocalSpace(const cv::Point2f& point) {
    float x = (height - point.y) / height * real_height;
    float y = (point.x - (width / 2)) / width * real_width;
    return cv::Point2f(x, y);
}

cv::Point2f LineDetector::rotateVector(const cv::Point2f& v, double r){
    double ca = cos(r);
    double sa = sin(r);
    return cv::Point2f(ca*v.x - sa*v.y, sa*v.x + ca*v.y);
}

void LineDetector::gammaCorrection(const cv::Mat& src, cv::Mat& dst, float fGamma)
{
    unsigned char lut[256];
    for (int i = 0; i < 256; i++)
    {
        lut[i] = cv::saturate_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f);
    }

    dst = src.clone();
    const int channels = dst.channels();
    switch (channels)
    {
        case 1:
        {
            cv::MatIterator_<uchar> it, end;
            for (it = dst.begin<uchar>(), end = dst.end<uchar>(); it != end; ++it)
            *it = lut[(*it)];
            break;
        }
        case 3:
        {
            cv::MatIterator_<cv::Vec3b> it, end;
            for (it = dst.begin<cv::Vec3b>(), end = dst.end<cv::Vec3b>(); it != end; ++it)
            {
            (*it)[0] = lut[((*it)[0])];
            (*it)[1] = lut[((*it)[1])];
            (*it)[2] = lut[((*it)[2])];
            }
            break; 
        }
    }
}