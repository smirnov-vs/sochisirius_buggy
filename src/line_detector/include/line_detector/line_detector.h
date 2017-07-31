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


//#define DEBUG

namespace enc = sensor_msgs::image_encodings;

#ifdef DEBUG
static const std::string SOURCE_WINDOW = "Source window";
static const std::string OPENCV_WINDOW = "Image window";
static const std::string TEST_WINDOW = "Test window";
#endif



typedef std::vector<cv::Point2f> point_vector;
typedef boost::shared_ptr<const nav_msgs::OccupancyGrid> grid_ptr;

typedef std::chrono::high_resolution_clock Time;

class LineDetector
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Subscriber grid_sub_;
    ros::Subscriber odometry_sub_;
    ros::Publisher grid_pub_;

    std::string image_topic = "/rgb/image_rect_color";
    std::string grid_topic = "/rtabmap/grid_map";
    std::string odometry_topic = "/zed/odom";
    std::string output_topic = "/line_detector/map";
    std::string image_out_topic = "/line_detector/image";

    float bottom_width = 1; // percentage of image width
    float top_width = 0.142; // percentage of image width
    float src_height = 0.46; // percentage of image height
    int car_hood = 10; //number of pixels to be cropped from bottom meant to get rid of car’s hood

    int height = 720;
    int width = 1280;

    boost::shared_ptr<const nav_msgs::Odometry> odometry;

    point_vector src;
    point_vector dst;
    cv::Mat M;

    std::mutex image_mutex;
    std::mutex grid_mutex;

    std::queue<point_vector> point_queue;
    std::queue<grid_ptr> grid_queue;

    float real_width = 2;
    float real_height = 2;

    double frequency = 20;

    double alpha = 2.5;
    double beta = -20.0;

    float gamma = 1;

    int max_point = 5;
    int max_grid = 5;

    bool white = true;

    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    void occupancyCb(const nav_msgs::OccupancyGrid::ConstPtr& grid);

    void odometryCb(const nav_msgs::Odometry::ConstPtr& odometry_);

    void process();

    //################################################
    //#       On image:       #    In worldspace:    #
    //#                       #                      #
    //#      O - - - > x      #          ^ x         #
    //#      |                #          |           #
    //#      |                #          |           #
    //#      |                #          |           #
    //#      " y              #          O - - - > y #
    //#                       #        camera        #
    //################################################

    inline cv::Point2f imgToLocalSpace(cv::Point2f point);

    inline cv::Point2f rotateVector(cv::Point2f v, double r);

    void gammaCorrection(cv::Mat& src, cv::Mat& dst, float fGamma);

public:
    LineDetector();

    ~LineDetector();
};