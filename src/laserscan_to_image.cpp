#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>

using namespace cv;

class LaserScanToImage
{
  public:
  LaserScanToImage(ros::NodeHandle& nh);
  ~LaserScanToImage();

  private:
  void handleScan(const sensor_msgs::LaserScanConstPtr& scan);

  ros::NodeHandle& nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher image_pub_;
  float image_resolution_;
};

LaserScanToImage::LaserScanToImage(ros::NodeHandle& nh)
  : nh_(nh)
{
  std::string laserscan_topic, image_pub_topic;
  if (!ros::param::get("~laserscan_topic", laserscan_topic))
  {
    ROS_WARN("No laserscan_topic Given, using default.");
    laserscan_topic = "/scan";
  }
  if (!ros::param::get("~image_pub_topic", image_pub_topic))
  {
    ROS_WARN("No image_pub_topic Given, using default.");
    image_pub_topic = "/laserscan_to_image/image";
  }
  if (!ros::param::get("~image_resolution", image_resolution_))
  {
    ROS_WARN("No image_resolution Given, using default.");
    image_resolution_ = 0.01;
  }

  scan_sub_ =
    nh_.subscribe<sensor_msgs::LaserScan>(laserscan_topic, 3, &LaserScanToImage::handleScan, this);
  image_pub_ = nh_.advertise<sensor_msgs::Image>(image_pub_topic, 3);
}
LaserScanToImage::~LaserScanToImage()
{
}
void LaserScanToImage::handleScan(const sensor_msgs::LaserScanConstPtr& scan)
{
  ROS_INFO_THROTTLE(5, "Got Scan Msg");

  float range_min, range_max, angle_min, angle_max, angle_increment;
  int radius_pix, img_size_pix;
  float axis_ratio = 1. / 20;
  cv::Point2i img_center, img_axis_x_loc, img_axis_y_loc;
  cv::Size2i img_size;
  Mat scan_img;

  range_min = scan->range_min;
  range_max = scan->range_max;
  angle_min = scan->angle_min;
  angle_max = scan->angle_max;
  angle_increment = scan->angle_increment;

  radius_pix = range_max / image_resolution_ + 0.5;
  img_size_pix = 2 * range_max / image_resolution_ + 0.5;

  img_center = cv::Point2i(radius_pix, radius_pix);
  img_axis_x_loc = cv::Point2i(img_center.x + radius_pix * axis_ratio, img_center.y);
  img_axis_y_loc = cv::Point2i(img_center.x, img_center.y - radius_pix * axis_ratio);
  img_size = cv::Size2i(img_size_pix, img_size_pix);

  scan_img = cv::Mat(img_size, CV_8UC3, cv::Scalar(125, 125, 125));

  cv::line(scan_img, img_center, img_axis_x_loc, CV_RGB(255, 0, 0), 2);
  cv::line(scan_img, img_center, img_axis_y_loc, CV_RGB(0, 255, 0), 2);

  size_t range_size = scan->ranges.size();
  for (int i = 0; i < range_size; i++)
  {
    float range = scan->ranges[i];
    if (std::isnan(range))
    {
      continue;
    }
    if (range < range_min || range > range_max)
    {
      continue;
    }

    float radian;
    cv::Point2i point_loc;

    radian = i * angle_increment + angle_min;
    point_loc.x = img_center.x + cvRound(range * std::cos(radian) / image_resolution_);
    point_loc.y = img_center.y - cvRound(range * std::sin(radian) / image_resolution_);
    scan_img.at<Vec3b>(point_loc) = Vec3b(0, 0, 255);
  }

  Mat img_resize;
  cv::resize(scan_img, img_resize, cv::Size2i(640, 640), 0, 0, INTER_NEAREST);

  sensor_msgs::ImagePtr p_image;
  p_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_resize).toImageMsg();
  image_pub_.publish(p_image);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserscan_to_image");
  ros::NodeHandle nh;
  LaserScanToImage li(nh);

  ROS_INFO("laserscan_to_image start.");

  ros::spin();
  return 0;
}
