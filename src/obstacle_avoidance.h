#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include "opencv2/videoio.hpp"

class ObstacleAvoidance
{
public:
  ObstacleAvoidance();
 ~ObstacleAvoidance();

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport* it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  void imageProcessCB(const sensor_msgs::ImageConstPtr& msg);
};
