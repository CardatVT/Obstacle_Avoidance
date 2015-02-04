#ifndef OBSTACLE_AVOIDANCE
#define OBSTACLE_AVOIDANCE

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "object_detector.h"
#include "color_object_detector.h"
#include "surf_object_detector.h"
//#include "mavros/Mavlink.h"
//#include "mavros/mavros_plugin.h"


//#include "opencv2/videoio.hpp" // opencv3 only

/**
 * Runs different object detection modes. Processes resulting images with detected obstacles and makes decisions based on resolved features.
 * Opens all windows and allows for selection of different detection modes.
 *
 * Issues:
 * -Windows cannot be closed or programmatically toggled to have one appear at a time. Perhaps unable due to memory management done by opencv highgui?
 *     *- tried to open and close windows with namedWindow and destroyWindow, program hanged, seemed to be a bad idea
 *     *- Can eventually switch to qt for gui implementation
 */

class ObstacleAvoidance
{
public:
  static ObstacleAvoidance* Instance();
  ObstacleAvoidance();
 ~ObstacleAvoidance();

private:

  static ObstacleAvoidance* instance_;

  int threadSupported_;
  int obstacle_detection_mode_;

  ros::NodeHandle nh_; // used to initialize image transport
  image_transport::ImageTransport* it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ObjectDetector* current_object_detector_;
  std::vector<ObjectDetector*> object_detectors_;

  ColorObjectDetector* color_object_detector_;
  SurfObjectDetector* surf_object_detector_;

  void imageProcessCB(const sensor_msgs::ImageConstPtr& msg);
  void findColoredObjects(cv_bridge::CvImagePtr cv_ptr);
  static void trackbarDetectionModeCB(int next_mode);
};

#endif //OBSTACLE_AVOIDANCE
