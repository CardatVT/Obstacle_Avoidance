#ifndef SURF_OBJECT_DETECTOR
#define SURF_OBJECT_DETECTOR

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "object_detector.h"

//just currently testing if another detection module can be integrated

class SurfObjectDetector : public ObjectDetector
{
public:
  static SurfObjectDetector* Instance();
  SurfObjectDetector();
  virtual ~SurfObjectDetector();

   void openWindow();
   void findObjects(cv::Mat original_image);

private:
    static SurfObjectDetector* instance_;

};

#endif //SURF_OBJECT_DETECTOR
