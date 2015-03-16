#ifndef BASIC_SHAPE_DETECTOR
#define BASIC_SHAPE_DETECTOR

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "object_detector.h"

/**
 * Detector to discern different colored objects. Must tune sliders according to your environment to optimize detection.
 * NOTE: May not be ideal for moving camera. or for very effective object detection. but fun to make.
 */

class BasicShapeDetector: public ObjectDetector
{
public:
  static BasicShapeDetector* Instance();
 ~BasicShapeDetector();

  void openWindow();
  void closeWindow();
  void findObjects(cv::Mat cv_ptr);

private:
  BasicShapeDetector();
  BasicShapeDetector& operator=(BasicShapeDetector const&){}; //assignment private

  static BasicShapeDetector* instance_;

};



#endif //BASIC_SHAPE_DETECTOR
