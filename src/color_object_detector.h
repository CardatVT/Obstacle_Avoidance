#ifndef COLOR_OBJECT_DETECTOR
#define COLOR_OBJECT_DETECTOR

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "object_detector.h"

/**
 * Detector to discern different colored objects. Must tune sliders according to your environment to optimize detection.
 * NOTE: May not be ideal for moving camera. or for very effective object detection. but fun to make.
 */

class ColorObjectDetector: public ObjectDetector
{
public:
  static ColorObjectDetector* Instance();
 ~ColorObjectDetector();

  void openWindow();
  void closeWindow();
  void findObjects(cv::Mat cv_ptr);

private:
  ColorObjectDetector();
  ColorObjectDetector& operator=(ColorObjectDetector const&){}; //assignment private

  static ColorObjectDetector* instance_;

  int lowH_; //hue
  int highH_;
  int lowS_; //saturation
  int highS_;
  int lowV_; //value
  int highV_;

  int threshold_;
  int morph_trackbar_;
  int morph_radius_;

  static void morphTrackbarCB(int next_morph_radius);
};



#endif //COLOR_OBJECT_DETECTOR
