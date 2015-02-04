#ifndef OBJECT_DETECTOR
#define OBJECT_DETECTOR

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/**
 * Describes the basis for a detector widget.
 */

class ObjectDetector
{
public:
  //pure virtual: requires derived type to implement these functions
  virtual void openWindow()=0;
  virtual void findObjects(cv::Mat original_image)=0;

};

#endif //OBJECT_DETECTOR
