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
  static ObjectDetector* Instance();
  ObjectDetector();
  virtual ~ObjectDetector();

  virtual void openWindow();
  virtual void findObjects(cv::Mat original_image);

};

#endif //OBJECT_DETECTOR
