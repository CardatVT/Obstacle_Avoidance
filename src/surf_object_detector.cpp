#include "surf_object_detector.h"

SurfObjectDetector* SurfObjectDetector::instance_ = NULL;

using namespace cv;

//constants
static const std::string SURF_WINDOW = "Obstacle Avoidance SURF";

SurfObjectDetector* SurfObjectDetector::Instance()
{
    if(!instance_)
        instance_ = new SurfObjectDetector();

    return instance_;
}


SurfObjectDetector::SurfObjectDetector()
{

}

SurfObjectDetector::~SurfObjectDetector()
{

}

void SurfObjectDetector::openWindow()
{
    cv::namedWindow(SURF_WINDOW);
}

void SurfObjectDetector::findObjects(cv::Mat original_image)
{
    cv::imshow(SURF_WINDOW,original_image);
}
