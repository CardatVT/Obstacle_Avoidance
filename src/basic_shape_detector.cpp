#include "basic_shape_detector.h"

//ensure only one instance of class
BasicShapeDetector* BasicShapeDetector::instance_ = NULL;

//Constants
static const std::string BASIC_SHAPE_DETECTOR_WINDOW = "Obstacle Avoidance Find Basic Shapes";

using namespace cv; //still prefixing some cv calls as I'm learning. Want to understand whats in cv and whats not

//only supports circles for now

BasicShapeDetector* BasicShapeDetector::Instance()
{
    if(!instance_)
        instance_ = new BasicShapeDetector();

    return instance_;
}

BasicShapeDetector::BasicShapeDetector()
{

}

BasicShapeDetector::~BasicShapeDetector()
{

}

void BasicShapeDetector::openWindow()
{
    //open window to show processed image
    cv::namedWindow(BASIC_SHAPE_DETECTOR_WINDOW);

}

void BasicShapeDetector::findObjects(cv::Mat original_image)
{
    //////////// Detect Circle Shape////////////////////
    cv::Mat original_gray;
    /// Convert it to gray
    cvtColor( original_image, original_gray, CV_BGR2GRAY );

    /// Reduce the noise so we avoid false circle detection
    GaussianBlur( original_gray, original_gray, Size(3, 3), 2, 2 );

    std::vector<Vec3f> circles;

    /// Apply the Hough Transform to find the circles
    HoughCircles( original_gray, circles, CV_HOUGH_GRADIENT, 1, original_gray.rows/8, 100, 100, 0, 0 );

    /// Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ )
    {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // circle center
      circle( original_image, center, 3, Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      circle( original_image, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }

    //show circle in window
    cv::imshow(BASIC_SHAPE_DETECTOR_WINDOW,original_image);
    ///////  End Circle Detection   ////////////////
}

