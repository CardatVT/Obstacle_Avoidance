#include "color_object_detector.h"

//ensure only one instance of class
ColorObjectDetector* ColorObjectDetector::instance_ = NULL;

//Constants
static const std::string FIND_COLOR_WINDOW = "Obstacle Avoidance Find Color";

using namespace cv; //still prefixing some cv calls as I'm learning. Want to understand whats in cv and whats not

cv::RNG rng(12345);

ColorObjectDetector* ColorObjectDetector::Instance()
{
    if(!instance_)
        instance_ = new ColorObjectDetector();

    return instance_;
}

ColorObjectDetector::ColorObjectDetector()
{
    //setup Find Color window///////////
    lowH_ = 0; //hue
    highH_ = 179;
    lowS_ = 0; //saturation
    highS_ = 255;
    lowV_ = 0; //value
    highV_ = 255;

    threshold_ = 100;
    low_radius_ = 4;
    //need to track seperately as we should never set morph_radius_ to 0 (seg fault)
    //trackbars in opencv always have 0 as min...
    morph_trackbar_ = 5;
    morph_radius_ = 5;
}

ColorObjectDetector::~ColorObjectDetector()
{

}

void ColorObjectDetector::openWindow()
{
    //open window to show processed image
    cv::namedWindow(FIND_COLOR_WINDOW);

    //add trackbars to window
    cvCreateTrackbar("Low Hue",FIND_COLOR_WINDOW.c_str(),&lowH_,179);
    cvCreateTrackbar("High Hue",FIND_COLOR_WINDOW.c_str(),&highH_,179);
    cvCreateTrackbar("Low Saturation",FIND_COLOR_WINDOW.c_str(),&lowS_,255);
    cvCreateTrackbar("High Saturation",FIND_COLOR_WINDOW.c_str(),&highS_,255);
    cvCreateTrackbar("Low Value",FIND_COLOR_WINDOW.c_str(),&lowV_,255);
    cvCreateTrackbar("High Value",FIND_COLOR_WINDOW.c_str(),&highV_,255);
   // cvCreateTrackbar("Threshold",FIND_COLOR_WINDOW.c_str(),&threshold_,255);
    cvCreateTrackbar("Low Radius",FIND_COLOR_WINDOW.c_str(),&low_radius_,1000);
    cvCreateTrackbar("Morph Radius",FIND_COLOR_WINDOW.c_str(),&morph_trackbar_,20,ColorObjectDetector::morphTrackbarCB);
}

void ColorObjectDetector::morphTrackbarCB(int next_morph_radius)
{    
    //minimum can't be 0, or seg fault
    if(next_morph_radius != 0)
        ColorObjectDetector::Instance()->morph_radius_ = next_morph_radius;
}

/**
  NOTE::May not work well with moving camera, needs testing

 *Hue values of basic colors
 * orange 0-22
 * yellow 22-38
 * green 38-75
 * blue 75-130
 * violet 130-160
 * red 160-179
 *
 * Need to tune values to mask out everything but objects with the appropriate color via sliders
 *
 * Can be used to get rid of background and just show foreground objects, so can be color independent as well...
 *
 * Updates a seperate window to track transformations
 */
void ColorObjectDetector::findObjects(cv::Mat original_image)
{        
//    cv::Mat img_gray;

//    cv::cvtColor(original_image,img_gray,CV_BGR2GRAY);
//    cv::blur(img_gray,img_gray,cv::Size(3,3));
    cv::Mat img_HSV;

    //convert original image into HSV and store in new image
    cv::cvtColor(original_image,img_HSV,CV_BGR2HSV);

    cv::Mat img_threshold;

//    cv::threshold(img_HSV,img_threshold,threshold_,255,THRESH_BINARY);

    //apply threshold to image       if within bounds show white else black
    cv::inRange(img_HSV,cv::Scalar(lowH_,lowS_,lowV_),cv::Scalar(highH_,highS_,highV_),img_threshold);

    cv::dilate(img_threshold,img_threshold,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph_radius_,morph_radius_)));
    cv::dilate(img_threshold,img_threshold,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph_radius_,morph_radius_)));
    cv::erode(img_threshold,img_threshold,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph_radius_,morph_radius_)));
    cv::erode(img_threshold,img_threshold,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph_radius_,morph_radius_)));
    cv::erode(img_threshold,img_threshold,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph_radius_,morph_radius_)));
    cv::erode(img_threshold,img_threshold,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph_radius_,morph_radius_)));
    cv::dilate(img_threshold,img_threshold,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph_radius_,morph_radius_)));
    cv::dilate(img_threshold,img_threshold,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph_radius_,morph_radius_)));


//    //morphological closing
//    cv::erode(img_threshold,img_threshold,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph_radius_,morph_radius_)));
//    cv::dilate(img_threshold,img_threshold,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph_radius_,morph_radius_)));

//    //morphological opening
//    cv::dilate(img_threshold,img_threshold,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph_radius_,morph_radius_)));
//    cv::erode(img_threshold,img_threshold,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph_radius_,morph_radius_)));

    //cv::Mat img_canny;
    //canny edge detection
   // cv::Canny(img_threshold,img_canny,threshold_,threshold_*2,3);


    //find contours on image
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(img_threshold,contours,hierarchy, CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));

    std::vector<std::vector<cv::Point> > contours_poly(contours.size());
    std::vector<cv::Rect> boundRect(contours.size());
    std::vector<cv::Point2f>center (contours.size());
    std::vector<float> radius(contours.size());
    //std::vector<cv::RotatedRect> minEllipse(contours.size());

    //remove small contours , use area or length?
    //for


    for(int i=0;i<contours.size();i++)
    {
        approxPolyDP(Mat(contours[i]),contours_poly[i],3,true);
        //get bounding shapes based on contours
        boundRect[i] = cv::boundingRect(Mat(contours_poly[i]));
        cv::minEnclosingCircle((Mat)contours_poly[i],center[i],radius[i]);
        //minEllipse[i] = cv::fitEllipse(contours_poly[i]);
    }

    //TODO::calculate area if under a certain area throw away
//    for(int i=0;i<contours.size();i++)
//    {
//        if (radius[i] < low_radius_)
//    }

    //draw
    cv::Mat drawing = cv::Mat::zeros(img_threshold.size(), CV_8UC3);
    for(int i=0;i<contours.size();i++)
    {
        cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
        cv::drawContours(drawing,contours_poly,i,color,1,8,vector<Vec4i>(),0,Point());
        cv::rectangle(drawing,boundRect[i].tl(),boundRect[i].br(),color,2,8,0);
        //cv::ellipse(drawing,minEllipse[i]);
        cv::circle(drawing,center[i],(int)radius[i],color,2,8,0);
    }

    //show masked colored objects in window
    cv::imshow(FIND_COLOR_WINDOW,drawing);
}

