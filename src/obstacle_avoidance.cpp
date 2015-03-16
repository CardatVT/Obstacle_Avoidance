#include "obstacle_avoidance.h"

ObstacleAvoidance* ObstacleAvoidance::instance_ = NULL;

//Constants
static const std::string ORIGINAL_IMAGE_WINDOW = "Obstacle Avoidance Original";

static const int MAX_OBJECT_DETECTION_MODES = 2; //indexed from 0
static const int COLOR_DETECTION_MODE = 0;
static const int SURF_DETECTION_MODE = 1;
static const int SHAPE_DETECTION_MODE = 2;


using namespace cv; //still prefixing some cv calls as I'm learning. Want to understand whats in cv and whats not


ObstacleAvoidance* ObstacleAvoidance::Instance()
{
    if(!instance_)
        instance_ = new ObstacleAvoidance();

    return instance_;
}

ObstacleAvoidance::ObstacleAvoidance()
{
    instance_ = this;

    //construct image transport from ros node handle
    it_ = new image_transport::ImageTransport(nh_);

    // Subscribe to input video feed
    image_sub_ = it_->subscribe("/camera/image_raw", 1, &ObstacleAvoidance::imageProcessCB, this);

    //open window to show original and processed image
    cv::namedWindow(ORIGINAL_IMAGE_WINDOW);       

    //initialize different tracking methods and their windows
    color_object_detector_ = ColorObjectDetector::Instance();
    color_object_detector_->openWindow();
    surf_object_detector_ = SurfObjectDetector::Instance();
    surf_object_detector_->openWindow();
    shape_object_detector_ = BasicShapeDetector::Instance();
    shape_object_detector_->openWindow();


    //store all object detectors for window management, order directly reflects trackbar
    object_detectors_.push_back(color_object_detector_);
    object_detectors_.push_back(surf_object_detector_);
    object_detectors_.push_back(shape_object_detector_);


    //default setup
    current_object_detector_ = color_object_detector_;
    obstacle_detection_mode_ = 0;
    //trackbar to select detection mode
    cvCreateTrackbar("Obstacle Detection Mode: 0-color 1-SURF 2-shape",ORIGINAL_IMAGE_WINDOW.c_str(),&obstacle_detection_mode_,MAX_OBJECT_DETECTION_MODES,ObstacleAvoidance::trackbarDetectionModeCB);

    //use seperate thread to handle window events returns 1 if can use another thread, 0 if cannot
    threadSupported_ = cv::startWindowThread();    
}

ObstacleAvoidance::~ObstacleAvoidance()
{
   delete it_;
   delete color_object_detector_;
   cv::destroyAllWindows();   
}


/**
 * callback had to be static or global, i used static
 * static singleton can now reference members inside member function for callback
 */
void ObstacleAvoidance::trackbarDetectionModeCB(int next_mode)
{    
    //switch detection modes
    ObstacleAvoidance::Instance()->current_object_detector_ = ObstacleAvoidance::Instance()->object_detectors_[next_mode];
}

void ObstacleAvoidance::imageProcessCB(const sensor_msgs::ImageConstPtr& msg)
{    
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        //convert to cvImage by copy so we can modify it
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& exception)
    {
        ROS_ERROR("cv_bridge exception: %s",exception.what());
        return;
    }
    //show original image
    cv::imshow(ORIGINAL_IMAGE_WINDOW,cv_ptr->image);

    //run object detection
    current_object_detector_->findObjects(cv_ptr->image);

    //TODO::need to average frames obtained to account for camera moving


    if(threadSupported_ == 0)
        cv::waitKey(3); //program waits for 3 ms before continuing, helps for window interaction?
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_avoidance_node");
  //run obstacle avoidance with ros
  ObstacleAvoidance oa;

  ros::spin();

  return 0;
}
