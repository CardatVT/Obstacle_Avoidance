#include "obstacle_avoidance.h"


static const std::string OPENCV_WINDOW = "Obstacle Avoidance";
int threadtest_;

ObstacleAvoidance::ObstacleAvoidance()
{
    //construct image transport from ros node handle
    it_ = new image_transport::ImageTransport(nh_);

    // Subscribe to input video feed
    image_sub_ = it_->subscribe("/camera/image_raw", 1, &ObstacleAvoidance::imageProcessCB, this);
    //publish output video feed
    //image_pub_ = it_->advertise("/image_converter/output_video", 1);

    //open window to show processed output
    cv::namedWindow(OPENCV_WINDOW);

    //use seperate thread to handle window events returns 1 if can use another thread, -1 if cannot
    threadtest_ = cv::startWindowThread();
}

ObstacleAvoidance::~ObstacleAvoidance()
{
   delete it_;
   cv::destroyWindow(OPENCV_WINDOW);
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

    //draw circle                           radius=10       B  G  R thickness=1
    cv::circle(cv_ptr->image,cv::Point(50,50),10,cv::Scalar(0,255,0),1);

    cv::imshow(OPENCV_WINDOW,cv_ptr->image);

    //program waits for 3 ms before continuing
    cv::waitKey(3);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_avoidance_node");
  //run obstacle avoidance with ros

  ObstacleAvoidance oa;

  ros::spin();

 
  return 0;
}
