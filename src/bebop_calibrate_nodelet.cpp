

#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h> // nav_msgs/Odometry

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>

/**************************************************** OpenCv Libraries*/
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>

/**************************************************** C++ Libraries*/
#include <iostream>
#include <string>

/*************************************************** Custom libraries */
#include "VisualControlToolbox/VisualControlToolbox.h"
/*
#include <algorithm>
#include <string>
#include <vector>

#include <nodelet/loader.h>
// #include <rclcpp/rclcpp.hpp>
#include <ros/ros.h>*/

//  TODO: Clean dependencies

#define HERTZ 10

class bebopVSNodelet : public nodelet::Nodelet
{
  //  Image
  cv::Mat img;
  bool success = false;
  std::string output;
  ros::Subscriber m_Img;

  //  timer
  ros::Timer _timer;
  int n_captures = 1;
  int capture_id = 0;

public:
  //  Constructor, destructor
  bebopVSNodelet(){}
  ~bebopVSNodelet(){}

  //  INIT
  void onInit(){
    ros::NodeHandle nh;
    nh = getNodeHandle();

    float wait = nh.param(std::string("wait"), 1.);
    output = nh.param(std::string("output"), std::string(""));
    n_captures = nh.param(std::string("n_captures"), 1);

    //  Image transport
    m_Img = nh.subscribe("image_raw",2, & bebopVSNodelet::imageCallback, this);

    //  Timer config
    _timer = nh.createTimer(ros::Duration(wait), boost::bind(& bebopVSNodelet::timerCallback, this));
    ROS_INFO("[CALIBRATE] INIT");
  }

  //  Timer, each time it interrupts, save last received image
  void timerCallback(){
    // ROS_INFO("[CALIBRATE] Timer callback");
    if(success)
    {
      std::string name = output+std::to_string(capture_id)+".png";
      cv::imwrite(name, img);
      ROS_INFO("Image %s saved", name.c_str());
      if(n_captures <= capture_id)
        ros::shutdown();
      capture_id++;
    }
  }

  //  Save received image in img matrix
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
    // ROS_INFO("[CALIBRATE] Image callback");
    try{
        // img=cv_bridge::toCvShare(msg,"bgr8")->image;
        img=cv_bridge::toCvCopy(msg,"bgr8")->image;
        success = true;
    }catch (cv_bridge::Exception& e){
        NODELET_ERROR("Could not convert from '%s' to 'bgr8'.",
                  msg->encoding.c_str());
    }
  }

};

PLUGINLIB_EXPORT_CLASS( bebopVSNodelet, nodelet::Nodelet )
// bebopVSNodelet::bebopVSNodelet(){}
//
// bebopVSNodelet::~bebopVSNodelet() {}


// int main(int argc, char **argv){
//
//   //  INIT
//   ros::init(argc,argv,"bebop_calibrate");
//   ros::NodeHandle nh;
//   bebop drone;
//   drone.onInit(nh);
//
//
//   //  ROS
//   ros::Rate rate(HERTZ);
//
//   while(ros::ok()){
//     ros::spinOnce();
//
//     //  Stop whe all images have been saved
//     if(drone.capture_id >= drone.n_captures)
//       break;
//
//     rate.sleep();
//   }
//
//
//   return 0;
// }
