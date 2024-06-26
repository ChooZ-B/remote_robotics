#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>
#include<sstream>
#include"img_manip.hpp"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"binary_manip");
    ros::NodeHandle nh;

    // deprecated
    //
    //if(argc < 2)
    //{
    //    ROS_INFO("usage: process <subscriber_topic const char*>");
    //    return 1;
    //}
    
    BinaryImg I("image/from_cam",10,10);

    ros::spin();
    
    //ROS_INFO("terminating node...\n");
    std::cout <<"\nterminating node..." << std::endl;
    return 0;
}
