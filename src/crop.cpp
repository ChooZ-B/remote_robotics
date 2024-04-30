#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>
#include<sstream>
#include"img_manip.hpp"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"crop");
    ros::NodeHandle nh;

    if(argc < 3)
    {
        std::cout << "usage: process <subscriber_topic const char*> <publisher_topic const char*>" << std::endl;
        return 1;
    }
    
    ImgSegmentSpecial s(argv[1],10,argv[2], 10);

    ros::spin();

    std::cout <<"\nterminating node..." << std::endl;
    return 0;
}
