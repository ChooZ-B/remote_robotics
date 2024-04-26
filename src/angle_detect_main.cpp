#include"angle_detector.hpp"

/*
 *
 * main function
 *
 */
int main(int argc, char** argv)
{
    ros::init(argc,argv,"angle_detection");

    if(argc < 3)
    {
        ROS_INFO("usage: process <subscriber_topic const char*>\n");
        return 1;
    }
 
    try
    {
        AngleServer a(argv[1],5);
    }
    catch(std::string msg)
    {
        ROS_ERROR("%s\n",msg.c_str());
        return 0;
    }
    ros::spin();
    return 0;
}