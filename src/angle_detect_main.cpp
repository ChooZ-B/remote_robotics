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
        ROS_INFO("usage: process <subscriber_topic const char*> <publisher_topic const char*>");
        return 1;
    }
 
    AngleServer a(argv[1],5);
    ros::spin();
    return 0;
}
