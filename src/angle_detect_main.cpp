#include"angle_detector.hpp"

/*
 *
 * main function
 *
 */
int main(int argc, char** argv)
{
    ros::init(argc,argv,"angle_detection");

    if(argc < 2)
    {
        std::cout << "usage: process <subscriber_topic const char*>" << std::endl;
        return 1;
    }

    try
    {
      AngleServer a(argv[1],5);
    }
    catch(std::string msg)
    {
        ROS_ERROR("%s\n",msg.c_str());
        return 1;
    }
    ros::spin();

    std::cout << "\nterminating node..." << std::endl;
    return 0;
}
