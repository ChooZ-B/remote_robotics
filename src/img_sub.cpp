#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>
#include<sstream>
#include<cstdlib>
#include"remote_robotics/MotorImg.h"

std::string win_name;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg,"");

        ROS_INFO("resolution: %dx%d\n",cv_ptr->image.cols,cv_ptr->image.rows);
        cv::imshow("view",cv_ptr->image); //eventually window title based in topic
        cv::waitKey(30);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s\n",e.what());
    }
    return;
}
void motorImageCallback(const remote_robotics::MotorImg& msg)
{
    int idx;
    cv_bridge::CvImagePtr cv_ptr;

    idx = msg.INDEX;
    //ROS_INFO("%i\n",idx); //debugging

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg.IMG,"");
        //char idx_str[3];
        //itoa(msg.INDEX,idx_str,10);
        win_name = "motor" + std::to_string(msg.INDEX);//(const char*)idx_str;
        
        ROS_INFO("resolution: %dx%d",cv_ptr->image.cols,cv_ptr->image.rows);
        cv::imshow("view",cv_ptr->image);
        cv::waitKey(30);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s\n",e.what());
        return;
    }

}



int main(int argc, char** argv)
{
    ros::init(argc, argv,"image_viewer");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    cv::namedWindow("view",cv::WINDOW_AUTOSIZE/* | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED*/);

    if(argc < 3)
    {
        std::cout << "usage: process <msg type (ImageTransport or MotorImg)> <subscriber_topic const char*>" << std::endl;
        return 1;
    }


    if ((std::string(argv[1]).compare(0,8,"MotorImg") == 0))
    {
        ROS_INFO("subscribing to MotorImage topic");
        ros::Subscriber sub = nh.subscribe(argv[2],5,motorImageCallback);
    }
    else // image transport msgs are default option
    {
        ROS_INFO("subscribing to ImageTransport topic");
        win_name="view";
        image_transport::Subscriber sub = it.subscribe(argv[2],5,imageCallback);
    }


    ros::spin();
    cv::destroyWindow(win_name);
    
    std::cout << "terminating node..." << std::endl;
    return 0;
}
