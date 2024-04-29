#include<cv_bridge/cv_bridge.h>
#include<sstream>
#include<cmath>
#include"angle_detector.hpp"

AngleServer::AngleServer(const char* s_topic, int sq_size)
{
    int i;
    for(i = 0; i < 23; i++) angles_[i] = 0;

    sub_ = nh_.subscribe(s_topic,sq_size,&AngleServer::callback,this);
    server_ = nh_.advertiseService("rotor_pos/angle",&AngleServer::sendAngle,this);
    ROS_INFO("service ready for processing requests\n");
}

void AngleServer::callback(const remote_robotics::MotorImg& msg)
{
    int idx;
    cv_bridge::CvImagePtr cv_ptr;

    idx = msg.INDEX;
    ROS_INFO("%i\n",idx); //debugging

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg.IMG,"mono8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s\n",e.what());
        return;
    }

    try
    {
        if((idx < 0) || (idx > 23))
            throw std::string("motor index out of bounds in member function callback()");

        angles_[idx] = getAngle(cv_ptr->image);

    }
    catch(std::string e)
    {
        ROS_INFO("%s\n",e.c_str());
    }

    return;
}


bool AngleServer::sendAngle(remote_robotics::rotor_angle::Request& req,
                remote_robotics::rotor_angle::Response& res)
{
    if((req.INDEX < 0) || (req.INDEX > 23))
    {
        res.ANGLE = -361;
        return true;
    }

    res.ANGLE = static_cast<int>(angles[req.INDEX]);
    return false;
}

AnglePublisher::AnglePublisher(const char* s_topic, int sq_size, const char* p_topic, int pq_size)
{
    sub_ = nh_.subscribe(s_topic,sq_size,&AnglePublisher::callback, this);
    pub_ = nh_.advertise<remote_robotics::RotorAngle>(p_topic,pq_size);
}

void AnglePublisher::callback(const remote_robotics::MotorImg& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    remote_robotics::RotorAngle angle_msg;

    //ROS_INFO("%i\n",msg.INDEX); //debugging

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg.IMG,"mono8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s\n",e.what());
        return;
    }

    try
    {
        angle_msg.ANGLE = static_cast<int>(getAngle(cv_ptr->image));
        angle_msg.INDEX = msg.INDEX;

        pub_.publish(angle_msg);
    }
    catch(std::string e)
    {
        ROS_INFO("%s\n",e.c_str());
    }

    return;
}

double AngleDetector::getAngle(cv::Mat& bin)
{
    if(!isBinary(bin))
        throw std::string("Image container isn't binary");
    cv::Moments m = moments(bin,true);
    double a = m.mu20;
    double b = m.mu11;
    double c = m.mu02;

    double theta = atan2(b, a-c)/2.0;
    
    // return minus theta since the algorithm is counting the angle clockwise
    return -theta*180/PI;
}

bool AngleDetector::isBinary(cv::Mat& m)
{
    if(m.channels()>1)
        return false;
    int i,j;
    for(i = 0; i < m.rows; i++)
        for(j = 0; j < m.cols; j++)
            if(!((m.at<uchar>(i,j) == 0) || (m.at<uchar>(i,j) == 255)))
                return false;
    return true;
}
