/**
 *
 * \file angle_detector.hpp
 *
 * \author Niels Toepler
 *
 * \date 04.03.2024
 *
 * Contains declaration for class AngleDetector
 *
 */
#ifndef ANGLE_DETECT_INCLUDED
#define ANGLE_DETECT_INCLUDED


#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<sensor_msgs/Image.h>
#include"remote_robotics/rotor_angle.h"
#include"remote_robotics/RotorAngle.h"
#include"remote_robotics/MotorImg.h"

#define PI 3.14159265

/**
 * \class AngleDetector
 *
 * \brief Detects angle of object.
 *
 */
class IAngleDetector{

    public:
        /**
         * \brief Computes object orientation using the least second moment of the binary image.
         *
         * \param bin a preferrably binary matrix
         *
         * \throws string error message if matrix isn't binary
         *
         * \return angle
         *
         */
        virtual double getAngle(cv::Mat& bin) = 0;
        //virtual ~IAngleDetector();

    protected:
        /**
         * \brief Checks if Mat is binary
         *
         * \param m an opencv matrix object
         *
         */
        virtual bool isBinary(cv::Mat& m) = 0;
};

class AngleDetector : public IAngleDetector{

    protected:
        bool isBinary(cv::Mat& m);

    public:
        double getAngle(cv::Mat& bin);
};

class AnglePublisher : public AngleDetector{

    protected:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;

    public:
        AnglePublisher(const char* s_topic, int sq_size, const char* p_topic, int pq_size);
        void callback(const remote_robotics::MotorImg& msg);
};

class AngleServer : public AngleDetector{

    protected:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::ServiceServer server_;

        /**
         * Array containing the current angle of every servo
         */
        double angles_[24];
    public:
        AngleServer(const char* s_topic, int sq_size);
        /**
         * receives indexed Images computes angle and saves angle in
         * corresponding index of member `angles[]`
         *
         * \param MotorImg& image of single servomotor
         */
        void callback(const remote_robotics::MotorImg& msg);
        /** 
         * sends current saved angle of requested servo motor
         *
         */
        bool sendAngle(remote_robotics::rotor_angle::Request& req,
                      remote_robotics::rotor_angle::Response& res);
};

#endif
