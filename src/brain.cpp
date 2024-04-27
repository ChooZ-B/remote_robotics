#include"brain.hpp"
#include"img_manip.hpp"
#include"angle_detector.hpp"
#include"remote_robotics/rotor_angle.h"
#include"remote_robotics/servo_motor_pos.h"



/*
 * main function
 *
 */

// int main(int argc, char** argv)
// {
//     ros::init(argc,argv,"brain");
//     ros::NodeHandle nh;
// 
//     ros::Rate loop_rate(50);
// 
//     Brain b("rotor_pos/angle","set_abs_servo_motor");
// 
//     //remote_robotics::motor_idx clt;
// 
//     unsigned int i = 0;
//     int idx;
//     int angle;
// 
//     while(nh.ok())
//     {
//         idx = i%24;
//         angle = b.getAngle(idx) + 90; // angle range: 0-180
//         ROS_INFO("Angle of rotor %i: %i\n",idx,angle);
// 
//         b.setPos(idx, (angle+45)%180-90);
// 
//         ros::spinOnce();
//         loop_rate.sleep();
// 
//         i++;
//     }
// 
//     return 0;
// }

//
// member function definitions
//
Brain::Brain()
{
    ros::ServiceClient curr_angle_client_ = nh_.serviceClient<remote_robotics::rotor_angle>("rotor_pos/angle");
    ros::ServiceClient set_pos_client_ = nh_.serviceClient<remote_robotics::servo_motor_pos>("set_abs_servo_motor");
}
int Brain::getAngle(int motor_idx)
{
    remote_robotics::rotor_angle srv;
    srv.request.INDEX = motor_idx;
    if (curr_angle_client_.call(srv))
    {
        return srv.response.ANGLE;
    }
    else
    {
        ROS_ERROR("Failed to call angle detector service.\n");
        return -361;
    }
}
bool Brain::setPos(int motor_idx, int new_pos)
{
    remote_robotics::servo_motor_pos srv;

    //new_pos = 

    srv.request.inpMotorIdxSrv = motor_idx;
    srv.request.inpPosUnit = std::string("DEG");
    srv.request.inpPosVal = new_pos;
    
    if (set_pos_client_.call(srv))
    {
        //ROS_INFO("\nreturned pos value: %i \nunit: %s \nof device: %i\n",
        //  (int) srv.response.outPosVal,
        //  ((std::string) srv.response.outPosUnit).c_str(),
        //  (int) srv.response.outMotorIdxSrv);
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call service servo_motor_pos.\n");
        return false;
    }
}

