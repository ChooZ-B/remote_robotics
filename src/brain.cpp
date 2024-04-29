#include"brain.hpp"
#include"img_manip.hpp"
#include"angle_detector.hpp"
#include"remote_robotics/rotor_angle.h"
#include"remote_robotics/servo_motor_pos.h"



//
// member function definitions
//
Brain::Brain()
{
    curr_angle_client_ = nh_.serviceClient<remote_robotics::rotor_angle>("rotor_pos/angle");
    set_pos_client_ = nh_.serviceClient<remote_robotics::servo_motor_pos>("set_abs_servo_motor");
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
        ROS_ERROR("Failed to call service %s.\n",curr_angle_client_.getService().c_str());
        return -361;
    }
}
bool Brain::setPos(int motor_idx, int new_pos)
{
    remote_robotics::servo_motor_pos srv;

    srv.request.inpMotorIdxSrv = motor_idx;
    srv.request.inpPosUnit = std::string("DEG");
    srv.request.inpPosVal = new_pos;
    
    if (set_pos_client_.call(srv))
    {
        ROS_INFO("\nreturned pos value: %i unit: %s of device: %i\n",
          (int) srv.response.outPosVal,
          ((std::string) srv.response.outPosUnit).c_str(),
          (int) srv.response.outMotorIdxSrv);
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call service %s.\n",set_pos_client_.getService().c_str());
        return false;
    }
}

