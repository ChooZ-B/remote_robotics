#ifndef BRAIN_INCLUDED
#define BRAIN_INCLUDED

#include<ros/ros.h>
#include"img_manip.hpp"
#include"angle_detector.hpp"
#include"remote_robotics/rotor_angle.h"
#include"remote_robotics/servo_motor_pos.h"

/**
 *
 * \class IBrain
 *
 * \brief interface
 */
class IBrain{
    public:
        /**
         *
         * receive angle of rotor from servomotor under given index
         *
         * \param motor_idx int index of servo motor
         *
         * \return int angle of rotor
         */
        virtual int getAngle(int motor_idx) = 0;
        /**
         *
         * member function to set rotor position of servomotor under given index
         *
         * \param motor_idx int index of servo motor
         *
         * \param new_pos int new position. has to be a value between -90 and 90
         *
         * \return bool true if communication was successful
         *              false if no answer from server
         */
        virtual bool setPos(int motor_idx, int new_pos) = 0;
};

class Brain : public IBrain{
    protected:
        ros::NodeHandle nh_;
        ros::ServiceClient curr_angle_client_;
        ros::ServiceClient set_pos_client_;

    public:
        Brain();
        int getAngle(int motor_idx);
        bool setPos(int motor_idx, int new_pos);
};

#endif
