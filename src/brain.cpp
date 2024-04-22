#include"img_manip.hpp"
#include"angle_detector.hpp"
#include"remote_robotics/rotor_angle.h"
#include"mex_ros_api/servo_motor_pos.h"

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
         * \param new_pos int new position
         *
         * \return bool true if communication was successful
         *              false if no answer from server
         */
        virtual bool setPos(int motor_idx, int new_pos) = 0;
};

class Brain : public IBrain{
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::ServiceClient curr_angle_client_;
        ros::ServiceClient set_pos_client_;

    public:
        Brain(const char* angle_service, const char* position_service);
        int getAngle(int motor_idx);
        bool setPos(int motor_idx, int new_pos);
};

/*
 * main function
 *
 */

int main(int argc, char** argv)
{
    ros::init(argc,argv,"brain");
    ros::NodeHandle nh;

    ros::Rate loop_rate(50);

    Brain b("rotor_pos/angle","set_abs_servo_motor");

    //remote_robotics::motor_idx clt;

    unsigned int i = 0;
    int idx;
    int angle;

    while(nh.ok())
    {
        idx = i%24;
        angle = b.getAngle(idx) + 90; // angle range: 0-180
        ROS_INFO("Angle of rotor %i: %i\n",idx,angle);

        b.setPos(idx, (angle+45)%180-90);

        ros::spinOnce();
        loop_rate.sleep();

        i++;
    }

    return 0;
}

//
// member function definitions
//
Brain::Brain(const char* angle_service, const char* position_service)
{
    ros::ServiceClient curr_angle_client_ = nh_.serviceClient<remote_robotics::rotor_angle>(angle_service);
    ros::ServiceClient set_pos_client_ = nh_.serviceClient<mex_ros_api::servo_motor_pos>(position_service);
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
    mex_ros_api::servo_motor_pos srv;

    new_pos = 

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

