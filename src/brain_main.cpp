#include<iostream>
#include<ctime>
#include"brain.hpp"

void test();

int main(int argc, char** argv)
{
    ros::init(argc,argv,"brain");
    //ros::NodeHandle nh;

    //ros::Rate loop_rate(50);
    test();

    std::cout <<"\nterminating node..." << std::endl;
    return 0;
}
void test()
{
    Brain b;

    unsigned int i = 0;
    int idx;
    int curr_angle;
    int new_angle;

    while(1)
    {
        std::cout << "input servo index (25 to end process): ";
        std::cin >> idx;
        if(std::cin.fail())
        {
            std::cout << "false input" << std::endl;
            std::cin.clear();
            std::cin.ignore(1000,'\n');
            continue;
        }
        if(idx == 25)
        {
            return;
        }

        std::cout << "input position(in degrees): ";
        std::cin >> new_angle;
        if(std::cin.fail())
        {
            std::cout << "false input" << std::endl;
            std::cin.clear();
            std::cin.ignore(1000,'\n');
            continue;
        }

        if(!b.setPos(idx,new_angle)) continue;

        sleep(2);
        try
        {
            curr_angle = b.getAngle(idx);
        }
        catch(std::string msg)
        {
            ROS_ERROR("%s\n",msg.c_str());
            continue;
        }
        std::cout << "angle: " << curr_angle << std::endl;
        //ros::spinOnce();
        //loop_rate.sleep();
    }
    return;
}


