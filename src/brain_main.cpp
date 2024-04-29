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

    ROS_INFO("terminating node...\n");
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
        std::cout << "input index (25 to end process): ";
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

        std::cout << "input position: ";
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
        b.getAngle(idx);
        std::cout << "angle: " << curr_angle << std::endl;
        //ros::spinOnce();
        //loop_rate.sleep();
    }
    return;
}


