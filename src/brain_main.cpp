#include<iostream>
#include<ctime>
#include<brain.hpp>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"brain");
    ros::NodeHandle nh;

    ros::Rate loop_rate(50);

    Brain b;

    unsigned int i = 0;
    int idx;
    int curr_angle;
    int new_angle;

    while(nh.ok())
    {
        std::cout << "input index: ";
        std::cin >> idx;
        if(std::cin.fail())
        {
            std::cin.clear();
            std::cin.ignore(1000,'\n');
            continue;
        }

        std::cout << "input position: ";
        std::cin >> new_angle;
        if(std::cin.fail())
        {
            std::cin.clear();
            std::cin.ignore(1000,'\n');
            continue;
        }

        if(!b.setPos(idx,new_angle)) continue;

        sleep(2);
        b.getAngle(idx);
        std::cout << "angle: " << curr_angle << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
}



