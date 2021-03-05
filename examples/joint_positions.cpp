//
// Created by Peter Klausing on 05.03.21.
//
#include <frankx/frankx.hpp>

#include <iostream>
#include <string>

using namespace frankx;

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    // Connect to the robot
    Robot robot(argv[1]);
    robot.automaticErrorRecovery();

    // Reduce the acceleration and velocity dynamic
    robot.setDynamicRel(0.15);

    double step = 0.1;
    std::cout << "Input 1 - 7 moving the according joint in positiv direction." << std::endl;
    std::cout << "Add Minus for negative direction. End with \'q\'." << std::endl;
    while(true)
    {
        std::array<double, 7> jointPositions = robot.currentJointPositions();
        Affine pose = robot.currentPose();
        std::cout << pose.toString() << std::endl;
        std::cout << "["
                << "1:" << jointPositions.at(0) << " "
                << "2:" << jointPositions.at(1) << " "
                << "3:" << jointPositions.at(2) << " "
                << "4:" << jointPositions.at(3) << " "
                << "5:" << jointPositions.at(4) << " "
                << "6:" << jointPositions.at(5) << " "
                << "7:" << jointPositions.at(6) << "]" << std::endl;
        std::string in;
        std::cin >> in;
        if (in == "q")
            break;
        int i;
        try {
            i = std::stoi(in);
        } catch (std::invalid_argument)
        {
            continue;
        }
        if ( i >= -7 && i != 0 && i <= 7)
        {
            int index;
            int sign;
            if(i > 0)
            {
                sign = 1;
                index = i;
            }
            else
            {
                sign = -1;
                index = -i;
            }
            jointPositions.at(index-1) += sign * step;
        }
        robot.move(JointMotion(jointPositions));
    }
    return 0;
}