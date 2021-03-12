//
// Created by Peter Klausing on 05.03.21.
//
#include <frankx/frankx.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include <map>

using namespace frankx;

typedef std::array<double, 7> JointPositionsType;

std::string jointPositionsToString(JointPositionsType positions, size_t precision = 7)
{
    std::stringstream ss;
    ss << std::setprecision(precision);
    for (int i = 0; i < 7; i++)
    {
        ss << positions.at(i) << " ";
    }
    return ss.str();
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname> <home_file>" << std::endl;
        return -1;
    }
    std::vector<std::pair<std::string, JointPositionsType>> jointPositionsMap {};
    std::string name;
    JointPositionsType jointPositions;
    std::ifstream infile(argv[2]);
    int i = 0;
    while (infile >> name
        >> jointPositions.at(0) >>jointPositions.at(1) >>jointPositions.at(2) >>jointPositions.at(3)
        >> jointPositions.at(4) >>jointPositions.at(5) >>jointPositions.at(6) )
    {
        jointPositionsMap.push_back(std::make_pair(name, jointPositions));
        std::cout << std::to_string(i++) << " " << name << " " << jointPositionsToString(jointPositions) << std::endl;
    }
    infile.close();

    // Connect to the robot
    Robot robot(argv[1]);
    robot.automaticErrorRecovery();

    // Reduce the acceleration and velocity dynamic
    robot.setDynamicRel(0.15);

    std::cout << "give number to move to according position or \'n\' for new" << std::endl;
    std::string in;
    std::cin >> in;
    if(in.size() == 1 && in[0] == 'n')
    {
        JointPositionsType curJointPositions = robot.currentJointPositions();
        std::cout << jointPositionsToString(curJointPositions) << std::endl;
        std::cout << "add a name:" << std::endl;
        std::string name;
        std::cin >> name;
        if (name.size() == 0 || name.find(" ") != -1)
        {
            std::cout << "no valid name:" << name << std::endl;
            return 0;
        }
        std::cout << "write back to file: "  << argv[2] << std::endl;
        std::ofstream outfile;
        outfile.open(argv[2], std::ios_base::app); // append instead of overwrite
        outfile << name << " " << jointPositionsToString(curJointPositions);
        outfile.close();
        return 0;
    }
    else
    {
        int num;
        try
        {
            num = std::stoi(in);
        }
        catch( ... )
        {
            std::cout << "could not parse input" << std::endl;
            return 0;
        }
        if (num < 0 ||  jointPositionsMap.size() <= num )
        {
            std::cout << "out of bound" << std::endl;
            return 0;
        }
        std::cout << "move to: " << jointPositionsToString(jointPositionsMap.at(num).second) << std::endl;
        robot.move(JointMotion(jointPositionsMap.at(num).second));
    }
    return 0;
}