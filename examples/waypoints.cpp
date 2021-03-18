//
// Created by Peter Klausing on 04.03.21.
//

#include <frankx/frankx.hpp>

#include <chrono>
#include <mutex>
#include <thread>
#include <iostream>

using namespace frankx;

Robot *robot {nullptr};
movex::WaypointMotion *waypointMotion {nullptr};
movex::MotionData *motionData {nullptr};
std::shared_ptr<std::mutex> motionDataMutex {nullptr};

void move()
{
    try {
        if (robot && waypointMotion && motionData)
            robot->move(*waypointMotion, *motionData);

    }
    catch (franka::CommandException exception) {
        std::cout << "cannot move in this mode. Quit!" << std::endl;
        std::this_thread::sleep_for (std::chrono::milliseconds (100));
    }
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    // Connect to the robot
    robot = new Robot(argv[1]);
    robot->automaticErrorRecovery();

    // Reduce the acceleration and velocity dynamic
    robot->setDynamicRel(0.05);
    robot->setDefaultBehavior();

    Affine currentPose = robot->currentPose();
    currentPose.translate({0,0,0});
    movex::Waypoint waypoint(currentPose);
    std::cout << currentPose.toString() << std::endl;

    motionData = new movex::MotionData();
    motionDataMutex = std::make_shared<std::mutex>();
    motionData->last_pose_lock = motionDataMutex;
    waypointMotion = new movex::WaypointMotion({waypoint}, false);

    std::thread moveThread(move);

    std::string q;
    while(true)
    {
        std::cout << "Press for next Pose" << std::endl;
        std::cin >> q;
        std::cout << "pressed" << std::endl;
        if(q=="q")
            break;
        else if (q=="w")
        {
            std::cout << "up" << std::endl;
            currentPose.translate({0.05,0,0});
        }
        else if (q=="s")
        {
            std::cout << "down" << std::endl;
            currentPose.translate({-0.05,0,0});
        }
        else if (q=="a")
        {
            std::cout << "left" << std::endl;
            currentPose.translate({0,0.05,0});
        }
        else if (q=="d")
        {
            std::cout << "right" << std::endl;
            currentPose.translate({0,-0.05,0});
        }
        waypoint = movex::Waypoint (currentPose);
        waypointMotion->setNextWaypoint(waypoint);
        int i = 0;
        do
        {
            std::this_thread::sleep_for (std::chrono::milliseconds (100));
            Affine curAffine;
            {
                const std::lock_guard<std::mutex> lock(*motionDataMutex);
                curAffine = motionData->last_pose;
            }
            std::cout << curAffine.toString() << std::endl;
        }
        while(/*i++ < 10 ||*/ motionData->is_moving);

    }
    return 0;
}