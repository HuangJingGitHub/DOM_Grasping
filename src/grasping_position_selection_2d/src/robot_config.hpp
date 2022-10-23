/**
 * @brief Flexiv robot
 */
#ifndef ROBOT_INCLUDED
#define ROBOT_INCLUDED

#include "arm_include/flexiv/Robot.hpp"
#include "arm_include/flexiv/Exception.hpp"
#include "arm_include/flexiv/Log.hpp"
#include "arm_include/flexiv/Scheduler.hpp"
#include "arm_include/flexiv/Utility.hpp"


#include <iostream>
#include <cmath>
#include <thread>


// IP of the robot server
std::string robotIP = "192.168.2.100";
// IP of the workstation PC running this program
std::string localIP = "192.168.2.109";
bool robotInitialized = false;
flexiv::Robot* robotPtr = nullptr;
bool direction_1 = true;
int cnt = 0;

namespace {
const unsigned int k_cartPoseSize = 7;
static bool isInitPoseSet = false;
bool isNewTcpPose = false;
static std::vector<double> preTcpPose;
static std::vector<double> targetTcpPose;
}

void setNewTcpPose(double delta_x, double delta_y) {
    if (isInitPoseSet == false)
        return;

    targetTcpPose = preTcpPose;
    if (direction_1 == true) {
        targetTcpPose[0] += 0.001;
        cnt++;
        if (cnt == 50) {
            direction_1 = false;
        }
    }
    else {
        targetTcpPose[0] -= 0.001;
        cnt--;
        if (cnt == 0) {
            direction_1 = true;
        }
    }

    //targetTcpPose[0] += 0.001;
    //targetTcpPose[1] += delta_y;
    isNewTcpPose = true;
}


/** Callback function for realtime periodic task */
void moveRobot(flexiv::Robot* robotPtr) {
    // Flag whether initial Cartesian position is set
    if (robotPtr == nullptr)
        return;

    try {
        // Monitor fault on robot server
        if (robotPtr->isFault()) {
            throw flexiv::ServerException(
                "periodicTask: Fault occurred on robot server, exiting ...");
        }

        flexiv::RobotStates robotStates;
        robotPtr->getRobotStates(robotStates);

        // Set initial TCP pose
        if (isInitPoseSet == false) {
            // Check vector size before saving
            if (robotStates.tcpPose.size() == k_cartPoseSize) {
                preTcpPose = robotStates.tcpPose;
                isInitPoseSet = true;
                std::cout <<
                    "Initial Cartesion pose of robot TCP set to [position "
                    "3x1 + rotation (quaternion) 4x1]: "
                    + flexiv::utility::vec2Str(preTcpPose);
            }
        }
        // Run control only after initial pose is set
        else if (isNewTcpPose == true) {
            robotPtr->streamTcpPose(targetTcpPose);
            robotPtr->getRobotStates(robotStates);
            preTcpPose = robotStates.tcpPose;
            isNewTcpPose = false;
        } 
        else {
            std::cout << "No move is made\n";
        }

    } catch (const flexiv::Exception& e) {
        std::cerr << e.what();
        return;
    }
}


int initRobotMain() {
    flexiv::Log log;

    static flexiv::Robot robot(robotIP, localIP);
    if (robot.isFault()) {
        robot.clearFault();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        // Check again
        if (robot.isFault()) {
            log.error("Fault cannot be cleared, exiting ...");
            return 1;
        }
        log.info("Fault on robot server is cleared");
    }

    log.info("Enabling robot ...");
    robot.enable();

    // Wait for the robot to become operational
    int secondsWaited = 0;
    while (robot.isOperational() == false) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (++secondsWaited == 10) {
            log.warn(
                "Still waiting for robot to become operational, please "
                "check that the robot 1) has no fault, 2) is booted "
                "into Auto mode");
        }
    }
    log.info("Robot is now operational");

    // Set mode after robot is operational
    robot.setMode(flexiv::MODE_CARTESIAN_IMPEDANCE_NRT);

    // Wait for the mode to be switched
    while (robot.getMode() != flexiv::MODE_CARTESIAN_IMPEDANCE_NRT) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    robot.switchTcp(0);
    robotPtr = &robot;

    return 1;
}

#endif