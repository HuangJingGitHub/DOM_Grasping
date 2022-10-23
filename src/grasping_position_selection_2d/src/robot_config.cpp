/**
 * @brief Flexiv robot
 */
#include "arm_include/flexiv/Robot.hpp"
#include "arm_include/flexiv/Exception.hpp"
#include "arm_include/flexiv/Log.hpp"
#include "arm_include/flexiv/Scheduler.hpp"
#include "arm_include/flexiv/Utility.hpp"

#include <iostream>
#include <cmath>
#include <thread>

namespace {
/** Size of Cartesian pose vector [position 3x1 + rotation (quaternion) 4x1 ] */
const unsigned int k_cartPoseSize = 7;
static bool isInitPoseSet = false;
bool isNewTcpPose = false;

/** Initial Cartesian-space pose (position + rotation) of robot TCP */
static std::vector<double> preTcpPose;

static std::vector<double> targetTcpPose;
}

void setNewTcpPose(double delta_x, double delta_y) {
    if (isInitPoseSet == false)
        return;

    targetTcpPose = preTcpPose;
    targetTcpPose[0] += delta_x;
    targetTcpPose[1] += delta_y;
    isNewTcpPose = true;
}

/** Callback function for realtime periodic task */
void periodicTask(flexiv::Robot* robot, flexiv::Scheduler* scheduler,
    flexiv::Log* log, flexiv::RobotStates& robotStates) {
    // Flag whether initial Cartesian position is set

    try {
        // Monitor fault on robot server
        if (robot->isFault()) {
            throw flexiv::ServerException(
                "periodicTask: Fault occurred on robot server, exiting ...");
        }

        // Read robot states
        robot->getRobotStates(robotStates);

        // Set initial TCP pose
        if (isInitPoseSet == false) {
            // Check vector size before saving
            if (robotStates.tcpPose.size() == k_cartPoseSize) {
                preTcpPose = robotStates.tcpPose;
                isInitPoseSet = true;
                log->info(
                    "Initial Cartesion pose of robot TCP set to [position "
                    "3x1 + rotation (quaternion) 4x1]: "
                    + flexiv::utility::vec2Str(preTcpPose));
            }
        }
        // Run control only after initial pose is set
        else if (isNewTcpPose == true) {
            robot->streamTcpPose(targetTcpPose);
            preTcpPose = robotStates.tcpPose;
            isNewTcpPose = false;
        } 
        else {
            log->error("Unknown motion type");
            log->info("Accepted motion types: hold, sine-sweep");
            exit(1);
        }

    } catch (const flexiv::Exception& e) {
        log->error(e.what());
        scheduler->stop();
    }
}


int main(int argc, char* argv[]) {
    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // IP of the robot server
    std::string robotIP = "192.168.2.100";

    // IP of the workstation PC running this program
    std::string localIP = "192.168.2.109";


    try {
        // RDK Initialization
        //=============================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robotIP, localIP);

        // Create data struct for storing robot states
        flexiv::RobotStates robotStates;

        // Clear fault on robot server if any
        if (robot.isFault()) {
            log.warn("Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            robot.clearFault();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Check again
            if (robot.isFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            log.info("Fault on robot server is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
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
        robot.setMode(flexiv::MODE_CARTESIAN_IMPEDANCE);

        // Wait for the mode to be switched
        while (robot.getMode() != flexiv::MODE_CARTESIAN_IMPEDANCE) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Choose the index of tool being used. No need to call this method if
        // the mounted tool on the robot has only one TCP, it'll be used by
        // default
        robot.switchTcp(0);

        // Periodic Tasks
        //=============================================================================
        flexiv::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.addTask(
            std::bind(periodicTask, &robot, &scheduler, &log, robotStates),
            "Servo periodic", 1, 45);
        // Start all added tasks, this is by default a blocking method
        scheduler.start();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
