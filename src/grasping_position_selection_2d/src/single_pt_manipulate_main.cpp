#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "data_processing.h"
#include "robot_config.hpp"

const int motion_interval = 45;   // in millisecond
const float motion_magnitude = 0.001; // in meter
const float error_threshold = 5; // in pixel


int main(int argc, char** argv) {
    std::ifstream camera_extrinsic_file("./src/grasping_position_selection_2d/src/"
                                        "parameters/camera_extrinsic_matrix.txt");
    if (camera_extrinsic_file.is_open()) {
        std::string item_str;
        float item;
        int row = 0, col = 0, cnt = 0;
        while (camera_extrinsic_file >> item_str) {
            item  = std::stof(item_str);
            row = cnt / 3;
            col = cnt % 3;
            camera_to_base(row, col) = item;
            cnt++;
        }
        camera_extrinsic_file.close();
    }
    else {
        std::cout << "Warning: Fail to initialize the camera to base matrix. \n"
                      "Default identity matix is used.\n";
    }
    std::cout << "Extrinsic camera matrix:\n" << camera_to_base << '\n';

    ros::init(argc, argv, "grasp_manipulate_main");
    ros::NodeHandle node_handle;
    ros::ServiceClient service_client = node_handle.serviceClient<grasping_position_selection_2d::visual_service>("visual_service");
    grasping_position_selection_2d::visual_service srv;
    srv.request.feedback_pt_num = 1;
    initRobotMain();

    std::string start_flag;
    std::cout << "Press 1 to start the manipulation experiment.\n";
    getline(std::cin, start_flag);
    if (start_flag != "1") {
        std::cout << "Exiting program.\n";
        return -1;
    }

    InitializeFiles(motion_interval, motion_magnitude, error_threshold);
    while (true) {
        if (!service_client.call(srv)) {
            std::cout << "Fail to call the service.\n";
            return 1;
        }

        ProcessServece(srv);
        WriteDataToFile();
        
        setNewTcpPose(0, 0);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (total_error_pt.norm() < error_threshold) {
            if (data_save_os.is_open())
                data_save_os.close();
            std::cout << "Manipulation completed. Exiting\n";
            break;
        }
    }
    return 0;
}