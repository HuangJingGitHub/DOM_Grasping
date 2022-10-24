#ifndef DATA_PROCESSING_INCLUDED
#define DATA_PROCESSING_INCLUDED

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include "grasping_position_selection_2d/visual_service.h"

const int total_data_dim = 12;
Eigen::Matrix3f camera_to_base = Eigen::Matrix3f::Identity();
Eigen::Vector2f feedback_pt, target_pt, ee_pt, grasp_pt,
                total_error_pt, ee_velocity_image;
Eigen::Vector3f ee_velocity_image_3D = Eigen::Array3f::Zero(),
                ee_velocity_3D;
Eigen::Matrix2f Jd = Eigen::Matrix2f::Identity();

std::string save_directory = "./src/grasping_position_selection_2d/src/data/";
std::ofstream  data_save_os;
std::vector<float> data_vec;
std::string data_save_order_info = "Format: time(ms)---feedback_pt---target_pt---ee_pt---Jd";
std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
int time_shift_ms = 0;

void InitializeFiles(const int& motion_interval, 
                    const float& motion_magnitude, 
                    const float& error_threshold) {
    std::string file_name_postfix, file_name, exp_specification_info;
    std::time_t cur_time = std::time(0);
    std::tm* cur_tm = std::localtime(&cur_time);
    file_name_postfix = std::to_string(cur_tm->tm_year + 1900) + "-"
                        + std::to_string(cur_tm->tm_mon + 1) + "-"
                        + std::to_string(cur_tm->tm_mday) + "_"
                        + std::to_string(cur_tm->tm_hour) + ":"
                        + std::to_string(cur_tm->tm_min) + ".txt";
    file_name = "single_pt_data_" + file_name_postfix;

    exp_specification_info = "motion_interval: " + std::to_string(motion_interval) + " ms "
                            + "motion_magnitude: " + std::to_string(motion_magnitude) + " m "
                            + "error_threshold: " + std::to_string(error_threshold) + " px"; 
    
    data_save_os.open(save_directory + file_name, std::ios::trunc);
    if (data_save_os.is_open() == false) {
        std::cerr << "Fail to open the data storage file!\n";
    }
    else {
        data_save_os << exp_specification_info << "\n" << data_save_order_info << "\n";
    }
}

void ProcessServece(const grasping_position_selection_2d::visual_service& srv) {
    data_vec.clear();
    feedback_pt(0, 0) = srv.response.feedback_pt[0];  
    feedback_pt(1, 0) = srv.response.feedback_pt[1]; 
    target_pt(0, 0) = srv.response.target_pt[0]; 
    target_pt(1, 0) = srv.response.target_pt[1]; 
    ee_pt(0, 0) = srv.response.ee_pt[0]; 
    ee_pt(1, 0) = srv.response.ee_pt[1];    
    grasp_pt(0, 0) = srv.response.grasp_pt[0];
    grasp_pt(1, 0) = srv.response.grasp_pt[1];   
    for (int row = 0, cnt = 0; row < 2; row++)
        for (int col = 0; col < 2; col++) {
            Jd(row, col) = srv.response.Jd[cnt];
            cnt++;
        }
    std::cout << "Jd: \n" << Jd << '\n' 
              << "Jd^-1:\n" << Jd.inverse() << '\n';    

    total_error_pt = feedback_pt - target_pt;
    ee_velocity_image = -Jd.inverse() * total_error_pt;
    ee_velocity_image_3D(0, 0) = ee_velocity_image(0, 0);
    ee_velocity_image_3D(1, 0) = ee_velocity_image(1, 0);
    ee_velocity_image_3D(2, 0) = 0;
    
    data_vec.push_back(feedback_pt(0, 0));
    data_vec.push_back(feedback_pt(1, 0));
    data_vec.push_back(target_pt(0, 0));
    data_vec.push_back(target_pt(1, 0));
    data_vec.push_back(ee_pt(0, 0));
    data_vec.push_back(ee_pt(1, 0));
    data_vec.push_back(grasp_pt(0, 0));
    data_vec.push_back(grasp_pt(1, 0));
    for (int i = 0; i < 4; i++)
        data_vec.push_back(srv.response.Jd[i]);
}

void WriteDataToFile() {
    if (data_vec.size() != total_data_dim)
        std::cout << "No valid data available. Saving data failed.\n";
    if (data_save_os.is_open() == false) 
        std::cerr << "Fail to open the file. Saving data failed.\n";
    
    std::chrono::high_resolution_clock::time_point t_now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t_start);
    if (time_shift_ms == 0)
        time_shift_ms = duration.count();

    data_save_os << duration.count() - time_shift_ms << ": ";
    for (float& data : data_vec)
        data_save_os << data << " ";
    data_save_os << "\n";
}

#endif