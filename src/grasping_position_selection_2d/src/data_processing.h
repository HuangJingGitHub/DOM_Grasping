#ifndef DATA_PROCESSING_INCLUDED
#define DATA_PROCESSING_INCLUDED

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>
#include <eigen3/Eigen/Dense>
#include "visual_module/visual_info_service_singlePt.h"

const int total_data_dim = 18;
int pre_motion_mode_flag = 0; // 0 - not constrained, 1 - constrained
int total_adjust_time = 0;
bool ee_path_planned = false;
bool track_ee_path_mode = false;

Eigen::Matrix3f camera_to_base = Eigen::Matrix3f::Identity();
Eigen::Vector2f feedback_pt, target_pt, ee_pt,
                path_error_pt, ee_path_error_pt, total_error_pt,
                ee_velocity_image;
Eigen::Vector3f ee_velocity_image_3D = Eigen::Array3f::Zero(),
                ee_velocity_3D;
Eigen::Matrix2f Jd = Eigen::Matrix2f::Identity();

std::string save_directory = "./src/grasping_position_selection_2d/src/data/";
std::ofstream  data_save_os;
std::vector<float> data_vec;
std::string data_save_order_info = "Format: time---feedback_pt---target_pt---ee_pt---Jd";

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

void ProcessServece(const visual_module::visual_info_service_singlePt& srv) {
    data_vec.clear();
    feedback_pt(0, 0) = srv.response.feedback_pt[0];  
    feedback_pt(1, 0) = srv.response.feedback_pt[1]; 
    target_pt(0, 0) = srv.response.target_pt[0]; 
    target_pt(1, 0) = srv.response.target_pt[1]; 
    ee_pt(0, 0) = srv.response.ee_pt[0]; 
    ee_pt(1, 0) = srv.response.ee_pt[1];
    ee_to_obs_obs_pt(0, 0) = srv.response.ee_to_obs_obs_pt[0];
    ee_to_obs_obs_pt(1, 0) = srv.response.ee_to_obs_obs_pt[1];
    DO_to_obs_DO_pt(0, 0) = srv.response.DO_to_obs_DO_pt[0]; 
    DO_to_obs_DO_pt(1, 0) = srv.response.DO_to_obs_DO_pt[1]; 
    DO_to_obs_obs_pt(0, 0) = srv.response.DO_to_obs_obs_pt[0]; 
    DO_to_obs_obs_pt(1, 0) = srv.response.DO_to_obs_obs_pt[1]; 
    projection_on_path_pt(0, 0) = srv.response.projection_on_path_pt[0]; 
    projection_on_path_pt(1, 0) = srv.response.projection_on_path_pt[1]; 
    for (int row = 0, cnt = 0; row < 2; row++)
        for (int col = 0; col < 2; col++) {
            Jd(row, col) = srv.response.Jd[cnt];
            cnt++;
        }
    
    ee_path_planned = srv.response.ee_path_planned;
    if (srv.response.ee_path_planned) {
        ee_target_pt(0, 0) = srv.response.ee_target_pt[0];
        ee_target_pt(1, 0) = srv.response.ee_target_pt[1];
        projection_on_ee_path_pt[0] = srv.response.projection_on_ee_path_pt[0];
        projection_on_ee_path_pt[1] = srv.response.projection_on_ee_path_pt[1];
        ee_path_error_pt = ee_pt - projection_on_ee_path_pt;
        ee_total_error_pt = ee_pt - ee_target_pt;
    }
    std::cout << "Jd: \n" << Jd << '\n' 
            << "Jd^-1:\n" << Jd.inverse() << '\n';    

    path_error_pt = feedback_pt - projection_on_path_pt;
    total_error_pt = feedback_pt - target_pt;
    ee_velocity_image = -Jd.inverse() * path_error_pt;
    ee_velocity_image_3D(0, 0) = ee_velocity_image(0, 0);
    ee_velocity_image_3D(1, 0) = ee_velocity_image(1, 0);
    
    data_vec.push_back(feedback_pt(0, 0));
    data_vec.push_back(feedback_pt(1, 0));
    data_vec.push_back(target_pt(0, 0));
    data_vec.push_back(target_pt(1, 0));
    data_vec.push_back(ee_pt(0, 0));
    data_vec.push_back(ee_pt(1, 0));
    data_vec.push_back(ee_to_obs_obs_pt(0, 0));
    data_vec.push_back(ee_to_obs_obs_pt(1, 0));
    data_vec.push_back(DO_to_obs_DO_pt(0, 0));
    data_vec.push_back(DO_to_obs_DO_pt(1, 0));
    data_vec.push_back(DO_to_obs_obs_pt(0, 0));
    data_vec.push_back(DO_to_obs_obs_pt(1, 0));
    data_vec.push_back(projection_on_path_pt(0, 0));
    data_vec.push_back(projection_on_path_pt(1, 0));
    for (int i = 0; i < 4; i++)
        data_vec.push_back(srv.response.Jd[i]);
}

void WriteDataToFile() {
    if (data_vec.size() != total_data_dim)
        std::cout << "No valid data available. Saving data failed.\n";
    if (data_save_os.is_open() == false) 
        std::cerr << "Fail to open the file. Saving data failed.\n";
    
    for (float& data : data_vec)
        data_save_os << data << " ";
    data_save_os << "\n";
}


#endif