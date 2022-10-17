#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "visual_perception.h"
#include "grasping_position_selection_2d/visual_service.h"

using namespace std;
using namespace cv;

class VisualServiceCore {
private:
    ros::NodeHandle node_handle_;
    ros::ServiceServer visual_info_service = node_handle_.advertiseService("visual_service", 
                                                            &VisualServiceCore::GetVisualInfoService, this);
    image_transport::ImageTransport image_trans_;
    image_transport::Subscriber image_subscriber_;

    const string kWindowName = "Main Window in service node";
    Mat cur_gray_img_;
    LK_Tracker tracker_;
    ImgExtractor extractor_;

public:
    VisualServiceCore(string ros_image_stream, 
                vector<int> DO_HSV_thresholds = vector<int>(6, 0)): image_trans_(node_handle_) {
        tracker_ = LK_Tracker(kWindowName);
        extractor_ = ImgExtractor(kWindowName, DO_HSV_thresholds);
        image_subscriber_ = image_trans_.subscribe(ros_image_stream, 30, &VisualServiceCore::ProcessImg, this);
        namedWindow(kWindowName);
    }

    ~VisualServiceCore() {
        destroyWindow(kWindowName);
    }

    void ProcessImg(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& exception_type) {
            ROS_ERROR("cv_bridge exception: %s", exception_type.what());
            return;
        }
        cvtColor(cv_ptr->image, cur_gray_img_, COLOR_BGR2GRAY);
        tracker_.Track(cv_ptr->image, cur_gray_img_);
        extractor_.Extract(cv_ptr->image);
        imshow(kWindowName, cv_ptr->image);
        waitKey(2);
    }

    bool GetVisualInfoService(grasping_position_selection_2d::visual_service::Request &request,
                              grasping_position_selection_2d::visual_service::Response &response) {
        if (tracker_.points_[0].empty() || tracker_.ee_point_[0].empty() || 
            extractor_.DO_extract_succeed_ == false)
            return false;

        response.feedback_pt[0] = tracker_.points_[0][0].x;
        response.feedback_pt[1] = tracker_.points_[0][0].y;
        response.target_pt[0] = tracker_.target_pts_[0].x;
        response.target_pt[1] = tracker_.target_pts_[0].y;
        response.ee_pt[0] = tracker_.ee_point_[0][0].x;
        response.ee_pt[1] = tracker_.ee_point_[0][0].y;
        for (int row = 0, cnt = 0; row < 2; row++)
            for (int col = 0; col < 2; col++) {
                response.Jd[cnt] = tracker_.cur_Jd_(row, col);
                cnt++;
            }
        return true;
    }
};

int main(int argc, char** argv) {
    vector<int> DO_HSV_thresholds{0, 0, 0, 180, 255, 255};
    ifstream HSV_file("./src/visual_module/src/data/HSV_thresholds.txt");
    if (HSV_file.is_open()) {
        string item_str;
        int cnt = 0, item;
        while (HSV_file >> item_str) {
            item = stoi(item_str);
            DO_HSV_thresholds[cnt] = item;
            cnt++;
        }
        HSV_file.close();
    }
    else {
        cout << "Fail to read DO HSV threshold file. Default value is used.\n";
    }

    string ros_image_stream = "cameras/source_camera/image";
    ros::init(argc, argv, "visual_core");
    VisualServiceCore visual_process_obj(ros_image_stream, DO_HSV_thresholds);
    ros::spin();
    return 0;
}