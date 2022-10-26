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
    ros::ServiceServer visual_service = node_handle_.advertiseService("visual_service", 
                                                            &VisualServiceCore::GetVisualService, this);
    image_transport::ImageTransport image_trans_;
    image_transport::Subscriber image_subscriber_;

    const string kWindowName = "Main Window in service node";
    Mat cur_gray_img_;
    Mat hand_logo_;
    LK_Tracker tracker_;
    ImgExtractor extractor_;
    GraspPositionSelector selector_;
    /*Mode1: Using Task-Oriented Metric in Selection  
      Mode2: Using Distance Sum in Selection*/
    int DEFAULT_SELECTION_MODE_ = 0;  

    vector<Point2f> grasp_pts_;
    Point2f first_feedback_pt_;
    

public:
    VisualServiceCore(string ros_image_stream, 
                vector<int> DO_HSV_thresholds = vector<int>(6, 0)): image_trans_(node_handle_) {
        tracker_ = LK_Tracker(kWindowName);
        extractor_ = ImgExtractor(kWindowName, DO_HSV_thresholds);
        selector_ = GraspPositionSelector();
        image_subscriber_ = image_trans_.subscribe(ros_image_stream, 30, &VisualServiceCore::ProcessImg, this);
        first_feedback_pt_ = Point2f(0, 0);
        namedWindow(kWindowName);
        hand_logo_ = imread("./src/grasping_position_selection_2d/src/parameters/blue_hand_icon_50X50.png", 
                            cv::ImreadModes::IMREAD_COLOR);
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
        if (tracker_.points_num_ == tracker_.points_[0].size() && DEFAULT_SELECTION_MODE_ != 0)
            SelectGraspingPosition(cv_ptr);


        // draw part
        drawContours(cv_ptr->image, extractor_.DO_contours_, extractor_.largest_DO_countor_idx_, 
                    Scalar(250, 0, 150), 1);
        imshow(kWindowName, cv_ptr->image);
        waitKey(2);
    }

    void SelectGraspingPosition(cv_bridge::CvImagePtr cv_ptr) {
        for (int i = 0; i < grasp_pts_.size() && tracker_.is_grasped_ == false; i++) {
            rectangle(cv_ptr->image, Rect(grasp_pts_[i].x - 6, grasp_pts_[i].y - 6, 12, 12), Scalar(0, 0, 255), 2);
            circle(cv_ptr->image, grasp_pts_[i], 3, Scalar(0, 0, 255), -1);
            putText(cv_ptr->image, "Sg" + to_string(i + 1), grasp_pts_[i] - Point2f(15, 15), 
                    0, 0.5, Scalar(0, 0, 0), 2);
/*             Mat destROI = cv_ptr->image(Rect(grasp_pts[i].x, grasp_pts[i].y, hand_logo_.cols, hand_logo_.rows));
            for (int row = 0; row < hand_logo_.rows; row++)
                for (int col = 0; col < hand_logo_.cols; col++) 
                    if (hand_logo_.at<Vec3b>(row, col) != Vec3b(0, 0, 0)) 
                        destROI.at<Vec3b>(row, col) = hand_logo_.at<Vec3b>(row, col); */
        }

        if (extractor_.DO_extract_succeed_ == false || tracker_.ValidFeedbackAndTargetPts() == false
            || cv::norm(first_feedback_pt_ - tracker_.points_[0][0]) < 1)
            return;
        
        first_feedback_pt_ = tracker_.points_[0][0];
        vector<double> cur_Q_value(extractor_.DO_contour_.size(), 0);
        Point2f single_grasp_pt;
        if (DEFAULT_SELECTION_MODE_ == 1)
            single_grasp_pt = selector_.SelectSingleGraspPosition(extractor_.DO_contour_, cur_Q_value,
                                                                    tracker_.points_[0], tracker_.target_pts_);
        else if (DEFAULT_SELECTION_MODE_ == 2)
            single_grasp_pt = selector_.SelectSingleGraspPositionbyDistance(extractor_.DO_contour_, tracker_.points_[0]);                                                                    
        grasp_pts_.clear();
        grasp_pts_.push_back(single_grasp_pt);
    }

    bool GetVisualService(grasping_position_selection_2d::visual_service::Request &request,
                          grasping_position_selection_2d::visual_service::Response &response) {
        DEFAULT_SELECTION_MODE_ = request.selection_mode;
        if (tracker_.points_[0].empty() || tracker_.ee_point_[0].empty() || 
            grasp_pts_.empty())
            return false;

        for (int i = 0; i < tracker_.points_num_; i++) {
            response.feedback_pt.push_back(tracker_.points_[0][i].x);
            response.feedback_pt.push_back(tracker_.points_[0][i].y);
            response.target_pt.push_back(tracker_.target_pts_[i].x);
            response.target_pt.push_back(tracker_.target_pts_[i].y);            
        }
        response.ee_pt.push_back(tracker_.ee_point_[0][0].x);
        response.ee_pt.push_back(tracker_.ee_point_[0][0].y);
        response.grasp_pt.push_back(grasp_pts_[0].x);
        response.grasp_pt.push_back(grasp_pts_[0].y);

        for (int row = 0; row < 2 * tracker_.points_num_; row++)
            for (int col = 0; col < 2; col++) {
                response.Jd.push_back(tracker_.cur_Jd_(row, col));
            }
        return true;

    }
};

int main(int argc, char** argv) {
    vector<int> DO_HSV_thresholds{0, 0, 0, 180, 255, 255};
    ifstream HSV_file("./src/grasping_position_selection_2d/src/parameters/HSV_thresholds.txt");
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