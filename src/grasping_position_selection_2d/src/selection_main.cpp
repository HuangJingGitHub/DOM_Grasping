#include <fstream>
#include <ctime>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "visual_perception.h"

std::time_t cur_time = std::time(0);
std::tm* cur_tm = std::localtime(&cur_time);
int file_cnt = 0;

class VisualProcessCore {
private:
    ros::NodeHandle node_handle_;
    image_transport::ImageTransport image_trans_;
    image_transport::Subscriber image_subscriber_;

    const string kWindowName = "Main Window in processing";
    Mat cur_gray_img_;
    Mat hand_logo_;
    LK_Tracker tracker_;
    ImgExtractor extractor_;
    GraspPositionSelector selector_;

public:
    VisualProcessCore(string ros_image_stream, 
                vector<int> DO_HSV_thresholds = vector<int>(6, 0)): image_trans_(node_handle_) {
        tracker_ = LK_Tracker(kWindowName);
        extractor_ = ImgExtractor(kWindowName, DO_HSV_thresholds);
        selector_ = GraspPositionSelector();
        image_subscriber_ = image_trans_.subscribe(ros_image_stream, 30, &VisualProcessCore::ProcessImg, this);
        namedWindow(kWindowName);
        // hand_logo_ = imread("./src/grasping_position_selection_2d/src/parameters/blue_hand_icon_50X50.png", cv::ImreadModes::IMREAD_COLOR);
    }

    ~VisualProcessCore() {
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
        extractor_.Extract(cv_ptr->image);   // Contour extraction is influenced by added icons or texts as are embedded in the image.
        tracker_.Track(cv_ptr->image, cur_gray_img_);        

        vector<Point2f> grasp_pts;
        if (extractor_.DO_extract_succeed_ && tracker_.ValidFeedbackAndTargetPts()) {
            vector<double> cur_Q_value(extractor_.DO_contour_.size(), 0);
            // Point2f grasp_pt = selector_.SelectSingleGraspPosition(extractor_.DO_contour_, cur_Q_value, 
            //                                    tracker_.points_[0], tracker_.target_pts_);
            //circle(cv_ptr->image, grasp_pt, 3, Scalar(255, 0, 0), -1);
            //if (tracker_.points_[0].size() <= 6)
            //    grasp_pts = selector_.SelectDualGraspPositions(extractor_.DO_contour_, cur_Q_value, 
            //                        tracker_.points_[0], tracker_.target_pts_);
            //else {
                grasp_pts = selector_.SelectDualGraspPositionsByCluster(extractor_.DO_contour_, cur_Q_value, 
                                    tracker_.points_[0], tracker_.target_pts_);           
                cout << "Using Clustering\n";
            //}
        }
        
        // display
        // if (extractor_.DO_extract_succeed_)
            // drawContours(cv_ptr->image, extractor_.DO_contours_, extractor_.largest_DO_countor_idx_, Scalar(250, 0, 150), 2);        
        if (tracker_.points_[0].size() == tracker_.target_pts_.size()) {
            for (int i = 0; i < tracker_.points_[0].size(); i++) {
                arrowedLine(cv_ptr->image, tracker_.points_[0][i], tracker_.target_pts_[i], Scalar(0, 255, 0), 2);
                putText(cv_ptr->image, "S" + to_string(i + 1), tracker_.points_[0][i] + Point2f(5, 5), 
                        0, 0.5, Scalar(0, 0, 0), 2);
            }
        }
        for (int i = 0; i < grasp_pts.size(); i++) {
            rectangle(cv_ptr->image, Rect(grasp_pts[i].x - 6, grasp_pts[i].y - 6, 12, 12), Scalar(0, 0, 255), 2);
            circle(cv_ptr->image, grasp_pts[i], 3, Scalar(0, 0, 255), -1);
            putText(cv_ptr->image, "Sg" + to_string(i + 1), grasp_pts[i] - Point2f(15, 15), 
                    0, 0.5, Scalar(0, 0, 0), 2);
/*             Mat destROI = cv_ptr->image(Rect(grasp_pts[i].x, grasp_pts[i].y, hand_logo_.cols, hand_logo_.rows));
            for (int row = 0; row < hand_logo_.rows; row++)
                for (int col = 0; col < hand_logo_.cols; col++) 
                    if (hand_logo_.at<Vec3b>(row, col) != Vec3b(0, 0, 0)) 
                        destROI.at<Vec3b>(row, col) = hand_logo_.at<Vec3b>(row, col); */
        }
        if (save_image_flag) {
            save_image_flag = false;
            std::string directory_info = "./src/grasping_position_selection_2d/src/images/";
            std::string image_name = std::to_string(cur_tm->tm_year + 1900) + "_"
                                    + std::to_string(cur_tm->tm_mon + 1) + "_"
                                    + std::to_string(cur_tm->tm_mday) + "_"
                                    + std::to_string(cur_tm->tm_hour) + "_"
                                    + std::to_string(cur_tm->tm_min) + "_"
                                    + std::to_string(cur_tm->tm_sec) + "_"
                                    + std::to_string(file_cnt) + ".jpg";
            imwrite(directory_info + image_name, cv_ptr->image);
            file_cnt++;
        }
        imshow(kWindowName, cv_ptr->image);
        waitKey(2);
    }
};


int main(int argc, char** argv) {
    vector<int> DO_HSV_thresholds{0, 0, 0, 180, 255, 255};
    ifstream HSV_file("./src/grasping_position_selection_2d/src/parameters/HSV_thresholds.txt");
    if (HSV_file.is_open()) {
        string item_str;
        int cnt = 0, item;
        while (HSV_file >> item_str && cnt < 6) {
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
    ros::init(argc, argv, "selection_core");
    VisualProcessCore visual_process_obj(ros_image_stream, DO_HSV_thresholds);
    ros::spin();
    return 0;    
} 