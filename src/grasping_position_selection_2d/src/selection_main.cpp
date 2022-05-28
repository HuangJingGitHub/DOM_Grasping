#include <fstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "visual_perception.h"

class VisualProcessCore {
private:
    ros::NodeHandle node_handle_;
    image_transport::ImageTransport image_trans_;
    image_transport::Subscriber image_subscriber_;

    const string kWindowName = "Main Window in processing";
    Mat cur_gray_img_;
    LK_Tracker tracker_;
    ImgExtractor extractor_;
    GraspPositionSelector selector_;
    vector<Point2f> target_feedback_pts_;
    Point2f ee_to_obs_obs_pt_;

public:
    VisualProcessCore(string ros_image_stream, 
                vector<int> DO_HSV_thresholds = vector<int>(6, 0)): image_trans_(node_handle_) {
        tracker_ = LK_Tracker(kWindowName);
        extractor_ = ImgExtractor(kWindowName, DO_HSV_thresholds);
        selector_ = GraspPositionSelector();
        image_subscriber_ = image_trans_.subscribe(ros_image_stream, 30, &VisualProcessCore::ProcessImg, this);
        namedWindow(kWindowName);
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
        tracker_.Track(cv_ptr->image, cur_gray_img_);
        extractor_.Extract(cv_ptr->image);         

        if (extractor_.DO_extract_succeed_)
            drawContours(cv_ptr->image, extractor_.DO_contours_, extractor_.largest_DO_countor_idx_, Scalar(250, 0, 150), 2);
        if (extractor_.DO_extract_succeed_ && tracker_.ValidFeedbackAndTargetPts()) {
            vector<double> cur_Q_value(extractor_.DO_contour_.size(), 0);
            // Point2f grasp_pt = selector_.SelectSingleGraspPosition(extractor_.DO_contour_, cur_Q_value, 
            //                                    tracker_.points_[0], tracker_.target_pts_[0]);
            //circle(cv_ptr->image, grasp_pt, 3, Scalar(255, 0, 0), -1);
            vector<Point2f> grasp_pts = selector_.SelectDualGraspPositions(extractor_.DO_contour_, cur_Q_value, 
                                        tracker_.points_[0], tracker_.target_pts_[0]);
            circle(cv_ptr->image, grasp_pts[0], 3, Scalar(255, 0, 0), -1);
            circle(cv_ptr->image, grasp_pts[1], 3, Scalar(255, 0, 0), -1);
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