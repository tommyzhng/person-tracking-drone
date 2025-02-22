#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/string_util.h>
#include <tensorflow/lite/model.h>

class PersonTracking {
    public:
        PersonTracking(ros::NodeHandle nh);
        ~PersonTracking();

    private:
        void ImageCb(const sensor_msgs::ImageConstPtr& msg);
        void Process(cv::Mat frame);
        void PublishPosition();
        std_msgs::PointStamped targetMsg_;
        std::vector<float> error_;

        // ROS
        ros::NodeHandle nh_;
        cv::Mat frame_;
        ros::Subscriber imageSub_;
        ros::Publisher targetPub_;

        // TF
        std::unique_ptr<tflite::Interpreter> interpreter_;
        std::unique_ptr<tflite::FlatBufferModel> model_;
        std::vector<std::string> labels_;
        std::ostringstream textStream_;
        int height_, width_;
        double frameRateCalc_;
        double freq_;

}
