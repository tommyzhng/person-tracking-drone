#ifndef PERSON_TRACKING_HPP
#define PERSON_TRACKING_HPP

#include <ros/ros.h>
#include <ros/package.h>
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
        void Update();

    private:
        void ImageCb(const sensor_msgs::ImageConstPtr& msg);
        void Process(cv::Mat frame);
        void PublishPosition();
        void PublishDetectionsImage();
        void CalculateError(cv::Mat& frame, cv::Point center);
        cv::Mat resizeAndPad(cv::Mat &image, int target_width, int target_height);
    
        geometry_msgs::PointStamped targetMsg_;

        // ROS
        ros::NodeHandle nh_;
        cv::Mat frame_ = cv::Mat(640, 480, CV_8UC3);
        ros::Subscriber imageSub_;
        ros::Publisher targetPub_;
        ros::Publisher detectionsPub_;

        // TF
        std::unique_ptr<tflite::Interpreter> interpreter_;
        TfLiteTensor* inputTensor_;
        TfLiteTensor* outputTensor_;
        std::unique_ptr<tflite::FlatBufferModel> model_;
        std::vector<std::string> labels_;
        std::ostringstream textStream_;
        int height_, width_;
        double frameRateCalc_;
        double freq_;
        double xError_;
        double yError_;
        double zError_;

};
#endif // PERSON_TRACKING_HPP
