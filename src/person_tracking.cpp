#include "person_tracking.hpp"

PersonTracking::PersonTracking(ros::NodeHandle nh) : nh_(nh)
{
    // paths
    const std::string modelPath = "./cv_model/detect.tflite";
    const std::string labelPath = "./cv_model/labelmap.txt";

    // load labels
    std::ifstream labelFile(labelPath);
    std::string line;
    while (std::getline(labelFile, line)) {
        labels_.push_back(line);
    }

    // Load TFLite model
    model_ = tflite::FlatBufferModel::BuildFromFile(modelPath.c_str());
    tflite::ops::builtin::BuiltinOpResolver resolver;
    tflite::InterpreterBuilder(*model_, resolver)(&interpreter_);
    interpreter_->AllocateTensors();

    // Get input and output tensor details
    TfLiteTensor* inputTensor = interpreter_->input_tensor(0);
    TfLiteTensor* outputTensor = interpreter_->output_tensor(0);

    // Set input dimensions
    height_ = 300;
    width_ = 300;

    // Initialize other variables
    frameRateCalc_ = 1;
    freq_ = cv::getTickFrequency();

    // ROS
    imageSub_ = nh_.subscribe("/camera/image_rect", 1, &PersonTracking::ImageCb, this);
    targetPub_ = nh_.advertise<std_msgs::PointStamped>("/person_tracking/target", 10);
}

PersonTracking::~PersonTracking()
{
}

void PersonTracking::ImageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    frame = cv_ptr->image;
}

void PersonTracking::Process()
{
    cv::flip(frame, frame, +1);
    int b = frame.cols, h = frame.rows;
    int64 t1 = cv::getTickCount();

    cv::Mat frameRGB, frameResized;
    cv::cvtColor(frame, frameRGB, cv::COLOR_BGR2RGB);
    cv::resize(frameRGB, frameResized, cv::Size(width_, height_));

    // Set input data
    memcpy(inputTensor->data.raw, frameResized.data, width_*height_*3);

    interpreter_->SetAllowFp16PrecisionForFp32(true);
    interpreter_->SetNumThreads(4);

    interpreter_->Invoke();

    const float* detection_locations = interpreter_->tensor(interpreter_->outputs()[0])->data.f;
    const float* detection_classes = interpreter_->tensor(interpreter_->outputs()[1])->data.f;
    const float* detection_scores = interpreter_->tensor(interpreter__->outputs()[2])->data.f;

    cv::Point center(0, 0);
    float area = 0.0;

    if (labels_[static_cast<int>(detection_classes[0])] == "person") {
        if (detection_scores[0] > 0.45 && detection_scores[0] < 1.0) {
            int det_index = (int)detection_classes[0] + 1;
            float miny = detection_locations[0]*h;
            float minx = detection_locations[1]*b;
            float maxy = detection_locations[2]*h;
            float maxx = detection_locations[3]*b;

            center.x = static_cast<int>(round((maxx + minx) / 2.0));
            center.y = static_cast<int>(round((maxy + miny) / 2.0));

            cv::circle(frame, center, 15, cv::Scalar(0, 0, 255), -1);
            cv::rectangle(frame, cv::Point(minx, miny), cv::Point(maxx, maxy), cv::Scalar(0, 255, 0), 2);
            area = std::abs(100.0 * ((static_cast<float>(maxy) / h) - (static_cast<float>(miny) / h)));

            
            textStream_ << std::fixed << std::setprecision(2) << area;
            cv::putText(frame, "Area: " + textStream_.str() + "%", cv::Point(30, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }
    }
    
    error_ = distanceFromCenter(frame, center);
    cv::putText(frame, "FPS: " + std::to_string(int(frameRateCalc_)), cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    
    // Calculate frame rate for next loop
    int64 t2 = cv::getTickCount();
    double time1 = (t2 - t1) / freq_;
    frameRateCalc_ = 1 / time1;      

    PublishPosition(error_);
}

void PersonTracking::PublishPosition(double error)
{
    targetMsg_.header.stamp = ros::Time::now();
    targetMsg_.point.x = error;
    targetMsg_.point.y = 0;
    targetMsg_.point.z = 0;

    targetPub_.publish(targetMsg_);
}

double distanceFromCenter(cv::Mat& frame, cv::Point center) {
    double error_ = 0.0;
    textStream_.str("");
    textStream_.clear();
    
    if (center != cv::Point(0, 0)) {
        // Draw center
        cv::circle(frame, cv::Point(frame.cols/2, frame.rows/2), 15, cv::Scalar(255, 0, 0), -1);
        
        // Draw line connecting two points
        cv::line(frame, cv::Point(frame.cols/2, frame.rows/2), cv::Point(center.x, center.y), cv::Scalar(255, 255, 0), 10);
        differences = 0.5 - (static_cast<float>(center.x) / static_cast<float>(frame.cols));
        textStream_ << std::fixed << std::setprecision(2) << differences * 100;
        cv::putText(frame, "X Delta = " + textStream_.str() + "%", cv::Point(30, 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
    textStream_.str("");
    textStream_.clear();
    return differences;
}