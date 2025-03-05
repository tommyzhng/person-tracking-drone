#include "person_tracking.hpp"

PersonTracking::PersonTracking(ros::NodeHandle nh) : nh_(nh)
{
    // paths
    std::string packagePath = ros::package::getPath("person_tracking_drone");
    const std::string modelPath = packagePath + "/src/cv_model/detect.tflite";
    const std::string labelPath = packagePath + "/src/cv_model/labelmap.txt";

    ROS_INFO("Model path: %s", modelPath.c_str());

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
    inputTensor_ = interpreter_->input_tensor(0);
    outputTensor_ = interpreter_->output_tensor(0);

    // Set input dimensions
    height_ = 300;
    width_ = 300;

    // Initialize other variables
    frameRateCalc_ = 1;
    freq_ = cv::getTickFrequency();

    // ROS
    imageSub_ = nh_.subscribe("/camera/image_rect", 1, &PersonTracking::ImageCb, this);
    targetPub_ = nh_.advertise<geometry_msgs::PointStamped>("/person_tracking/target", 10);
    detectionsPub_ = nh_.advertise<sensor_msgs::Image>("/person_tracking/detections_image", 1);
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
    frame_ = cv_ptr->image;
}

cv::Mat PersonTracking::resizeAndPad(cv::Mat &image, int target_width, int target_height) {
    if (image.empty()) {
        ROS_ERROR("Input image is empty");
        return cv::Mat();
    }
    if (image.cols == 0 || image.rows == 0) {
        ROS_ERROR("Image has empty rows or columns");
        return cv::Mat();
    }

    float aspect_ratio = float(image.cols) / float(image.rows);
    int new_width = target_width;
    int new_height = target_height;

    if (aspect_ratio > 1.0) {
        new_height = int(target_width / aspect_ratio);
    } else {
        new_width = int(target_height * aspect_ratio);
    }

    // new image with the target dimensions and padding
    cv::Mat padded_image(target_height, target_width, image.type(), cv::Scalar(0, 0, 0));

    int x_offset = (target_width - new_width) / 2;
    int y_offset = (target_height - new_height) / 2;

    // Using ptr for more efficient memory access
    for (int y = 0; y < new_height; ++y) {
        uchar* row_ptr = padded_image.ptr<uchar>(y + y_offset);  // Pointer to row y in the padded image
        for (int x = 0; x < new_width; ++x) {
            int orig_x = int(x * float(image.cols) / new_width);
            int orig_y = int(y * float(image.rows) / new_height);

            orig_x = std::min(orig_x, image.cols - 1);   // Prevent out of bounds
            orig_y = std::min(orig_y, image.rows - 1);

            // Use ptr to access the original image pixels efficiently
            const uchar* orig_row_ptr = image.ptr<uchar>(orig_y);
            row_ptr[x * 3] = orig_row_ptr[orig_x * 3];        // B
            row_ptr[x * 3 + 1] = orig_row_ptr[orig_x * 3 + 1];  // G
            row_ptr[x * 3 + 2] = orig_row_ptr[orig_x * 3 + 2];  // R
        }
    }
    return padded_image;
}

void PersonTracking::Process(cv::Mat frame)
{
    cv::flip(frame, frame, +1);
    int b = frame.cols, h = frame.rows;
    
    cv::Mat frameRGB, frameResized;
    frameResized = resizeAndPad(frame, width_, height_);
    
    if (!inputTensor_) {
        ROS_ERROR("Input tensor is null!");
        return;
    }

    // Set input data
    memcpy(inputTensor_->data.raw, frameResized.data, width_*height_*3);

    interpreter_->SetNumThreads(4);
    interpreter_->Invoke();

    const float* detection_locations = interpreter_->tensor(interpreter_->outputs()[0])->data.f;
    const float* detection_classes = interpreter_->tensor(interpreter_->outputs()[1])->data.f;
    const float* detection_scores = interpreter_->tensor(interpreter_->outputs()[2])->data.f;

    cv::Point center(0, 0);
    float area = 0.0;

    if (labels_[static_cast<int>(detection_classes[0])] == "person") {
        if (detection_scores[0] > 0.45 && detection_scores[0] < 1.0) {
            int det_index = (int)detection_classes[0] + 1;
            float miny = detection_locations[0]*h;
            float minx = detection_locations[1]*b;
            float maxy = detection_locations[2]*h;
            float maxx = detection_locations[3]*b;

            center.x = (int)(round((maxx + minx) / 2.0));
            center.y = (int)(round((maxy + miny) / 2.0));

            cv::circle(frame, center, 15, cv::Scalar(0, 0, 255), -1);
            cv::rectangle(frame, cv::Point(minx, miny), cv::Point(maxx, maxy), cv::Scalar(0, 255, 0), 2);
            area = std::abs(100.0 * ((static_cast<float>(maxy) / h) - (static_cast<float>(miny) / h)));
        }

    }
    
    CalculateError(frame, center); 
    
}

void PersonTracking::PublishPosition()
{
    targetMsg_.header.stamp = ros::Time::now();
    targetMsg_.point.x = xError_;
    targetMsg_.point.y = 0;
    targetMsg_.point.z = 0;

    targetPub_.publish(targetMsg_);
}

void PersonTracking::PublishDetectionsImage()
{
    cv_bridge::CvImage cvImage;
    cvImage.header.stamp = ros::Time::now();
    cvImage.encoding = sensor_msgs::image_encodings::BGR8;
    cvImage.image = frame_;
    detectionsPub_.publish(cvImage.toImageMsg());
}

void PersonTracking::CalculateError(cv::Mat& frame, cv::Point center) {
    textStream_.str("");
    textStream_.clear();
    
    if (center != cv::Point(0, 0)) {
        // Draw center
        cv::circle(frame, cv::Point(frame.cols/2, frame.rows/2), 15, cv::Scalar(255, 0, 0), -1);
        xError_ = 0.5 - ((double)(center.x) / (double)(frame.cols));
    }
}

void PersonTracking::Update()
{
    Process(frame_);
    PublishPosition();
    PublishDetectionsImage();
    ros::spinOnce(); // ros subs
}