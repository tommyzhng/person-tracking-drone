#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/core/ocl.hpp>
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/string_util.h"
#include "tensorflow/lite/model.h"
#include <cmath>

class Tracker {
public:
    Tracker() {
        // Initialize variables
        const std::string modelPath = "./model/detect.tflite";
        const std::string labelPath = "./model/labelmap.txt";

        // Load labels
        std::ifstream labelFile(labelPath);
        std::string line;
        while (std::getline(labelFile, line)) {
            labels.push_back(line);
        }

        // Load TFLite model
        model = tflite::FlatBufferModel::BuildFromFile(modelPath.c_str());
        tflite::ops::builtin::BuiltinOpResolver resolver;
        tflite::InterpreterBuilder(*model, resolver)(&interpreter);
        interpreter->AllocateTensors();

        // Get input and output tensor details
        inputTensor = interpreter->input_tensor(0);
        outputTensor = interpreter->output_tensor(0);

        // Set input dimensions
        height = 300;
        width = 300;

        // Initialize other variables
        frameRateCalc = 1;
        freq = cv::getTickFrequency();
    }

    std::tuple<cv::Mat, double, double> process(cv::Mat frame) {
        frame = cv::flip(frame, 1);
        int b = frame.cols, h = frame.rows;
        int64 t1 = cv::getTickCount();

        cv::Mat frameRGB, frameResized;
        cv::cvtColor(frame, frameRGB, cv::COLOR_BGR2RGB);
        cv::resize(frameRGB, frameResized, cv::Size(width, height));

        // Set input data
        memcpy(inputTensor->data.raw, frameResized.data, width*height*3);

        interpreter->SetAllowFp16PrecisionForFp32(true);
        interpreter->SetNumThreads(4);      //quad core

        interpreter->Invoke();

        const float* detection_locations = interpreter->tensor(interpreter->outputs()[0])->data.f;
        const float* detection_classes=interpreter->tensor(interpreter->outputs()[1])->data.f;
        const float* detection_scores = interpreter->tensor(interpreter->outputs()[2])->data.f;

        

        double differences = distanceFromCenter(frame, cv::Point(0, 0));
        int64 t2 = cv::getTickCount();
        double time1 = (t2 - t1) / freq;
        frameRateCalc = 1 / time1;

        return {frame, differences, 0.0};  // Placeholder for area calculation
    }

private:
    std::unique_ptr<tflite::Interpreter> interpreter;
    TfLiteTensor* inputTensor = nullptr;
    TfLiteTensor* outputTensor = nullptr;
    std::unique_ptr<tflite::FlatBufferModel> model;
    std::vector<std::string> labels;
    int height, width;
    double frameRateCalc;
    double freq;

    double distanceFromCenter(cv::Mat& frame, cv::Point center) {
    double differences;
    
    if (center != cv::Point(0, 0)) {
        cv::circle(frame, cv::Point(frame.cols/2, frame.rows/2), 15, cv::Scalar(255, 0, 0), -1);
        cv::line(frame, cv::Point(frame.cols/2, frame.rows/2), cv::Point(center.x, center.y), cv::Scalar(255, 255, 0), 10);
        
        differences = 0.5 - center.x / frame.cols;
        cv::putText(frame, "X Delta = " + std::to_string(round(differences * 100, 2)) + "%", cv::Point(30, 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    } else {
        differences = 0;
    }
    
    return differences;
}

};