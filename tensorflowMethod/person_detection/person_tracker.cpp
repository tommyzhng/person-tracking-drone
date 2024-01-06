#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

// Include necessary headers
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/core/ocl.hpp>
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/string_util.h"
#include "tensorflow/lite/model.h"
#include <Python.h>

// Include your MyTracker class definition here
class MyTracker {
    public:
        void init() {
            // Initialize variables
            const std::string modelPath = "./person_detection/detect.tflite";
            const std::string labelPath = "./person_detection/labelmap.txt";

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
            cv::flip(frame, frame, +1);
            int b = frame.cols, h = frame.rows;
            int64 t1 = cv::getTickCount();

            cv::Mat frameRGB, frameResized;
            cv::cvtColor(frame, frameRGB, cv::COLOR_BGR2RGB);
            cv::resize(frameRGB, frameResized, cv::Size(width, height));

            // Set input data
            memcpy(inputTensor->data.raw, frameResized.data, width*height*3);

            interpreter->SetAllowFp16PrecisionForFp32(true);
            interpreter->SetNumThreads(4);

            interpreter->Invoke();

            const float* detection_locations = interpreter->tensor(interpreter->outputs()[0])->data.f;
            const float* detection_classes = interpreter->tensor(interpreter->outputs()[1])->data.f;
            const float* detection_scores = interpreter->tensor(interpreter->outputs()[2])->data.f;

            cv::Point center(0, 0);
            float area = 0.0;

            if (labels[static_cast<int>(detection_classes[0])] == "person") {
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

                    
                    textStream << std::fixed << std::setprecision(2) << area;
                    cv::putText(frame, "Area: " + textStream.str() + "%", cv::Point(30, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                }
            }
            
            double differences = distanceFromCenter(frame, center);
            cv::putText(frame, "FPS: " + std::to_string(int(frameRateCalc)), cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            
            // Calculate frame rate for next loop
            int64 t2 = cv::getTickCount();
            double time1 = (t2 - t1) / freq;
            frameRateCalc = 1 / time1;      
            return {frame, differences, area};
        }

    private:
        std::unique_ptr<tflite::Interpreter> interpreter;
        TfLiteTensor* inputTensor;
        TfLiteTensor* outputTensor;
        std::unique_ptr<tflite::FlatBufferModel> model;
        std::vector<std::string> labels;
        std::ostringstream textStream;
        int height, width;
        double frameRateCalc;
        double freq;
        

        double distanceFromCenter(cv::Mat& frame, cv::Point center) {
            double differences = 0.0;
            textStream.str("");
            textStream.clear();
            
            if (center != cv::Point(0, 0)) {
                // Draw center
                cv::circle(frame, cv::Point(frame.cols/2, frame.rows/2), 15, cv::Scalar(255, 0, 0), -1);
                
                // Draw line connecting two points
                cv::line(frame, cv::Point(frame.cols/2, frame.rows/2), cv::Point(center.x, center.y), cv::Scalar(255, 255, 0), 10);
                differences = 0.5 - (static_cast<float>(center.x) / static_cast<float>(frame.cols));
                textStream << std::fixed << std::setprecision(2) << differences * 100;
                cv::putText(frame, "X Delta = " + textStream.str() + "%", cv::Point(30, 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            }
            textStream.str("");
            textStream.clear();
            return differences;
        }
};

namespace py = pybind11;

// convert an np.array to a cv::Mat
cv::Mat from_array(const py::array& ar) {
    if (!ar.dtype().is(py::dtype::of<uchar>())) {
        std::cout << "ERROR unsupported dtype!" << std::endl;
        return cv::Mat();
    }

    auto shape = ar.shape();
    int rows = shape[0];
    int cols = shape[1];
    int channels = shape[2];
    int type = CV_MAKETYPE(CV_8U, channels); // CV_8UC3
    cv::Mat mat = cv::Mat(rows, cols, type);
    memcpy(mat.data, ar.data(), sizeof(uchar) * rows * cols * channels);

    return mat;
}

py::array to_array(const cv::Mat& im) {
    const ssize_t channels = im.channels();
    const ssize_t height = im.rows;
    const ssize_t width = im.cols;
    const ssize_t dim = sizeof(uchar) * height * width * channels;
    auto data = new uchar[dim];
    std::copy(im.data, im.data + dim, data);
    return py::array_t<uchar>(
        py::buffer_info(
            data,
            sizeof(uchar), //itemsize
            py::format_descriptor<uchar>::format(),
            channels, // ndim
            std::vector<ssize_t> { height, width, channels }, // shape
            std::vector<ssize_t> { width * channels, channels, sizeof(uchar) } // strides
        ),
        py::capsule(data, [](void* f){
            // handle releasing data
            delete[] reinterpret_cast<uchar*>(f);
        })
    );
}

PYBIND11_MODULE(person_tracker, m) {
    py::class_<MyTracker>(m, "MyTracker")
        .def(py::init()) // Binding the constructor
        .def("init", &MyTracker::init) // Binding the init method
        // Bind other methods of MyTracker as needed
        .def("process", [](MyTracker& self, py::array_t<unsigned char> &frame) {
            cv::Mat frameMat = from_array(frame);
    
            //Process
            auto result = self.process(frameMat);

            py::array_t<unsigned char> frameOut = to_array(frameMat);
            return py::make_tuple(frameOut, std::get<1>(result), std::get<2>(result));
    },  py::arg("frame"));
}
