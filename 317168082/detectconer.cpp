/*
CS5330 Project 4
Yunyu Guo
March 9 2025

This program uses OpenCV to detect ARuco markers in real-time video.
 - Captures video from camera
 - Detects 6x6 ARuco markers (250 unique IDs)
 - Draws detected markers with their IDs
 - Shows marker corners in clockwise order:
   [0] = Top-left
   [1] = Top-right
   [2] = Bottom-right
   [3] = Bottom-left
 - Press 'q' to quit
*/


#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

class ArucoDetector {
private:
    cv::VideoCapture cap;
    cv::aruco::ArucoDetector detector;
    const std::string WINDOW_NAME = "ARuco Marker Detection";

public:
    ArucoDetector(int camera_id = 1) {
        // Initialize camera
        cap.open(camera_id);
        if (!cap.isOpened()) {
            throw std::runtime_error("Could not open camera");
        }

        // Initialize ARuco detector
        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        detector = cv::aruco::ArucoDetector(dictionary, detectorParams);
    }

    void detectMarkers(cv::Mat& frame) {
        std::vector<std::vector<cv::Point2f>> markerCorners;
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> rejected;

        // Detect markers
        detector.detectMarkers(frame, markerCorners, markerIds, rejected);

        // Draw detected markers
        if (markerIds.size() > 0) {
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        }
    }

    void run() {
        while (true) {
            cv::Mat frame;
            cap.read(frame);
            if (frame.empty()) {
                throw std::runtime_error("Failed to capture frame");
            }

            detectMarkers(frame);
            
            // Display the frame
            cv::imshow(WINDOW_NAME, frame);

            // Break loop with 'q' key
            if (cv::waitKey(1) == 'q') {
                break;
            }
        }

        cleanup();
    }

    void cleanup() {
        cap.release();
        cv::destroyAllWindows();
    }

    //Destructor,It's called automatically when: The object goes out of scope; The program ends; delete is called on a pointer to the object
    ~ArucoDetector() {
        cleanup();
    }
};

int main() {
    try {
        ArucoDetector detector(1);  // Use camera 1
        detector.run();
    }
    catch (const std::exception& e) {
        // std::cerr is used to print error messages to the standard error stream
        //std::endl is used to flush the output buffer and add a newline character
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}
