/*
CS5330 Project 4
Yunyu Guo
March 9 2025

Camera Calibration using ARuco Markers

 - Captures video from camera
 - Detects ARuco markers
 - Allows user to select frames for calibration
 - Stores 2D corner points and corresponding 3D world points
 - Press 's' to save current frame for calibration
 - Press 'q' to quit
 */

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include "calibration.hpp" // implements all methods in header file

// Constructor implementation
ArucoCalibrator::ArucoCalibrator(int camera_id, float marker_size) 
    : validDetection(false), markerSize(marker_size) {
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

char ArucoCalibrator::processFrame() {
    cv::Mat frame;
    cap.read(frame);
    if (frame.empty()) return 'q';

    detectMarkers(frame);
    cv::imshow("Camera Calibration", frame);

    return (char)cv::waitKey(1);
}

void ArucoCalibrator::detectMarkers(cv::Mat& frame) {
    std::vector<std::vector<cv::Point2f>> markerCorners;
    markerIds.clear();  // Clear previous IDs
    std::vector<std::vector<cv::Point2f>> rejected;

    detector.detectMarkers(frame, markerCorners, markerIds, rejected);

    if (markerIds.size() > 0) {
        // Draw detected markers
        cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        lastCorners = markerCorners;
        frame.copyTo(lastFrame);
        validDetection = true;

        // Display number of markers detected
        std::string text = "Detected " + std::to_string(markerIds.size()) + " markers";
        cv::putText(frame, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                    0.8, cv::Scalar(0, 255, 0), 2);
    } else {
        validDetection = false;
        cv::putText(frame, "No markers detected", cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    }
}

void ArucoCalibrator::saveCalibrationFrame() {
    if (!validDetection) {
        std::cout << "No valid markers detected!" << std::endl;
        return;
    }

    std::vector<cv::Point2f> imagePoints;
    std::vector<cv::Vec3f> objectPoints;

    // Debug information
    std::cout << "\nDetected " << markerIds.size() << " markers with IDs: ";
    for (int id : markerIds) {
        std::cout << id << " ";
    }
    std::cout << std::endl;

    // Process each detected marker
    for (size_t i = 0; i < lastCorners.size(); i++) {
        int markerId = markerIds[i];
        
        // Calculate marker position based on its ID
        // Assuming markers are arranged in a grid with markerSize spacing
        int row = i / 3;  // Assuming 3 markers per row
        int col = i % 3;
        float x = col * (markerSize * 1.5f);  // 1.5 times marker size spacing
        float y = row * (markerSize * 1.5f);

        // Print corner positions for debugging
        std::cout << "Marker " << markerId << " corners:" << std::endl;
        for (const auto& corner : lastCorners[i]) {
            std::cout << "(" << corner.x << ", " << corner.y << ") ";
        }
        std::cout << std::endl;

        // Add the four corners of this marker
        objectPoints.push_back(cv::Vec3f(x, y, 0.0f));
        objectPoints.push_back(cv::Vec3f(x + markerSize, y, 0.0f));
        objectPoints.push_back(cv::Vec3f(x + markerSize, y + markerSize, 0.0f));
        objectPoints.push_back(cv::Vec3f(x, y + markerSize, 0.0f));

        // Add the image points
        imagePoints.insert(imagePoints.end(), lastCorners[i].begin(), lastCorners[i].end());
    }

    // Save frame if any markers are detected
    if (imagePoints.size() > 0) {  // Changed from 12 to 0
        point_list.push_back(objectPoints);
        corner_list.push_back(imagePoints);
        calibration_images.push_back(lastFrame.clone());
        
        std::cout << "Calibration frame " << point_list.size() << " saved with " 
                  << imagePoints.size()/4 << " markers" << std::endl;
        std::cout << "3D-2D point pairs in this frame: " << imagePoints.size() << std::endl;
    } else {
        std::cout << "No markers detected in this frame" << std::endl;
    }
}

void ArucoCalibrator::run() {
    while (true) {
        char key = processFrame();
        if (key == 'q') {
            break;
        }
        else if (key == 's') {
            saveCalibrationFrame();
        }
    }
}

ArucoCalibrator::~ArucoCalibrator() {
    if (cap.isOpened()) {
        cap.release();
    }
    cv::destroyAllWindows();
}

/*************************************** 
int main() {
    try {
        ArucoCalibrator calibrator(1);  
        calibrator.run();
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    return 0;
} 
***************************************/