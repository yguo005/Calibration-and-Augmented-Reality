/*
CS5330 Project 4
Yunyu Guo
March 9 2025

   Camera Calibration using ARuco Markers
   - Calculates camera matrix and distortion coefficients
   - Prints calibration results and reprojection error
   - Saves calibration parameters to file
   
 */

#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <map>

class ArucoCalibrator {
private:
    // Camera and detection members
    cv::VideoCapture cap;
    cv::aruco::ArucoDetector detector;
    
    // Lists for storing calibration data
    std::vector<std::vector<cv::Vec3f>> point_list;      // 3D world points
    std::vector<std::vector<cv::Point2f>> corner_list;   // 2D image points
    std::vector<cv::Mat> calibration_images;             // Saved frames
    
    // Cache for last successful detection
    std::vector<std::vector<cv::Point2f>> lastCorners;
    std::vector<int> markerIds;
    cv::Mat lastFrame;
    bool validDetection;
    
    // Marker configuration
    float markerSize;
    std::map<int, cv::Vec3f> markerPositions;

public:
    
    explicit ArucoCalibrator(int camera_id = 0, float marker_size = 0.02);
    
    // Process a single frame and return key pressed
    char processFrame();
    
    // Save current frame for calibration
    void saveCalibrationFrame();
    
    // Getters for calibration data
    std::vector<std::vector<cv::Vec3f>> getPointList() const { return point_list; }
    std::vector<std::vector<cv::Point2f>> getCornerList() const { return corner_list; }
    std::vector<cv::Mat> getCalibrationImages() const { return calibration_images; }
    
    // Destructor to release resources
    ~ArucoCalibrator();

    void run();

private:
    // Helper function to detect markers in a frame
    void detectMarkers(cv::Mat& frame);
};

#endif 
