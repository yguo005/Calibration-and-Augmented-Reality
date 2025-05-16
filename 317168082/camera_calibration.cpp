/*
CS5330 Project 4
Yunyu Guo
March 9 2025

  Camera Calibration using ARuco Markers
  - Requires minimum 5 calibration frames
  - Calculates camera matrix and distortion coefficients
  - Prints calibration results and reprojection error
  - Saves calibration parameters to file
  
 */

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <fstream>
#include "calibration.hpp"

class CameraCalibrator {
private:
    static const int MIN_FRAMES = 5;
    cv::Mat camera_matrix;
    cv::Mat distortion_coeffs;
    cv::Size image_size;
    double last_error;

public:
    CameraCalibrator(const cv::Size& frameSize) 
        : image_size(frameSize), last_error(-1) {
        
        // Always initialize with default values
        initializeDefaultParameters();
    }

    void initializeDefaultParameters() {
        // Initialize camera matrix with default values
        camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        camera_matrix.at<double>(0,0) = 1.0;  // fx
        camera_matrix.at<double>(1,1) = 1.0;  // fy
        camera_matrix.at<double>(0,2) = image_size.width/2.0;   // cx (principal point)
        camera_matrix.at<double>(1,2) = image_size.height/2.0;  // cy (principal point)

        // Initialize distortion coefficients to zero
        distortion_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    }

    double calibrate(const std::vector<std::vector<cv::Vec3f>>& point_list,
                    const std::vector<std::vector<cv::Point2f>>& corner_list) {
        if (point_list.size() < MIN_FRAMES) {
            return -1;
        }

        // Reset parameters to default values before each calibration
        initializeDefaultParameters();

        // Print initial values
        std::cout << "\nInitial camera matrix:\n" << camera_matrix << std::endl;
        std::cout << "Initial distortion coeffs:\n" << distortion_coeffs << std::endl;

        // Use fewer constraints for better calibration with multiple markers
        int flags = cv::CALIB_FIX_ASPECT_RATIO;  

        // Perform calibration
        std::vector<cv::Mat> rvecs, tvecs;
        last_error = cv::calibrateCamera(
            point_list, 
            corner_list,
            image_size,
            camera_matrix,
            distortion_coeffs,
            rvecs,
            tvecs,
            flags
        );

        // Print results
        std::cout << "\nFinal camera matrix:\n" << camera_matrix << std::endl;
        std::cout << "Final distortion coeffs:\n" << distortion_coeffs << std::endl;
        std::cout << "Reprojection error: " << last_error << " pixels" << std::endl;

        
        if (last_error > 1.0) {
            std::cout << "WARNING: High reprojection error!" << std::endl;
            
        }

        return last_error;
    }

    void saveCalibration(const std::string& filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        fs << "camera_matrix" << camera_matrix;
        fs << "distortion_coefficients" << distortion_coeffs;
        fs << "reprojection_error" << last_error;
        fs.release();
        std::cout << "Calibration saved to " << filename << std::endl;
    }

    cv::Mat getCameraMatrix() const { return camera_matrix; }
    cv::Mat getDistortionCoeffs() const { return distortion_coeffs; }

    std::vector<double> calculatePerFrameErrors(
        const std::vector<std::vector<cv::Vec3f>>& point_list,
        const std::vector<std::vector<cv::Point2f>>& corner_list) {
        
        std::vector<double> frame_errors;
        
        // For each calibration frame
        for (size_t i = 0; i < point_list.size(); i++) {
            // Project 3D points to 2D using current calibration
            std::vector<cv::Point2f> projected_points;
            cv::Mat rvec, tvec;
            
            // Solve for rotation and translation of this frame
            cv::solvePnP(point_list[i], corner_list[i], camera_matrix, 
                         distortion_coeffs, rvec, tvec);
            
            // Project points using the estimated pose
            cv::projectPoints(point_list[i], rvec, tvec, camera_matrix, 
                              distortion_coeffs, projected_points);
            
            // Calculate error for this frame
            double err = 0;
            for (size_t j = 0; j < projected_points.size(); j++) {
                // Squared Euclidean distance
                double dx = projected_points[j].x - corner_list[i][j].x;
                double dy = projected_points[j].y - corner_list[i][j].y;
                err += dx*dx + dy*dy;
            }
            
            // Average error per point for this frame
            err = std::sqrt(err / projected_points.size());
            frame_errors.push_back(err);
        }
        
        return frame_errors;
    }
};


int main() {
    cv::Size frameSize(640, 480);
    CameraCalibrator calibrator(frameSize);
    ArucoCalibrator arucoDetector(1);  

    while (true) {
        char key = arucoDetector.processFrame();
        
        if (key == 's') {  // Save frame
            arucoDetector.saveCalibrationFrame();
            
            // Get current frame counts
            auto points = arucoDetector.getPointList();
            auto corners = arucoDetector.getCornerList();
            
            // Print current frame count
            std::cout << "\nFrame " << points.size() << " saved." << std::endl;
            
            // Calculate and show per-frame errors for all saved frames
            std::vector<double> frame_errors = calibrator.calculatePerFrameErrors(points, corners);
            
            // Print frame errors to console
            std::cout << "Per-frame reprojection errors:" << std::endl;
            for (size_t i = 0; i < frame_errors.size(); i++) {
                std::cout << "Frame " << i+1 << ": " << frame_errors[i] 
                          << " pixels" << std::endl;
            }
            
            // If have minimum frames, run calibration automatically
            if (points.size() >= 5) {  // Minimum 5 frames
                double error = calibrator.calibrate(points, corners);
                if (error >= 0) {  // Successful calibration
                    std::cout << "Current overall calibration error: " << error 
                              << " pixels (Frame count: " << points.size() << ")" << std::endl;
                }
            } else {
                std::cout << "Need " << 5 - points.size() 
                          << " more frames for calibration" << std::endl;
            }
        }
        else if (key == 'q') {  // Quit
            // Save calibration only when quitting and if have good results
            auto points = arucoDetector.getPointList();
            if (points.size() >= 5) {
                double error = calibrator.calibrate(points, arucoDetector.getCornerList());
                if (error >= 0 && error < 1.0) {  // Only save if error is less than 1 pixel
                    calibrator.saveCalibration("camera_calibration.yml");
                    std::cout << "Final calibration saved with error: " << error << " pixels" << std::endl;
                } else {
                    std::cout << "Calibration not saved due to high error: " << error << " pixels" << std::endl;
                }
            }
            break;
        }
    }
    return 0;
}
