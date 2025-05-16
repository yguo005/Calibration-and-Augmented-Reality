/*
CS5330 Project4
Yunyu Guo
March 13 2025

Load calibration parameters from the saved file
Start video capture
For each frame:
Detect ArUco markers
Estimate board pose
Display real-time position and rotation
Draw 3D axes on the board
*/

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

class CameraPoseEstimator {
private:
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Ptr<cv::aruco::GridBoard> gridboard;
    cv::aruco::Dictionary dictionary;
    const float MARKER_LENGTH = 0.05f;    // 5cm
    const float MARKER_SEPARATION = 0.01f; // 1cm

public:
    CameraPoseEstimator(const std::string& calibration_file) {
        // Load calibration data
        cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            throw std::runtime_error("Could not open calibration file");
        }
        
        fs["camera_matrix"] >> camera_matrix;
        fs["distortion_coefficients"] >> dist_coeffs;
        fs.release();

        // Setup ArUco board
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        gridboard = cv::makePtr<cv::aruco::GridBoard>(
            cv::Size(5, 7),
            MARKER_LENGTH,
            MARKER_SEPARATION,
            dictionary
        );
    }

    void estimatePose(const cv::Mat& frame) {
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::aruco::DetectorParameters detectorParams;
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);
        
        // Detect markers
        detector.detectMarkers(frame, marker_corners, marker_ids);
        
        if (marker_ids.size() > 0) {
            cv::Mat rvec, tvec;
            
            // Estimate board pose
            bool valid = cv::aruco::estimatePoseBoard(
                marker_corners, marker_ids, gridboard,
                camera_matrix, dist_coeffs,
                rvec, tvec
            );

            if (valid) {
                // Convert rotation vector to matrix
                cv::Mat R;
                cv::Rodrigues(rvec, R);
                
                // Print position data
                std::cout << "\nCamera Position (meters):" << std::endl;
                std::cout << "X: " << std::fixed << std::setprecision(3) << tvec.at<double>(0) << std::endl;
                std::cout << "Y: " << std::fixed << std::setprecision(3) << tvec.at<double>(1) << std::endl;
                std::cout << "Z: " << std::fixed << std::setprecision(3) << tvec.at<double>(2) << std::endl;
                
                // Get Euler angles
                std::vector<double> euler = rotationMatrixToEulerAngles(R);
                std::cout << "\nCamera Rotation (degrees):" << std::endl;
                std::cout << "Roll : " << std::fixed << std::setprecision(1) << euler[0] << std::endl;
                std::cout << "Pitch: " << std::fixed << std::setprecision(1) << euler[1] << std::endl;
                std::cout << "Yaw  : " << std::fixed << std::setprecision(1) << euler[2] << std::endl;
                
                // Draw axes
                cv::drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1);
            }
            
            // Draw detected markers
            cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);
        }
    }

private:
    std::vector<double> rotationMatrixToEulerAngles(const cv::Mat& R) {
        std::vector<double> euler(3);
        
        double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) + 
                        R.at<double>(1,0) * R.at<double>(1,0));
        
        bool singular = sy < 1e-6;
        
        if (!singular) {
            euler[0] = atan2(R.at<double>(2,1), R.at<double>(2,2)) * 180/CV_PI;
            euler[1] = atan2(-R.at<double>(2,0), sy) * 180/CV_PI;
            euler[2] = atan2(R.at<double>(1,0), R.at<double>(0,0)) * 180/CV_PI;
        } else {
            euler[0] = atan2(-R.at<double>(1,2), R.at<double>(1,1)) * 180/CV_PI;
            euler[1] = atan2(-R.at<double>(2,0), sy) * 180/CV_PI;
            euler[2] = 0;
        }
        
        return euler;
    }
};

int main() {
    try {
        cv::VideoCapture cap(0);
        if (!cap.isOpened()) {
            std::cerr << "Could not open camera" << std::endl;
            return -1;
        }

        CameraPoseEstimator estimator("camera_calibration.yml");
        
        cv::Mat frame;
        while (true) {
            cap >> frame;
            if (frame.empty()) break;

            estimator.estimatePose(frame);
            
            cv::imshow("Camera Pose Estimation", frame);
            
            char key = (char)cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q')
                break;
        }

        cap.release();
        cv::destroyAllWindows();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}