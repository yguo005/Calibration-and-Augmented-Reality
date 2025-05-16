/*
CS5330 Project4
Yunyu Guo
March 13 2025

Only captures frame when markers are detected
Shows continuous frame preview
Displays frame count and error metrics
manual frame capture with 's' key
Saves calibration data only when quitting with enough frames
*/

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <iostream>
#include <ctime>
#include <iomanip>

class CameraCalibrator {
private:
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Size image_size;
    std::vector<std::vector<std::vector<cv::Point2f>>> allCorners;
    std::vector<std::vector<int>> allIds;
    std::vector<cv::Mat> rvecs, tvecs;
    const int MIN_CALIBRATION_FRAMES = 5;
    cv::Ptr<cv::aruco::GridBoard> gridboard;
    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::Dictionary dictionary;
    const float MARKER_LENGTH = 0.05f;    // 5cm - adjust based on your marker size
    const float MARKER_SEPARATION = 0.01f; // 1cm separation between markers
    double last_reprojection_error = 0.0;
    int calibrationFlags;

public:
    CameraCalibrator(const cv::Size& frameSize) : image_size(frameSize) {
        // Initialize calibration flags
        calibrationFlags = cv::CALIB_ZERO_TANGENT_DIST;
        
        // Initialize camera matrix with rough estimates
        camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        camera_matrix.at<double>(0,0) = frameSize.width;
        camera_matrix.at<double>(1,1) = frameSize.width;
        camera_matrix.at<double>(0,2) = frameSize.width/2.0;
        camera_matrix.at<double>(1,2) = frameSize.height/2.0;
        
        dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

        // Create ArUco dictionary and board
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        
        // Fix: Create GridBoard using makePtr
        gridboard = cv::makePtr<cv::aruco::GridBoard>(
            cv::Size(5, 7),
            MARKER_LENGTH,
            MARKER_SEPARATION,
            dictionary
        );
    }

    bool processFrame(const cv::Mat& frame, bool shouldCapture = false) {
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners, rejected;
        
        // Create detector with parameters and dictionary
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);
        detector.detectMarkers(frame, marker_corners, marker_ids, rejected);

        if (marker_ids.size() > 0) {
            // Draw detected markers
            cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);

            // Only store corners and IDs when explicitly capturing
            if (shouldCapture) {
                allCorners.push_back(marker_corners);
                allIds.push_back(marker_ids);
            }

            return true;
        }
        return false;
    }

    void printCameraParameters(const std::string& when = "current") const {
        std::cout << "\n=== Camera Parameters " << when << " ===" << std::endl;
        std::cout << "Camera Matrix:" << std::endl;
        for(int i = 0; i < 3; i++) {
            std::cout << "[";
            for(int j = 0; j < 3; j++) {
                std::cout << std::fixed << std::setprecision(6) << camera_matrix.at<double>(i,j);
                if(j < 2) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }
        
        std::cout << "\nDistortion Coefficients:" << std::endl;
        std::cout << "[";
        for(int i = 0; i < 5; i++) {
            std::cout << std::fixed << std::setprecision(6) << dist_coeffs.at<double>(i);
            if(i < 4) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    bool calibrate() {
        if (allCorners.size() < MIN_CALIBRATION_FRAMES) {
            return false;
        }

        // Print parameters before calibration
        printCameraParameters("before calibration");

        // Prepare data for calibration
        std::vector<std::vector<cv::Point2f>> allCornersConcatenated;
        std::vector<int> allIdsConcatenated;
        std::vector<int> markerCounterPerFrame;
        
        markerCounterPerFrame.reserve(allCorners.size());
        for(unsigned int i = 0; i < allCorners.size(); i++) {
            markerCounterPerFrame.push_back((int)allCorners[i].size());
            for(unsigned int j = 0; j < allCorners[i].size(); j++) {
                allCornersConcatenated.push_back(allCorners[i][j]);
                allIdsConcatenated.push_back(allIds[i][j]);
            }
        }

        // Perform calibration
        last_reprojection_error = cv::aruco::calibrateCameraAruco(
            allCornersConcatenated,
            allIdsConcatenated,
            markerCounterPerFrame,
            gridboard,
            image_size,
            camera_matrix,
            dist_coeffs,
            rvecs,
            tvecs,
            calibrationFlags
        );

        std::cout << "Calibration complete with error: " << last_reprojection_error << std::endl;
        
        // Print parameters after calibration
        printCameraParameters("after calibration");
        
        return true;
    }

    void saveCalibration(const std::string& filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        
        time_t tt;
        time(&tt);
        struct tm* t2 = localtime(&tt);
        char buf[1024];
        strftime(buf, sizeof(buf) - 1, "%c", t2);

        fs << "calibration_time" << buf;
        fs << "image_width" << image_size.width;
        fs << "image_height" << image_size.height;
        fs << "flags" << calibrationFlags;
        fs << "camera_matrix" << camera_matrix;
        fs << "distortion_coefficients" << dist_coeffs;
        fs << "avg_reprojection_error" << last_reprojection_error;
        
        fs.release();
        std::cout << "Calibration saved to " << filename << std::endl;
    }

    int getMinCalibrationFrames() const {
        return MIN_CALIBRATION_FRAMES;
    }

    double getReprojectionError() const {
        return cv::norm(dist_coeffs);
    }

    double getLastReprojectionError() const {
        return last_reprojection_error;
    }
};

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cout << "Error opening camera" << std::endl;
        return -1;
    }

    cv::Mat frame;
    cap >> frame;
    CameraCalibrator calibrator(frame.size());
    
    int frame_count = 0;
    
    // Print instructions
    std::cout << "Camera Calibration Instructions:" << std::endl;
    std::cout << "--------------------------------" << std::endl;
    std::cout << "Show the ArUco marker board to the camera" << std::endl;
    std::cout << "Press 's' to capture a frame" << std::endl;
    std::cout << "Press 'q' or 'ESC' to quit and save" << std::endl;
    std::cout << "--------------------------------" << std::endl;
    
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cv::Mat frameCopy = frame.clone();
        bool markersDetected = calibrator.processFrame(frameCopy, false);  // Don't capture by default
        
        // Display current frame count and status
        cv::putText(frameCopy, 
            "Frames: " + std::to_string(frame_count), 
            cv::Point(10, 30), 
            cv::FONT_HERSHEY_SIMPLEX, 0.7, 
            cv::Scalar(0, 255, 0), 2);

        if (frame_count >= calibrator.getMinCalibrationFrames()) {
            cv::putText(frameCopy,
                "Error: " + std::to_string(calibrator.getLastReprojectionError()) + " px", 
                cv::Point(10, 60), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                cv::Scalar(0, 255, 0), 2);
        }

        // Show capture instructions
        std::string instructions = (markersDetected) ? 
            "Markers detected - Press 's' to capture" : 
            "No markers detected";
        cv::putText(frameCopy, instructions,
            cv::Point(10, frame.rows - 20),
            cv::FONT_HERSHEY_SIMPLEX, 0.7,
            cv::Scalar(0, 255, 0), 2);

        char key = (char)cv::waitKey(1);
        
        // Capture frame when 's' is pressed and markers are detected
        if (key == 's' && markersDetected) {
            if (calibrator.processFrame(frameCopy, true)) {
                frame_count++;
                std::cout << "\n=== Frame " << frame_count << " captured ===" << std::endl;
                
                if (frame_count >= calibrator.getMinCalibrationFrames()) {
                    if (calibrator.calibrate()) {
                        double error = calibrator.getLastReprojectionError();
                        if (error > 0.0) {
                            std::cout << "Calibration updated with " << frame_count << " frames" << std::endl;
                            std::cout << "Current reprojection error: " << error << " pixels" << std::endl;
                        } else {
                            std::cout << "Warning: Calibration produced zero error - possible failure" << std::endl;
                        }
                    }
                }
            }
        }
        
        cv::imshow("Camera Calibration", frameCopy);
        
        if (key == 27 || key == 'q' || key == 'Q') {
            if (frame_count >= calibrator.getMinCalibrationFrames()) {
                std::cout << "\nFinal calibration results:" << std::endl;
                std::cout << "Total frames used: " << frame_count << std::endl;
                std::cout << "Final reprojection error: " << calibrator.getLastReprojectionError() 
                         << " pixels" << std::endl;
                calibrator.saveCalibration("camera_calibration.yml");
            }
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
