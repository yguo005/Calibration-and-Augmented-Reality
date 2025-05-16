/*
CS5330 Project 4
Yunyu Guo
March 9 2025

Projects 8 3D points onto the image plane
points 0-3 shows the 4 corners of the board 
Draws colored circles with point indices
Shows 3D axes at the board origin
Displays distance to the target
Updates visualization in real-time
*/

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>

class PoseVisualizer {
private:
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Ptr<cv::aruco::GridBoard> gridboard;
    cv::aruco::Dictionary dictionary;
    const float MARKER_LENGTH = 0.05f;    // 5cm
    const float MARKER_SEPARATION = 0.01f; // 1cm
    std::vector<cv::Point3f> object_points;

public:
    PoseVisualizer(const std::string& calibration_file) {
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

        // Create 3D points for visualization
        createVisualizationPoints();
    }

    void processFrame(cv::Mat& frame) {
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
                // Project 3D points onto image plane
                std::vector<cv::Point2f> projected_points;
                cv::projectPoints(object_points, rvec, tvec, 
                                camera_matrix, dist_coeffs, 
                                projected_points);

                // Draw projected points
                drawProjectedPoints(frame, projected_points);
                
                // Draw 3D axes
                cv::drawFrameAxes(frame, camera_matrix, dist_coeffs, 
                                rvec, tvec, MARKER_LENGTH * 2);

                // Add visualization info
                addVisualizationInfo(frame, rvec, tvec);
            }
            
            // Draw detected markers
            cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);
        }
    }

private:
    void createVisualizationPoints() {
        // Get board corners in 3D
        float width = gridboard->getGridSize().width * (MARKER_LENGTH + MARKER_SEPARATION);
        float height = gridboard->getGridSize().height * (MARKER_LENGTH + MARKER_SEPARATION);
        
        object_points = {
            cv::Point3f(0, 0, 0),           // Bottom-left
            cv::Point3f(width, 0, 0),       // Bottom-right
            cv::Point3f(width, height, 0),  // Top-right
            cv::Point3f(0, height, 0),      // Top-left
            // Still include axis endpoints for reference
            cv::Point3f(MARKER_LENGTH*2, 0, 0),  // X axis
            cv::Point3f(0, MARKER_LENGTH*2, 0),  // Y axis
            cv::Point3f(0, 0, MARKER_LENGTH*2),  // Z axis
            cv::Point3f(0, 0, 0)                 // Origin
        };
    }

    void drawProjectedPoints(cv::Mat& frame, 
                           const std::vector<cv::Point2f>& projected_points) {
        // Draw points with different colors
        cv::Scalar colors[] = {
            cv::Scalar(0, 0, 255),     // Red
            cv::Scalar(0, 255, 0),     // Green
            cv::Scalar(255, 0, 0),     // Blue
            cv::Scalar(255, 255, 0),   // Cyan
            cv::Scalar(255, 0, 255),   // Magenta
            cv::Scalar(0, 255, 255),   // Yellow
            cv::Scalar(128, 128, 255), // Light blue
            cv::Scalar(255, 128, 128)  // Light red
        };

        for (size_t i = 0; i < projected_points.size(); i++) {
            cv::circle(frame, projected_points[i], 3, colors[i % 8], -1);
            cv::putText(frame, std::to_string(i), 
                       projected_points[i] + cv::Point2f(5, 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, colors[i % 8], 2);
        }
    }

    void addVisualizationInfo(cv::Mat& frame, const cv::Mat& rvec, 
                             const cv::Mat& tvec) {
        std::string info = cv::format("Distance: %.2fm", cv::norm(tvec));
        cv::putText(frame, info, cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }
};

int main() {
    try {
        cv::VideoCapture cap(0);
        if (!cap.isOpened()) {
            std::cerr << "Could not open camera" << std::endl;
            return -1;
        }

        PoseVisualizer visualizer("camera_calibration.yml");
        
        cv::Mat frame;
        while (true) {
            cap >> frame;
            if (frame.empty()) break;

            visualizer.processFrame(frame);
            
            cv::imshow("3D Visualization", frame);
            
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