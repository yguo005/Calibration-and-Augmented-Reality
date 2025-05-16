/*
CS5330 Project 4
Yunyu Guo
March 9 2025

Creates a virtual snowman 
Projects the 3D points onto the image plane
*/

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>

class VirtualObjectRenderer {
private:
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Ptr<cv::aruco::GridBoard> gridboard;
    cv::aruco::Dictionary dictionary;
    const float MARKER_LENGTH = 0.05f;    // 5cm
    const float MARKER_SEPARATION = 0.01f; // 1cm
    
    // 3D points for virtual object (snowman)
    std::vector<cv::Point3f> object_vertices;
    std::vector<std::pair<int, int>> object_lines;

    void createVirtualObject() {
        float base_height = 0.1f;     // Height from board
        float bottom_radius = 0.06f;  // Bottom circle radius
        float head_radius = 0.04f;    // Head circle radius
        
        // Define snowman vertices 
        object_vertices = {
            // Bottom circle center
            cv::Point3f(0, 0, base_height + bottom_radius),               // 0
            // Head circle center
            cv::Point3f(0, 0, base_height + 2*bottom_radius + head_radius), // 1
            
            // Left eye
            cv::Point3f(-0.01f, -0.01f, base_height + 2*bottom_radius + head_radius), // 2
            // Right eye
            cv::Point3f(0.01f, -0.01f, base_height + 2*bottom_radius + head_radius),  // 3
            
            // Straight arm points (left)
            cv::Point3f(-bottom_radius, 0, base_height + bottom_radius), // 4
            cv::Point3f(-bottom_radius - 0.05f, 0, base_height + bottom_radius), // 5
            
            // Triangle arm points (right)
            cv::Point3f(bottom_radius, 0, base_height + bottom_radius),  // 6
            cv::Point3f(bottom_radius + 0.05f, 0.03f, base_height + bottom_radius), // 7
            cv::Point3f(bottom_radius + 0.05f, -0.03f, base_height + bottom_radius) // 8
        };
    }

public:
    VirtualObjectRenderer(const std::string& calibration_file) {
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

        createVirtualObject();
    }

    void processFrame(cv::Mat& frame) {
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::aruco::DetectorParameters detectorParams;
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);
        
        detector.detectMarkers(frame, marker_corners, marker_ids);
        
        if (marker_ids.size() > 0) {
            cv::Mat rvec, tvec;
            
            bool valid = cv::aruco::estimatePoseBoard(
                marker_corners, marker_ids, gridboard,
                camera_matrix, dist_coeffs,
                rvec, tvec
            );

            if (valid) {
                // Project 3D points to image plane
                std::vector<cv::Point2f> image_points;
                cv::projectPoints(object_vertices, rvec, tvec, 
                                camera_matrix, dist_coeffs, 
                                image_points);

                // Draw virtual object
                drawVirtualObject(frame, image_points);
                
                // Draw board axes for reference
                cv::drawFrameAxes(frame, camera_matrix, dist_coeffs, 
                                rvec, tvec, MARKER_LENGTH);
            }
            
            cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);
        }
    }

private:
    void drawVirtualObject(cv::Mat& frame, 
                          const std::vector<cv::Point2f>& image_points) {
        // Calculate radii based on Z-distance for perspective effect
        float z_scale = 1.0f / (cv::norm(image_points[1] - image_points[0]) / 100.0f);
        int bottom_radius = static_cast<int>(40 * z_scale);
        int head_radius = static_cast<int>(30 * z_scale);
        int eye_radius = static_cast<int>(3 * z_scale);
        
        // Draw bottom circle
        cv::circle(frame, image_points[0], bottom_radius, cv::Scalar(255, 255, 255), -1);
        cv::circle(frame, image_points[0], bottom_radius, cv::Scalar(0, 0, 0), 2);
        
        // Draw head circle
        cv::circle(frame, image_points[1], head_radius, cv::Scalar(255, 255, 255), -1);
        cv::circle(frame, image_points[1], head_radius, cv::Scalar(0, 0, 0), 2);
        
        // Draw eyes
        cv::circle(frame, image_points[2], eye_radius, cv::Scalar(0, 0, 0), -1);
        cv::circle(frame, image_points[3], eye_radius, cv::Scalar(0, 0, 0), -1);
        
        // Draw straight arm (left)
        cv::line(frame, image_points[4], image_points[5], cv::Scalar(139, 69, 19), 2);
        
        // Draw triangle arm outline (right)
        cv::line(frame, image_points[6], image_points[7], cv::Scalar(139, 69, 19), 2);
        cv::line(frame, image_points[7], image_points[8], cv::Scalar(139, 69, 19), 2);
        cv::line(frame, image_points[8], image_points[6], cv::Scalar(139, 69, 19), 2);
    }
};

int main() {
    try {
        cv::VideoCapture cap(0);
        if (!cap.isOpened()) {
            std::cerr << "Could not open camera" << std::endl;
            return -1;
        }

        VirtualObjectRenderer renderer("camera_calibration.yml");
        
        cv::Mat frame;
        while (true) {
            cap >> frame;
            if (frame.empty()) break;

            renderer.processFrame(frame);
            
            cv::imshow("Virtual Object", frame);
            
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