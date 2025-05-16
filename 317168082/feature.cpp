/*
CS5330 Project 4
Yunyu Guo
March 9 2025

Implements both Harris corner and SURF feature detection

*/

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>  // Required for SURF
#include <iostream>

class FeatureDetector {
private:
    // Increase default threshold for Harris corners
    double k = 0.04;
    int blockSize = 2;
    int apertureSize = 3;
    double minResponse = 200;  // Increased from 100 to 200
    
    // Adjust SURF parameters
    int minHessian = 800;     // Increased from 400 to 800
    cv::Ptr<cv::xfeatures2d::SURF> surf_detector;
    
    bool use_harris = true;  // Toggle between Harris and SURF

public:
    FeatureDetector() {
        surf_detector = cv::xfeatures2d::SURF::create(minHessian);
        surf_detector->setExtended(true);
        surf_detector->setUpright(false);
    }
    
    void detectAndDrawFeatures(cv::Mat& frame) {
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        
        if (use_harris) {
            detectHarrisCorners(gray, frame);
        } else {
            detectSURFFeatures(gray, frame);
        }
        
        // Add UI text
        std::string method = use_harris ? "Harris Corners" : "SURF Features";
        cv::putText(frame, method, cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, "Press 'h' to switch method", cv::Point(10, 60),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }
    
private:
    void detectHarrisCorners(const cv::Mat& gray, cv::Mat& output) {
        cv::Mat dst = cv::Mat::zeros(gray.size(), CV_32FC1);
        
        // Add Gaussian blur to reduce noise
        cv::Mat blurred;
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
        
        // Detect Harris corners
        cv::cornerHarris(blurred, dst, blockSize, apertureSize, k);
        
        // Normalize and scale
        cv::Mat dst_norm;
        cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
        
        // Draw corners with non-maximum suppression
        int radius = 10;  // Minimum distance between corners
        for(int i = radius; i < dst_norm.rows-radius; i++) {
            for(int j = radius; j < dst_norm.cols-radius; j++) {
                float currentValue = dst_norm.at<float>(i,j);
                if(currentValue > minResponse) {
                    // Check if it's a local maximum
                    bool isMax = true;
                    for(int di = -radius; di <= radius && isMax; di++) {
                        for(int dj = -radius; dj <= radius; dj++) {
                            if(di == 0 && dj == 0) continue;
                            if(dst_norm.at<float>(i+di,j+dj) >= currentValue) {
                                isMax = false;
                                break;
                            }
                        }
                    }
                    if(isMax) {
                        cv::circle(output, cv::Point(j,i), 5, 
                                 cv::Scalar(0,0,255), 2, 8, 0);
                    }
                }
            }
        }
    }
    
    void detectSURFFeatures(const cv::Mat& gray, cv::Mat& output) {
        std::vector<cv::KeyPoint> keypoints;
        
        // Detect SURF features
        surf_detector->detect(gray, keypoints);
        
        // Draw keypoints with size and orientation
        cv::drawKeypoints(gray, keypoints, output, 
                         cv::Scalar(0,0,255),
                         cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    }
    
public:
    void toggleMethod() {
        use_harris = !use_harris;
    }
    
    void adjustThreshold(bool increase) {
        if (use_harris) {
            minResponse += increase ? 20 : -20;  // Larger step size
            minResponse = std::max(100.0, minResponse);  // Higher minimum
        } else {
            minHessian += increase ? 100 : -100;  // Larger step size
            minHessian = std::max(400, minHessian);  // Higher minimum
            surf_detector = cv::xfeatures2d::SURF::create(minHessian);
        }
    }
};

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Could not open camera" << std::endl;
        return -1;
    }

    FeatureDetector detector;
    
    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        detector.detectAndDrawFeatures(frame);
        
        cv::imshow("Feature Detection", frame);
        
        char key = (char)cv::waitKey(1);
        if (key == 27 || key == 'q') break;
        else if (key == 'h') detector.toggleMethod();
        else if (key == '+') detector.adjustThreshold(true);
        else if (key == '-') detector.adjustThreshold(false);
    }

    cap.release();
    cv::destroyAllWindows();
    
    return 0;
}
