#ifndef matching2D_hpp
#define matching2D_hpp

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"

class Descriptor{    
public:
    enum DescriptorType{
        BRISK,
        BRIEF,
        ORB,
        FREAK,
        AKAZE,
        SIFT
    };

    Descriptor(std::string descriptorType);
    void compute(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors);
    const Descriptor::DescriptorType getType();

private:
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    DescriptorType descriptorType_;
};

class Detector{
    public:
    enum DetectorType{
        FAST,
        BRISK,
        ORB,
        AKAZE,
        SIFT,
        HARRIS,
        SHITOMASI,
    };

    Detector(std::string detectorType);
    void detectKp(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis);
    // Detect keypoints in image using the traditional Shi-Thomasi detector
    void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis);
    // Detect keypoints in image using the traditional Harris detector
    void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis);
    const Detector::DetectorType getType();    
private:
    cv::Ptr<cv::FeatureDetector> detector_;
    DetectorType detectorType_;
};

void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType);

#endif /* matching2D_hpp */
