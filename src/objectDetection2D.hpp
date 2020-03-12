
#ifndef objectDetection2D_hpp
#define objectDetection2D_hpp

#include <stdio.h>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include "dataStructures.h"

class ObjectDetector{
public:
ObjectDetector(std::string basePath, std::string classesFile, std::string modelConfiguration, std::string modelWeights);
void detectObjects(cv::Mat& img, std::vector<BoundingBox>& bBoxes, float confThreshold, 
        float nmsThreshold, bool bVis);    
private:
    std::vector<std::string> classes_; // detector classes
    cv::dnn::Net net_; // cv neural network

    std::vector<cv::String> names_; //names of output layers
};


#endif /* objectDetection2D_hpp */
