
#include <numeric>
#include "matching2D.hpp"

using namespace std;

Descriptor::Descriptor(std::string descriptorType){
    if (descriptorType.compare("BRISK") == 0)
    {
        descriptorType_ = DescriptorType::BRISK;
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor_ = cv::BRISK::create(threshold, octaves, patternScale);
    }else if(descriptorType.compare("BRIEF") == 0){
        descriptorType_ = DescriptorType::BRIEF;
        const int bytes = 32;
        const bool use_orientation = false;
        extractor_ = cv::xfeatures2d::BriefDescriptorExtractor::create(bytes, use_orientation);

    }else if(descriptorType.compare("ORB") == 0){
        descriptorType_ = DescriptorType::ORB;
        const int nfeatures = 500;  // The maximum number of features to retain. 
        const float scaleFactor = 1.2f; // Pyramid decimation ratio
        const int nlevels = 8;   //  number of pyramid levels
        const int edgeThreshold = 31; // size of the border where the features are not detected
        const int firstLevel = 0; //  level of pyramid to put source image to
        const int WTA_K = 2; //  level of pyramid to put source image to
        const cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE; //FAST_SCORE
        const int patchSize = 31; // size of the patch used by the oriented BRIEF descriptor
        const int fastThreshold = 20; //the fast threshold     
        extractor_ = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, 
                                    firstLevel, WTA_K, scoreType, patchSize, fastThreshold);  

    }else if(descriptorType.compare("FREAK") == 0){
        descriptorType_ = DescriptorType::FREAK;
        const bool orientationNormalized = true;
        const bool scaleNormalized = true;
        const float patternScale = 22.0f;
        const int nOctaves = 4;
        extractor_ = cv::xfeatures2d::FREAK::create(orientationNormalized, scaleNormalized, patternScale, nOctaves);

    }else if(descriptorType.compare("AKAZE") == 0){        
        descriptorType_ = DescriptorType::AKAZE;
        const cv::AKAZE::DescriptorType descriptor_type = cv::AKAZE::DESCRIPTOR_MLDB; // DESCRIPTOR_KAZE_UPRIGHT, DESCRIPTOR_MLDB_UPRIGHT, DESCRIPTOR_KAZE(rot invariant), DESCRIPTOR_MLDB (rot invariant) 
        const int descriptor_size = 0; // Size of the descriptor in bits. 0 -> Full size 
        const int descriptor_channels = 3; // Number of channels in the descriptor (1, 2, 3) 
        const float threshold = 0.001f; // 	Detector response threshold to accept point
        const int nOctaves = 4; 
        const int nOctaveLayers = 4;
        const cv::KAZE::DiffusivityType diffusivity = cv::KAZE::DIFF_PM_G2; // DIFF_PM_G1, DIFF_PM_G2, DIFF_WEICKERT or DIFF_CHARBONNIER 
        extractor_ = cv::AKAZE::create(descriptor_type, descriptor_size, 
                                    descriptor_channels, threshold, nOctaves, 
                                    nOctaveLayers, diffusivity);

    }else if(descriptorType.compare("SIFT") == 0){
        descriptorType_ = DescriptorType::SIFT;
        const int nfeatures = 0;
        const int nOctaveLayers = 3;
        const double contrastThreshold = 0.04;
        const double edgeThreshold = 10;
        const double sigma = 1.6;
        extractor_ = cv::xfeatures2d::SIFT::create( nfeatures, nOctaveLayers, 
                                        contrastThreshold,  edgeThreshold, sigma);
    }else{        
        throw std::runtime_error(descriptorType + " Keypoint descriptor not implemented");
    }  
}

void Descriptor::compute(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors){
    extractor_->compute(img, keypoints, descriptors);
}    

const Descriptor::DescriptorType Descriptor::getType(){
    return descriptorType_;
}


/// DETECTOR ////
Detector::Detector(std::string detectorType)
{
    if(detectorType.compare("FAST") == 0){        
        // Fast detector
        detectorType_ = DetectorType::FAST;
        const int threshold = 15; // difference between intensity of the central pixel and pixels of a circle around this pixel
        const bool bNMS = true;   // perform non-maxima suppression on keypoints
        const cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16; // TYPE_9_16, TYPE_7_12, TYPE_5_8
        detector_ = cv::FastFeatureDetector::create(threshold, bNMS, type);

    }else if(detectorType.compare("BRISK") == 0){        
        detectorType_ = DetectorType::BRISK;
        const int thresh = 30;
        const int octaves = 3;
        const float pattern_scale = 1.0;
        detector_ = cv::BRISK::create(thresh, octaves, pattern_scale);

    }else if(detectorType.compare("ORB") == 0){
        detectorType_ = DetectorType::ORB;            
        const int nfeatures = 500;  // The maximum number of features to retain. 
        const float scaleFactor = 1.2f; // Pyramid decimation ratio
        const int nlevels = 8;   //  number of pyramid levels
        const int edgeThreshold = 31; // size of the border where the features are not detected
        const int firstLevel = 0; //  level of pyramid to put source image to
        const int WTA_K = 2; //  level of pyramid to put source image to
        const cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE; //FAST_SCORE
        const int patchSize = 31; // size of the patch used by the oriented BRIEF descriptor
        const int fastThreshold = 20; //the fast threshold     
        detector_ = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, 
                                    firstLevel, WTA_K, scoreType, patchSize, fastThreshold);  

    }else if(detectorType.compare("AKAZE") == 0){        
        detectorType_ = DetectorType::AKAZE;            
        const cv::AKAZE::DescriptorType descriptor_type = cv::AKAZE::DESCRIPTOR_MLDB; // DESCRIPTOR_KAZE_UPRIGHT, DESCRIPTOR_MLDB_UPRIGHT, DESCRIPTOR_KAZE(rot invariant), DESCRIPTOR_MLDB (rot invariant) 
        const int descriptor_size = 0; // Size of the descriptor in bits. 0 -> Full size 
        const int descriptor_channels = 1; // Number of channels in the descriptor (1, 2, 3) 
        const float threshold = 0.001f; // 	Detector response threshold to accept point
        const int nOctaves = 4; 
        const int nOctaveLayers = 4;
        const cv::KAZE::DiffusivityType diffusivity = cv::KAZE::DIFF_PM_G2; // DIFF_PM_G1, DIFF_PM_G2, DIFF_WEICKERT or DIFF_CHARBONNIER 
        detector_ = cv::AKAZE::create(descriptor_type, descriptor_size, 
                                    descriptor_channels, threshold, nOctaves, 
                                    nOctaveLayers, diffusivity);

    }else if(detectorType.compare("SIFT") == 0){        
        detectorType_ = DetectorType::SIFT;            
        const int nfeatures = 0;
        const int nOctaveLayers = 3;
        const double contrastThreshold = 0.04;
        const double edgeThreshold = 10;
        const double sigma = 1.6;
        detector_ = cv::xfeatures2d::SIFT::create( nfeatures, nOctaveLayers, 
                                        contrastThreshold,  edgeThreshold, sigma);
    }else if(detectorType.compare("HARRIS") == 0){
        detectorType_ = DetectorType::HARRIS;       
    }else if(detectorType.compare("SHITOMASI") == 0){
        detectorType_ = DetectorType::SHITOMASI;       
    }else{        
        throw std::runtime_error(detectorType + " Keypoint detector not implemented");
    }
}

void Detector::detectKp(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis){
    if (detectorType_ == DetectorType::SHITOMASI)  
    {
        detKeypointsShiTomasi(keypoints, img, bVis);
    }
    else if(detectorType_ == DetectorType::HARRIS)
    {
        detKeypointsHarris(keypoints, img, bVis);
    }else{
        detector_->detect(img, keypoints, cv::Mat());
        // visualize results
        if (bVis)
        {
            cv::Mat visImage = img.clone();
            cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            std::string windowName = " Corner Detector Results";
            cv::namedWindow(windowName, 6);
            imshow(windowName, visImage);
            cv::waitKey(0);
        }            
    }
}

void Detector::detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / std::max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection    
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    
    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void Detector::detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis){
    const int blockSize = 2;
    const int apertureSize = 3;
    const double k = 0.04;
    const int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    const double maxOverlap = 0.0;

    cv::Mat dst = cv::Mat::zeros( img.size(), CV_32FC1 );
    cornerHarris( img, dst, blockSize, apertureSize, k );
 

    cv::Mat dst_norm;
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
   
    for(size_t j = 0; j<dst_norm.rows; j++){
        for(size_t i = 0; i<dst_norm.cols; i++){
            const auto& response = dst_norm.at<float>(j,i);
            if(response > minResponse){
                const auto size = std::sqrt(2) * apertureSize;
                cv::KeyPoint new_kp(static_cast<float>(i), static_cast<float>(j), size, -1, response);     

                // non-maximum suppression in local neighbourhood
                bool b_overlap = false;
                for(auto it = keypoints.begin(); it != keypoints.end(); it++){
                    double kp_overlap = cv::KeyPoint::overlap(*it, new_kp);

                    if(kp_overlap > maxOverlap){
                        b_overlap = true;
                        if(new_kp.response > it->response){
                            (*it) = new_kp; 
                        }
                        break;
                    }
                }

                if(!b_overlap){
                    keypoints.push_back(new_kp);
                }
            }
        }
    }

    // visualize results
    if (bVis)
    {
        cv::Mat dst_norm_scaled;
        cv::convertScaleAbs(dst_norm, dst_norm_scaled);
        cv::Mat visImage = dst_norm_scaled.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

const Detector::DetectorType Detector::getType(){
    return detectorType_;
}

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        if(descriptorType.compare("SIFT") == 0 && normType == cv::NORM_HAMMING){
            std::cerr << "Using L2 norm since SIFT not support hamming distance " << std::endl;
            normType = cv::NORM_L2;
        }
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        if(descSource.type()!=CV_32F) {
            descSource.convertTo(descSource, CV_32F);
        }
        if(descRef.type()!=CV_32F) {
            descRef.convertTo(descRef, CV_32F);
        }
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher->knnMatch( descSource, descRef, knn_matches, 2 );
        const float ratio_thresh = 0.8f;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                matches.push_back(knn_matches[i][0]);
            }
        }
    }
}