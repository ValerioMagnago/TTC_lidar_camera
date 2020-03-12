/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <list>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"

#include <fstream>      // std::fstream

using namespace std;

#define LOOP

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */
#ifdef LOOP    
    std::vector<string> detectorTypes = {"FAST", "SHITOMASI", "ORB", "BRISK", "SIFT", "HARRIS", "AKAZE"}; //, "SHITOMASI", "AKAZE", "ORB", "FAST", "BRISK", "SIFT", "HARRIS"};
    std::vector<string> kpDescriptorTypes = {"ORB", "FREAK", "SIFT", "BRISK", "BRIEF", "AKAZE"};
    
    for(const string detectorType : detectorTypes){
        for(const string kpDescriptorType : kpDescriptorTypes){            
#else
    const string detectorType = "FAST"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
    const string kpDescriptorType = "BRIEF"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
#endif

    if ((kpDescriptorType.compare("AKAZE") == 0 && detectorType.compare("AKAZE") != 0) ||
        (detectorType.compare("SIFT") == 0 && kpDescriptorType.compare("ORB") == 0))
    {
        // AKAZE descriptors can only be used with KAZE or AKAZE keypoints.
        // ORB descriptors are not compatible with SIFT detetor
        #ifdef LOOP 
            continue;
        #else
            return -1;
        #endif
    }

    // Detectors    
    Detector detector(detectorType);

    // Descriptors    
    Descriptor descriptor(kpDescriptorType);

    // data location
    const string fileName = "./logs/" + detectorType + "_" + kpDescriptorType + ".txt";
    string dataPath = "../";
    
    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;   // last file index to load
    int imgStepWidth = 1; 
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    string yoloBasePath = dataPath + "dat/yolo/";
    string yoloClassesFile = yoloBasePath + "coco.names";
    string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    string yoloModelWeights = yoloBasePath + "yolov3.weights";
    ObjectDetector objectDetector(yoloBasePath, yoloClassesFile, yoloModelConfiguration, yoloModelWeights);

    // Lidar
    string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    string lidarFileType = ".bin";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    
    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;
    
    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;
    
    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;    

    // misc
    double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    //vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    list<DataFrame> dataBuffer;
    bool bVis = false;            // visualize results

    // Open log file
    std::fstream fs;
    fs.open (fileName, std::fstream::out);

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex+=imgStepWidth)
    {
        /* LOAD IMAGE INTO BUFFER */        
        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file 
        cv::Mat img = cv::imread(imgFullFilename);

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = img;
        dataBuffer.push_back(frame);
        if(dataBuffer.size() > dataBufferSize){
            dataBuffer.pop_front();
        }
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;


        /* DETECT & CLASSIFY OBJECTS */
        bVis = false;
        float confThreshold = 0.2;
        float nmsThreshold = 0.4;
        double t = (double)cv::getTickCount();       
        objectDetector.detectObjects(dataBuffer.back().cameraImg, dataBuffer.back().boundingBoxes, 
                confThreshold, nmsThreshold, bVis);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();                
        bVis = false;
        cout << "#2 : DETECT & CLASSIFY OBJECTS done in " << t << "[s]" << endl;


        /* CROP LIDAR POINTS */

        // load 3D Lidar points from file
        string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
        std::vector<LidarPoint> lidarPoints;
        loadLidarFromFile(lidarPoints, lidarFullFilename);

        // remove Lidar points based on distance properties
        float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
        t = (double)cv::getTickCount();       
        cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();                        
        dataBuffer.back().lidarPoints = lidarPoints;

        cout << "#3 : CROP LIDAR POINTS done in " << t << "[s]" << endl;


        /* CLUSTER LIDAR POINT CLOUD */

        // associate Lidar points with camera-based ROI
        t = (double)cv::getTickCount();
        float shrinkFactor = 0.10; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
        clusterLidarWithROI(dataBuffer.back().boundingBoxes, dataBuffer.back().lidarPoints, shrinkFactor, P_rect_00, R_rect_00, RT);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();                        
        // Visualize 3D objects        
        bVis = false;
        if(bVis)
        {
            bool bWait = false;
            show3DObjects(dataBuffer.back().boundingBoxes, cv::Size(4.0, 20.0), cv::Size(1000, 1000), bWait);
        }
        bVis = false;

        cout << "#4 : CLUSTER LIDAR POINT CLOUD done  in " << t << "[s]" << endl;
        
        
        /* DETECT IMAGE KEYPOINTS */

        // convert current image to grayscale
        cv::Mat imgGray;
        cv::cvtColor(dataBuffer.back().cameraImg, imgGray, cv::COLOR_BGR2GRAY);

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        bVis = false;
        detector.detectKp(keypoints, imgGray, bVis);
        bVis = false;

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        dataBuffer.back().keypoints = keypoints;

        cout << "#5 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        cv::Mat descriptors;
        descriptor.compute(dataBuffer.back().keypoints, dataBuffer.back().cameraImg, descriptors);
        //descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);

        // push descriptors for current frame to end of data buffer
        dataBuffer.back().descriptors = descriptors;

        cout << "#6 : EXTRACT DESCRIPTORS done " << dataBuffer.size() << endl;


        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {
            auto prevData = dataBuffer.rbegin();
            std::advance(prevData, 1);

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType    = "MAT_FLANN";        // MAT_BF, MAT_FLANN            
            string selectorType   = "SEL_KNN";       // SEL_NN, SEL_KNN
            
            string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
            if(kpDescriptorType.compare("SIFT")==0){
                descriptorType = "DES_HOG";
            }
            matchDescriptors((prevData)->keypoints, dataBuffer.back().keypoints,
                             (prevData)->descriptors, dataBuffer.back().descriptors,
                             matches, descriptorType, matcherType, selectorType);

            // store matches in current data frame
            dataBuffer.back().kptMatches = matches;

            cout << "#7 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            
            /* TRACK 3D OBJECT BOUNDING BOXES */

            //// STUDENT ASSIGNMENT
            //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between current and previous frame (implement ->matchBoundingBoxes)
            map<int, int> bbBestMatches;
            matchBoundingBoxes(matches, bbBestMatches, *(prevData), dataBuffer.back()); // associate bounding boxes between current and previous frame using keypoint matches
            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            dataBuffer.back().bbMatches = bbBestMatches;

            cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done" << endl;


            /* COMPUTE TTC ON OBJECT IN FRONT */

            // loop over all BB match pairs
            for (auto it1 = dataBuffer.back().bbMatches.begin(); it1 != dataBuffer.back().bbMatches.end(); ++it1)
            {
                // find bounding boxes associates with current match
                BoundingBox *prevBB, *currBB;
                for (auto it2 = dataBuffer.back().boundingBoxes.begin(); it2 != dataBuffer.back().boundingBoxes.end(); ++it2)
                {
                    if (it1->second == it2->boxID) // check wether current match partner corresponds to this BB
                    {
                        currBB = &(*it2);
                    }
                }

                for (auto it2 = (prevData)->boundingBoxes.begin(); it2 != (prevData)->boundingBoxes.end(); ++it2)
                {
                    if (it1->first == it2->boxID) // check wether current match partner corresponds to this BB
                    {
                        prevBB = &(*it2);
                    }
                }

                // compute TTC for current match
                if( currBB->lidarPoints.size()>0 && prevBB->lidarPoints.size()>0 ) // only compute TTC if we have Lidar points
                {                    
                    //// STUDENT ASSIGNMENT
                    //// TASK FP.2 -> compute time-to-collision based on Lidar data (implement -> computeTTCLidar)
                    double ttcLidar; 
                    computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar);
                    //// EOF STUDENT ASSIGNMENT

                    //// STUDENT ASSIGNMENT
                    //// TASK FP.3 -> assign enclosed keypoint matches to bounding box (implement -> clusterKptMatchesWithROI)
                    //// TASK FP.4 -> compute time-to-collision based on camera (implement -> computeTTCCamera)
                    double ttcCamera;                    
                    clusterKptMatchesWithROI(*prevBB, *currBB, (prevData)->keypoints, dataBuffer.back().keypoints, dataBuffer.back().kptMatches);                                        

                    bVis = false;
                    if (bVis)  // VISUALIZE SELECTED MATCH
                    {
                        cv::Mat matchImg = dataBuffer.back().cameraImg.clone();
                        cv::Mat prevImg = (prevData)->cameraImg.clone();
                        cv::Mat currImg = dataBuffer.back().cameraImg.clone();
                        cv::rectangle(prevImg, cv::Point(prevBB->roi.x, prevBB->roi.y), cv::Point(prevBB->roi.x + prevBB->roi.width, prevBB->roi.y + prevBB->roi.height), cv::Scalar(0, 255, 0), 2);
                        cv::rectangle(currImg, cv::Point(currBB->roi.x, currBB->roi.y), cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height), cv::Scalar(0, 255, 0), 2);
                        cv::drawMatches(prevImg, (prevData)->keypoints, currImg, dataBuffer.back().keypoints, currBB->kptMatches,
                                        matchImg, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                        string windowName = "Matching keypoints between two camera images (best 50)";
                        cv::namedWindow(windowName, 7);
                        cv::imshow(windowName, matchImg);
                        cv::waitKey(0);
                    }

                    computeTTCCamera((prevData)->keypoints, dataBuffer.back().keypoints, currBB->kptMatches, sensorFrameRate, ttcCamera);
                    //// EOF STUDENT ASSIGNMENT

                    fs << ttcCamera << ", " << ttcLidar << std::endl;
                    bVis = false;
                    if (bVis)
                    {
                        cv::Mat visImg = dataBuffer.back().cameraImg.clone();
                        showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
                        cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y), cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height), cv::Scalar(0, 255, 0), 2);
                        
                        char str[200];
                        sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttcLidar, ttcCamera);
                        putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255), 3);

                        string windowName = "Final Results : TTC";
                        cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
                        cv::imshow(windowName, visImg);
                        cout << "Press key to continue to next frame" << endl;
                        cv::waitKey(0);
                    }
                    bVis = false;

                } // eof TTC computation
            } // eof loop over all BB matches 
        }
    } // eof loop over all images
    fs.close();
#ifdef LOOP 
        }
    }
#endif 
    return 0;
}
