
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

#include <list>
using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));    
    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {        
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point            
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);            
        }        
        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &prev_boundingBox, BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
    boundingBox.kptMatches.clear();
    for(const auto match : kptMatches){    
        const auto kp = kptsCurr[match.trainIdx];
        const auto kp_prev = kptsPrev[match.queryIdx];
        if(boundingBox.roi.contains(kp.pt) && prev_boundingBox.roi.contains(kp_prev.pt)){
            
            boundingBox.keypoints.push_back(kp);

            //Compute the delta
            //const auto kp_prev = kptsPrev[match.trainIdx];
            //const auto delta_x = std::abs(kp_prev.pt.x - kp.pt.x);
            //const auto delta_y = std::abs(kp_prev.pt.y - kp.pt.y);
            //if(delta_x < 50 && delta_y < 15){                
            boundingBox.kptMatches.push_back(match);
            //}            
        }        
    }
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    //list<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios    
    std::sort(distRatios.begin(), distRatios.end());
    
    /*distRatios.sort();
    const auto n0 = distRatios.size();
    const double ratio = 0.3;
    const size_t n_max = std::ceil(n0*ratio);
    while(distRatios.size() > n_max){
        auto it = distRatios.begin();
        auto it_r = distRatios.rbegin();
        size_t r_step = 0;
        size_t step = 0;
        const double min = distRatios.front();
        const double max = distRatios.back();
        const double delta = (max - min)/4.;

        do{
            it_r++;
            r_step++;
        }while((*it_r) > (max - delta));

        do{
            it++;
            step++;
        }while((*it) < (min + delta));

        if(step > r_step){
            distRatios.erase(it_r.base(), distRatios.end());
        }else{
            distRatios.erase(distRatios.begin(), it);
        }
    }

    const double scale = 1/static_cast<double>(distRatios.size());
    double meanDistRatio = 0;
    for(const auto ratio : distRatios){
        meanDistRatio += scale*ratio;
    }*/
    
    const long medIndex = floor(distRatios.size()/2.0);
    const double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex];

    double dT = 1 / frameRate;
    //TTC = -dT / (1 - meanDistRatio);
    TTC = -dT / (1 - medDistRatio);
}


bool isOnLane(const LidarPoint& pt, const double laneWidth){
	return 2*std::abs(pt.y) < laneWidth;
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // auxiliary variables    
    double laneWidth = 4.0; // assumed width of the ego lane

    /*
    // find closest distance to Lidar points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
      	if(isOnLane(*it, laneWidth))
          minXPrev = minXPrev > it->x ? it->x : minXPrev;
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {	
      	if(isOnLane(*it, laneWidth))
        	minXCurr = minXCurr > it->x ? it->x : minXCurr;
    }    

    // compute TTC from both measurements
    TTC = minXCurr / (frameRate*(minXPrev - minXCurr));
    */

    vector<double> distsPrev, distsCurr;

    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
      	if(isOnLane(*it, laneWidth))
          distsPrev.push_back(it->x);
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
      	if(isOnLane(*it, laneWidth))
          distsCurr.push_back(it->x);
    }

    std::cout << distsPrev.size() << std::endl;
    std::cout << distsCurr.size() << std::endl;
    std::sort(distsPrev.begin(), distsPrev.end());
    std::sort(distsCurr.begin(), distsCurr.end());

    const long medIndexPrev = floor(distsPrev.size()/2.0);
    const double medPrev = distsPrev.size() % 2 == 0 ? (distsPrev[medIndexPrev - 1] + distsPrev[medIndexPrev]) / 2.0 : distsPrev[medIndexPrev];

    const long medIndexCurr = floor(distsCurr.size()/2.0);
    const double medCurr = distsCurr.size() % 2 == 0 ? (distsCurr[medIndexCurr - 1] + distsCurr[medIndexCurr]) / 2.0 : distsCurr[medIndexCurr];    

    // compute TTC from both measurements
    TTC = medCurr / (frameRate*(medPrev - medCurr));
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // ...
    const auto n_prev = prevFrame.boundingBoxes.size();
    const auto n_curr = currFrame.boundingBoxes.size();   
    const double min_conf = 0.2;
    const double min_overlap = 0.6;

    for(auto i = 0; i < n_curr; i++){

        const auto bb_curr = currFrame.boundingBoxes[i];
        const double area_curr = bb_curr.roi.height * bb_curr.roi.width;

        size_t best_j = 0;
        double best_overlap = min_overlap;
        for(auto j = 0; j < n_prev; j++){            
            const auto bb_prev = prevFrame.boundingBoxes[j];

            if(bb_prev.classID != bb_curr.classID || bb_prev.confidence < min_conf || bb_curr.confidence < min_conf) continue;

            const double area_prev = bb_prev.roi.height * bb_prev.roi.width;            
            if(area_prev * 0.5 < area_curr && area_curr < 1.5*area_prev){
                const int min_x = std::max(bb_prev.roi.x, bb_curr.roi.x);
                const int min_y = std::max(bb_prev.roi.y, bb_curr.roi.y);

                const int max_x = std::min(bb_prev.roi.x + bb_prev.roi.width, bb_curr.roi.x + bb_curr.roi.width);
                const int max_y = std::min(bb_prev.roi.y + bb_prev.roi.height, bb_curr.roi.y + bb_curr.roi.height);

                const auto delta_x = max_x - min_x;
                const auto delta_y = max_y - min_y;
                const double overlap = delta_x*delta_y/area_curr;
                if(delta_x > 0 && delta_y > 0 && overlap > best_overlap){
                    best_j = j;
                    best_overlap = overlap;
                }
            }
        }

        if(best_overlap > (min_overlap + 1e-4)){
            bbBestMatches.insert({best_j, i});

            bool bVis = false;
            if(false){
                const auto bb_prev = prevFrame.boundingBoxes[best_j];
                int top, left, width, height;
                top    = bb_prev.roi.y;
                left   = bb_prev.roi.x;
                width  = bb_prev.roi.width;
                height = bb_prev.roi.height;      
                cv::Mat prev = prevFrame.cameraImg.clone();
                cv::rectangle(prev, cv::Point(left, top), cv::Point(left+width, top+height),cv::Scalar(0, 255, 0), 2);

                cv::Mat curr = currFrame.cameraImg.clone();
                cv::rectangle(curr, cv::Point(left, top), cv::Point(left+width, top+height),cv::Scalar(0, 255, 0), 2);
                top    = bb_curr.roi.y;
                left   = bb_curr.roi.x;
                width  = bb_curr.roi.width;
                height = bb_curr.roi.height;                        
                cv::rectangle(curr, cv::Point(left, top), cv::Point(left+width, top+height),cv::Scalar(255, 0, 0), 2);
                

                
                string windowName = "Prev";
                cv::namedWindow(windowName, 1);
                cv::imshow(windowName, prev);

                windowName = "Curr";
                cv::namedWindow(windowName, 1);
                cv::imshow(windowName, curr);
                cv::waitKey(0);
            }
        }
    }
    /*
    const size_t min_pts = 1;
    const auto n_prev = prevFrame.boundingBoxes.size();
    const auto n_curr = currFrame.boundingBoxes.size();
    std::vector<std::vector<int>> score(n_curr,std::vector<int>(n_prev,0));
    
    for(const auto match : matches){
        const auto img1_idx =  match.queryIdx;    
        const auto img2_idx =  match.trainIdx;
        const auto img1_kp  =  prevFrame.keypoints[match.queryIdx].pt;
        const auto img2_kp  =  currFrame.keypoints[match.trainIdx].pt;

        std::list<int> img1_bb, img2_bb;
        for(size_t idx = 0; idx < n_prev; idx++){
            const auto bb = prevFrame.boundingBoxes[idx];
            if(bb.roi.x < img1_kp.x && img1_kp.x < (bb.roi.x + bb.roi.width) && 
                bb.roi.y < img1_kp.y && img1_kp.y < (bb.roi.y + bb.roi.height)){
                img1_bb.push_back(idx);
            }
        }
        if(img1_bb.size() < 1) continue;

        for(size_t idx = 0; idx < n_curr; idx++){
            const auto bb = currFrame.boundingBoxes[idx];
            if(bb.roi.x < img2_kp.x && img2_kp.x < (bb.roi.x + bb.roi.width) && 
                bb.roi.y < img2_kp.y && img2_kp.y < (bb.roi.y + bb.roi.height)){
                img2_bb.push_back(idx);
            }
        }
        if(img2_bb.size() < 1) continue;

        for(const auto idx1 : img1_bb){
            for(const auto idx2 : img2_bb){
                score[idx2][idx1]++;
            }
        }
    }

    // Check best score
    for(size_t i = 0; i < n_curr; i++){
        size_t min_score = std::numeric_limits<size_t>::max();
        size_t max_score = 0, max_idx = 0;

        for(size_t j = 0; j < n_prev; j++){
            if(min_score > score[i][j]){
                min_score = score[i][j];
            }
            if(max_score < score[i][j]){
                max_score = score[i][j];
                max_idx = i;
            }
        }

        // TODO: check that score[x][j] is always less thatn score[i][j]
        if(max_score > min_pts){
            const auto res = bbBestMatches.insert({max_idx, i});
            // TODO res.second == false check who has the best score!
        }
    }
    */


}
