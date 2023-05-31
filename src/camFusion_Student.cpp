
#include <iostream>
#include <algorithm>
#include <numeric>
#include <utility>
#include <unordered_map>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

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
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

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

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
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
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    
    for (cv::DMatch &match: kptMatches)
    {
        if(boundingBox.roi.contains(kptsCurr[match.trainIdx].pt)) boundingBox.kptMatches.push_back(match);
    }

    // Compute mean distance
    std::vector<double> kptDistances(boundingBox.kptMatches.size());
    auto compute_kptDist = [&](auto& match){return cv::norm(kptsPrev[match.queryIdx].pt - kptsCurr[match.trainIdx].pt);};
    std::transform(boundingBox.kptMatches.begin(), boundingBox.kptMatches.end(), kptDistances.begin(), compute_kptDist);
    double kptDistMean = std::accumulate(kptDistances.begin(), kptDistances.end(), 0.0) / boundingBox.kptMatches.size();
    // Compute standard deviation
    std::vector<double> deviations(boundingBox.kptMatches.size());
    auto squared_error = [&](auto& dist){return std::pow(dist - kptDistMean, 2);};
    std::transform(kptDistances.begin(), kptDistances.end(), deviations.begin(), squared_error);
    double KptDistSD = std::sqrt(std::accumulate(deviations.begin(), deviations.end(), 0.0) / deviations.size());

    // A match is outlier if the distance is > 1.8 times the standatd deviation
    auto is_outlier = [&](auto& dist){return dist > (1.8 * KptDistSD);};
    boundingBox.kptMatches.erase(std::remove_if(kptDistances.begin(), kptDistances.end(), is_outlier), boundingBox.kptMatches.end());

    std::vector<cv::DMatch> inliers;
    double lowerLimit = kptDistMean - (1.8 * KptDistSD);
    double upperLimit = kptDistMean + (1.8 * KptDistSD);
    for(size_t i = 0; i < kptDistances.size(); i++)
    {
        if(kptDistances[i] >= lowerLimit && kptDistances[i] <= upperLimit)
        {
            inliers.push_back(boundingBox.kptMatches[i]);
        }
    }
    boundingBox.kptMatches = inliers;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    double minDistanceThreshold = 100.0;
    std::vector<double> distanceRatios;
    for(cv::DMatch &match: kptMatches)
    {
            // Get a keypoint in previous frame and its match in current frame.
            cv::KeyPoint prevKpt = kptsPrev[match.queryIdx];
            cv::KeyPoint currKpt = kptsCurr[match.trainIdx];

            // Compute the distance of this keypoint (both in previous and current frame) to all other keypoints

            for(cv::DMatch &other: kptMatches)
            {
                bool isSame = other.queryIdx == match.queryIdx && other.trainIdx == match.trainIdx;
                if(!isSame)
                {
                    cv::KeyPoint prevKptOther = kptsPrev[other.queryIdx];
                    cv::KeyPoint currKptOther = kptsCurr[other.trainIdx];
                    double distPrev = cv::norm(prevKpt.pt - prevKptOther.pt);
                    double distCurr = cv::norm(currKpt.pt - currKptOther.pt);
                    // Make sure we dont divide by zero
                    bool divisorNotZero = distPrev > std::numeric_limits<double>::epsilon();
                    if( divisorNotZero && distCurr >= minDistanceThreshold) distanceRatios.push_back(distCurr / distPrev);

                }
            }
    }

    // Now we have distance ratios for all keypoint match pairs
    // Compute the median 
    if (distanceRatios.size() > 0)
    {
        std::sort(distanceRatios.begin(), distanceRatios.end());
        int medianIdx = floor(distanceRatios.size() / 2.0);

        double medianDistanceRatio = medianIdx % 2 == 0 ? (distanceRatios[medianIdx] + distanceRatios[medianIdx - 1]) / 2.0 : distanceRatios[medianIdx];

        double dT = (1.0 / frameRate);
        TTC = -dT / (1 - medianDistanceRatio);

    }
    else
        TTC = std::numeric_limits<double>::quiet_NaN();
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // Get all x coordinates for current and previous frame
    std::vector<double> xPrev(lidarPointsPrev.size()), xCurr(lidarPointsCurr.size());

    std::transform(lidarPointsPrev.begin(), lidarPointsPrev.end(), xPrev.begin(), [](auto& point){return point.x;}); 
    std::transform(lidarPointsCurr.begin(), lidarPointsCurr.end(), xCurr.begin(), [](auto& point){return point.x;});
    std::cout<<"Transform done "<<std::endl;
    double xPrevMean = std::accumulate(xPrev.begin(), xPrev.end(), 0.0) / xPrev.size();
    double xCurrMean = std::accumulate(xCurr.begin(), xCurr.end(), 0.0) / xCurr.size();
    std::cout<< " Mean calculated "<<std::endl;
    double dT = 1.0 / frameRate;
    TTC =  (xCurrMean * dT) / (xPrevMean - xCurrMean);
    std::cout << "TTC computed "<<std::endl;

}




void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{

    std::vector<cv::DMatch> uniqueMatches;
    // Remove matches with keypoints that are in more than one bounding box
    int prevBoxCount, currBoxCount;
    for(cv::DMatch &match: matches)
    {
            prevBoxCount = 0;
            currBoxCount = 0;

            for(BoundingBox &box: currFrame.boundingBoxes) if(box.roi.contains(currFrame.keypoints[match.trainIdx].pt)) currBoxCount++;
            for(BoundingBox &box: prevFrame.boundingBoxes) if(box.roi.contains(prevFrame.keypoints[match.queryIdx].pt)) prevBoxCount++;

            if(prevBoxCount == 1 && currBoxCount == 1) uniqueMatches.push_back(match);

    }

    // replace the original with unqiue only
    matches = uniqueMatches;

    // For each bounding box in current frame, go through the matches
    std::map<int, int> prevBoxCounter;
    for(BoundingBox &boxCurr: currFrame.boundingBoxes)
    {
        prevBoxCounter.clear();
        for(cv::DMatch &match: matches)
        {
            for(BoundingBox &boxPrev: prevFrame.boundingBoxes)
            {
                bool isCurr = boxCurr.roi.contains(currFrame.keypoints[match.trainIdx].pt);
                bool isPrev = boxPrev.roi.contains(prevFrame.keypoints[match.queryIdx].pt);
                
                if(isCurr && isPrev) prevBoxCounter[boxPrev.boxID]++;
            }
        }

        // Find the box from previous frame with maximum matches
        auto less_than = [](const auto& a, const auto& b){return a.second < b.second;};
        auto maxPrev = *std::max_element(prevBoxCounter.begin(), prevBoxCounter.end(), less_than);
        bbBestMatches[maxPrev.first] = boxCurr.boxID; 
    }

}
