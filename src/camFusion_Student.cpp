
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
    // ...
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // Find the closest point along x axis for both set of lidar points

    double minPrev1 = 1e9;
    double minPrev2 = 1e9;
    double minCurr1 = 1e9;
    double minCurr2 = 1e9;

    for(const auto &point: lidarPointsPrev)
    {
        if(point.x < minPrev1)
        {
            minPrev1 = point.x;
            std::swap(minPrev2, minPrev1);
        }
    }

    for(const auto &point: lidarPointsCurr)
    {
        if(point.x < minCurr1)
        {
            minCurr1 = point.x;
            std::swap(minCurr2, minCurr1);
        }
    }
    

    // Find distance between 1st and 2nd closest point
    double thresh = 0.02; // 5cm
    double s0, s1, dt;

    s0 = abs(minPrev1 - minPrev2) < thresh ? minPrev1 : minPrev2;
    s1 = abs(minCurr1 - minCurr2) < thresh ? minCurr1 : minCurr2;
    dt = 1 / frameRate; 

    TTC =  (s1 * dt) / abs(s0 - s1);  

}


std::pair<size_t, int> getBoundingBoxCount(cv::KeyPoint &keypoint, std::vector<BoundingBox> &boxes)
{
    size_t count = 0;
    int id;
    for(BoundingBox &box: boxes)
    {
        if(box.roi.contains(keypoint.pt))
        {
            count++;
            id = box.boxID;
        }
        
    }
    return std::make_pair(count, id);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
   // Remove matches with keypoints that are in more than one bounding box

   int cols = prevFrame.boundingBoxes.size();
   int rows = currFrame.boundingBoxes.size();

   cv::Mat  pairCount = cv::Mat::zeros(rows, cols, CV_32S);

   for (auto it = matches.begin(); it != matches.end(); it++)
   {
        cv::KeyPoint prevKpt = prevFrame.keypoints[(*it).queryIdx];
        cv::KeyPoint currKpt = currFrame.keypoints[(*it).trainIdx];

        for (auto boxPrev: prevFrame.boundingBoxes)
        {
            if (boxPrev.roi.contains(prevKpt.pt))
            {
                for(auto boxCurr: currFrame.boundingBoxes)
                {
                    if(boxCurr.roi.contains(currKpt.pt))
                    {
                        // std::cout << "Found Pair " << boxPrev.boxID << ", " << boxCurr.boxID << std::endl;
                        pairCount.at<int>(boxCurr.boxID, boxPrev.boxID)++;
                    }
                }
            }
        }
   }
   // For each row (bounding box in previous frame), find the column (bounding box in current frame)
   // that has highest count
   cv::Point minLoc, maxLoc;
   double minVal, maxVal;
   std::map<int, int> tempPairs;
   for(BoundingBox &box: currFrame.boundingBoxes)
   {
        cv::minMaxLoc(pairCount.row(box.boxID), &minVal, &maxVal, &minLoc, &maxLoc);
        std::cout << " BB PAIR: " << maxLoc.x << ", " << box.boxID << std::endl;
        tempPairs[maxLoc.x]=box.boxID;
   }
 
  // Filter based on highest number of pairs
  int maxCount = 0;
  std::pair<int, int> maxPair;
  for(BoundingBox &box: currFrame.boundingBoxes)
  {
     for(auto &bbPair: tempPairs)
     {
        if(bbPair.second == box.boxID)
        {
            auto c = pairCount.at<double>(box.boxID, bbPair.first);
            if( c > maxCount)
            {
                maxCount = c;
                maxPair = std::make_pair(bbPair.first, box.boxID);
            }
        }
     }
     bbBestMatches[maxPair.first] = maxPair.second;
  }

}
