
#include <iostream>
#include <algorithm>
#include <numeric>
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
    vector<cv::DMatch> boundingBoxMatches;
    vector<double> eucDistances;

    // Looping over the matches in the current frame
    for (cv::DMatch match_i : kptMatches) 
    {
        // Check if ROI contains the keypoints
        if (boundingBox.roi.contains(kptsCurr[match_i.trainIdx].pt))
        {
            boundingBoxMatches.push_back(match_i);
            eucDistances.push_back(cv::norm(kptsCurr[match_i.trainIdx].pt - kptsPrev[match_i.queryIdx].pt));
        }
    }

    // compute the threshhold of matches Eucliadian Distance
    double distTh = 1.5 * std::accumulate(eucDistances.begin(), eucDistances.end(), 0.0) / eucDistances.size();

    // associate best matches to given bounding box
    auto it1 = boundingBoxMatches.begin();
    for (auto it2 = eucDistances.begin(); it2 != eucDistances.end(); it2++, it1++)
    {
        if((*it2) <  distTh)
        {
            boundingBox.kptMatches.push_back((*it1));
            boundingBox.keypoints.push_back(kptsCurr[(*it1).trainIdx]);
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    vector<double> distanceRatios;
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; it1++) 
    {
        cv::KeyPoint kptOutCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kptOutPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2) 
        {
            cv::KeyPoint kptInCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kptInPrev = kptsPrev.at(it2->queryIdx);

            // Calculate the current and previous Euclidean distances between each keypoint in the paircompute the current and previous Euclidean distances between the pair's keypoints
            double distCurr = cv::norm(kptOutCurr.pt - kptInCurr.pt);
            double distPrev = cv::norm(kptOutPrev.pt - kptInPrev.pt);

            double minDist = 100.0;  // Set a minimum current distance between keypoints to limit the calculated distanceRatios.

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist) 
            {
                double distRatio = distCurr / distPrev;
                distanceRatios.push_back(distRatio);
            }
        }
    }

    // Continue only if the vector of distanceRatios isn't empty
    if (distanceRatios.size() == 0)
    {
        TTC = std::numeric_limits<double>::quiet_NaN();
        return;
    }

    std::sort(distanceRatios.begin(), distanceRatios.end());
    double medianDistRatio = distanceRatios[distanceRatios.size() / 2];

    // Based on these 2D camera features, compute a TTC estimate
    TTC = (-1.0 / frameRate) / (1 - medianDistRatio);
}

// Helper function to sort lidar points based on their X (longitudinal) coordinate
void sortPtsDistance(std::vector<LidarPoint> &points)
{
    std::sort(points.begin(), points.end(), [](LidarPoint a, LidarPoint b){return a.x < b.x;});
}
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // Take the median x-distance as our more robust estimate in each frame.
    // Consider taking the median of a random subset of the points if performance is suffering.
    sortPtsDistance(lidarPointsPrev);
    sortPtsDistance(lidarPointsCurr);
    double d1 = lidarPointsPrev[lidarPointsPrev.size()/2].x;
    double d2 = lidarPointsCurr[lidarPointsCurr.size()/2].x;

    // The constant-velocity model is used (as opposed to a constant-acceleration model)
    TTC = d2 * (1.0 / frameRate) / (d1 - d2);
    if (TTC <= 0) // if the ego car is decelerating
        TTC = NAN;
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    std::multimap<int, int> mmap{};
    int maxPrevBoxID = 0;

    for (auto match_i : matches) 
    {
        cv::KeyPoint prevKp = prevFrame.keypoints[match_i.queryIdx];
        cv::KeyPoint currKp = currFrame.keypoints[match_i.trainIdx];
        
        int currBoxID = -1;
        int prevBoxID = -1;

        for (auto bbox : prevFrame.boundingBoxes) 
        {
            if (bbox.roi.contains(prevKp.pt)) 
                prevBoxID = bbox.boxID;
        }

        for (auto bbox : currFrame.boundingBoxes) 
        {
            if (bbox.roi.contains(currKp.pt)) 
                currBoxID = bbox.boxID;
        }
        
        mmap.insert({currBoxID, prevBoxID});
        maxPrevBoxID = std::max(maxPrevBoxID, prevBoxID);
    }

    vector<int> currFrameBoxIDs {};
    for (auto box : currFrame.boundingBoxes) 
        currFrameBoxIDs.push_back(box.boxID);

    for (int k : currFrameBoxIDs) 
    {
        auto rangePrevBoxIDs = mmap.equal_range(k);
        std::vector<int> counts(maxPrevBoxID + 1, 0);

        for (auto it = rangePrevBoxIDs.first; it != rangePrevBoxIDs.second; ++it) 
        {
            if (-1 != (*it).second) counts[(*it).second] += 1;
        }

        int modeIndex = std::distance(counts.begin(), std::max_element(counts.begin(), counts.end()));

        bbBestMatches.insert({modeIndex, k});
    }
}
