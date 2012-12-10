//
//  cvarOpenCVDetector.m
//  cvar
//
//  Created by Oleksandr Shkil on 23.11.12.
//  Copyright (c) 2012 Codeminders. All rights reserved.
//

#ifdef __cplusplus

#include "cvarOpenCVDetector.h"
#include <opencv2/opencv.hpp>

#include <ctime>
#include <iostream>
#include <numeric>

#define SCALEF 0.3f
#define FEATURES_CNT 200
#define SIZE 100

cvarOpenCVDetector::cvarOpenCVDetector():winSize(10, 10), termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03)
{
}

// Optical Flow test
void cvarOpenCVDetector::detectPlace(const cvarOpenCVFrameHolder& frame, std::vector<cv::Point2f>& aKeypoints, cv::Mat& aDesriptors, int aX, int aY)
{
    cv::Mat mask(frame.getFrame().rows, frame.getFrame().cols, CV_8UC1);
    cv::rectangle(mask, cv::Rect(aX-SIZE/2, aY-SIZE/2,  SIZE, SIZE), cv::Scalar(1.0), -1);

    //memset(frame.getFrame().data, frame.getFrame().rows*frame.getFrame().cols, 0);
    cv::goodFeaturesToTrack(frame.getFrame(), aKeypoints, FEATURES_CNT, 0.01, 10, mask);

    //cv::cornerSubPix(frame.getFrame(), aKeypoints, winSize, cv::Size(-1, -1), termcrit);
    image_prev = frame.getFrame();
    features_prev = aKeypoints;
    prev_point.x = aX;
    prev_point.y = aY;
}

bool cvarOpenCVDetector::matchPlace(const cvarOpenCVFrameHolder& frame, const stDetectInfo& info, int& x, int& y)
{
    std::vector<uchar> status;
    std::vector<float> err;
    int matches;

    if(image_prev.data == NULL )
        return false;

    clock_t beg = clock();
    cv::calcOpticalFlowPyrLK(image_prev, frame.getFrame(), features_prev, features_next, status, err, winSize, 3, termcrit, 0);
    matches = std::accumulate(status.begin(), status.end(), 0);

    std::vector<cv::Point2f> src, dst;

    for( size_t i =0; i<features_prev.size(); i++)
    {
        if(status[i])
        {
            src.push_back(features_prev[i]);
            dst.push_back(features_next[i]);
        }
    }

    /*if( src.size() < FEATURES_CNT/4 )
    {
        return false;
    }*/

    std::cout << "Best inliers" << src.size() << std::endl;
    if( src.size() <= /*FEATURES_CNT/2*/10 )
    {
        std::cout << "No Object\n";
        cv::goodFeaturesToTrack(frame.getFrame(), features_prev, FEATURES_CNT, 0.01, 10);
        //cv::cornerSubPix(frame.getFrame(), features_prev, winSize, cv::Size(-1, -1), termcrit);
    }
    else
    {
        features_prev = dst;
    }

    std::vector<uchar> mask;
    std::vector<cv::Point2f> res;
    cv::Mat H = cv::findHomography(src, dst, CV_RANSAC, 3, mask);
    matches = std::accumulate(mask.begin(), mask.end(), 0);
    std::cout << "Best inliers" << matches << std::endl;
    if( matches < 10 )
    {
        std::cout << "No Object\n";
        return false;
    }
    std::vector<cv::Point2f> dot;
    dot.push_back(prev_point);
    cv::perspectiveTransform(dot, res, H);

    x = res[0].x;
    y = res[0].y;

    prev_point = res[0];
    image_prev = frame.getFrame();

    std::cout << "RESULT X=" << x << "Y=" << y << " TIME=" << std::fixed << getElapsed(beg) << std::endl;
    return true;
}

//void cvarOpenCVDetector::detectPlace(const cvarOpenCVFrameHolder& frame, std::vector<cv::Point2f>& aKeypoints, cv::Mat& aDesriptors)
//{
//    // Feature detecting
//#if CV_MAJOR_VERSION==2 && CV_MINOR_VERSION==3
//    cv::ORB::CommonParams orbParams(1.2f, 8, 31, 0);
//    cv::OrbFeatureDetector orb(200, orbParams);
//#else
//    cv::OrbFeatureDetector orb(200, 1.2f, 8.0, 31,
//                               0, 2, cv::ORB::HARRIS_SCORE, 31);
//#endif
//    // Feature Extracting
//    orb.detect(frame.getFrame(), tmp_keypoints);
//    cv::KeyPoint::convert(tmp_keypoints, aKeypoints);
//#if CV_MAJOR_VERSION==2 && CV_MINOR_VERSION==3
//    cv::OrbDescriptorExtractor orbExt(orbParams);
//    orbExt.compute(frame.getFrame(), tmp_keypoints, aDesriptors);
//#else
//    orb.compute(frame.getFrame(), tmp_keypoints, aDesriptors);
//#endif
//}


//bool cvarOpenCVDetector::matchPlace(const cvarOpenCVFrameHolder& frame, const stDetectInfo& info, int& x, int& y)
//{
//    float ratio = 0.82;
//    double distance = 1.0;
//    double confidence = 0.85;
//    double RansacThreshold = 4.0;
    
//    if(info.keypoints.size() == 0)
//        return false;

//    clock_t beg = clock();
//    detectPlace(frame, keypoints, descriptors);
//    std::cout << "keypoints Size " << keypoints.size() << std::endl;
//    //std::cout << "Descriptor extracting time: " << getElapsed(beg) << std::endl;
    
//    matches1.clear();
//    matches2.clear();

//#if CV_MAJOR_VERSION==2 && CV_MINOR_VERSION==3
//    cv::BruteForceMatcher<cv::Hamming> matcher;
//#else
//    cv::BFMatcher matcher(cv::NORM_HAMMING);
//#endif
//    matcher.knnMatch(descriptors, info.descriptors, matches1, 2);
//    matcher.knnMatch(info.descriptors, descriptors, matches2, 2);
//    //std::cout << "Matching time: " << getElapsed(beg) << " match1 size "<< matches1.size() << " match2 size" << matches2.size() << std::endl;
    
//    // ratio
//    ratioTest(matches1, ratio);
//    ratioTest(matches2, ratio);
    
//    // check the sizes
//    if(matches1.size()==0 || matches2.size()==0){
//        return false;
//    }
    
//    // symmetry
//    std::vector<cv::DMatch> symMatches2;
//    symmetryTest(matches1,matches2,symMatches2);
    
//    // matches to points ransac
//    std::vector<cv::Point2f> points1symAfterRatio, points2symAfterRatio;
//    matchesToPoints(points1symAfterRatio, points2symAfterRatio, symMatches2, info.keypoints, keypoints);
//    if (points1symAfterRatio.size() == 0)
//        return false;
//    std::vector<cv::DMatch> ransacMatchesWithRatio;
//    ransacTest(points1symAfterRatio, points2symAfterRatio, symMatches2, ransacMatchesWithRatio, distance, confidence);
    
//    // transform point with ratio filtering
//    std::vector<cv::Point2f> externalRatio, internalRatio;
//    matchesToPoints(externalRatio, internalRatio, ransacMatchesWithRatio, info.keypoints, keypoints);
//    if(externalRatio.size()==0)
//        return false;
//    cv::Point2f pPointExternal(info.x, info.y);
//    cv::Point2f pPointInternalCorrespondent;
//    transformPointHomography(externalRatio,internalRatio, pPointExternal, pPointInternalCorrespondent, RansacThreshold);
//    x = pPointInternalCorrespondent.x;
//    y = pPointInternalCorrespondent.y;
    
//    std::cout << "RESULT X=" << x << "Y=" << y << " TIME=" << getElapsed(beg) << std::endl;
    
//    return true;
//}

void cvarOpenCVDetector::ratioTest(std::vector<std::vector<cv::DMatch> >& matches, float ratio)
{
    int removed = 0;
    for (std::vector<std::vector<cv::DMatch> >::iterator matchIterator= matches.begin();
         matchIterator!= matches.end(); ++matchIterator) {
        // if 2 NN has been identified
        if (matchIterator->size() <= 1 || (*matchIterator)[0].distance/(*matchIterator)[1].distance > ratio) {
            matchIterator->clear(); // remove match
            removed++;
        }
    }
}

void cvarOpenCVDetector::symmetryTest(const std::vector<std::vector<cv::DMatch> >& matches1,
                  const std::vector<std::vector<cv::DMatch> >& matches2,
                  std::vector<cv::DMatch>& symMatches)
{
    for (std::vector<std::vector<cv::DMatch> >::const_iterator matchIterator1= matches1.begin();
         matchIterator1!= matches1.end(); ++matchIterator1) {
        if (matchIterator1->size() < 2) // ignore deleted matches
            continue;
        // for all matches image 2 -> image 1
        for (std::vector<std::vector<cv::DMatch> >::const_iterator matchIterator2= matches2.begin();
             matchIterator2!= matches2.end(); ++matchIterator2) {
            if (matchIterator2->size() < 2) // ignore deleted matches
                continue;
            // Match symmetry test
            if ((*matchIterator1)[0].queryIdx == (*matchIterator2)[0].trainIdx  &&
                (*matchIterator2)[0].queryIdx == (*matchIterator1)[0].trainIdx) {
                // add symmetrical match
                symMatches.push_back(cv::DMatch((*matchIterator1)[0].queryIdx,
                                                (*matchIterator1)[0].trainIdx,
                                                (*matchIterator1)[0].distance));
                break; // next match in image 1 -> image 2
            }
        }
    }
}

void cvarOpenCVDetector::matchesToPoints(std::vector<cv::Point2f>& points1,
                     std::vector<cv::Point2f>& points2,
                     const std::vector<cv::DMatch>& matches,
                     const std::vector<cv::Point2f>& pointsExternal,
                     const std::vector<cv::Point2f>& pointsInternal)
{
    for (std::vector<cv::DMatch>::const_iterator it= matches.begin(); it!= matches.end(); ++it) {
        points1.push_back( pointsExternal[it->trainIdx] );
        points2.push_back( pointsInternal[it->queryIdx] );
    }
}

void cvarOpenCVDetector::ransacTest(std::vector<cv::Point2f>& points1,
                std::vector<cv::Point2f>& points2,
                std::vector<cv::DMatch>& matches,
                std::vector<cv::DMatch>& ransacMatches,
                double distance,
                double confidence)
{
    // Compute F matrix using RANSAC
    std::vector<uchar> inliers(points1.size(),0);
    cv::Mat fundemental= cv::findFundamentalMat(
                                                cv::Mat(points1),cv::Mat(points2), // matching points
                                                inliers,      // match status (inlier ou outlier)
                                                cv::FM_RANSAC, // RANSAC method
                                                distance,     // distance to epipolar line
                                                confidence);  // confidence probability
    // extract the surviving (inliers) matches
    std::vector<uchar>::const_iterator itIn= inliers.begin();
    std::vector<cv::DMatch>::const_iterator itM= matches.begin();
    // for all matches
    for ( ;itIn!= inliers.end(); ++itIn, ++itM) {
        if (*itIn) { // it is a valid match
            ransacMatches.push_back(*itM);
        }
    }
}

void cvarOpenCVDetector::transformPointHomography(std::vector<cv::Point2f>& external,
                              std::vector<cv::Point2f>& internal,
                              const cv::Point2f& pointExternal,
                              cv::Point2f& pointInternal,
                              double ransacThreshold)
{
    cv::Mat homographyMat = cv::findHomography(external, internal, cv::FM_RANSAC, ransacThreshold);
    std::vector<cv::Point2f> pExternalPointVector;
    pExternalPointVector.push_back(pointExternal);
    std::vector<cv::Point2f> pInternalPointVector;
    cv::perspectiveTransform(pExternalPointVector,pInternalPointVector,homographyMat);
    pointInternal = pInternalPointVector[0];
}

double cvarOpenCVDetector::getElapsed(clock_t beg)
{
    double d = (double(clock() - beg) / (double)CLOCKS_PER_SEC);
    return (double(clock() - beg) / (double)CLOCKS_PER_SEC);
}

#endif
