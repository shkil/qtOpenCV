//
//  cvarOpenCVDetector.h
//  cvar
//
//  Created by Oleksandr Shkil on 23.11.12.
//  Copyright (c) 2012 Codeminders. All rights reserved.
//

#ifndef CVAROPENCVDETECTOR
#define CVAROPENCVDETECTOR

#ifdef __cplusplus

#include <vector>
#include <opencv2/opencv.hpp>
#include "cvarDetectInfo.h"
#include "cvarOpenCVFrameHolder.h"

class cvarOpenCVDetector {
public:
    void detectPlace(const cvarOpenCVFrameHolder& frame, std::vector<cv::Point2f>& keypoints, cv::Mat& desriptors);
    bool matchPlace(const cvarOpenCVFrameHolder& frame, const stDetectInfo& info, int& x, int& y);

private:
    std::vector<cv::KeyPoint> tmp_keypoints;
    std::vector<cv::Point2f> keypoints;
    cv::Mat descriptors;

    std::vector< std::vector<cv::DMatch> > matches1;
    std::vector< std::vector<cv::DMatch> > matches2;

    double getElapsed(clock_t beg);
    
    // Some tests
    void ratioTest(std::vector<std::vector<cv::DMatch> >& matches, float ratio);
    void symmetryTest(const std::vector<std::vector<cv::DMatch> >& matches1,
                      const std::vector<std::vector<cv::DMatch> >& matches2,
                      std::vector<cv::DMatch>& symMatches);
    void matchesToPoints(std::vector<cv::Point2f>& points1,
                         std::vector<cv::Point2f>& points2,
                         const std::vector<cv::DMatch>& matches,
                         const std::vector<cv::Point2f>& pointsExternal,
                         const std::vector<cv::Point2f>& pointsInternal);
    void ransacTest(std::vector<cv::Point2f>& points1,
                    std::vector<cv::Point2f>& points2,
                    std::vector<cv::DMatch>& matches,
                    std::vector<cv::DMatch>& ransacMatches,
                    double distance,
                    double confidence);
    void transformPointHomography(std::vector<cv::Point2f>& external,
                                  std::vector<cv::Point2f>& internal,
                                  const cv::Point2f& pointExternal,
                                  cv::Point2f& pointInternal,
                                  double ransacThreshold);
};

#endif

#endif
