//
//  cvarFrameHolder.h
//  cvar
//
//  Created by Oleksandr Shkil on 23.11.12.
//  Copyright (c) 2012 Codeminders. All rights reserved.
//

#ifndef CVAROPENCVFRAMEHOLDER
#define CVAROPENCVFRAMEHOLDER


#ifdef __cplusplus
#include <opencv2/opencv.hpp>

class cvarOpenCVFrameHolder {
    cv::Mat frame1Channel;
    
public:
    cvarOpenCVFrameHolder();
    void update(unsigned char* img, int width, int height);
    const cv::Mat& getFrame() const;
};

#endif

#endif
