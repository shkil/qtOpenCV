//
//  cvarFrameHolder.m
//  cvar
//
//  Created by Oleksandr Shkil on 23.11.12.
//  Copyright (c) 2012 Codeminders. All rights reserved.
//

#ifdef __cplusplus

#include "cvarOpenCVFrameHolder.h"
#ifdef CV_NEON
    #include "cvarImageUtils.h"
#endif

cvarOpenCVFrameHolder::cvarOpenCVFrameHolder()
{
}

void cvarOpenCVFrameHolder::update(unsigned char* img, int width, int height)
{
    if( (width != frame1Channel.rows) || (height != frame1Channel.cols) )
    {
        frame1Channel = cv::Mat(width, height, CV_8UC1);
    }
#ifdef CV_NEON
    cvarImageutils::convertRGBA2GrayNEON(img, frame1Channel.data, width*height*4);
#else
    cv::Mat frame4(height, width, CV_8UC3, img, width*3);
    cvtColor(frame4, frame1Channel, CV_BGR2GRAY);
    //cvtColor(frame1Channel, f, CV_GRAY2BGR);              // DEBUG
    //memcpy(img, f.data, width*height*3);
#endif
}

const cv::Mat& cvarOpenCVFrameHolder::getFrame() const
{
    return frame1Channel;
}

#endif
