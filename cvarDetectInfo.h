//
//  cvarDetectInfo.h
//  cvar
//
//  Created by Oleksandr Shkil on 27.11.12.
//  Copyright (c) 2012 Codeminders. All rights reserved.
//

#ifndef cvar_cvarDetectInfo_h
#define cvar_cvarDetectInfo_h

#include <vector>

struct stDetectInfo{
    std::vector<cv::Point2f> keypoints;
    cv::Mat descriptors;
    int x;                  // Coords of place
    int y;
};

#endif
