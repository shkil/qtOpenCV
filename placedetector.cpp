#include "placedetector.h"

#define DETECT_WIDTH 100
#define DETECT_HEIGHT 100

PlaceDetector::PlaceDetector(): x(0), y(0)
{
}

void PlaceDetector::setPlace(int x, int y)
{
    //this->x = x;
    //this->y = y;

    // crop image to detect some place
    cv::Rect myROI(x-DETECT_WIDTH, y-DETECT_HEIGHT, DETECT_WIDTH, DETECT_HEIGHT);
    cv::Mat cropped = imgSrc(myROI);

    int minHessian = 400;
    cv::SurfFeatureDetector detector( minHessian );
    detector.detect( imgSrc, keypoints_to_find );
}

void PlaceDetector::getPlace(int& x, int& y)
{
    x = this->x;
    y = this->y;
}

void PlaceDetector::setImage(const cv::Mat& img)
{
    img.copyTo(imgSrc);
}
