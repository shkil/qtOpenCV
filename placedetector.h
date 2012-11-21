#ifndef PLACEDETECTOR_H
#define PLACEDETECTOR_H

#include <opencv2/opencv.hpp>

class PlaceDetector
{
public:
    PlaceDetector();
    void setPlace(int x, int y);
    void getPlace(int& x, int& y);
    void setImage(const cv::Mat& img);

protected:
    int x;              // X coordinate of the object
    int y;              // Y coordinate of the object

    cv::Mat imgSrc;
    std::vector<cv::KeyPoint> keypoints_to_find;
};

#endif // PLACEDETECTOR_H
