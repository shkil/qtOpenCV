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

    void preProcessImage(cv::Mat& img);
    void processImage();

    bool isShowObject();
protected:
    int x;              // X coordinate of the object
    int y;              // Y coordinate of the object

    cv::Mat imgSrc;
    cv::Mat croppedImg;
    bool bShowObject;

    std::vector<cv::KeyPoint> keypoints_to_find;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors_to_find;
    cv::Mat descriptors;

    std::vector< cv::DMatch > matches;
    std::vector< cv::DMatch > good_matches;

};

#endif // PLACEDETECTOR_H
