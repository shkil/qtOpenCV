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

//    std::vector<cv::KeyPoint> keypoints_to_find;
//    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors_to_find;
    cv::Mat descriptors;

//  std::vector< cv::DMatch > matches;
//  std::vector< cv::DMatch > good_matches;

    std::vector<cv::Point2f> points_to_find;
    std::vector<cv::Point2f> points;

    cv::Point2f pPointExternal;

    void detect(std::vector<cv::Point2f>& points, cv::Mat& descs);
    void ratioTest(std::vector<std::vector<cv::DMatch> >& matches, float ratio);
    void symmetryTest(std::vector<std::vector<cv::DMatch> >& matches1,
                        std::vector<std::vector<cv::DMatch> >& matches2,
                        std::vector<cv::DMatch>& symMatches);
    void matchesToPoints(std::vector<cv::Point2f>& points1,
                        std::vector<cv::Point2f>& points2,
                        std::vector<cv::DMatch> matches,
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
                                    cv::Point2f pointExternal,
                                    cv::Point2f& pointInternal,
                                    double ransacThreshold);
};

#endif // PLACEDETECTOR_H
