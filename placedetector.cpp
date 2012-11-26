#include "placedetector.h"
#include <QDebug>

#define DETECT_WIDTH 150
#define DETECT_HEIGHT 150


PlaceDetector::PlaceDetector(): x(0), y(0), bShowObject(false)
{
}


void PlaceDetector::preProcessImage(cv::Mat& img)
{
    std::vector<cv::KeyPoint> keypoints;
    int minHessian = 400;
    //cv::SurfFeatureDetector detector( minHessian );
    cv::FastFeatureDetector detector;
    detector.detect(img, keypoints);

    cv::drawKeypoints(img, keypoints, img);
    cv::rectangle(img, cv::Rect(x-DETECT_WIDTH/2, y-DETECT_HEIGHT/2, DETECT_WIDTH, DETECT_HEIGHT),
                  cv::Scalar(0x0, 0x0, 0, 0));

}

void PlaceDetector::setPlace(int x, int y)
{
    this->x = x;
    this->y = y;

    // crop image to detect some place
    cv::Rect myROI(x-DETECT_WIDTH/2, y-DETECT_HEIGHT/2, DETECT_WIDTH, DETECT_HEIGHT);
    croppedImg = imgSrc(myROI);

    // Getting keypoints
    int minHessian = 400;
    cv::SurfFeatureDetector detector( minHessian );
    detector.detect( croppedImg, keypoints_to_find );
    qDebug() << "Keypoints of Place size: " << keypoints_to_find.size();

    // Extract descriptors
    cv::SurfDescriptorExtractor extractor;
    extractor.compute(croppedImg, keypoints_to_find, descriptors_to_find);
}

void PlaceDetector::getPlace(int& x, int& y)
{
    x = this->x;
    y = this->y;
}

void PlaceDetector::setImage(const cv::Mat& img)
{
    img.copyTo(imgSrc);
    processImage();             // here for debug - actual matching
}

void PlaceDetector::processImage()
{
    int minHessian = 400;
    cv::SurfFeatureDetector detector( minHessian );
    detector.detect( imgSrc, keypoints );
    qDebug() << "Keypoints of whole image size: " << keypoints.size();

    // Extract descriptors
    cv::SurfDescriptorExtractor extractor;
    extractor.compute(imgSrc, keypoints, descriptors);

    //-- Этап 3: Необходимо сматчить вектора дескрипторов.
    cv::FlannBasedMatcher matcher;
    matcher.match( descriptors_to_find, descriptors, matches );
    double max_dist = 0; double min_dist = 100;

    //-- Вычисление максимального и минимального расстояния среди всех дескрипторов
                           // в пространстве признаков
    for( int i = 0; i < descriptors_to_find.rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    qDebug() << "-- Max dist: " << max_dist;
    qDebug() << "-- Min dist :" << min_dist;

    good_matches.clear();
    //-- Отобрать только хорошие матчи, расстояние меньше чем 3 * min_dist
    for( int i = 0; i < descriptors_to_find.rows; i++ )
    {
        if( matches[i].distance < 3 * min_dist )
        {
            good_matches.push_back( matches[i]);
        }
    }
    qDebug() << "Number of good matches: " << good_matches.size();
    if( good_matches.size() < 4 )
    {
        bShowObject = false;
        return;
    }

    //-- Локализация объектов
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;
    for( int i = 0; i < good_matches.size(); i++ )
    {
        obj.push_back( keypoints_to_find[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints[ good_matches[i].trainIdx ].pt );
    }
    cv::Mat H = cv::findHomography( obj, scene, CV_RANSAC );

    /*std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
    obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
    std::vector<cv::Point2f> scene_corners(4);*/

    // SHKIL
    std::vector<cv::Point2f> obj_corners(1);
    std::vector<cv::Point2f> scene_corners(1);
    obj_corners[0] = cv::Point2f(DETECT_WIDTH/2, DETECT_HEIGHT/2);

    //-- Отобразить углы целевого объекта, используя найденное преобразование, на сцену
    cv::perspectiveTransform( obj_corners, scene_corners, H);

    this->x = scene_corners[0].x;
    this->y = scene_corners[0].y;
    bShowObject = true;
}

 bool PlaceDetector::isShowObject()
 {
    return bShowObject;
 }
