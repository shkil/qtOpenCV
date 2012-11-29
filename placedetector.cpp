#include "placedetector.h"
#include <QDebug>
#include <QTime>

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
    cv::OrbFeatureDetector detector;
    detector.detect(img, keypoints);

    cv::drawKeypoints(img, keypoints, img);
    cv::rectangle(img, cv::Rect(x-DETECT_WIDTH/2, y-DETECT_HEIGHT/2, DETECT_WIDTH, DETECT_HEIGHT),
                  cv::Scalar(0x0, 0x0, 0, 0));

}

void PlaceDetector::detect(std::vector<cv::Point2f>& points, cv::Mat& descs)
{
    cv::OrbFeatureDetector orb;
    std::vector<cv::KeyPoint> keypoints1;
    orb.detect( imgSrc, keypoints1 );
    cv::KeyPoint::convert(keypoints1, points);
    cv::OrbDescriptorExtractor orbD;
    orbD.compute( imgSrc, keypoints1, descs);
}

void PlaceDetector::setPlace(int x, int y)
{
//    this->x = x;
//    this->y = y;

//    // crop image to detect some place
//    cv::Rect myROI(x-DETECT_WIDTH/2, y-DETECT_HEIGHT/2, DETECT_WIDTH, DETECT_HEIGHT);
//    croppedImg = imgSrc(myROI);

//    // Getting keypoints
//    int minHessian = 400;
//    cv::OrbFeatureDetector detector( minHessian );
//    detector.detect( croppedImg, keypoints_to_find );
//    qDebug() << "Keypoints of Place size: " << keypoints_to_find.size();

//    // Extract descriptors
//    cv::OrbDescriptorExtractor extractor;
//    extractor.compute(croppedImg, keypoints_to_find, descriptors_to_find);
    detect(points_to_find, descriptors_to_find);
    pPointExternal.x = x;
    pPointExternal.y = y;
}

void PlaceDetector::getPlace(int& x, int& y)
{
    x = this->x;
    y = this->y;
}

void PlaceDetector::setImage(const cv::Mat& img)
{
    img.copyTo(imgSrc);
    //processImage();             // here for debug - actual matching
}

void PlaceDetector::processImage()
{
    float ratio = 0.65f;
    double distance = 3.0;
    double confidence = 0.99;
    double ransacThreshold = 4.0;

    detect(points, descriptors);
    // +++++++++++++++ knnMatching +++++++++++++++++++++++++++
    cv::BruteForceMatcher<cv::Hamming> matcher;
    std::vector<std::vector<cv::DMatch> > matches1;
    matcher.knnMatch(descriptors,descriptors_to_find,
                                matches1, // vector of matches (up to 2 per entry)
                                2);       // return 2 nearest neighbours

    std::vector<std::vector<cv::DMatch> > matches2;
    matcher.knnMatch(descriptors_to_find,descriptors,
                        matches2, // vector of matches (up to 2 per entry)
                        2);       // return 2 nearest neighbours
    // +++++++++++++++++ ratioTest ++++++++++++++++++++++++++++
    // ratio test Internal -> External
    ratioTest(matches1, ratio);
    // ratio test External -> Internal
    ratioTest(matches2, ratio);

    // +++++++++++++++++ Symmetry test after ratio test ++++++++
    std::vector<cv::DMatch> symMatches2;
    if(matches1.size()==0 || matches2.size()==0){
        bShowObject = false;
        return;
    }
    symmetryTest(matches1,matches2,symMatches2);

    // +++++++++++++++++ Ransac test with ratio filtering +++++++
    std::vector<cv::Point2f> points1symAfterRatio, points2symAfterRatio;
    matchesToPoints(points1symAfterRatio, points2symAfterRatio, symMatches2, points_to_find, points);
    if (points1symAfterRatio.size() == 0) {
        bShowObject = false;
        return;
    }
    std::vector<cv::DMatch> ransacMatchesWithRatio;
    ransacTest(points1symAfterRatio, points2symAfterRatio, symMatches2, ransacMatchesWithRatio, distance, confidence);

    // +++++++++++++++++ transform point with ratio filtering ++++++++
    std::vector<cv::Point2f> externalRatio, internalRatio;
    matchesToPoints(externalRatio, internalRatio, ransacMatchesWithRatio, points_to_find, points);
    if(externalRatio.size()==0) {
        bShowObject = false;
        return;
    }
    cv::Point2f pPointInternalCorrespondent;
    transformPointHomography(externalRatio,internalRatio, pPointExternal, pPointInternalCorrespondent, ransacThreshold);
    this->x = pPointInternalCorrespondent.x;
    this->y = pPointInternalCorrespondent.y;
    bShowObject = true;
    /*int minHessian = 400;

    if( !keypoints_to_find.size() )
        return;

    QTime myTimer;
    myTimer.start();

    //cv::SurfFeatureDetector detector( minHessian );
    cv::OrbFeatureDetector orb;
    orb.detect(imgSrc, keypoints );

    qDebug() << "Keypoints of whole image size: " << keypoints.size();

    // Extract descriptors
    //cv::SurfDescriptorExtractor extractor;
    cv::OrbDescriptorExtractor extractor;
    extractor.compute(imgSrc, keypoints, descriptors);

    qDebug() << "Time feature detecting whole image: " <<  myTimer.elapsed();

    //-- Этап 3: Необходимо сматчить вектора дескрипторов.
    //cv::FlannBasedMatcher matcher;
    cv::BruteForceMatcher<cv::Hamming> matcher;
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

    //qDebug() << "-- Max dist: " << max_dist;
    //qDebug() << "-- Min dist :" << min_dist;

    good_matches.clear();
    //-- Отобрать только хорошие матчи, расстояние меньше чем 3 * min_dist
    for( int i = 0; i < descriptors_to_find.rows; i++ )
    {
        if( matches[i].distance < 3 * min_dist )
        {
            good_matches.push_back( matches[i]);
        }
    }
    //qDebug() << "Number of good matches: " << good_matches.size();
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

    // SHKIL
    std::vector<cv::Point2f> obj_corners(1);
    std::vector<cv::Point2f> scene_corners(1);
    obj_corners[0] = cv::Point2f(DETECT_WIDTH/2, DETECT_HEIGHT/2);

    //-- Отобразить углы целевого объекта, используя найденное преобразование, на сцену
    cv::perspectiveTransform( obj_corners, scene_corners, H);

    this->x = scene_corners[0].x;
    this->y = scene_corners[0].y;
    bShowObject = true;*/
}

void PlaceDetector::ratioTest(std::vector<std::vector<cv::DMatch> >& matches, float ratio){
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

void PlaceDetector::symmetryTest(std::vector<std::vector<cv::DMatch> >& matches1,
                    std::vector<std::vector<cv::DMatch> >& matches2,
                    std::vector<cv::DMatch>& symMatches){
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

void PlaceDetector::matchesToPoints(std::vector<cv::Point2f>& points1,
                        std::vector<cv::Point2f>& points2,
                        std::vector<cv::DMatch> matches,
                        const std::vector<cv::Point2f>& pointsExternal,
                        const std::vector<cv::Point2f>& pointsInternal){
    for (std::vector<cv::DMatch>::const_iterator it= matches.begin(); it!= matches.end(); ++it) {
        points1.push_back(pointsExternal[it->trainIdx]);
        points2.push_back(pointsInternal[it->queryIdx]);
    }
}

void PlaceDetector::ransacTest(std::vector<cv::Point2f>& points1,
                std::vector<cv::Point2f>& points2,
                std::vector<cv::DMatch>& matches,
                std::vector<cv::DMatch>& ransacMatches,
                double distance,
                double confidence){

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

void PlaceDetector::transformPointHomography(std::vector<cv::Point2f>& external,
                                std::vector<cv::Point2f>& internal,
                                cv::Point2f pointExternal,
                                cv::Point2f& pointInternal,
                                double ransacThreshold){
//    timespec time1, time2;
//    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
    cv::Mat homographyMat = cv::findHomography(external, internal, cv::FM_RANSAC, ransacThreshold);
//    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
//    ss << "\n Time: transform: homMat: " << diff(time1,time2).tv_sec << ":" << diff(time1,time2).tv_nsec/1000000000.0;
    std::vector<cv::Point2f> pExternalPointVector;
    pExternalPointVector.push_back(pointExternal);
    std::vector<cv::Point2f> pInternalPointVector;
    cv::perspectiveTransform(pExternalPointVector,pInternalPointVector,homographyMat);
    pointInternal = pInternalPointVector[0];
}

bool PlaceDetector::isShowObject()
{
    return bShowObject;
}
