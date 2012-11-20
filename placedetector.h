#ifndef PLACEDETECTOR_H
#define PLACEDETECTOR_H

#include <opencv2/opencv.hpp>

class PlaceDetector
{
public:
    PlaceDetector();
    void setPlace(int x, int y);
    void getPlace(int& x, int& y);
protected:
    int x;              // X coordinate of the object
    int y;              // Y coordinate of the object
};

#endif // PLACEDETECTOR_H
