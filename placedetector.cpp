#include "placedetector.h"

PlaceDetector::PlaceDetector(): x(0), y(0)
{
}

void PlaceDetector::setPlace(int x, int y)
{
    this->x = x;
    this->y = y;
}

void PlaceDetector::getPlace(int& x, int& y)
{
    x = this->x;
    y = this->y;
}
