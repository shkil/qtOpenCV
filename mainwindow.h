#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include <placedetector.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    // GUI events
    void paintEvent(QPaintEvent *event );
    void mousePressEvent (QMouseEvent * event);

private:
    // QT
    Ui::MainWindow *ui;
    QTimer *timer;
    QImage *img;
    int nImgWidth2;
    int nImgHeight2;

    // OpenCV
    QPoint placePoint;
    bool bShowObject;
    cv::VideoCapture capture;
    cv::Mat capImg;                 // image captured from camera
    PlaceDetector detector;         // detector of the place
};

#endif // MAINWINDOW_H
