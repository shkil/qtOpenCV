#include <QDebug>
#include <QPainter>
#include <QImage>
#include <QPoint>
#include <QStyle>
#include <QDesktopWidget>
#include <QMouseEvent>

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    capture(0),
    placePoint(0,0)
{
    ui->setupUi(this);

    assert(capture.isOpened());

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(100);

    // creating image
    img = new QImage(":/images/res/pikas.png");
    nImgWidth2 = img->width()/2;
    nImgHeight2 = img->height()/2;

    // No resizing
    this->setFixedSize(capture.get(CV_CAP_PROP_FRAME_WIDTH), capture.get(CV_CAP_PROP_FRAME_HEIGHT));

    // center the window on the screen
    setGeometry(QStyle::alignedRect( Qt::LeftToRight, Qt::AlignCenter, this->size(), qApp->desktop()->availableGeometry()));
}

MainWindow::~MainWindow()
{
    delete img;
    delete ui;
}

void MainWindow::paintEvent( QPaintEvent *event )
{
    QPainter painter(this);
    int x, y;

    capture >> capImg;
    cv::flip(capImg, capImg, 1);
    detector.setImage(capImg);
    detector.processImage();
    //cv::Mat ss;
    //capImg.copyTo(ss);

    //cv::Rect myROI(10, 10, 100, 100);
    //capImg = capImg(myROI);

    detector.preProcessImage(capImg);      //debug

    painter.drawImage(QPoint(0,0), QImage(capImg.data, capImg.cols, capImg.rows, capImg.step, QImage::Format_RGB888));
    if( !detector.isShowObject() )
        return;
    detector.getPlace(x, y);
    painter.drawImage(QPoint(x-nImgWidth2,y-nImgHeight2), *img);
}

void MainWindow::mousePressEvent (QMouseEvent * event)
{
    placePoint = event->pos();
    detector.setPlace(placePoint.x(), placePoint.y());
}



