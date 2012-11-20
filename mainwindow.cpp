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
    placePoint(0,0),
    bShowObject(false)
{
    ui->setupUi(this);

    assert(capture.isOpened());

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(50);

    this->setFixedSize(capture.get(CV_CAP_PROP_FRAME_WIDTH), capture.get(CV_CAP_PROP_FRAME_HEIGHT));

    // center the window on the screen
    setGeometry(QStyle::alignedRect( Qt::LeftToRight, Qt::AlignCenter, this->size(), qApp->desktop()->availableGeometry()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent( QPaintEvent *event )
{
    QPainter painter(this);
    capture >> capImg;
    painter.drawImage(QPoint(0,0), QImage(capImg.data, capImg.cols, capImg.rows, capImg.step, QImage::Format_RGB888));

}

void MainWindow::mousePressEvent (QMouseEvent * event)
{
    placePoint = event->pos();
    bShowObject = true;
}



