#ifndef TOURDIALOG_H
#define TOURDIALOG_H

// pcl
#include <pcl/visualization/pcl_visualizer.h>

// QT
#include <QDialog>
#include <QAbstractButton>
#include <QReadWriteLock>

// opencv
#include <opencv2/opencv.hpp>

// This project
#include "worker.h"

class PCDViewerMainWindow;

namespace Ui {
    class TourDialog;
}

class TourDialog : public QDialog
{
    Q_OBJECT

public:
    TourDialog(QWidget *parent = 0);
    ~TourDialog();

private:
    Ui::TourDialog *ui;

    double alpha;
    cv::VideoWriter* videoWriter_;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_; // display widget
    PCDViewerMainWindow* pclViewer_; // parent widget

    int frame_width, frame_height, image_quality;


    QThread* thread_;
    Worker* worker_;

    int  num_worker;
    QReadWriteLock locker;

    void button_apply();
    void button_cancel() {}
    void button_ok() {}

    int getTourStyleSelection();

    void initRecording();
    void recordOneFrame();

    void detectFrameSize();

    void lockWindowFrameSize();
    void unlockWindowFrameSize();


public slots:
    void onButton(QAbstractButton *button);

    void errorString(QString str) { std::cout<<str.toStdString()<<std::endl;}

    void oneStepAroundY();
    void oneStepAroundX();
    void oneStepAroundZ();
    void tourFinished();
    void onRec();
    void onStop();


signals:
    void click_apply();
    void terminate();
    void error(QString err);
};

#endif // TOURDIALOG_H
