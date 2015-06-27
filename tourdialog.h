#ifndef TOURDIALOG_H
#define TOURDIALOG_H

// pcl
#include <pcl/visualization/pcl_visualizer.h>

// QT
#include <QDialog>
#include <QAbstractButton>

// opencv
#include <opencv2/opencv.hpp>

// This project

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


    void button_apply();
    void button_cancel() {}
    void button_ok() {}

    int getTourStyleSelection();

    void initRecording();

    void detectFrameSize();

public slots:
    void onButton(QAbstractButton *button);

    void errorString(QString str) { std::cout<<str.toStdString()<<std::endl;}

    void oneStepAroundY();
    void tourFinished();
    void onRec();


signals:
    void click_apply();
    void finished();
    void error(QString err);
};

#endif // TOURDIALOG_H
