#include <unistd.h>
#include <vector>
#include <stdio.h>

// This project
#include "tourdialog.h"
#include "../build/ui_tourdialog.h"

#include "pcdviewermainwindow.h"

#include <qtconcurrentrun.h>

#include "boundingbox.h"
#include "pcdviewermainwindow.h"

TourDialog::TourDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TourDialog),
    alpha(0),
    videoWriter_(0),
    frame_width(0),
    frame_height(0),
    image_quality(-1),
    thread_(0),
    worker_(0),
    num_worker(0)
{
    ui->setupUi(this);

    pclViewer_ = (PCDViewerMainWindow*)parentWidget();
    viewer_ = pclViewer_->getViewer();

    connect(ui->buttonBox, SIGNAL(clicked(QAbstractButton*)), this, SLOT(onButton(QAbstractButton*)));
    connect(ui->pushButtonRec, SIGNAL(clicked()), this, SLOT(onRec()));
    connect(ui->pushButtonStop, SIGNAL(clicked()), this, SLOT(onStop()));


    ui->comboBox->addItem("Rotate around Y");
    ui->comboBox->addItem("Rotate around X");
    ui->comboBox->addItem("Rotate around Z");

}

TourDialog::~TourDialog() {
    delete ui;
    delete videoWriter_;
}

void TourDialog::onButton(QAbstractButton *button) {
    QDialogButtonBox::StandardButton standardButton = ui->buttonBox->standardButton(button);

    switch(standardButton) {
    // Standard buttons:
    case QDialogButtonBox::Ok:
        //button_apply();
        break;
    case QDialogButtonBox::Cancel:
        button_cancel();
        break;
    case QDialogButtonBox::Apply:
        //button_apply();
        break;
    }
}

void TourDialog::onRec() {
    int _num_worker = 0;

    locker.lockForRead();
    _num_worker = num_worker;
    locker.unlock();

    if (_num_worker >= 1)
        return;

    thread_ = new QThread;
    worker_ = new Worker(&num_worker, &locker, this);

    worker_->moveToThread(thread_);

    connect(worker_, SIGNAL(error(QString)), this, SLOT(errorString(QString)));    
    connect(thread_, SIGNAL(started()), worker_, SLOT(process()));
    connect(worker_, SIGNAL(finished()), thread_, SLOT(quit()));
    connect(worker_, SIGNAL(finished()), worker_, SLOT(deleteLater()));
    connect(worker_, SIGNAL(finished()), this, SLOT(tourFinished()));
    connect(thread_, SIGNAL(finished()), thread_, SLOT(deleteLater()));

    if (getTourStyleSelection() == 0)
        connect(worker_, SIGNAL(moveOneStep()), this, SLOT(oneStepAroundY()));

    if (ui->checkBox->isChecked())
        initRecording();

    locker.lockForWrite();
    num_worker++;
    locker.unlock();

    thread_->start();
}

void TourDialog::onStop()
{
    locker.lockForWrite();
    num_worker --;
    locker.unlock();
}

void TourDialog::initRecording()
{
    lockWindowFrameSize();
    detectFrameSize();

    videoWriter_ = new cv::VideoWriter(
                "video.avi",                    // filename
                CV_FOURCC('D','I','V','X'),     // fourcc, http://www.fourcc.org/codecs.php
                10,                             // fps
                cv::Size(frame_width, frame_height), // frameSize
                true);                          // isColor
}

void TourDialog::detectFrameSize()
{
    QRect rect = pclViewer_->getSnapshotGeometry();
    QPixmap pixmap(rect.size());
    pclViewer_->renderASnapshot(pixmap, QPoint(), QRegion(rect));

    char buffer[50];

    tmpnam(buffer);

    pixmap.save(buffer, "PNG", image_quality);

    cv::Mat frame;
    frame = cv::imread(buffer, CV_LOAD_IMAGE_COLOR);   // Read the file

    frame_width = frame.size().width;
    frame_height = frame.size().height;

}

void TourDialog::oneStepAroundY() {
    double pos_x, pos_y, pos_z,
            view_x, view_y, view_z,
            up_x, up_y, up_z;


    std::vector<pcl::visualization::Camera> cameras;
    viewer_ -> getCameras(cameras);

    BoundingBox bb = pclViewer_->getData()->getBoundingBox();

    double mid_x = bb.get_mid(0)/2.;
    double mid_z = bb.get_mid(2)/2.;


    pos_x = cameras[0].pos[0];
    pos_y = cameras[0].pos[1];
    pos_z = cameras[0].pos[2];

    view_x = cameras[0].focal[0];
    view_y = cameras[0].focal[1];
    view_z = cameras[0].focal[2];

    up_x = cameras[0].view[0];
    up_y = cameras[0].view[1];
    up_z = cameras[0].view[2];

    double alpha = 0.1;

    pos_x = cos(alpha)*(pos_x - mid_x) + sin(alpha)*(pos_z - mid_z) + mid_x;
    pos_z = -sin(alpha)*(pos_x - mid_x) + cos(alpha)*(pos_z - mid_z) + mid_z;

    //sleep(0.01);
    viewer_ -> setCameraPosition(pos_x, pos_y, pos_z,
                                 view_x, view_y, view_z,
                                 up_x, up_y, up_z);

    if (videoWriter_) {
        QRect rect = pclViewer_->getSnapshotGeometry();
        QPixmap pixmap(rect.size());
        pclViewer_->renderASnapshot(pixmap, QPoint(), QRegion(rect));

        char buffer[50];

        tmpnam(buffer);

        pixmap.save(buffer, "PNG", image_quality);

        cv::Mat frame;
        frame = cv::imread(buffer, CV_LOAD_IMAGE_COLOR);   // Read the file

        assert(frame.size().width == frame_width);
        assert(frame.size().height == frame_height);

        videoWriter_->write(frame);
    }
}


int TourDialog::getTourStyleSelection() {
    return ui->comboBox->currentIndex();
}

void TourDialog::tourFinished() {
    videoWriter_->release();
    delete videoWriter_;
    videoWriter_ = 0;
    unlockWindowFrameSize();
}

void TourDialog::lockWindowFrameSize() {
    pclViewer_->setMaximumSize(pclViewer_->size());
    pclViewer_->setMinimumSize(pclViewer_->size());
}

void TourDialog::unlockWindowFrameSize() {
    pclViewer_->setMaximumSize(QSize(500000,500000));
    pclViewer_->setMinimumSize(QSize(50,50));
}
