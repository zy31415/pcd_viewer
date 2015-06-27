#include <unistd.h>
#include <vector>
#include <stdio.h>

// This project
#include "tourdialog.h"
#include "../build/ui_tourdialog.h"

#include "pcdviewermainwindow.h"

#include <qtconcurrentrun.h>

#include "worker.h"
#include "boundingbox.h"

TourDialog::TourDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TourDialog),
    alpha(0),
    videoWriter_(0),
    frame_width(0),
    frame_height(0),
    image_quality(100) {

    ui->setupUi(this);

    connect(ui->buttonBox, SIGNAL(clicked(QAbstractButton*)), this, SLOT(onButton(QAbstractButton*)));


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
        button_apply();
        break;
    case QDialogButtonBox::Cancel:
        button_cancel();
        break;
    case QDialogButtonBox::Apply:
        button_apply();
        break;
    }
}

void TourDialog::button_apply() {

    QThread* thread_ = new QThread;

    Worker* worker_ = new Worker(this);

    worker_->moveToThread(thread_);

    connect(worker_, SIGNAL(error(QString)), this, SLOT(errorString(QString)));
    connect(worker_, SIGNAL(finished()), this, SLOT(tourFinished()));
    connect(thread_, SIGNAL(started()), worker_, SLOT(process()));
    connect(worker_, SIGNAL(finished()), thread_, SLOT(quit()));
    connect(worker_, SIGNAL(finished()), worker_, SLOT(deleteLater()));
    connect(thread_, SIGNAL(finished()), thread_, SLOT(deleteLater()));

    if (getTourStyleSelection() == 0)
        connect(worker_, SIGNAL(moveOneStep()), this, SLOT(oneStepAroundY()));

    if (ui->checkBox->isChecked())
        initRecording();

    thread_->start();
}

void TourDialog::initRecording() {

    videoWriter_ = new cv::VideoWriter(
                "video.avi",                    // filename
                CV_FOURCC('D','I','V','X'),     // fourcc, http://www.fourcc.org/codecs.php
                1,                             // fps
                cv::Size(frame_width, frame_height), // frameSize
                true);                          // isColor
}

void TourDialog::detectFrameSize(){
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


    pos_x = cameras[0].pos[0];
    pos_y = cameras[0].pos[1];
    pos_z = cameras[0].pos[2];

    view_x = cameras[0].focal[0];
    view_y = cameras[0].focal[1];
    view_z = cameras[0].focal[2];

    up_x = cameras[0].view[0];
    up_y = cameras[0].view[1];
    up_z = cameras[0].view[2];

    pos_y = (bb.get_max_y() + bb.get_min_y())/2.;

    double r = 4 * sqrt(pow(bb.get_max_x(),2.) + pow(bb.get_max_z(),2.));

    pos_z = r * cos(alpha);
    pos_x = r * sin(alpha);

    alpha += 0.04;
    sleep(0.01);
    viewer_ -> setCameraPosition(pos_x, pos_y, pos_z,
                                 view_x, view_y, view_z,
                                 up_x, up_y, up_z);

    if (videoWriter_) {
        static int ii = 0;

        char filename[50];
        sprintf(filename, "out/%d.png", ii++);
        cv::Mat frame;
        frame = cv::imread(filename, CV_LOAD_IMAGE_COLOR);   // Read the file

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
    //videoWriter_->release();
    //delete videoWriter_;
    videoWriter_ = 0;
    pclViewer_->enableResize();
}
