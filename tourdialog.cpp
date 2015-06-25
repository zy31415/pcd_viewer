#include <unistd.h>

// PCL
#include <pcl/visualization/common/common.h>

// This project
#include "tourdialog.h"
#include "../build/ui_tourdialog.h"

#include "pclviewer.h"

#include <qtconcurrentrun.h>

#include "worker.h"
#include "boundingbox.h"

TourDialog::TourDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TourDialog),
    alpha(0) {

    ui->setupUi(this);

    connect(ui->buttonBox, SIGNAL(clicked(QAbstractButton*)), this, SLOT(onButton(QAbstractButton*)));

    pclViewer_ = (PCLViewer*)parentWidget();
    viewer_ = pclViewer_ -> getViewer();

    std::vector<pcl::visualization::Camera> cameras;
    viewer_ -> getCameras(cameras);

    ui->pos_x->setText(QString::number(cameras[0].pos[0]));
    ui->pos_y->setText(QString::number(cameras[0].pos[1]));
    ui->pos_z->setText(QString::number(cameras[0].pos[2]));

    ui->view_x->setText(QString::number(cameras[0].focal[0]));
    ui->view_y->setText(QString::number(cameras[0].focal[1]));
    ui->view_z->setText(QString::number(cameras[0].focal[2]));

    ui->up_x->setText(QString::number(cameras[0].view[0]));
    ui->up_y->setText(QString::number(cameras[0].view[1]));
    ui->up_z->setText(QString::number(cameras[0].view[2]));

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
    connect(thread_, SIGNAL(started()), worker_, SLOT(process()));
    connect(worker_, SIGNAL(finished()), thread_, SLOT(quit()));
    connect(worker_, SIGNAL(finished()), worker_, SLOT(deleteLater()));
    connect(thread_, SIGNAL(finished()), thread_, SLOT(deleteLater()));

    connect(worker_, SIGNAL(click_apply()), this, SLOT(tour()));

    thread_->start();
}


void TourDialog::tour() {
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

    BoundingBox bb = pclViewer_->getBoundingBox();

    pos_z = (bb.get_max_z() + bb.get_min_z())/2.;

    double r = 2 * sqrt(pow(bb.get_max_x(),2.) + pow(bb.get_max_y(),2.));

        pos_x = r * cos(alpha);
        pos_y = r * sin(alpha);

        alpha += 0.04;
        sleep(0.01);
        viewer_ -> setCameraPosition(pos_x, pos_y, pos_z,
                                     view_x, view_y, view_z,
                                     up_x, up_y, up_z);

}

