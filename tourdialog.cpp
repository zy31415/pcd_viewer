#include <unistd.h>

// PCL
#include <pcl/visualization/common/common.h>

// This project
#include "tourdialog.h"
#include "../build/ui_tourdialog.h"

#include "pclviewer.h"

#include <qtconcurrentrun.h>

#include "worker.h"

TourDialog::TourDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TourDialog)
{
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

TourDialog::~TourDialog()
{
    delete ui;
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
    worker_->setViewer(viewer_);
    worker_->setBoundingBox(pclViewer_->getBoundingBox());

    worker_->moveToThread(thread_);

    connect(worker_, SIGNAL(error(QString)), this, SLOT(errorString(QString)));
    connect(thread_, SIGNAL(started()), worker_, SLOT(process()));
    connect(worker_, SIGNAL(finished()), thread_, SLOT(quit()));
    connect(worker_, SIGNAL(finished()), worker_, SLOT(deleteLater()));
    connect(thread_, SIGNAL(finished()), thread_, SLOT(deleteLater()));

    thread_->start();
    //thread_->wait();



//    double pos_x, pos_y, pos_z,
//            view_x, view_y, view_z,
//            up_x, up_y, up_z;

////    pos_x = ui->pos_x->text().toFloat();
////    pos_y = ui->pos_y->text().toFloat();
////    pos_z = ui->pos_z->text().toFloat();
//    view_x = ui->view_x->text().toFloat();
//    view_y = ui->view_y->text().toFloat();
//    view_z = ui->view_z->text().toFloat();
//    up_x = ui->up_x->text().toFloat();
//    up_y = ui->up_y->text().toFloat();
//    up_z = ui->up_z->text().toFloat();

//    BoundingBox bb = pclViewer_->getBoundingBox();

//    pos_z = (bb.get_max_z() + bb.get_min_z())/2.;

//    double r = 2 * sqrt(pow(bb.get_max_x(),2.) + pow(bb.get_max_y(),2.));

//    double alpha = 0;
//    while (alpha < 3.14*4) {
//        pos_x = r * cos(alpha);
//        pos_y = r * sin(alpha);

//        alpha += 0.002;
//        sleep(0.01);
//        viewer_ -> setCameraPosition(pos_x, pos_y, pos_z,
//                                     view_x, view_y, view_z,
//                                     up_x, up_y, up_z);
//    }
}


