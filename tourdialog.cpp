#include <unistd.h>

// PCL
#include <pcl/visualization/common/common.h>

// This project
#include "tourdialog.h"
#include "../build/ui_tourdialog.h"


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
        button_ok();
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

    double pos_x, pos_y, pos_z,
            view_x, view_y, view_z,
            up_x, up_y, up_z;

    pos_x = ui->pos_x->text().toFloat();
    pos_y = ui->pos_y->text().toFloat();
    pos_z = ui->pos_z->text().toFloat();
    view_x = ui->view_x->text().toFloat();
    view_y = ui->view_y->text().toFloat();
    view_z = ui->view_z->text().toFloat();
    up_x = ui->up_x->text().toFloat();
    up_y = ui->up_y->text().toFloat();
    up_z = ui->up_z->text().toFloat();

    for (int n=0; n<1000; n++) {
        pos_x *= 0.9;
        pos_y *= 0.9;
        pos_z *= 0.9;
        sleep(0.01);
        viewer_ -> setCameraPosition(pos_x, pos_y, pos_z,
                                     view_x, view_y, view_z,
                                     up_x, up_y, up_z);
    }



    //viewer_ -> spin();

}
