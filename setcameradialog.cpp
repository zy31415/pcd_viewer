// This project
#include "setcameradialog.h"
#include "../build/ui_setcameradialog.h"
#include "pcdviewermainwindow.h"

SetCameraDialog::SetCameraDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SetCameraDialog) {

    ui->setupUi(this);

    connect(ui->buttonBox, SIGNAL(clicked(QAbstractButton*)), this, SLOT(onButton(QAbstractButton*)));

    pclViewer_ = (PCDViewerMainWindow*)parentWidget();
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

SetCameraDialog::~SetCameraDialog()
{
    delete ui;
}

void SetCameraDialog::onButton(QAbstractButton *button) {
    QDialogButtonBox::StandardButton standardButton = ui->buttonBox->standardButton(button);

    switch(standardButton) {
    // Standard buttons:
    case QDialogButtonBox::Ok:
        button_apply();
        break;
    case QDialogButtonBox::Cancel:
        break;
    case QDialogButtonBox::Apply:
        button_apply();
        break;
    }
}

void SetCameraDialog::button_apply() {
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

    viewer_ -> setCameraPosition(pos_x, pos_y, pos_z,
                                 view_x, view_y, view_z,
                                 up_x, up_y, up_z);

}


