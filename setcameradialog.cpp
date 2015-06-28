// This project
#include "setcameradialog.h"
#include "../build/ui_setcameradialog.h"
#include "pcdviewermainwindow.h"

SetCameraDialog::SetCameraDialog(
        const pcl::visualization::Camera& camera,
        QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SetCameraDialog)
{
    ui->setupUi(this);

    connect(ui->buttonBox, SIGNAL(clicked(QAbstractButton*)),
            this, SLOT(onButton(QAbstractButton*)));

    // set current value
    ui->pos_x->setText(QString::number(camera.pos[0]));
    ui->pos_y->setText(QString::number(camera.pos[1]));
    ui->pos_z->setText(QString::number(camera.pos[2]));

    ui->view_x->setText(QString::number(camera.focal[0]));
    ui->view_y->setText(QString::number(camera.focal[1]));
    ui->view_z->setText(QString::number(camera.focal[2]));

    ui->up_x->setText(QString::number(camera.view[0]));
    ui->up_y->setText(QString::number(camera.view[1]));
    ui->up_z->setText(QString::number(camera.view[2]));

    // set validator
    ui->pos_x->setValidator( new QDoubleValidator(this));
    ui->pos_y->setValidator( new QDoubleValidator(this));
    ui->pos_z->setValidator( new QDoubleValidator(this));

    ui->view_x->setValidator( new QDoubleValidator(this));
    ui->view_y->setValidator( new QDoubleValidator(this));
    ui->view_z->setValidator( new QDoubleValidator(this));

    ui->up_x->setValidator( new QDoubleValidator(this));
    ui->up_y->setValidator( new QDoubleValidator(this));
    ui->up_z->setValidator( new QDoubleValidator(this));

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

void SetCameraDialog::button_apply()
{
    pos_x = ui->pos_x->text().toFloat();
    pos_y = ui->pos_y->text().toFloat();
    pos_z = ui->pos_z->text().toFloat();
    view_x = ui->view_x->text().toFloat();
    view_y = ui->view_y->text().toFloat();
    view_z = ui->view_z->text().toFloat();
    up_x = ui->up_x->text().toFloat();
    up_y = ui->up_y->text().toFloat();
    up_z = ui->up_z->text().toFloat();

    std::vector<double> par;
    par.push_back(pos_x);
    par.push_back(pos_y);
    par.push_back(pos_z);

    par.push_back(view_x);
    par.push_back(view_y);
    par.push_back(view_z);

    par.push_back(up_x);
    par.push_back(up_y);
    par.push_back(up_z);


    emit onSetCamera(par);
}


