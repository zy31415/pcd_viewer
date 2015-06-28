#include "triangulationdialog.h"
#include "../build/ui_triangulationdialog.h"

#include <stdio.h>
#include <string>

#include "pcdviewermainwindow.h"
#include "triangulation_meshes.h"


TriangulationDialog::TriangulationDialog(DataModel* data_, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TriangulationDialog),
    data_(data_)
{
    ui->setupUi(this);

    setParametersToDialog();

    connect(ui->buttonBox, SIGNAL(clicked(QAbstractButton*)), this, SLOT(onComputeTriangulationButton(QAbstractButton*)));
    connect(ui->checkBox_if_plot_meshes, SIGNAL(toggled(bool)), this, SLOT(setEnabled()));
}

TriangulationDialog::~TriangulationDialog()
{
    delete ui;
}

void TriangulationDialog::setParametersToDialog()
{
    TriangulationParameters par(data_->getTriangulationParameters());
    ui->lineEdit_k->setText(QString::number(par.k));
    ui->lineEdit_SearchRadius->setText(QString::number(par.search_radius));
    ui->lineEdit_Mu->setText(QString::number(par.mu));
    ui->lineEdit_MaxNN->setText(QString::number(par.max_NN));
    ui->lineEdit_MaxSurfaceAngle->setText(QString::number(par.max_surface_angle*180./PI));
    ui->lineEdit_MinAngle->setText(QString::number(par.min_angle*180./PI));
    ui->lineEdit_MaximumAngle->setText(QString::number(par.max_angle*180./PI));
    ui->checkBox_is_normal_consistency->setChecked(par.is_normal_consistency);

    ui->checkBox_if_plot_meshes->setChecked(data_->getIfShowMeshes());

    // Set validator
    ui->lineEdit_k->setValidator( new QIntValidator(0, 1000000, this));
    ui->lineEdit_SearchRadius->setValidator( new QDoubleValidator(0, 1e99, 10, this));
    ui->lineEdit_Mu->setValidator( new QDoubleValidator(0, 1e99, 10, this));
    ui->lineEdit_MaxNN->setValidator( new QIntValidator(0, 1000000, this));
    ui->lineEdit_MaxSurfaceAngle->setValidator( new QDoubleValidator(-360, 360, 10, this));
    ui->lineEdit_MinAngle->setValidator( new QDoubleValidator(-360, 360, 10, this));
    ui->lineEdit_MaximumAngle->setValidator( new QDoubleValidator(-360, 360, 10, this));


    setEnabled();
}

void TriangulationDialog::setEnabled()
{
    bool if_plot_meshes = ui->checkBox_if_plot_meshes->isChecked();
    if (if_plot_meshes) {
        ui->lineEdit_k->setEnabled(true);
        ui->lineEdit_SearchRadius->setEnabled(true);
        ui->lineEdit_Mu->setEnabled(true);
        ui->lineEdit_MaxNN->setEnabled(true);
        ui->lineEdit_MaxSurfaceAngle->setEnabled(true);
        ui->lineEdit_MinAngle->setEnabled(true);
        ui->lineEdit_MaximumAngle->setEnabled(true);
        ui->checkBox_is_normal_consistency->setEnabled(true);
    } else {
        ui->lineEdit_k->setEnabled(false);
        ui->lineEdit_SearchRadius->setEnabled(false);
        ui->lineEdit_Mu->setEnabled(false);
        ui->lineEdit_MaxNN->setEnabled(false);
        ui->lineEdit_MaxSurfaceAngle->setEnabled(false);
        ui->lineEdit_MinAngle->setEnabled(false);
        ui->lineEdit_MaximumAngle->setEnabled(false);
        ui->checkBox_is_normal_consistency->setEnabled(false);
    }
}

TriangulationParameters TriangulationDialog::getTriangulationParametersFromDialog()
{
    TriangulationParameters par;

    par.k = ui->lineEdit_k->text().toInt();
    par.search_radius = ui->lineEdit_SearchRadius->text().toDouble();
    par.mu = ui->lineEdit_Mu->text().toDouble();
    par.max_NN = ui->lineEdit_MaxNN->text().toDouble();
    par.max_surface_angle = ui->lineEdit_MaxSurfaceAngle->text().toDouble()*PI/180.;
    par.min_angle = ui->lineEdit_MinAngle->text().toDouble()*PI/180.;
    par.max_angle = ui->lineEdit_MaximumAngle->text().toDouble()*PI/180.;
    par.is_normal_consistency = ui->checkBox_is_normal_consistency->isChecked();

    return par;
}

void TriangulationDialog::computeTriangulationMesh()
{
    TriangulationParameters par = getTriangulationParametersFromDialog();
    data_->setMeshing(ui->checkBox_if_plot_meshes->isChecked(), par);
}

void TriangulationDialog::onComputeTriangulationButton(QAbstractButton *button) {
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


void TriangulationDialog::button_apply() {      
    computeTriangulationMesh();
}

void TriangulationDialog::button_ok() {
    button_apply();
}

void TriangulationDialog::button_cancel() {
}




