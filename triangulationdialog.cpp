#include <stdio.h>
#include <string>

#include "triangulationdialog.h"
#include "../build/ui_triangulationdialog.h"

#include "pcdviewermainwindow.h"

TriangulationDialog::TriangulationDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TriangulationDialog),
    if_plot_meshes(false)
{
    ui->setupUi(this);
    setParametersToDialog();

    meshes_.reset(new pcl::PolygonMesh);

    connect(ui->buttonBox, SIGNAL(clicked(QAbstractButton*)), this, SLOT(onComputeTriangulationButton(QAbstractButton*)));
    connect(ui->checkBox_if_plot_meshes, SIGNAL(toggled(bool)), this, SLOT(setEnabled()));

    pclViewer_ = (PCDViewerMainWindow*)parentWidget();
    viewer_ = pclViewer_ -> getViewer();
}

void TriangulationDialog::setParametersToDialog() {
    ui->lineEdit_k->setText(QString::number(triangulation_parameters.k));
    ui->lineEdit_SearchRadius->setText(QString::number(triangulation_parameters.search_radius));
    ui->lineEdit_Mu->setText(QString::number(triangulation_parameters.mu));
    ui->lineEdit_MaxNN->setText(QString::number(triangulation_parameters.max_NN));
    ui->lineEdit_MaxSurfaceAngle->setText(QString::number(triangulation_parameters.max_surface_angle*180./PI));
    ui->lineEdit_MinAngle->setText(QString::number(triangulation_parameters.min_angle*180./PI));
    ui->lineEdit_MaximumAngle->setText(QString::number(triangulation_parameters.max_angle*180./PI));
    ui->checkBox_is_normal_consistency->setChecked(triangulation_parameters.is_normal_consistency);

    ui->checkBox_if_plot_meshes->setChecked(if_plot_meshes);

    setEnabled();
}

void TriangulationDialog::setEnabled() {
    if_plot_meshes = ui->checkBox_if_plot_meshes->isChecked();
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

void TriangulationDialog::getParametersFromDialog() {
    triangulation_parameters.k = ui->lineEdit_k->text().toInt();
    triangulation_parameters.search_radius = ui->lineEdit_SearchRadius->text().toDouble();
    triangulation_parameters.mu = ui->lineEdit_Mu->text().toDouble();
    triangulation_parameters.max_NN = ui->lineEdit_MaxNN->text().toDouble();
    triangulation_parameters.max_surface_angle = ui->lineEdit_MaxSurfaceAngle->text().toDouble()*PI/180.;
    triangulation_parameters.min_angle = ui->lineEdit_MinAngle->text().toDouble()*PI/180.;
    triangulation_parameters.max_angle = ui->lineEdit_MaximumAngle->text().toDouble()*PI/180.;
    triangulation_parameters.is_normal_consistency = ui->checkBox_is_normal_consistency->isChecked();
    if_plot_meshes = ui->checkBox_if_plot_meshes->isChecked();
}

void TriangulationDialog::computeTriangulationMesh() {

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ = pclViewer_->getPointsData();

    compute_triangulation_meshes(
                cloud_,
                meshes_,
                triangulation_parameters
                );
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
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ = pclViewer_ -> getPointsData();

    if (if_plot_meshes) {
        getParametersFromDialog();
        computeTriangulationMesh();

        if (viewer_->contains("polygon"))
            viewer_->updatePolygonMesh<pcl::PointXYZRGBA>(cloud_, meshes_->polygons, "polygon");
        else
            viewer_->addPolygonMesh<pcl::PointXYZRGBA>(cloud_, meshes_->polygons, "polygon");

    } else
        if (viewer_->contains("polygon"))
            viewer_ -> removePolygonMesh("polygon");

    pclViewer_ -> update();

}

void TriangulationDialog::button_ok() {
    button_apply();
}

void TriangulationDialog::button_cancel() {
    setParametersToDialog();
}

TriangulationDialog::~TriangulationDialog() {
    delete ui;
}


