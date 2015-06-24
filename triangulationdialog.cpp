#include <stdio.h>
#include <string>



#include "triangulationdialog.h"
#include "../build/ui_triangulationdialog.h"

#include "pclviewer.h"



TriangulationDialog::TriangulationDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TriangulationDialog),
    k(20),
    search_radius(0.025),
    mu(2.5),
    max_NN(100),
    max_surface_angle(45.*PI/180.),
    min_angle(10.*PI/180.),
    max_angle(120.*PI/180.),
    is_normal_consistency(true),
    meshes_(new pcl::PolygonMesh),
    if_plot_meshes(false)
{
    ui->setupUi(this);
    setParametersToDialog();

    connect(ui->buttonBox, SIGNAL(clicked(QAbstractButton*)), this, SLOT(onComputeTriangulationButton(QAbstractButton*)));

    connect(ui->checkBox_if_plot_meshes, SIGNAL(toggled(bool)), this, SLOT(setEnabled()));

}

void TriangulationDialog::setParametersToDialog() {
    ui->lineEdit_k->setText(QString::number(k));
    ui->lineEdit_SearchRadius->setText(QString::number(search_radius));
    ui->lineEdit_Mu->setText(QString::number(mu));
    ui->lineEdit_MaxNN->setText(QString::number(max_NN));
    ui->lineEdit_MaxSurfaceAngle->setText(QString::number(max_surface_angle*180./PI));
    ui->lineEdit_MinAngle->setText(QString::number(min_angle*180./PI));
    ui->lineEdit_MaximumAngle->setText(QString::number(max_angle*180./PI));
    ui->checkBox_is_normal_consistency->setChecked(is_normal_consistency);
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
    k = ui->lineEdit_k->text().toInt();
    search_radius = ui->lineEdit_SearchRadius->text().toDouble();
    mu = ui->lineEdit_Mu->text().toDouble();
    max_NN = ui->lineEdit_MaxNN->text().toDouble();
    max_surface_angle = ui->lineEdit_MaxSurfaceAngle->text().toDouble()*PI/180.;
    min_angle = ui->lineEdit_MinAngle->text().toDouble()*PI/180.;
    max_angle = ui->lineEdit_MaximumAngle->text().toDouble()*PI/180.;
    is_normal_consistency = ui->checkBox_is_normal_consistency->isChecked();
    if_plot_meshes = ui->checkBox_if_plot_meshes->isChecked();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
copy_PointXYZRGBA_to_PointXYZ
(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_RGBA (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_no_RGBA->resize(cloud->size());
    for (size_t i = 0;
         i < cloud -> points.size();
         i++) {
        cloud_no_RGBA -> points[i].x = cloud -> points[i].x;
        cloud_no_RGBA -> points[i].y = cloud -> points[i].y;
        cloud_no_RGBA -> points[i].z = cloud -> points[i].z;
    }
    return cloud_no_RGBA;
}

void TriangulationDialog::computeTriangulationMesh() {

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ = ((PCLViewer*)parentWidget())->getPointsData();


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_RGBA = copy_PointXYZRGBA_to_PointXYZ(cloud_);

    // Normal estimation*
    int nr_cores = boost::thread::hardware_concurrency();
    pcl::NormalEstimationOMP <pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    tree->setInputCloud (cloud_no_RGBA);
    n.setInputCloud (cloud_no_RGBA);
    n.setSearchMethod (tree);
    n.setKSearch (k);
    n.setNumberOfThreads(nr_cores);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud_no_RGBA, *normals, *cloud_with_normals);
    // cloud_with_normals = cloud + normals;

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (search_radius);

    // Set typical values for the parameters
    gp3.setMu (mu);
    gp3.setMaximumNearestNeighbors (max_NN);
    gp3.setMaximumSurfaceAngle(max_surface_angle); // 45 degrees
    gp3.setMinimumAngle(min_angle); // 10 degrees
    gp3.setMaximumAngle(max_angle); // 120 degrees
    gp3.setNormalConsistency(is_normal_consistency);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (*meshes_);

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
    PCLViewer* pclviewer_ = (PCLViewer*)parentWidget();
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ = pclviewer_ -> getViewer();

    if (if_plot_meshes) {
        getParametersFromDialog();
        computeTriangulationMesh();

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ = pclviewer_ -> getPointsData();

        if (!viewer_->updatePolygonMesh<pcl::PointXYZRGBA>(cloud_, meshes_->polygons, "polygon"))
            viewer_->addPolygonMesh<pcl::PointXYZRGBA>(cloud_, meshes_->polygons, "polygon");

    } else {
        viewer_ -> removePolygonMesh("polygon");
    }

    pclviewer_ -> update();

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


