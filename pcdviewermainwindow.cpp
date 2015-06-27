#include "pcdviewermainwindow.h"
#include "../build/ui_pcdviewermainwindow.h"

#include <QThread>
#include <string>
#include <boost/filesystem.hpp>

#include "colordialog.h"
#include "worker.h"
#include "setcameradialog.h"
#include "colordialog.h"


PCDViewerMainWindow::PCDViewerMainWindow (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCDViewerMainWindow),
    // Note: the order of initialization is very important.
    //  viewer_ should be initialized before other dialogs.
    viewer_(new pcl::visualization::PCLVisualizer ("viewer", false)),
    triangulationDialog_(new TriangulationDialog(this)),
    td_(new TourDialog(this))
{
    ui->setupUi (this);

    this->setWindowTitle ("PCD file viewer");

    data_ = new DataModel(this);

    // add status bar message
    ui->statusBar->showMessage("Open a .pcd file to start.");


    // Set up the QVTK window
    setUpQVTKWindow();

    connect_SIGNAL_SLOT();

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(data_->getCloud());
    viewer_->addPointCloud(data_->getCloud(), rgb, "cloud");
    viewer_->resetCamera();
    ui->qvtkWidget->update();
}

PCDViewerMainWindow::~PCDViewerMainWindow ()
{
    delete ui;
    delete data_;
    delete triangulationDialog_;
    delete td_;
}

void PCDViewerMainWindow::connect_SIGNAL_SLOT() {
    // Connect "Load" and "Save" buttons with their slots
    connect(ui->action_Open, SIGNAL(triggered()), this, SLOT(onLoadFileButton ()));
    connect(ui->action_Save, SIGNAL(triggered()), this, SLOT(onSaveFileButton ()));

    // Connect File -> Close
    connect(ui->action_Close, SIGNAL(triggered()), this, SLOT(onClose()));

    // Connect Help -> About
    connect(ui->action_About, SIGNAL(triggered()), this, SLOT(onAbout()));

    // connect View -> Color Mode
    connect(ui->action_Color_Mode, SIGNAL(triggered()), this, SLOT(onColorMode()));

    // connet View -> Triangulation
    connect(ui->action_Triangulation, SIGNAL(triggered()), this, SLOT(onTriangulation()));

    // connet View -> Tour
    connect(ui->actionTour, SIGNAL(triggered()), this, SLOT(onTour()));

    connect(ui->actionSnapshot, SIGNAL(triggered()), this, SLOT(onSnapshot()));

    connect(ui->actionSet_Camera, SIGNAL(triggered()), this, SLOT(onSetCamera()));


    // connect DataModel to PCDViewerMainWindow, data changing signals
    connect(data_, SIGNAL(onDrawCloudData()), this, SLOT(onDrawCloudData()));
    connect(data_, SIGNAL(onDrawPointSize()), this, SLOT(onDrawPointSize()));
    connect(data_, SIGNAL(onIfShowDataPoints()), this, SLOT(onIfShowDataPoints()));
    connect(data_, SIGNAL(onIfShowMeshes()), this, SLOT(onIfShowMeshes()));


}

void PCDViewerMainWindow::setUpQVTKWindow()
{
    viewer_->setBackgroundColor (0.2, 0.2, 0.2);
    ui->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
    viewer_->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();
}

void PCDViewerMainWindow::onLoadFileButton ()
{
    QString filename = QFileDialog::getOpenFileName(
                this,
                tr ("Open point cloud"),
                ".",
                tr ("Point cloud data (*.pcd *.ply)"));

    data_->readPCDFile(filename);
}


void PCDViewerMainWindow::onSaveFileButton ()
{    
    QString filename = QFileDialog::getSaveFileName(
              this,
              tr ("Open point cloud"),
              "pointcloud.pcd",
              tr ("Point cloud data (*.pcd *.ply)"));

    data_->savePCDFile(filename);
}


void PCDViewerMainWindow::axisChosen() {
//    filtering_axis_= cdialog_ -> get_color_changing_axis();
//    updatePointCloud();
}

void PCDViewerMainWindow::lookUpTableChosen() {
//    color_mode_ = cdialog_ -> get_look_up_table();
//    updatePointCloud();
}

void PCDViewerMainWindow::updatePointCloud() {
//    colorCloudDistances ();
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud_);
//    viewer_->updatePointCloud (cloud_, rgb, "cloud");
//    ui->qvtkWidget->update ();
}



void PCDViewerMainWindow::onAbout() {
    QMessageBox msgBox(this);
    msgBox.setText("This is a poit cloud file GUI viewer.<br>"
                   "<b>Author</b>: Yang Zhang <br>"
                   "<b>Email</b>: <a href='mailto:zy31415@gmail.com?Subject=About point cloud viewer' target='_top'>zy31415@gmail.com</a><br>");
    msgBox.exec();
}

void PCDViewerMainWindow::onColorMode() {
    ColorDialog cdialog(data_);
    cdialog.exec();
}

void PCDViewerMainWindow::removePointsCloudFromView() {
    viewer_->removePointCloud("cloud");
    ui->qvtkWidget->update();
}

void PCDViewerMainWindow::addPointsCloudToView() {
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud_);
//    viewer_->addPointCloud(cloud_, rgb, "cloud");
//    ui->qvtkWidget->update();
}

void PCDViewerMainWindow::onTriangulation(){
    TriangulationDialog tridia(data_);
    tridia.exec();
}

void PCDViewerMainWindow::update() {
    ui->qvtkWidget->update();
}


void PCDViewerMainWindow::onTour() {
    td_->show();
}

void PCDViewerMainWindow::onSnapshot(){
    QFileDialog fdialog(this);
    fdialog.setDefaultSuffix("png");

    QString filter = "Images (*.bmp *.gif *.jpg *.jpeg *.png);;All files (*.*)";
    QString filename = fdialog.getSaveFileName(
                this, tr ("Save snapshot"), "snapshot.png",
                filter, &filter);

    boost::filesystem::path p(filename.toStdString());

    std::string extension(p.extension().string());

    if ( extension == "")
        filename += ".png";


    QRect rect = getSnapshotGeometry();
    QPixmap pixmap(rect.size());
    renderASnapshot(pixmap, QPoint(), QRegion(rect));
    pixmap.save(filename);

}

const QRect& PCDViewerMainWindow::getSnapshotGeometry() {
    return ui->qvtkWidget->geometry();
}

void PCDViewerMainWindow::renderASnapshot(QPixmap& pixmap,
            const QPoint & targetOffset,
            const QRegion & sourceRegion) {
//    ui->qvtkWidget->render(&updateViewerpixmap, targetOffset, sourceRegion);
}

void PCDViewerMainWindow::onSetCamera() {
    SetCameraDialog dialog(this);
    dialog.exec();
}

void PCDViewerMainWindow::disableResize() {
    this->setFixedSize(this->size());
}


void PCDViewerMainWindow::enableResize() {
    this->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

}

void PCDViewerMainWindow::onDrawCloudData() {
    // add / update point cloud data:
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(data_->getCloud());

    if (viewer_->contains("cloud"))
        viewer_->updatePointCloud (data_->getCloud(), rgb, "cloud");
    else
        viewer_->addPointCloud (data_->getCloud(), rgb, "cloud");

    // set point size
    viewer_->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                data_->getPointSize(),
                "cloud");

    viewer_->resetCamera ();
    ui->qvtkWidget->update ();

}

void PCDViewerMainWindow::onDrawPointSize(){
    viewer_->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                data_->getPointSize(),
                "cloud");
    std::cout<<data_->getPointSize()<<std::endl;
    ui->qvtkWidget->update ();
}

void PCDViewerMainWindow::onIfShowDataPoints(){
    if (data_->getIfShowDataPoints())
        onDrawCloudData();
    else
        if (viewer_->contains("cloud"))
            viewer_->removePointCloud("cloud");

    ui->qvtkWidget->update();
}

void PCDViewerMainWindow::onIfShowMeshes()
{
    if (data_->getIfShowMeshes()) {
        if (viewer_->contains("meshes"))
            viewer_->updatePolygonMesh<pcl::PointXYZRGBA>(cloud_, meshes_->polygons, "meshes");
        else
            viewer_->addPolygonMesh<pcl::PointXYZRGBA>(cloud_, meshes_->polygons, "meshes");
    } else
        if (viewer_->contains("meshes"))
            viewer_ -> removePolygonMesh("meshes");

}

void PCDViewerMainWindow::onDrawMeshes()
{
    if (viewer_->contains("meshes"))
        viewer_->updatePolygonMesh<pcl::PointXYZRGBA>(cloud_, meshes_->polygons, "meshes");
    else
        viewer_->addPolygonMesh<pcl::PointXYZRGBA>(cloud_, meshes_->polygons, "meshes");

}
