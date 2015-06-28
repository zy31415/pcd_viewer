#include "pcdviewermainwindow.h"
#include "../build/ui_pcdviewermainwindow.h"

#include <QThread>
#include <string>
#include <boost/filesystem.hpp>

#include "colordialog.h"
#include "worker.h"
#include "setcameradialog.h"
#include "colordialog.h"
#include "helpdialog.h"

PCDViewerMainWindow::PCDViewerMainWindow (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCDViewerMainWindow),
    viewer_(new pcl::visualization::PCLVisualizer ("viewer", false)),
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

    onDrawCloudData();
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

    // connect View -> Tour
    connect(ui->actionTour, SIGNAL(triggered()), this, SLOT(onTour()));

    connect(ui->actionSnapshot, SIGNAL(triggered()), this, SLOT(onSnapshot()));

    // set camera position
    connect(ui->actionSet_Camera, SIGNAL(triggered()), this, SLOT(onSetCameraButtonPressed()));

    // help info.
    connect(ui->actionHelpInfo, SIGNAL(triggered(bool)), this, SLOT(onHelpInfo()));

    // connect DataModel to PCDViewerMainWindow, data changing signals
    connect(data_, SIGNAL(onDrawCloudData()), this, SLOT(onDrawCloudData()));
    connect(data_, SIGNAL(onDrawPointSize()), this, SLOT(onDrawPointSize()));
    connect(data_, SIGNAL(onIfShowDataPoints()), this, SLOT(onIfShowDataPoints()));
    connect(data_, SIGNAL(onDrawMeshes()), this, SLOT(onDrawMeshes()));



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

void PCDViewerMainWindow::onAbout() {
    QMessageBox msgBox(this);
    msgBox.setText("This is a point cloud file GUI viewer.<br>"
                   "<b>Author</b>: Yang Zhang <br>"
                   "<b>Email</b>: <a href='mailto:zy31415@gmail.com?Subject=About point cloud viewer' target='_top'>zy31415@gmail.com</a><br>");
    msgBox.exec();
}

void PCDViewerMainWindow::onColorMode() {
    ColorDialog cdialog(data_);
    cdialog.exec();
}


void PCDViewerMainWindow::onTriangulation(){
    TriangulationDialog tridia(data_, this);
    tridia.exec();
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

const QRect& PCDViewerMainWindow::getSnapshotGeometry()
{
    return ui->qvtkWidget->geometry();
}

void PCDViewerMainWindow::renderASnapshot(QPixmap& pixmap,
            const QPoint & targetOffset,
            const QRegion & sourceRegion)
{
    ui->qvtkWidget->render(&pixmap, targetOffset, sourceRegion);
}

void PCDViewerMainWindow::onSetCameraButtonPressed()
{
    std::vector<pcl::visualization::Camera> cameras;
    viewer_ -> getCameras(cameras);

    SetCameraDialog dialog(cameras[0], this);

    connect(&dialog, SIGNAL(onSetCamera(std::vector<double>)), this, SLOT(onSetCamera(std::vector<double>)));

    dialog.exec();
}

void PCDViewerMainWindow::onSetCamera(std::vector<double> pars)
{
    viewer_ -> setCameraPosition(pars[0], pars[1], pars[2],
                                 pars[3], pars[4], pars[5],
                                 pars[6], pars[7], pars[8]);
}

void PCDViewerMainWindow::disableResize()
{
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

void PCDViewerMainWindow::onDrawMeshes()
{
    if (data_->getIfShowMeshes()) {
        if (viewer_->contains("meshes"))
            viewer_->updatePolygonMesh<pcl::PointXYZRGBA>(
                        data_->getCloud(),
                        data_->getMesh()->polygons,
                        "meshes");
        else
            viewer_->addPolygonMesh<pcl::PointXYZRGBA>(
                        data_->getCloud(),
                        data_->getMesh()->polygons,
                        "meshes");
    } else
        if (viewer_->contains("meshes"))
            viewer_ -> removePolygonMesh("meshes");

    ui->qvtkWidget->update();
}

void PCDViewerMainWindow::onHelpInfo()
{
    HelpDialog helpDialog(this);
    helpDialog.exec();
}
