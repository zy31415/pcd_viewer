#include "pcdviewermainwindow.h"
#include "../build/ui_pcdviewermainwindow.h"

#include "colordialog.h"
#include "worker.h"
#include "setcameradialog.h"

#include <QThread>
#include <string>

#include <boost/filesystem.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

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

    //ui->centralwidget->layout()->addWidget(new );

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

    cdialog_ = new ColorDialog(this);
}

PCDViewerMainWindow::~PCDViewerMainWindow ()
{
    delete ui;
    delete data_;
    delete cdialog_;
    delete triangulationDialog_;
    delete td_;
}

void PCDViewerMainWindow::connect_SIGNAL_SLOT() {
    // Connect "Load" and "Save" buttons and their functions
    connect(ui->action_Open, SIGNAL(triggered()), this, SLOT(loadFileButtonPressed ()));
    connect(ui->action_Save, SIGNAL(triggered()), this, SLOT(saveFileButtonPressed ()));

    // Connet File -> Close
    connect(ui->action_Close, SIGNAL(triggered()), this, SLOT(close()));

    // Connet Help -> About
    connect(ui->action_About, SIGNAL(triggered()), this, SLOT(about()));

    // connet View -> Color Mode
    connect(ui->action_Color_Mode, SIGNAL(triggered()), this, SLOT(color_mode_dialog()));

    // connet View -> Triangulation
    connect(ui->action_Triangulation, SIGNAL(triggered()), this, SLOT(onTriangulation()));

    // connet View -> Tour
    connect(ui->actionTour, SIGNAL(triggered()), this, SLOT(onTour()));

    connect(ui->actionSnapshot, SIGNAL(triggered()), this, SLOT(onSnapshot()));

    connect(ui->actionSet_Camera, SIGNAL(triggered()), this, SLOT(onSetCamera()));

    connect(data_, SIGNAL(updateViewer()), this, SLOT(updateViewer()));

}

void PCDViewerMainWindow::setUpQVTKWindow()
{
    viewer_->setBackgroundColor (0.2, 0.2, 0.2);
    ui->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
    viewer_->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();
}

void PCDViewerMainWindow::loadFileButtonPressed ()
{
    QString filename = QFileDialog::getOpenFileName(
                this,
                tr ("Open point cloud"),
                ".",
                tr ("Point cloud data (*.pcd *.ply)"));

    data_->readPCDFile(filename);

//    // clear screen:
//    viewer_->removeAllShapes();
//    viewer_->removeAllPointClouds();

//    // add new point could
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud_);
//    viewer_->addPointCloud (cloud_, rgb, "cloud");
//    viewer_->resetCamera ();
//    ui->qvtkWidget->update ();

//    // Update widgests:
//    triangulationDialog_->close();
//    delete triangulationDialog_;
//    triangulationDialog_ = new TriangulationDialog(this);

//    cdialog_->close();
//    delete cdialog_;
//    cdialog_ = new ColorDialog(this);

}


void PCDViewerMainWindow::saveFileButtonPressed ()
{
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *data_->getCloud());
  else if (filename.endsWith (".thisply", Qt::CaseInsensitive))
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *data_->getCloud());
  else
  {
    filename.append(".ply");
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *data_->getCloud());
  }

  if (return_status != 0)
  {
    PCL_ERROR("Error writing point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }
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



void PCDViewerMainWindow::about() {
    QMessageBox msgBox(this);
    msgBox.setText("This is a poit cloud file GUI viewer.<br>"
                   "<b>Author</b>: Yang Zhang <br>"
                   "<b>Email</b>: <a href='mailto:zy31415@gmail.com?Subject=About point cloud viewer' target='_top'>zy31415@gmail.com</a><br>");
    msgBox.exec();
}

void PCDViewerMainWindow::color_mode_dialog() {
    cdialog_->show();
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
    triangulationDialog_->show();
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

void PCDViewerMainWindow::updateViewer() {
    // clear screen:
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(data_->getCloud());

    if (viewer_->contains("cloud"))
        viewer_->updatePointCloud (data_->getCloud(), rgb, "cloud");
    else
        viewer_->addPointCloud (data_->getCloud(), rgb, "cloud");

    viewer_->resetCamera ();
    ui->qvtkWidget->update ();


    // Update widgests:
//    triangulationDialog_->close();
//    delete triangulationDialog_;
//    triangulationDialog_ = new TriangulationDialog(this);

//    cdialog_->close();
//    delete cdialog_;
//    cdialog_ = new ColorDialog(this);

}
