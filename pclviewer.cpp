#include "pclviewer.h"
#include "../build/ui_pclviewer.h"

#include "colordialog.h"
#include "worker.h"
#include "setcameradialog.h"

#include <QThread>
#include <string>

#include <boost/filesystem.hpp>

PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer),
    filtering_axis_ (1),  // = y
    color_mode_ (4), // = Rainbow
    // Note: the order of initialization is very important.
    //  viewer_ should be initialized before other dialogs.
    viewer_(new pcl::visualization::PCLVisualizer ("viewer", false)),
    triangulationDialog_(new TriangulationDialog(this)),
    td_(new TourDialog(this)),
    cloud_ (new pcl::PointCloud<pcl::PointXYZRGBA>)
{

    ui->setupUi (this);
    this->setWindowTitle ("PCD viewer");

    // add status bar message
    ui->statusBar->showMessage("Open a .pcd file to start.");


    // Set up the QVTK window
    setUpQVTKWindow();


    connect_SIGNAL_SLOT();


    // The number of points in the cloud
    cloud_->resize(500);

    // Fill the cloud with random points
    for (size_t i = 0; i < cloud_->points.size (); i++)
    {
        cloud_->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    bb.update(cloud_);

    // Color the randomly generated cloud
    colorCloudDistances();

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud_);
    viewer_->addPointCloud(cloud_, rgb, "cloud");
    viewer_->resetCamera();
    ui->qvtkWidget->update();

    cdialog_ = new ColorDialog(this);
}

PCLViewer::~PCLViewer ()
{
    delete ui;
    delete cdialog_;
    delete triangulationDialog_;
    delete td_;
}

void PCLViewer::connect_SIGNAL_SLOT() {
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


}

void PCLViewer::setUpQVTKWindow() {
    viewer_->setBackgroundColor (0.2, 0.2, 0.2);
    ui->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
    viewer_->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();
}

void PCLViewer::loadFileButtonPressed ()
{
    QString filename = QFileDialog::getOpenFileName(
                this,
                tr ("Open point cloud"),
                ".",
                tr ("Point cloud data (*.pcd *.ply)"));

    PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);

    if (filename.isEmpty ())
        return;

    int return_status;
    if (filename.endsWith (".pcd", Qt::CaseInsensitive))
        return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloud_tmp);
    else
        return_status = pcl::io::loadPLYFile (filename.toStdString (), *cloud_tmp);

    if (return_status != 0) {
        PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
        return;
    }

    // If point cloud contains NaN values, remove them before updating the visualizer point cloud
    if (cloud_tmp->is_dense)
        pcl::copyPointCloud (*cloud_tmp, *cloud_);
    else {
        PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
        std::vector<int> vec;
        pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_, vec);
    }

    //bb.update(cloud_);

    colorCloudDistances ();

    // clear screen:
    viewer_->removeAllShapes();
    viewer_->removeAllPointClouds();

    // add new point could
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud_);
    viewer_->addPointCloud (cloud_, rgb, "cloud");
    viewer_->resetCamera ();
    ui->qvtkWidget->update ();

    // Update widgests:
    triangulationDialog_->close();
    delete triangulationDialog_;
    triangulationDialog_ = new TriangulationDialog(this);

    cdialog_->close();
    delete cdialog_;
    cdialog_ = new ColorDialog(this);

}


void PCLViewer::saveFileButtonPressed ()
{
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *cloud_);
  else if (filename.endsWith (".thisply", Qt::CaseInsensitive))
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  else
  {
    filename.append(".ply");
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  }

  if (return_status != 0)
  {
    PCL_ERROR("Error writing point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }
}

void PCLViewer::axisChosen() {
    filtering_axis_= cdialog_ -> get_color_changing_axis();
    updatePointCloud();
}

void PCLViewer::lookUpTableChosen() {
    color_mode_ = cdialog_ -> get_look_up_table();
    updatePointCloud();
}

void PCLViewer::updatePointCloud() {
    colorCloudDistances ();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud_);
    viewer_->updatePointCloud (cloud_, rgb, "cloud");
    ui->qvtkWidget->update ();
}



void PCLViewer::about() {
    QMessageBox msgBox(this);
    msgBox.setText("This is a poit cloud file GUI viewer.<br>"
                   "<b>Author</b>: Yang Zhang <br>"
                   "<b>Email</b>: <a href='mailto:zy31415@gmail.com?Subject=About point cloud viewer' target='_top'>zy31415@gmail.com</a><br>");
    msgBox.exec();
}

void PCLViewer::color_mode_dialog() {
    cdialog_->show();
}

void PCLViewer::removePointsCloudFromView() {
    viewer_->removePointCloud("cloud");
    ui->qvtkWidget->update();
}

void PCLViewer::addPointsCloudToView() {
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud_);
    viewer_->addPointCloud(cloud_, rgb, "cloud");
    ui->qvtkWidget->update();
}

void PCLViewer::onTriangulation(){
    triangulationDialog_->show();
}

void PCLViewer::update() {
    ui->qvtkWidget->update();
}


void PCLViewer::onTour() {
    td_->show();
}

void PCLViewer::onSnapshot(){
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

const QRect& PCLViewer::getSnapshotGeometry() {
    return ui->qvtkWidget->geometry();
}

void PCLViewer::renderASnapshot(QPixmap& pixmap,
            const QPoint & targetOffset,
            const QRegion & sourceRegion) {
    ui->qvtkWidget->render(&pixmap, targetOffset, sourceRegion);
}

void PCLViewer::onSetCamera() {
    SetCameraDialog dialog(this);
    dialog.exec();
}

void PCLViewer::disableResize() {
    this->setFixedSize(this->size());
}


void PCLViewer::enableResize() {
    this->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

}
