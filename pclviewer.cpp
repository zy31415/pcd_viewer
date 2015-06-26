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

void PCLViewer::colorCloudDistances () {
  // Find the minimum and maximum values along the selected axis
  double min, max;
  min = bb.get_min(filtering_axis_);
  max = bb.get_max(filtering_axis_);

  // Compute LUT scaling to fit the full histogram spectrum
  double lut_scale = 255.0 / (max - min);  // max is 255, min is 0

  if (min == max)  // In case the cloud is flat on the chosen direction (x,y or z)
    lut_scale = 1.0;  // Avoid rounding error in boost

  for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator cloud_it = cloud_->begin ();
       cloud_it != cloud_->end ();
       cloud_it++)
  {
    int value;
    switch (filtering_axis_)
    {
      case 0:  // x
        value = boost::math::iround ( (cloud_it->x - min) * lut_scale);  // Round the number to the closest integer
        break;
      case 1:  // y
        value = boost::math::iround ( (cloud_it->y - min) * lut_scale);
        break;
      default:  // z
        value = boost::math::iround ( (cloud_it->z - min) * lut_scale);
        break;
    }

    // Apply color to the cloud
    switch (color_mode_)
    {
      case 0:
        // Blue (= min) -> Red (= max)Ui_ColorDialog
        cloud_it->r = value;
        cloud_it->g = 0;
        cloud_it->b = 255 - value;
        break;
      case 1:
        // Green (= min) -> Magenta (= max)
        cloud_it->r = value;
        cloud_it->g = 255 - value;
        cloud_it->b = value;
        break;
      case 2:
        // White (= min) -> Red (= max)
        cloud_it->r = 255;
        cloud_it->g = 255 - value;
        cloud_it->b = 255 - value;
        break;
      case 3:
        // Grey (< 128) / Red (> 128)
        if (value > 128)
        {
          cloud_it->r = 255;http://stackoverflow.com/questions/13184555/creating-a-qwidget-in-a-non-gui-thread
          cloud_it->g = 0;
          cloud_it->b = 0;
        }
        else
        {
          cloud_it->r = 128;
          cloud_it->g = 128;
          cloud_it->b = 128;
        }
        break;
      default:
        // Blue -> Green -> Red (~ rainbow)
        cloud_it->r = value > 128 ? (value - 128) * 2 : 0;  // r[128] = 0, r[255] = 255
        cloud_it->g = value < 128 ? 2 * value : 255 - ( (value - 128) * 2);  // g[0] = 0, g[128] = 255, g[255] = 0
        cloud_it->b = value < 128 ? 255 - (2 * value) : 0;  // b[0] = 255, b[128] = 0
    }
  }
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
