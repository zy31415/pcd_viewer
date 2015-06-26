#ifndef PCLVIEWER_H
#define PCLVIEWER_H

// Qt
#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>

// Boost
#include <boost/math/special_functions/round.hpp>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// this project
#include "colordialog.h"
#include "triangulationdialog.h"
#include "boundingbox.h"
#include "tourdialog.h"

// typedef pcl::PointXYZRGBA PointT;
// typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudT;

namespace Ui {
  class PCLViewer;
}

class PCLViewer : public QMainWindow {
    Q_OBJECT

private:
    /** @brief ui pointer */
    Ui::PCLViewer *ui;

    /** @brief The PCL visualizer object */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

    /** @brief The point cloud displayed */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
    BoundingBox bb;

    ColorDialog* cdialog_;
    TriangulationDialog* triangulationDialog_;

    TourDialog* td_;

    void setUpQVTKWindow(void);

    void connect_SIGNAL_SLOT();

public:
    /** @brief Constructor */
    explicit PCLViewer (QWidget *parent = 0);

    /** @brief Destructor */
    ~PCLViewer ();

    void removePointsCloudFromView();
    void addPointsCloudToView();
    void addMeshes();
    void update();

    inline pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPointsData() {
        return cloud_;
    }

    inline boost::shared_ptr<pcl::visualization::PCLVisualizer> getViewer() {
        return viewer_;
    }

    inline BoundingBox& getBoundingBox () {return bb;}

    void renderASnapshot(QPixmap& pixmap,
                const QPoint & targetOffset= QPoint(),
                const QRegion & sourceRegion = QRegion()
            );

    const QRect& getSnapshotGeometry();

    void disableResize();

    void enableResize();


protected:

    void updatePointCloud();

public slots:
    /** @brief Triggered whenever the "Save file" button is clicked */
    void saveFileButtonPressed ();

    /** @brief Triggered whenever the "Load file" button is clicked */
    void loadFileButtonPressed ();

    /** @brief Triggered whenever a button in the "Color on axis" group is clicked */
    void axisChosen ();

    /** @brief Triggered whenever a button in the "Color mode" group is clicked */
    void lookUpTableChosen ();

    /** @brief About the program.*/
    void about();

    void color_mode_dialog();

    void onTriangulation();

    void onTour();

    void onSnapshot();

private slots:
    void onSetCamera();

};

#endif // PCLVIEWER_H
