#ifndef PCDVIEWERMAINWINDOW_H
#define PCDVIEWERMAINWINDOW_H

// Qt
#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>

// Point Cloud Library
#include <pcl/visualization/pcl_visualizer.h>

// Boost
#include <boost/math/special_functions/round.hpp>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// this project
#include "triangulationdialog.h"
#include "boundingbox.h"
#include "tourdialog.h"
#include "datamodel.h"

// typedef pcl::PointXYZRGBA PointT;
// typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudT;

class DataModel;

namespace Ui {
  class PCDViewerMainWindow;
}

class PCDViewerMainWindow : public QMainWindow {
    Q_OBJECT

private:
    /** @brief ui pointer */
    Ui::PCDViewerMainWindow *ui;

    /** @brief The PCL visualizer object */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

    DataModel* data_;

    TriangulationDialog* triangulationDialog_;

    TourDialog* td_;

    cv::VideoWriter* videoWriter_;

    void setUpQVTKWindow(void);

    void connect_SIGNAL_SLOT();

public:
    /** @brief Constructor */
    explicit PCDViewerMainWindow (QWidget *parent = 0);

    /** @brief Destructor */
    ~PCDViewerMainWindow ();

    void removePointsCloudFromView();
    void addPointsCloudToView();
    void addMeshes();
    void update();


    inline boost::shared_ptr<pcl::visualization::PCLVisualizer> getViewer() {
        return viewer_;
    }

    DataModel* getData() {return data_;}


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
    void onSaveFileButton ();

    /** @brief Triggered whenever the "Load file" button is clicked */
    void onLoadFileButton ();

    /** @brief About the program.*/
    void onAbout();

    void onClose() {close();}

    void onColorMode();

    void onTriangulation();

    void onTour();

    void onSnapshot();

    void onIfShowDataPoints();

    void onDrawCloudData();
    void onDrawPointSize();
    void onDrawMeshes();

    void onSetCamera(std::vector<double>);


private slots:
    void onSetCameraButtonPressed();

};

#endif // PCLVIEWER_H
