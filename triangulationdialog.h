#ifndef TRIANGULATIONDIALOG_H
#define TRIANGULATIONDIALOG_H

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>

// QT
#include <QAbstractButton>
#include <QDialog>

// This project
#include "triangulation_meshes.h"

#define PI 3.1415926

class PCDViewerMainWindow;

namespace Ui {
class TriangulationDialog;
}

class TriangulationDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TriangulationDialog(QWidget *parent = 0);
    ~TriangulationDialog();

private:
    Ui::TriangulationDialog *ui;

    pcl::PolygonMesh::Ptr meshes_;

    TriangulationParameters triangulation_parameters;

    bool if_plot_meshes;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_; // display widget
    PCDViewerMainWindow* pclViewer_; // parent widget

    void setParametersToDialog();
    void getParametersFromDialog();
    void computeTriangulationMesh();
    void button_apply();
    void button_ok();
    void button_cancel();
    void clearViewer();

public slots:
    void onComputeTriangulationButton(QAbstractButton *button);
    void setEnabled();

};

#endif // TRIANGULATIONDIALOG_H
