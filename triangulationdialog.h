#ifndef TRIANGULATIONDIALOG_H
#define TRIANGULATIONDIALOG_H

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/conversions.h>

// QT
#include <QAbstractButton>
#include <QDialog>

#define PI 3.1415926

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

    int k, max_NN;
    double search_radius, mu, max_surface_angle, min_angle, max_angle;
    bool is_normal_consistency, if_plot_meshes;

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
