#ifndef TRIANGULATIONDIALOG_H
#define TRIANGULATIONDIALOG_H

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


// QT
#include <QAbstractButton>
#include <QDialog>

#include "datamodel.h"


class PCDViewerMainWindow;

namespace Ui {
class TriangulationDialog;
}

/**
 * @brief Dialog for plotting meshes.
 */
class TriangulationDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TriangulationDialog(DataModel* data_, QWidget *parent = 0);
    ~TriangulationDialog();

private:
    Ui::TriangulationDialog *ui;
    DataModel* data_;

    void setParametersToDialog();
    TriangulationParameters getTriangulationParametersFromDialog();
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
