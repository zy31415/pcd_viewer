#ifndef COLORDIALOG_H
#define COLORDIALOG_H

// pcl
#include <pcl/visualization/pcl_visualizer.h>

#include <QDialog>

#include "datamodel.h"

class PCDViewerMainWindow;

namespace Ui {
  class ColorDialog;
}

class ColorDialog : public QDialog {
    Q_OBJECT

private:
    Ui::ColorDialog* ui;
    DataModel* data_;

public:
    ColorDialog(DataModel* data,
                QWidget *parent = 0);
    virtual ~ColorDialog();

public slots:
    void onIfShowDataPoints();
    void onChangePointSize();
    void onAxisChosen();
    void onLookUpTableChosen();

};


#endif // COLORDIALOG_H
