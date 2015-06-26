#ifndef COLORDIALOG_H
#define COLORDIALOG_H

// pcl
#include <pcl/visualization/pcl_visualizer.h>

#include <QDialog>

class PCDViewerMainWindow;

namespace Ui {
  class ColorDialog;
}

class ColorDialog : public QDialog {
    Q_OBJECT

private:
    Ui::ColorDialog* ui;
    int point_size;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_; // display widget
    PCDViewerMainWindow* pclViewer_; // parent widget

public:
    ColorDialog(QWidget *parent = 0);
    virtual ~ColorDialog();

    static int DEFAULT_POINT_SIZE;

    int get_color_changing_axis();
    int get_look_up_table();

public slots:
    void onIfShowDataPoints();
    void onChangePointSize();
};


#endif // COLORDIALOG_H
