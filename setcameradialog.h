#ifndef SETCAMERADIALOG_H
#define SETCAMERADIALOG_H

// pcl
#include <pcl/visualization/pcl_visualizer.h>

// QT
#include <QDialog>
#include <QAbstractButton>

class PCDViewerMainWindow;

namespace Ui {
class SetCameraDialog;
}

class SetCameraDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SetCameraDialog(QWidget *parent = 0);
    ~SetCameraDialog();

private:
    Ui::SetCameraDialog *ui;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_; // display widget
    PCDViewerMainWindow* pclViewer_; // parent widget

    void button_apply();

public slots:
    void onButton(QAbstractButton *button);
};

#endif // TOURDIALOG_H
