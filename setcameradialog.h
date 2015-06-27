#ifndef SETCAMERADIALOG_H
#define SETCAMERADIALOG_H

// pcl
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
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
    explicit SetCameraDialog(
            const pcl::visualization::Camera& camera,
            QWidget *parent = 0);

    ~SetCameraDialog();

    double getPosX() {return pos_x;}
    double getPosY() {return pos_y;}
    double getPosZ() {return pos_z;}

    double getViewX() {return view_x;}
    double getViewY() {return view_y;}
    double getViewZ() {return view_z;}

    double getUpX() {return up_x;}
    double getUpY() {return up_y;}
    double getUpZ() {return up_z;}



private:
    Ui::SetCameraDialog *ui;

    double pos_x, pos_y, pos_z,
            view_x, view_y, view_z,
            up_x, up_y, up_z;

    void button_apply();

public slots:
    void onButton(QAbstractButton *button);

signals:
    void onSetCamera(std::vector<double> vec);

};

#endif // TOURDIALOG_H
