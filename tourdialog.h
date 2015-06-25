#ifndef TOURDIALOG_H
#define TOURDIALOG_H

// pcl
#include <pcl/visualization/pcl_visualizer.h>

// QT
#include <QDialog>
#include <QAbstractButton>

// This project

class PCLViewer;

namespace Ui {
class TourDialog;
}

class TourDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TourDialog(QWidget *parent = 0);
    ~TourDialog();

private:
    Ui::TourDialog *ui;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_; // display widget
    PCLViewer* pclViewer_; // parent widget

    double alpha;

    void button_apply();
    void button_cancel() {}
    void button_ok() {}

public slots:
    void onButton(QAbstractButton *button);

    void errorString(QString str) { std::cout<<str.toStdString()<<std::endl;}

    void tour();

signals:
    void click_apply();
};

#endif // TOURDIALOG_H
