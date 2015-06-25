#ifndef WORKER_H
#define WORKER_H

#include <QObject>

// pcl
#include <pcl/visualization/pcl_visualizer.h>

#include "boundingbox.h"

class Worker : public QObject
{
    Q_OBJECT
public:
    explicit Worker(QObject *parent = 0) {}
    ~Worker(){}

    void setViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_) {
        this->viewer_ = viewer_;}

    void setBoundingBox(BoundingBox bb) {
        this->bb = bb;}

signals:
    void finished();
    void error(QString err);

public slots:
    void process();

private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_; // display widget
    BoundingBox bb;
};

#endif // WORKER_H
