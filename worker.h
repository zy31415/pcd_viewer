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

signals:
    void moveOneStep();
    void finished();
    void error(QString err);

public slots:
    void process();
};

#endif // WORKER_H
