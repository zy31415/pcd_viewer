#ifndef WORKER_H
#define WORKER_H

#include <QObject>
#include <QReadWriteLock>

// pcl
#include <pcl/visualization/pcl_visualizer.h>

#include "boundingbox.h"

/**
 * @brief This class cooperates with the TourDialog to do the scene rotation
 */
class Worker : public QObject
{
    Q_OBJECT

private:
    int* num_worker_;
    QReadWriteLock* locker_;

public:
    explicit Worker(
            int* num_worker_,
            QReadWriteLock* locker_,
            QObject *parent = 0): num_worker_(num_worker_), locker_(locker_) {}

    ~Worker(){}

signals:
    void moveOneStep();
    void finished();
    void error(QString err);

public slots:
    void process();
};

#endif // WORKER_H
