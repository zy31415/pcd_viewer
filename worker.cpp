#include "worker.h"

void Worker::process() {
    int key = 1;

    while (key) {
        emit moveOneStep();
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        locker_->lockForRead();
        key = *num_worker_;
        locker_->unlock();
    }

    emit finished();
}
