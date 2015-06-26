#include "worker.h"

void Worker::process() {
    for (int ii=0; ii<100; ii++) {
        emit moveOneStep();
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    emit finished();
}
