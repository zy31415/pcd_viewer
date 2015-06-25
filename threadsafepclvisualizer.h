#ifndef THREADSAFEPCLVISUALIZER_H
#define THREADSAFEPCLVISUALIZER_H

#include <pcl/visualization/pcl_visualizer.h>

class ThreadSafePCLVisualizer : public pcl::visualization::PCLVisualizer
{
private:
    boost::mutex mutex;

public:
    ThreadSafePCLVisualizer(const std::string &name="", const bool create_interactor=true) :
        pcl::visualization::PCLVisualizer(name, create_interactor) {}

    ThreadSafePCLVisualizer(int &argc, char **argv, const std::string &name="",
                            PCLVisualizerInteractorStyle *style=PCLVisualizerInteractorStyle::New(),
                            const bool create_interactor=true) :
        pcl::visualization::PCLVisualizer(argc, argv, name, style, create_interactor) {}


    void spinOnce (int time=1, bool force_redraw=false) {
           mutex.lock();
           pcl::visualization::PCLVisualizer::spinOnce(time, force_redraw);
           mutex.unlock();
       }

       void setCameraPosition (double pos_x, double pos_y, double pos_z,
                               double view_x, double view_y, double view_z,
                               double up_x, double up_y, double up_z,
                               int viewport=0) {
           mutex.lock();
           pcl::visualization::PCLVisualizer::setCameraPosition(
                   pos_x, pos_y, pos_z,
                   view_x, view_y, view_z,
                   up_x, up_y, up_z,
                   viewport=0);
           mutex.unlock();
       }
};

#endif // THREADSAFEPCLVISUALIZER_H
