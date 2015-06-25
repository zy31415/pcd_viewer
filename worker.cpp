#include "worker.h"

void Worker::process() {
    double pos_x, pos_y, pos_z,
            view_x, view_y, view_z,
            up_x, up_y, up_z;

    std::cout<<"asdfasdfsdsadf"<<std::endl;

    std::vector<pcl::visualization::Camera> cameras;
    viewer_ -> getCameras(cameras);


    pos_x = cameras[0].pos[0];
    pos_y = cameras[0].pos[1];
    pos_z = cameras[0].pos[2];

    view_x = cameras[0].focal[0];
    view_y = cameras[0].focal[1];
    view_z = cameras[0].focal[2];

    up_x = cameras[0].view[0];
    up_y = cameras[0].view[1];
    up_z = cameras[0].view[2];

    pos_z = (bb.get_max_z() + bb.get_min_z())/2.;

    double r = 2 * sqrt(pow(bb.get_max_x(),2.) + pow(bb.get_max_y(),2.));


    double alpha = 0;
    while (alpha < 3.14*4) {
        pos_x = r * cos(alpha);
        pos_y = r * sin(alpha);

        alpha += 0.002;
        sleep(0.01);
        viewer_ -> setCameraPosition(pos_x, pos_y, pos_z,
                                     view_x, view_y, view_z,
                                     up_x, up_y, up_z);
    }
}
