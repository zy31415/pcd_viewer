#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class BoundingBox
{
private:
    double min_x;
    double max_x;
    double max_y;
    double min_y;
    double max_z;
    double min_z;

public:
    inline BoundingBox() :
        min_x(0), max_x(0),
        min_y(0), max_y(0),
        min_z(0), max_z(0) {}

    // copy constructor
    BoundingBox(const BoundingBox& other) :
        min_x(other.min_x),
        max_x(other.max_x),
        min_y(other.min_y),
        max_y(other.max_y),
        min_z(other.min_z),
        max_z(other.max_z) {}

    double get_min_x() {return min_x;}
    double get_max_x() {return max_x;}
    double get_min_y() {return min_y;}
    double get_max_y() {return max_y;}
    double get_min_z() {return min_z;}
    double get_max_z() {return max_z;}

    double get_max(int axis);
    double get_min(int axis);

    void update(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_);

};

#endif // BOUNDINGBOX_H
