#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H


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

    double get_min_x() {return min_x;}
    double get_max_x() {return max_x;}
    double get_min_y() {return min_y;}
    double get_max_y() {return max_y;}
    double get_min_z() {return min_z;}
    double get_max_z() {return max_z;}

    void update(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_);

};

#endif // BOUNDINGBOX_H
