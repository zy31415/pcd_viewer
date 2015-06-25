#include "boundingbox.h"

void BoundingBox::update(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_) {
    min_x = cloud_->points[0].x;
    max_x = cloud_->points[0].x;

    min_y = cloud_->points[0].y;
    max_y = cloud_->points[0].y;

    min_z = cloud_->points[0].z;
    max_z = cloud_->points[0].z;

    // Search for the minimum/maximum
    for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator cloud_it = cloud_->begin ();
         cloud_it != cloud_->end ();
         cloud_it++)
    {
        if (min_x > cloud_it->x)
            min_x = cloud_it->x;

        if (max_x < cloud_it->x)
            max_x = cloud_it->x;

        if (min_y > cloud_it->y)
            min_y = cloud_it->y;

        if (max_y < cloud_it->y)
            max_y = cloud_it->y;

        if (min_z > cloud_it->z)
            min_z = cloud_it->z;

        if (max_z < cloud_it->z)
            max_z = cloud_it->z;
    }
}
