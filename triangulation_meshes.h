#ifndef COMPUTE_TRIANGULATION_MESH_H
#define COMPUTE_TRIANGULATION_MESH_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>

#define PI 3.1415926

struct TriangulationParameters {
    int k;
    double search_radius;
    double mu;
    int max_NN;
    double max_surface_angle;
    double min_angle;
    double max_angle;
    bool is_normal_consistency;

    TriangulationParameters():
        k(20),
        search_radius(0.025),
        mu(2.5),
        max_NN(100),
        max_surface_angle(45.*PI/180.),
        min_angle(10.*PI/180.),
        max_angle(120.*PI/180.),
        is_normal_consistency(true)
    {}
};

pcl::PointCloud<pcl::PointXYZ>::Ptr
copy_PointXYZRGBA_to_PointXYZ
(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

void compute_triangulation_meshes(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_,
        pcl::PolygonMesh::Ptr meshes_,
        TriangulationParameters &par
        );

#endif // COMPUTE_TRIANGULATION_MESH_H
