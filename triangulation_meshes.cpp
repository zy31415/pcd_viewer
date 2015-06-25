#include "triangulation_meshes.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr
copy_PointXYZRGBA_to_PointXYZ
(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_RGBA (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_no_RGBA->resize(cloud->size());
    for (size_t i = 0;
         i < cloud -> points.size();
         i++) {
        cloud_no_RGBA -> points[i].x = cloud -> points[i].x;
        cloud_no_RGBA -> points[i].y = cloud -> points[i].y;
        cloud_no_RGBA -> points[i].z = cloud -> points[i].z;
    }
    return cloud_no_RGBA;
}

void compute_triangulation_meshes(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_,
        pcl::PolygonMesh::Ptr meshes_,
        TriangulationParameters &par
        ) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_RGBA = copy_PointXYZRGBA_to_PointXYZ(cloud_);

    // Normal estimation*
    int nr_cores = boost::thread::hardware_concurrency();
    pcl::NormalEstimationOMP <pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    tree->setInputCloud (cloud_no_RGBA);
    n.setInputCloud (cloud_no_RGBA);
    n.setSearchMethod (tree);
    n.setKSearch (par.k);
    n.setNumberOfThreads(nr_cores);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud_no_RGBA, *normals, *cloud_with_normals);
    // cloud_with_normals = cloud + normals;

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (par.search_radius);

    // Set typical values for the parameters
    gp3.setMu (par.mu);
    gp3.setMaximumNearestNeighbors (par.max_NN);
    gp3.setMaximumSurfaceAngle(par.max_surface_angle); // 45 degrees
    gp3.setMinimumAngle(par.min_angle); // 10 degrees
    gp3.setMaximumAngle(par.max_angle); // 120 degrees
    gp3.setNormalConsistency(par.is_normal_consistency);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (*meshes_);

}
