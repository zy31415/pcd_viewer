#include "datamodel.h"

// pcl
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>

DataModel::DataModel(QObject *parent) :
    QObject(parent),
    filtering_axis_(1),  // = y
    color_mode_(4), // = Rainbow
    point_size(3),
    if_show_data_points(true),
    if_show_meshes(false),
    meshes_(new pcl::PolygonMesh)
{
    genRandomPCDWithinUnitBox();
}

DataModel::~DataModel() {}

void DataModel::genRandomPCDWithinUnitBox()
{
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

    // The number of points in the cloud
    cloud_->resize(500);

    // Fill the cloud with random points
    for (size_t i = 0; i < cloud_->points.size (); i++)
    {
        cloud_->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    // Update boundary
    bb.update(cloud_);

    // Color the randomly generated cloud
    colorPCDAlongAxis();
}

void DataModel::colorPCDAlongAxis ()
{
  // Find the minimum and maximum values along the selected axis
  double min, max;
  min = bb.get_min(filtering_axis_);
  max = bb.get_max(filtering_axis_);

  // Compute LUT scaling to fit the full histogram spectrum
  double lut_scale = 255.0 / (max - min);  // max is 255, min is 0

  if (min == max)  // In case the cloud is flat on the chosen direction (x,y or z)
    lut_scale = 1.0;  // Avoid rounding error in boost

  for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator cloud_it = cloud_->begin ();
       cloud_it != cloud_->end ();
       cloud_it++)
  {
    int value;
    switch (filtering_axis_)
    {
      case 0:  // x
        value = boost::math::iround ( (cloud_it->x - min) * lut_scale);  // Round the number to the closest integer
        break;
      case 1:  // y
        value = boost::math::iround ( (cloud_it->y - min) * lut_scale);
        break;
      default:  // z
        value = boost::math::iround ( (cloud_it->z - min) * lut_scale);
        break;
    }

    // Apply color to the cloud
    switch (color_mode_)
    {
      case 0:
        // Blue (= min) -> Red (= max)Ui_ColorDialog
        cloud_it->r = value;
        cloud_it->g = 0;
        cloud_it->b = 255 - value;
        break;
      case 1:
        // Green (= min) -> Magenta (= max)
        cloud_it->r = value;
        cloud_it->g = 255 - value;
        cloud_it->b = value;
        break;
      case 2:
        // White (= min) -> Red (= max)
        cloud_it->r = 255;
        cloud_it->g = 255 - value;
        cloud_it->b = 255 - value;
        break;
      case 3:
        // Grey (< 128) / Red (> 128)
        if (value > 128)
        {
          cloud_it->r = 255;http://stackoverflow.com/questions/13184555/creating-a-qwidget-in-a-non-gui-thread
          cloud_it->g = 0;
          cloud_it->b = 0;
        }
        else
        {
          cloud_it->r = 128;
          cloud_it->g = 128;
          cloud_it->b = 128;
        }
        break;
      default:
        // Blue -> Green -> Red (~ rainbow)
        cloud_it->r = value > 128 ? (value - 128) * 2 : 0;  // r[128] = 0, r[255] = 255
        cloud_it->g = value < 128 ? 2 * value : 255 - ( (value - 128) * 2);  // g[0] = 0, g[128] = 255, g[255] = 0
        cloud_it->b = value < 128 ? 255 - (2 * value) : 0;  // b[0] = 255, b[128] = 0
    }
  }
}

void DataModel::readPCDFile(const QString filename)
{
    PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);

    if (filename.isEmpty ())
        return;

    int return_status;
    if (filename.endsWith (".pcd", Qt::CaseInsensitive))
        return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloud_tmp);
    else
        return_status = pcl::io::loadPLYFile (filename.toStdString (), *cloud_tmp);

    if (return_status != 0) {
        PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
        return;
    }

    this->filename = filename;

    // If point cloud contains NaN values, remove them before updating the visualizer point cloud
    if (cloud_tmp->is_dense)
        pcl::copyPointCloud (*cloud_tmp, *cloud_);
    else {
        PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
        std::vector<int> vec;
        pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_, vec);
    }

    bb.update(cloud_);

    colorPCDAlongAxis ();

    // Tell the MainWindow to update the viewer.
    emit onDrawCloudData();
}


void DataModel::computeTriangulationMesh() {
    compute_triangulation_meshes(
                cloud_,
                meshes_,
                triangulation_parameters
                );
}

void DataModel::savePCDFile(QString filename) {

    PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

    if (filename.isEmpty ())
        return;

    int return_status;

    if (filename.endsWith (".pcd", Qt::CaseInsensitive))
        return_status = pcl::io::savePCDFileBinary (filename.toStdString(), *cloud_);

    else if (filename.endsWith (".ply", Qt::CaseInsensitive))
        return_status = pcl::io::savePLYFileBinary (filename.toStdString(), *cloud_);
    else
    {
      filename.append(".pcd");
      return_status = pcl::io::savePCDFileBinary (filename.toStdString(), *cloud_);
    }

    if (return_status != 0)
    {
      PCL_ERROR("Error writing point cloud %s\n", filename.toStdString().c_str());
      return;
    }
}

void DataModel::setPointSize(int point_size)
{
    this->point_size = point_size;
    emit onDrawPointSize();
}

void DataModel::setIfShowDataPoints(bool if_show_data_points) {
    this -> if_show_data_points = if_show_data_points;
    emit onIfShowDataPoints();
}


int DataModel::getColorAxis() {
    return filtering_axis_;
}

void DataModel::setColorAxis(int axis) {
    filtering_axis_ = axis;
    colorPCDAlongAxis();
    emit onDrawCloudData();
}

int DataModel::getColorLookUpTable() {
    return color_mode_;
}

void DataModel::setColorLookUpTable(int table)
{
    color_mode_ = table;
    colorPCDAlongAxis();
    emit onDrawCloudData();
}

void DataModel::setMeshing(bool if_show_meshes,
                           const TriangulationParameters& par)
{
    this->if_show_meshes = if_show_meshes;
    if (if_show_meshes)
    {
        triangulation_parameters = par;
        computeTriangulationMesh();
    }

    emit onDrawMeshes();
}
