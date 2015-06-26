#ifndef DATAMODEL_H
#define DATAMODEL_H

#include <QObject>

#include "boundingbox.h"

class DataModel : public QObject
{
    Q_OBJECT
private:
    /** @brief 0 = x | 1 = y | 2 = z */
    int filtering_axis_;

    /** @brief Holds the color mode for @ref colorCloudDistances */
    int color_mode_;

    /** @brief Bounding box of the points cloud data. */
    BoundingBox bb;

    /** @brief Points Cloud data */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;

    /** @brief Points Cloud meshes */
    pcl::PolygonMesh::Ptr meshes_;

    /** @brief Color point cloud on X,Y or Z axis using a Look-Up Table (LUT)
     * Computes a LUT and color the cloud accordingly, available color palettes are :
     *
     *  Values are on a scale from 0 to 255:
     *  0. Blue (= 0) -> Red (= 255), this is the default value
     *  1. Green (= 0) -> Magenta (= 255)
     *  2. White (= 0) -> Red (= 255)
     *  3. Grey (< 128) / Red (> 128)
     *  4. Blue -> Green -> Red (~ rainbow)
     *
     * @warning If there's an outlier in the data the color may seem uniform because of this outlier!
     * @note A boost rounding exception error will be thrown if used with a non dense point cloud
     */
    void colorPCDAlongAxis();

public:
    explicit DataModel(QObject *parent = 0);
    ~DataModel();

    void readPCDFile(QString filename);

signals:

public slots:
};

#endif // DATAMODEL_H
