#ifndef DATAMODEL_H
#define DATAMODEL_H

#include <QObject>

#include "boundingbox.h"
#include "triangulation_meshes.h"

/**
 * @brief Used to store application data (Model-view-controller architectural pattern).
 */
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

    QString filename;

    int point_size;
    bool if_show_data_points;

    /** @brief Points Cloud data */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;

    /** @brief Points Cloud meshes */
    pcl::PolygonMesh::Ptr meshes_;
    bool if_show_meshes;
    TriangulationParameters triangulation_parameters;

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
    void genRandomPCDWithinUnitBox();

public:
    explicit DataModel(QObject *parent = 0);
    ~DataModel();

    void readPCDFile(const QString filename);
    void savePCDFile(QString filename);

    void computeTriangulationMesh();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloud() {return cloud_;}

    int getPointSize() { return point_size;}
    void setPointSize(int point_size);

    bool getIfShowDataPoints() {return if_show_data_points; }
    void setIfShowDataPoints(bool if_show_data_points);

    int getColorAxis();
    void setColorAxis(int axis);

    int getColorLookUpTable();
    void setColorLookUpTable(int table);

    TriangulationParameters getTriangulationParameters() { return triangulation_parameters;}

    bool getIfShowMeshes() {return if_show_meshes;}
    void setMeshing(bool if_show_meshes,
                    const TriangulationParameters& par);
    pcl::PolygonMesh::Ptr getMesh() {return meshes_;}

    BoundingBox getBoundingBox() {return bb;}

signals:
    void onDrawCloudData();
    void onDrawPointSize();
    void onIfShowDataPoints();
    void onDrawMeshes();

public slots:
};

#endif // DATAMODEL_H
