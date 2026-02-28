#pragma once


#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/mls.h>
#include <pcl/common/time.h>
#include <pcl/surface/gp3.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <pcl/io/vtk_io.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkAxesActor.h>
#include <vtkTransform.h>
#include <vtkSmartPointer.h>

using namespace pcl;

typedef pcl::PointXYZRGBA PointRGBA;
typedef pcl::PointCloud<PointRGBA> Cloud;
typedef Cloud::Ptr CloudPtr;

typedef pcl::PointCloud<pcl::Normal> CloudNorm;
typedef CloudNorm::Ptr CloudNormPtr;
typedef CloudNorm::ConstPtr NormalPtr;

typedef pcl::PointXYZ PointMono;
typedef pcl::PointCloud<PointMono> CloudMono;
typedef CloudMono::Ptr CloudMonoPtr;

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> CloudNT;
typedef CloudNT::Ptr CloudNTPtr;

typedef pcl::PointXYZRGBNormal  PointNTRGB;
typedef pcl::PointCloud<PointNTRGB> CloudNTRGB;
typedef CloudNTRGB::Ptr CloudNTRGBPtr;

typedef pcl::PointXYZRGBNormal  PointNTRGBA;
typedef pcl::PointCloud<PointNTRGBA> CloudNTRGBA;
typedef CloudNTRGBA::Ptr CloudNTRGBAPtr;

typedef pcl::PointXYZRGBL  PointL;
typedef pcl::PointCloud<PointL> CloudL;
typedef CloudL::Ptr CloudLPtr;
typedef CloudL::ConstPtr CloudLConstPtr;

typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

typedef pcl::visualization::PointCloudColorHandlerCustom<PointRGBA> ColorHandlerT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerNT;


struct mycolor
{
    int r, g, b;
    mycolor() : r(0), g(0), b(0)
    {
    }
    mycolor(int re, int gr, int bl)
    {
        r = re;
        g = gr;
        b = bl;
    }
};

struct cropvalues
{
    double xmin, xmax, ymin, ymax, zmin, zmax;
    cropvalues() : xmin(0.0), xmax(0.0), ymin(0.0), ymax(0.0), zmin(0.0), zmax(0.0)
    {
    }
};

class dxVision
{
public:
    dxVision();

    ~dxVision();

    /**----- Grabbing + Visualisation + Utils ------*/
    void viewPointCloud(CloudPtr source, std::string viewid = " ", std::string msg = " ", int pointSize = 2);
    void viewPointClouds(std::vector<CloudPtr> source, std::string viewid = "", std::string msg = " ");
    void viewPointClouds(std::vector<CloudNTPtr> source, std::string viewid = "", std::string msg = " ");
    void viewPointCloudMono(CloudMonoPtr source, std::string viewid = " ");
    void viewPointCloudWithNormals(CloudPtr source, NormalPtr normals);
    void viewPointNormalCloud(CloudNTPtr cloud, bool showNormals = true);
    void viewPointCloudWithNormals(CloudNTPtr source, std::string viewid = "default", bool viewNormals = false);
    void viewPointCloudWithNormalsAndTexture(CloudNTRGBAPtr source, std::string viewid = "", bool viewNormals = false);
    void addCoordinateSystem(pcl::visualization::PCLVisualizer& view, double x = 0, double y = 0, double z = 0);
    CloudPtr cropPointCloud(CloudPtr source, const cropvalues& limits);

    //View Point cloud with normal
    template <typename PointType, class CloudType>
    void viewPointCloudWithNormalsAndTexture(typename CloudType::Ptr cloud, int point_size);

private:
    //Grabber* grabber;

    vtkSmartPointer<vtkAxesActor> getCoordinateSystem(double x, double y, double z);
    void generateColor(mycolor& color);

    Eigen::MatrixXi camera_source_; ///< binary matrix: (i,j) = 1 if point j is seen by camera i
    Eigen::Matrix3Xd normals_; ///< the surface normal for each point in the point cloud

};

