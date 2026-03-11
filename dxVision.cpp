#include "dxVision.h"

#include <string>
#include <vector>

using namespace std;

namespace
{
	constexpr double bR = 0.9f, bG = 0.9f, bB = 0.9f;
	constexpr double fontsize = 0.5;
	constexpr double fR = 0.9f, fG = 0.0f, fB = 0.0f;
	constexpr double framesize = 0.5;
}

dxVision::dxVision() = default;

dxVision::~dxVision() = default;

bool dxVision::initMujocoCamera(const MuJoCoCameraParams& mujoparams)
{
	mModel = mujoparams.model;
	mData = mujoparams.data;
	mOpt = mujoparams.opt;
	mCtx = mujoparams.ctx;
	mCameraName = mujoparams.cameraName;
	mBaseBodyName = mujoparams.baseBodyName;
	if (mujoparams.width > 0)
	{
		mWidth = mujoparams.width;
	}
	if (mujoparams.height > 0)
	{
		mHeight = mujoparams.height;
	}

	if (!mModel || !mData || !mOpt || !mCtx)
	{
		return false;
	}

	mMujocoCamera.setModel(mModel, mData);
	mMujocoCamera.setCameraName(mCameraName);
	mMujocoCamera.setBaseBodyName(mBaseBodyName);
	mMujocoCamera.setResolution(mWidth, mHeight);
	if (mujoparams.fovyDeg > 0.0)
	{
		mMujocoCamera.setOverrideFovy(mujoparams.fovyDeg);
	}
	else
	{
		mMujocoCamera.clearOverrideFovy();
	}
	return true;
}

cv::Mat dxVision::acquireMujocoRgb()
{
	if (!mModel || !mData || !mOpt || !mCtx)
	{
		return cv::Mat();
	}
	dxMuJoCoRealSense::Frame frame;
	mMujocoCamera.setResolution(mWidth, mHeight);
	if (!mMujocoCamera.captureRgbDepth(mOpt, mCtx, frame))
	{
		return cv::Mat();
	}

	cv::Mat image(frame.height, frame.width, CV_8UC3, frame.rgb.data());
	return image.clone();
}

CloudPtr dxVision::acquireMujocoPointCloud()
{
	if (!mModel || !mData || !mOpt || !mCtx)
	{
		return CloudPtr();
	}

	dxMuJoCoRealSense::Frame frame;
	mMujocoCamera.setResolution(mWidth, mHeight);
	if (!mMujocoCamera.captureRgbDepth(mOpt, mCtx, frame))
	{
		return CloudPtr();
	}

	std::vector<std::array<float, 3>> points;
	std::vector<std::array<unsigned char, 3>> colors;
	if (!mMujocoCamera.computePointCloudInBaseWithColor(frame.depth, frame.rgb, points, colors))
	{
		return CloudPtr();
	}

	CloudPtr cloud(new Cloud());
	cloud->points.resize(points.size());
	cloud->width = static_cast<uint32_t>(points.size());
	cloud->height = 1;
	cloud->is_dense = false;
	for (size_t i = 0; i < points.size(); ++i)
	{
		const auto& p = points[i];
		const auto& c = colors[i];
		PointRGBA pt;
		pt.x = p[0];
		pt.y = p[1];
		pt.z = p[2];
		pt.r = c[0];
		pt.g = c[1];
		pt.b = c[2];
		pt.a = 255;
		cloud->points[i] = pt;
	}
	return cloud;
}



void dxVision::viewPointCloud(CloudPtr source, string viewid, string msg, int pointSize)
{
	// Compute 3D centroid of the point cloud - for computing normals with a centered cloud
	//Eigen::Vector4f centroid;
	//compute3DCentroid(*source, centroid);

	visualization::PCLVisualizer view(viewid);
	view.setBackgroundColor(bR, bG, bB);
	view.initCameraParameters();
	view.addText(msg, 10, 10, fontsize, fR, fG, fB, "text", 0);
	view.addPointCloud(source, "localcloud");
	//view.addCoordinateSystem(0.5);
	//this->addCoordinateSystem(view);
	//this->addCoordinateSystem(view, centroid[0], centroid[1], centroid[2]);
	view.loadCameraParameters("camera.cam");
	view.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "localcloud");
	while (!view.wasStopped())
	{
		// view.spin();
		// std::this_thread::sleep_for(std::chrono::microseconds(100000));
		view.spinOnce();
	}
	view.close();
}

void dxVision::viewPointClouds(vector<CloudPtr> source, string viewid, string msg)
{
	mycolor color;
	srand(time(nullptr));
	Eigen::Vector4f centroid;

	if (source.size() > 1)
	{
		visualization::PCLVisualizer view(viewid);
		view.setBackgroundColor(bR, bG, bB);
		view.initCameraParameters();
		view.addText(msg, 10, 10, fontsize, fR, fG, fB, "text", 0);

		char idname[50] = "";
		for (auto i = 0; i < source.size(); i++)
		{
			sprintf(idname, "local%d", i);
			this->generateColor(color);

			if (i == 0)
			{
				compute3DCentroid(*source[i], centroid);
				visualization::PointCloudColorHandlerCustom<PointXYZRGBA>colsource(source[0], 232, 232, 232);
				view.addPointCloud(source[i], colsource, idname);
				this->addCoordinateSystem(view, centroid[0], centroid[1], centroid[2]);
				//view.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, idname);
			}
			else
			{
				compute3DCentroid(*source[i], centroid);
				visualization::PointCloudColorHandlerCustom<PointRGBA>colsource(source[i], color.r, color.g, color.b);
				view.addPointCloud(source[i], colsource, idname);
				//this->addCoordinateSystem(view, centroid[0], centroid[1], centroid[2]);
				view.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, idname);
			}
			//this->addCoordinateSystem(view);
		}
		view.loadCameraParameters("camera.cam");

		while (!view.wasStopped())
		{
			view.spinOnce();
		}
		view.close();

	}
	else
	{
		viewPointCloud(source[0], "default view");
	}

}

void dxVision::viewPointClouds(vector<CloudNTPtr> source, string viewid, string msg)
{
	mycolor color;
	srand(time(nullptr));
	Eigen::Vector4f centroid;

	if (source.size() > 1)
	{
		visualization::PCLVisualizer view(viewid);
		view.setBackgroundColor(bR, bG, bB);
		view.initCameraParameters();
		view.addText(msg, 10, 10, fontsize, fR, fG, fB, "text", 0);

		char idname[50] = "";
		for (auto i = 0; i < source.size(); i++)
		{
			sprintf(idname, "local%d", i);
			this->generateColor(color);

			if (i == 0)
			{
				int _color = 150;
				compute3DCentroid(*source[i], centroid);
				visualization::PointCloudColorHandlerCustom<PointNT>colsource(source[0], _color, _color, _color);
				view.addPointCloud(source[i], colsource, idname);
				this->addCoordinateSystem(view, centroid[0], centroid[1], centroid[2]);
				//view.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, idname);
			}
			else
			{
				compute3DCentroid(*source[i], centroid);
				visualization::PointCloudColorHandlerCustom<PointNT>colsource(source[i], color.r, color.g, color.b);
				view.addPointCloud(source[i], colsource, idname);
				//this->addCoordinateSystem(view, centroid[0], centroid[1], centroid[2]);
				view.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, idname);
			}
			//this->addCoordinateSystem(view);
		}
		view.loadCameraParameters("camera.cam");

		while (!view.wasStopped())
		{
			view.spinOnce();
		}
		view.close();

	}

}

void dxVision::viewPointCloudMono(CloudMonoPtr source, string viewid)
{
	// Compute 3D centroid of the point cloud - for computing normals with a centered cloud
	//Eigen::Vector4f centroid;
	//compute3DCentroid(*source, centroid);
	//std::cout << "Centroid\n" << centroid.head<3>() << std::endl;

	visualization::PCLVisualizer view(viewid);
	view.setBackgroundColor(bR, bG, bB);
	view.initCameraParameters();
	view.loadCameraParameters("camera.cam");
	//this->addCoordinateSystem(view);
	//this->addCoordinateSystem(view, centroid[0], centroid[1], centroid[2]);
	view.addPointCloud(source, "localcloud");
	view.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "localcloud");

	while (!view.wasStopped())
	{
		view.spin();
	}
	view.close();
}

void dxVision::viewPointCloudWithNormals(CloudPtr source, NormalPtr normals)
{
	visualization::Camera camera;

	visualization::PCLVisualizer Nviewer("Normal Viewer");
	Nviewer.setBackgroundColor(bR, bG, bB);
	Nviewer.initCameraParameters();
	Nviewer.loadCameraParameters("camera.cam");

	visualization::PointCloudColorHandlerCustom<PointXYZRGBA>colsource(source, 0.5, 0.5, 0.5);
	Nviewer.addPointCloud(source, colsource, "localcloud");
	Nviewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 6, "localcloud");

	Nviewer.addPointCloudNormals<PointXYZRGBA, Normal>(source, normals, 1, 0.01, "normals");
	Nviewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "normals");
	//Nviewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "normals");
	Nviewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "normals");
	//this->addCoordinateSystem(Nviewer);

	while (!Nviewer.wasStopped())
	{
		Nviewer.spin();
	}
	Nviewer.close();
}

void dxVision::viewPointNormalCloud(CloudNTPtr cloud, bool showNormals)
{
	visualization::Camera camera;

	visualization::PCLVisualizer viewer("Normal Viewer");
	viewer.setBackgroundColor(bR, bG, bB);
	viewer.initCameraParameters();
	viewer.loadCameraParameters("camera.cam");
	visualization::PointCloudColorHandlerCustom<PointNT>colsource(cloud, 0, 0, 255);
	viewer.addPointCloud<PointNT>(cloud, colsource, "localcloud");
	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 6, "localcloud");
	if (showNormals)
	{
		viewer.addPointCloudNormals<pcl::PointNormal>(cloud, 1, 0.01, "normals");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "normals");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "normals");
	}
	this->addCoordinateSystem(viewer);
	while (!viewer.wasStopped())
	{
		viewer.spin();
	}
	viewer.close();
}

void dxVision::viewPointCloudWithNormals(CloudNTPtr source, std::string viewid, bool viewNormals)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> Nviewer(new pcl::visualization::PCLVisualizer(viewid));
	Nviewer->setBackgroundColor(0.3, 0.3, 0.3);
	Nviewer->initCameraParameters();
	Nviewer->loadCameraParameters("camera.cam");
	visualization::PointCloudColorHandlerCustom<PointNT>colsource(source, 225, 0, 0);
	Nviewer->addPointCloud(source, colsource, "localcloud");
	Nviewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "localcloud");

	if (viewNormals)
		Nviewer->addPointCloudNormals<PointNT, PointNT>(source, source, 20, 0.01, "normals");
	Nviewer->addCoordinateSystem(0.2);

	while (!Nviewer->wasStopped())
	{
		Nviewer->spin();
	}
	Nviewer->close();
}

void dxVision::viewPointCloudWithNormalsAndTexture(CloudNTRGBAPtr source, std::string viewid, bool viewNormals)
{
	CloudPtr temp(new Cloud);
	pcl::copyPointCloud(*source, *temp);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> Nviewer(new pcl::visualization::PCLVisualizer(viewid));
	Nviewer->setBackgroundColor(bR, bG, bB);
	Nviewer->initCameraParameters();
	Nviewer->loadCameraParameters("camera.cam");
	Nviewer->addPointCloud<PointRGBA>(temp, "localcloud");

	//Nviewer->addCoordinateSystem(0.2);
	if (viewNormals)
		Nviewer->addPointCloudNormals<PointNTRGBA, PointNTRGBA>(source, source, 50, 0.01, "normals");

	while (!Nviewer->wasStopped())
	{
		Nviewer->spin();
	}
	Nviewer->close();
}

CloudPtr dxVision::cropPointCloud(CloudPtr source, const cropvalues& limits)
{
	if (!source)
	{
		return CloudPtr();
	}
	CloudPtr cropped(new Cloud());
	pcl::CropBox<PointRGBA> crop;
	crop.setInputCloud(source);
	crop.setMin(Eigen::Vector4f(static_cast<float>(limits.xmin),
								static_cast<float>(limits.ymin),
								static_cast<float>(limits.zmin), 1.0f));
	crop.setMax(Eigen::Vector4f(static_cast<float>(limits.xmax),
								static_cast<float>(limits.ymax),
								static_cast<float>(limits.zmax), 1.0f));
	crop.filter(*cropped);
	return cropped;
}

template <typename PointType, class CloudType>
void dxVision::viewPointCloudWithNormalsAndTexture(typename CloudType::Ptr cloud, int point_size)
{
	if (!cloud || cloud->empty())
	{
		std::cerr << "Error: Point cloud is empty or not loaded." << std::endl;
		return;
	}

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
	viewer->setBackgroundColor(0.9, 0.9, 0.9);

	// Add the point cloud based on its type
	//if constexpr (std::is_same<PointType, pcl::PointXYZRGBA>::value || std::is_same<PointType, pcl::PointXYZRGB>::value)
	//{
	//    viewer->addPointCloud<PointType>(cloud, "cloud");
	//}
	//else if constexpr (std::is_same<PointType, pcl::PointXYZI>::value)
	//{
	//    pcl::visualization::PointCloudColorHandlerGenericField<PointType> intensity_distribution(cloud, "intensity");
	//    viewer->addPointCloud<PointType>(cloud, intensity_distribution, "intensity_cloud");
	//}
	//else
	//{
	//    viewer->addPointCloud<PointType>(cloud, "cloud");
	//}
	viewer->addPointCloud<PointType>(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "cloud");

	// Compute bounding box manually
	PointType minPt, maxPt;
	minPt.x = minPt.y = minPt.z = std::numeric_limits<float>::max();
	maxPt.x = maxPt.y = maxPt.z = std::numeric_limits<float>::lowest();

	for (const auto& point : *cloud)
	{
		if (point.x < minPt.x) minPt.x = point.x;
		if (point.y < minPt.y) minPt.y = point.y;
		if (point.z < minPt.z) minPt.z = point.z;

		if (point.x > maxPt.x) maxPt.x = point.x;
		if (point.y > maxPt.y) maxPt.y = point.y;
		if (point.z > maxPt.z) maxPt.z = point.z;
	}

	// Compute centroid
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);

	// Calculate cloud dimensions
	float xRange = maxPt.x - minPt.x;
	float yRange = maxPt.y - minPt.y;
	float zRange = maxPt.z - minPt.z;
	float maxRange = std::max({ xRange, yRange, zRange });

	// Adjust camera position and focus
	viewer->setCameraPosition(centroid[0], centroid[1], centroid[2] + 2.5 * maxRange,  // Camera position
							  centroid[0], centroid[1], centroid[2],                // Viewpoint
							  0, -1, 0);                                           // Up vector

	// Access renderer and adjust the camera view angle for 80% occupancy
		//vtkSmartPointer<vtkRenderer> renderer = viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer();
		//if (renderer)
		//{
		//    renderer->GetActiveCamera()->SetViewAngle(20.0);  // Adjust to control zoom level (80% view)
		//}

	viewer->loadCameraParameters("camera.cam");
	//viewer->addCoordinateSystem(1.0);
	viewer->addText("Cloud", 10, 10, "message");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
}


void dxVision::addCoordinateSystem(pcl::visualization::PCLVisualizer& view, double x, double y, double z)
{
	vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
	axes = this->getCoordinateSystem(x, y, z);
	view.getRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(axes);
}


vtkSmartPointer<vtkAxesActor> dxVision::getCoordinateSystem(double x, double y, double z)
{
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->Translate(x, y, z);

	vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
	axes->SetTotalLength(framesize, framesize, framesize);
	axes->SetUserTransform(transform);
	axes->SetShaftTypeToCylinder();
	axes->SetCylinderRadius(1.00 * axes->GetCylinderRadius());
	axes->SetConeRadius(1.7 * axes->GetConeRadius());
	axes->AxisLabelsOff();

	return axes;
}


void dxVision::generateColor(mycolor& color)
{

	color.r = rand() % 256;
	color.g = rand() % 256;
	color.b = rand() % 256;
}

template void dxVision::viewPointCloudWithNormalsAndTexture<PointRGBA, Cloud>(CloudPtr, int);
template void dxVision::viewPointCloudWithNormalsAndTexture<PointMono, CloudMono>(CloudMonoPtr, int);
template void dxVision::viewPointCloudWithNormalsAndTexture<PointNT, CloudNT>(CloudNTPtr, int);
template void dxVision::viewPointCloudWithNormalsAndTexture<PointNTRGBA, CloudNTRGBA>(CloudNTRGBAPtr, int);
