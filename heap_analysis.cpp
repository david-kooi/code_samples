#include <iterator>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>

#include <boost/thread/thread.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>




boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer 
		  (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties 
		  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}



void computeMinPoints(pcl::PointCloud<pcl::PointXYZ>& pc, float& minX, 
														  float& minY,
														  float& minZ){

	minX = pc.points[0].x;
	minY = pc.points[0].y;
	minZ = pc.points[0].z;
	for(size_t i = 0; i < pc.points.size(); i++){
		if(pc.points[i].x < minX){
			minX = pc.points[i].x;
		}	

		if(pc.points[i].y < minY){
			minY = pc.points[i].y;
		}	

		if(pc.points[i].z < minZ){
			minZ = pc.points[i].z;
		}	
	}
}	

void computeMaxPoints(pcl::PointCloud<pcl::PointXYZ>& pc, float& maxX, 
														  float& maxY,
														  float& maxZ,
														  uint16_t& idxX,
														  uint16_t& idxY,
														  uint16_t& idxZ){

	maxX = pc.points[0].x;
	maxY = pc.points[0].y;
	maxZ = pc.points[0].z;
	for(size_t i = 0; i < pc.points.size(); i++){
		if(pc.points[i].x > maxX){
			maxX = pc.points[i].x;
			idxX = i;
		}	

		if(pc.points[i].y > maxY){
			maxY = pc.points[i].y;
			idxY = i;
		}	

		if(pc.points[i].z > maxZ){
			maxZ = pc.points[i].z;
			idxZ = i;
		}	
	}
}

/*
 * Take the original value o and perform a normalization.
 * Return the normalized value.
 * */
float calculateNormal(float o, float min, float max){
	return (o - min)/(max - min);
}

/*
 * Normalize the XYZ points within a point cloud.
 * The scale factor N will move the range from 0-1 to 0-N
 * */
void normalizePointCloud(pcl::PointCloud<pcl::PointXYZ>& pc, float min[3], 
															 float max[3],
															 uint16_t scale){
	for(size_t i = 0; i < pc.points.size(); i ++){
		float newX = calculateNormal(pc.points[i].x, min[0], max[0]); 
		float newY = calculateNormal(pc.points[i].y, min[1], max[1]); 
		float newZ = calculateNormal(pc.points[i].z, min[2], max[2]);

		pc.points[i].x = newX*scale;
		pc.points[i].y = newY*scale;
		pc.points[i].z = newZ*scale;

	}
}


int main (int argc, char** argv){


	// Create cloud and it's ptr
	pcl::PointCloud<pcl::PointXYZ> pclCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(&pclCloud);
	std::cout << "Created pc" << std::endl;

	// Create polygon
	pcl::PolygonMesh polyMesh;

	std::cout << "Visualizer Created" << std::endl;

	// Read the ply
	pcl::io::loadPLYFile("heap.ply", polyMesh);
	std::cout << "Read ply" << std::endl;

	pcl::fromPCLPointCloud2(polyMesh.cloud, pclCloud);

	// Normalize the pointcloud
	float minX, minY, minZ;
	float maxX, maxY, maxZ;
	uint16_t idxX, idxY, idxZ;

	computeMinPoints(pclCloud, minX, minY, minZ);
	computeMaxPoints(pclCloud, maxX, maxY, maxZ, 
					           idxX, idxY, idxZ);

	float minArr[3] = {minX, minY, minZ};
	float maxArr[3] = {maxX, maxY, maxZ};

	std::cout << "MAX X: " << maxX << std::endl;
	std::cout << "MAY Y: " << maxY << std::endl;
	std::cout << "MAX Z: " << maxZ << std::endl;

	std::cout << "MIN X: " << minX << std::endl;
	std::cout << "MIN Y: " << minY << std::endl;
	std::cout << "MIN Z: " << minZ << std::endl;

	normalizePointCloud(pclCloud, minArr, 
						          maxArr,
    								  1);

	// View min and max points again
	computeMinPoints(pclCloud, minX, minY, minZ);
	computeMaxPoints(pclCloud, maxX, maxY, maxZ, 
					           idxX, idxY, idxZ);

	std::cout << "MAX X: " << maxX << std::endl;
	std::cout << "MAY Y: " << maxY << std::endl;
	std::cout << "MAX Z: " << maxZ << std::endl;

	std::cout << "MIN X: " << minX << std::endl;
	std::cout << "MIN Y: " << minY << std::endl;
	std::cout << "MIN Z: " << minZ << std::endl;

	// Rotate point cloud 90 degrees (PI radians)
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX()));
	transform.translation() << 0.0, -0.5, 1.0;

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud
															<pcl::PointXYZ>());
	pcl::transformPointCloud(pclCloud, *transformed_cloud, transform);

	// Print the transformation
	printf ("\nTransforming using an Affine3f\n");
	std::cout << transform.matrix() << std::endl;

	// Remove all points below Y = 0
	pcl::PointCloud<pcl::PointXYZ>::Ptr sliced_cloud(new pcl::PointCloud
															<pcl::PointXYZ>());

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(transformed_cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(0, 10);
	pass.filter(*sliced_cloud);


	
	// Create visualizer 
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = simpleVis(sliced_cloud);

	// Add Ground Plane @ Y = 0
	pcl::ModelCoefficients plane_coeff;
	plane_coeff.values.resize(4);
	plane_coeff.values[0] = 0;
	plane_coeff.values[1] = 1;
	plane_coeff.values[2] = 0;
	plane_coeff.values[3] = 0;
	viewer->addPlane(plane_coeff, "ground_plane");

	
	// Compute the peak
	computeMaxPoints(*sliced_cloud, maxX, maxY, maxZ, 
					           idxX, idxY, idxZ);
	float x, y, z;
	x = sliced_cloud->points[idxY].x; 
	y = sliced_cloud->points[idxY].y; 
	z = sliced_cloud->points[idxY].z; 
	// Add a sphere at the peak
	pcl::PointXYZ peak(x, y, z);
	viewer->addSphere(peak, 0.03, "peak");


	while(!viewer->wasStopped()){
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));	
	}



	

	return(0);


}
