#include "sambuca_pcl/heap_analysis.h"
#include <cmath>

/* Written by David Kooi
 * */


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



/* Find the maximum Y value where X = 0*/
void computeMaxPointAhead(pcl::PointCloud<pcl::PointXYZ>& pc,  pcl::PointXYZ& maxPoint){

		maxPoint.x = -1000;
		maxPoint.y = -1000;
		maxPoint.z = -1000;


		for(size_t i = 0; i < pc.points.size(); i++){


			// Only checking within a threshold above and below
			// x == 0
			if( pc.points[i].x < 0.5  && pc.points[i].x > -0.5){
				if(pc.points[i].y > maxPoint.y){
					//printf("IN\n");
					maxPoint = pc.points[i];
				}
			}
		}

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

/* Creates a `width X `height grid of points that have a maximum value of
 * `scale.
 *
 *  E.g: width = height = 10, scale = 1, forms square of 10x10 points
 *  each sepearted by 1 unit.
 * */
void createReferencePlane(pcl::PointCloud<pcl::PointXYZ> *refPlane,
				          uint32_t width, uint32_t height, uint32_t xScale, uint32_t yScale){

	// Create red
	// From:
	// http://docs.pointclouds.org/1.5.1/structpcl_1_1_point_x_y_z_r_g_b.html
	uint8_t r= 255, g = 0, b = 0;
	uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

	// Create the point cloud
	for(size_t i = 0; i < height; i+=4){
		for(size_t j = 0; j < width; j+=4){
			//cout << "i: " << i << "\n";
			//cout << "j: " << j << "\n";

			refPlane->at(j, i).x = (float)xScale*(j/(float)width);
			refPlane->at(j, i).y = (float)yScale*(i/(float)height);
			//refPlane->at(j, i).rgb = *reinterpret_cast<float*>(&rgb);
		}
	}

	//cout << "PT: " << refPlane->at(999,999).x << "," << refPlane->at(999,999).y << "\n";

}


/*
 * Get the intersection between the heapCloud and the refPlane.
 * http://docs.pointclouds.org/trunk/kdtree_2include_2pcl_2kdtree_2impl_2io_8hpp_source.html#l00068
 * */
void getIntersection(pcl::PointCloud<pcl::PointXYZ>::Ptr heapCloudPtr,
					 pcl::PointCloud<pcl::PointXYZ>::Ptr refPlanePtr,
					 std::vector<int> &indices,
					 std::vector<pcl::PointXYZ> &intersectionPoints){

   pcl::KdTreeFLANN<pcl::PointXYZ> tree;
   tree.setInputCloud (heapCloudPtr);

   std::vector<int> nn_idx (1);
   std::vector<float> nn_dists (1);
   //indices.resize (heapCloudPtr->points.size());
   // Setup KD-Search
   tree.setEpsilon(0.001);

   float threshold = 0.05;
   int numNN = 0;
   for (size_t i = 0; i < refPlanePtr->points.size (); ++i)
   {
     numNN = tree.nearestKSearchT ((*refPlanePtr)[i], 1, nn_idx, nn_dists);
	 if(numNN > 0){
		// Save indices if within a threshold
		if(nn_dists[0] < threshold){

			indices.insert(indices.end(), nn_idx[0]);
		}
	 }
   }
   // Save intersection points
   intersectionPoints.resize(indices.size() + 1);


   int i = 0;
   for(std::vector<int>::iterator ptr = indices.begin(); ptr!=indices.end(); ptr++){
		//intersectionPoints.insert(intersectionPoints.end(), heapCloudPtr->points[*ptr]);
		intersectionPoints[i] = heapCloudPtr->points[*ptr];
		i++;
	}
}


float areaBeneathIntersection(std::vector<pcl::PointXYZ> &intersectionPoints,
				              pcl::PointXYZ& peakPoint){
	float area = 0;
	float width = 0;
	float height = 0;
	pcl::PointXYZ *p1;
	pcl::PointXYZ *p2;

  //printf("IntersectionPoints: %d\n", intersectionPoints.size());
  if(intersectionPoints.size() < 2){
    return 0.0;
  }

	// Start not at point[0] but at point[1]
	int i = 1;
	for(std::vector<pcl::PointXYZ>::iterator ptr = intersectionPoints.begin()+1;
		ptr!=intersectionPoints.end(); ptr++, i++){

		p1 = &intersectionPoints[i-1];
		p2 = &intersectionPoints[i];

		// Ignore anything above the peak
		if(p1->y > peakPoint.y){
			continue;
		}


		// width = sqrt(A^2 + B^2)
	    //printf("p1y: %lf p2y: %lf\n", p1->y, p2->y);
		width = pow( fabs(p1->x) - fabs(p2->x), 2) + pow( fabs(p1->z) - fabs(p2->z) , 2);
		width = sqrt(width);

		height = (fabs((p1)->y) - fabs(p2->y)) / 2.0;

		area += fabs(width * height);
	}

	return area;
}


/*
 * Scans from 0 - PI and returns the angle of maximum area
 * */
float scanForEntry(pcl::PointCloud<pcl::PointXYZ>::Ptr heapCloudPtr,
				  pcl::PointCloud<pcl::PointXYZ>::Ptr refPlanePtr,
				  pcl::PointXYZ& peakPoint,
				  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
				  int useViewer,
				  int usePly){

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedRefPlanePtr (new pcl::PointCloud
															<pcl::PointXYZ>());




	// Get maximum area beneath intersection
	float resolution = 0.0873*2.0; // 10 degrees
	float maxArea = 0;
  float maxAngle = 0;
	float averageArea = 0;
	std::vector<pcl::PointXYZ> maxPoints;

  // Rotate reference plane to starting position
  Eigen::Affine3f rotate = Eigen::Affine3f::Identity();
  rotate.rotate(Eigen::AngleAxisf(0.7, Eigen::Vector3f::UnitY()));
  //transform.translation() << 0.0, 0.0, 0.0;
  // transform.translation() << originX, originY, originZ;
  pcl::transformPointCloud(*refPlanePtr, *refPlanePtr, rotate);


	for(float angle = 3.84; angle < 5.59; angle += resolution){
//	for(float angle = M_PI; angle < M_PI+(M_PI/2); angle += resolution){
		
		cout << "-----------\n";
		cout<< "angle " << angle << "\n";

    	// Rotate refPlane point cloud
    	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		// Incrementally rotate refPlane by resolution
    	transform.rotate(Eigen::AngleAxisf(resolution, Eigen::Vector3f::UnitY()));

    	pcl::transformPointCloud(*refPlanePtr, *transformedRefPlanePtr, transform);
    	*refPlanePtr = *transformedRefPlanePtr;

    	// Perform KD search to extract the intersection between the
    	// referencePlane and the heapCloud
    	std::vector<int> indices;
    	std::vector<pcl::PointXYZ> intersectionPoints;
		intersectionPoints.resize(0);
    	getIntersection(heapCloudPtr, refPlanePtr, indices, intersectionPoints);


    	cout << "Size of intersection " << indices.size() << "\n";
    	//cout << "Size of points " << intersectionPoints.size() << "\n";


    	// Take area beneath intersection
    	float area = areaBeneathIntersection(intersectionPoints, peakPoint);

    	cout << "Area beneath intersection " << area << "\n";
		cout << "-----------\n";

		if(usePly){

				// Abs sets ply values to zero.
				// For ply demo, just use the area because we know they are
				// positive. 
				if(area > maxArea){
					cout << "MAX\n";
					maxArea = area;
					maxPoints = intersectionPoints;
					maxAngle = angle;

				}
		}else{
				// But for real data take the absolute value of the area,
				// in case the point cloud is below Y = 0.

				if(abs(area) > maxArea){
					cout << "MAX\n";
					maxArea = area;
					maxPoints = intersectionPoints;
					maxAngle = angle;

				}
		}


	}

	cout << "-_-_-_-_-_-_-\n\n";
	cout << "Optimal Approach Angle " << maxAngle << "\n";
	cout << "Area Under Curve " << maxArea << "\n"; 
	cout << "-_-_-_-_-_-_-\n\n";


	if(useViewer){
			/* Draw spheres on the intersection*/
			char name [10];
			int i = 0;
			for(std::vector<pcl::PointXYZ>::iterator ptr = maxPoints.begin();
				ptr < maxPoints.end(); ptr++){

				i++;
	
				// Put spheres on all points
				sprintf(name, "%d", i);

				// Don't plot as many points on Ply demonstration
				// Also, don't make them as big
				if(usePly){
					if(i%15)
						continue;
					viewer->addSphere(*ptr, 0.01, name);
				}else{
					//point = &(heapCloudPtr->points[*ptr]);
					viewer->addSphere(*ptr, 0.5, name);
				}
			}
	}

  return maxAngle;


}


int main (int argc, char** argv){


	// Create cloud and it's ptr
	pcl::PointCloud<pcl::PointXYZ> pclCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(&pclCloud);
	std::cout << "Created pc" << std::endl;




	pcl::PolygonMesh polyMesh;

	// Read the ply
	pcl::io::loadPLYFile("heap.ply", polyMesh);
	std::cout << "Read ply" << std::endl;

	pcl::fromPCLPointCloud2(polyMesh.cloud, pclCloud);

	// Normalize the pointcloud
	float minX, minY, minZ;
	float maxX, maxY, maxZ;
	uint16_t idxX, idxY, idxZ;

	computeMinPoints(pclCloud, minX, minY, minZ);
	computeMaxPoints(pclCloud, maxX, maxY, maxZ, idxX, idxY, idxZ);

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
	computeMaxPoints(pclCloud, maxX, maxY, maxZ, idxX, idxY, idxZ);

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
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(pclCloud, *transformed_cloud, transform);

	// Print the transformation
	printf ("\nTransforming using an Affine3f\n");
	std::cout << transform.matrix() << std::endl;


	// Rotate point cloud 180 degrees (PI radians)
	Eigen::Affine3f transformA = Eigen::Affine3f::Identity();
	transformA.rotate(Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitY()));
	pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transformA);


	// Remove all points below Y = 0
	pcl::PointCloud<pcl::PointXYZ>::Ptr sliced_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(transformed_cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(0, 10);
	pass.filter(*sliced_cloud);

	// Add Ground Plane @ Y = 0
	pcl::ModelCoefficients plane_coeff;
	plane_coeff.values.resize(4);
	plane_coeff.values[0] = 0;
	plane_coeff.values[1] = 1;
	plane_coeff.values[2] = 0;
	plane_coeff.values[3] = 0;
	//viewer->addPlane(plane_coeff, "ground_plane");

	// Compute the peak
	computeMaxPoints(*sliced_cloud, maxX, maxY, maxZ, idxX, idxY, idxZ);
	float x, y, z;
	x = sliced_cloud->points[idxY].x;
	y = sliced_cloud->points[idxY].y;
	z = sliced_cloud->points[idxY].z;
	pcl::PointXYZ peak(x, y, z);

	// Set oh heap origin at peak
	transform = Eigen::Affine3f::Identity();
	transform.translation() << -peak.x, 0.0, -peak.z;
	pcl::transformPointCloud(*sliced_cloud, *transformed_cloud, transform);
	*sliced_cloud = *transformed_cloud;

	// Create visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = simpleVis(sliced_cloud);
	std::cout << "Visualizer Created" << std::endl;
	peak.x = 0.0;
	peak.z = 0.0;
	viewer->addSphere(peak, 0.03, "peak");

	// Create reference plane
  uint32_t width = 100;
	uint32_t height = width;
	pcl::PointXYZ def(0,0,0); // Default value
	pcl::PointCloud<pcl::PointXYZ> refPlane(width, height, def);
	pcl::PointCloud<pcl::PointXYZ>::Ptr refPlanePtr(&refPlane);
	cout << "Size: " << refPlane.size();
	createReferencePlane(&refPlane, width, height, 1, 1);

	// Transform origin of refPlane to y=0 value of peak
	float new_x, new_y, new_z = 0;
	new_x = peak.x;
  new_y = 0;
	new_z = peak.z;

	Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
	transform2.translation() << new_x, new_y, new_z;
	pcl::transformPointCloud(refPlane, *transformed_cloud, transform2);
	refPlane = *transformed_cloud;

	// Scan for optimal entry
	pcl::PointCloud<pcl::PointXYZ>::Ptr ref2;
	pcl::PointXYZ point(100.0, 100.0, 100.0); // Place holder
	scanForEntry(sliced_cloud, refPlanePtr, point, viewer, 1, 1);
	viewer->addPointCloud<pcl::PointXYZ>(refPlanePtr, "refPlane");


	while(!viewer->wasStopped()){
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return(0);
}
