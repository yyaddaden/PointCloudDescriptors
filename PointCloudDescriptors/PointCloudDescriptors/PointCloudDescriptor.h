#include <iostream>
#include <vector>
#include <io.h>
#include <fcntl.h>

// load pcl libraries
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

#include <cnpy.h>

#include <pcl/io/vlp_grabber.h>


/* 
	used references for the code source :
		* http://pointclouds.org/documentation/
		* http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_4:_3D_object_recognition_(descriptors)
*/

/* local 3D descriptors */

// descriptor - USC : Unique Shape Context
#include <pcl/features/usc.h>
// descriptor - SHOT : Unique Signatures of Histograms for Local Surface 
#include <pcl/features/shot.h>

// histogram visualization
#include <pcl/visualization/histogram_visualizer.h>

class PointCloudDescriptor {
public:
	PointCloudDescriptor(std::string& pcdFileName, bool rgb, double radiusSearch);
	~PointCloudDescriptor();

	void ComputeSHOTFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud, pcl::PointCloud<pcl::Normal>::Ptr _normals);
	void ComputeSHOTRGBFeature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud, pcl::PointCloud<pcl::Normal>::Ptr _normals);
	void ComputeUSCFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud);

	void LoadPcdFile();
	void DownSampling(float voxelSize);

	void ShowPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud);
	void ShowPointCloudRgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloudRgb);

	void ComputeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud);
	void ComputeNormalsRgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);

	pcl::PointCloud<pcl::SHOT352>::Ptr GetDescriptorSHOT();
	std::vector<std::vector<float>> GetVectDescriptorSHOT();

	pcl::PointCloud<pcl::SHOT1344>::Ptr GetDescriptorSHOTRgb();
	std::vector<std::vector<float>> GetVectDescriptorSHOTRgb();

	pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr GetDescriptorUSC();
	std::vector<std::vector<float>> GetVectDescriptorUSC();

	pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud();
	pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloudDownSample();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloudRgb();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloudDownSampleRgb();
	pcl::PointCloud<pcl::Normal>::Ptr GetNormals();

	void ExportDescriptorNpy(std::vector<std::vector<float>> inputVector, std::string npyFileName);

private:
	bool _rgb;
	double _radiusSearch;
	std::string _pcdFileName;
	pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr _cloudDownSample;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloudRgb;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloudDownSampleRgb;
	pcl::PointCloud<pcl::Normal>::Ptr _normals;

	pcl::PointCloud<pcl::SHOT352>::Ptr _descriptorSHOT;
	std::vector<std::vector<float>> _vectDescriptorSHOT;
	pcl::PointCloud<pcl::SHOT1344>::Ptr _descriptorSHOTRgb;
	std::vector<std::vector<float>> _vectDescriptorSHOTRgb;
	pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr _descriptorUSC;
	std::vector<std::vector<float>> _vectDescriptorUSC;
};