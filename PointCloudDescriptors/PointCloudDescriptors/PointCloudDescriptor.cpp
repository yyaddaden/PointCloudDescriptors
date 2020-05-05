#include "PointCloudDescriptor.h"

/* Constructor & Destructor */

PointCloudDescriptor::PointCloudDescriptor(std::string& pcdFileName, bool rgb, double radiusSearch) {

	this->_pcdFileName = pcdFileName;
	this->_rgb = rgb;
	this->_radiusSearch = radiusSearch;

	this->LoadPcdFile();
}

PointCloudDescriptor::~PointCloudDescriptor() {

}

/* Accessors */

pcl::PointCloud<pcl::SHOT352>::Ptr PointCloudDescriptor::GetDescriptorSHOT() {
	return this->_descriptorSHOT;
}

std::vector<std::vector<float>> PointCloudDescriptor::GetVectDescriptorSHOT() {
	return this->_vectDescriptorSHOT;
}

pcl::PointCloud<pcl::SHOT1344>::Ptr PointCloudDescriptor::GetDescriptorSHOTRgb() {
	return this->_descriptorSHOTRgb;
}

std::vector<std::vector<float>> PointCloudDescriptor::GetVectDescriptorSHOTRgb() {
	return this->_vectDescriptorSHOTRgb;
}

pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr PointCloudDescriptor::GetDescriptorUSC() {
	return this->_descriptorUSC;
}

std::vector<std::vector<float>> PointCloudDescriptor::GetVectDescriptorUSC() {
	return this->_vectDescriptorUSC;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudDescriptor::GetCloud() {
	return this->_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudDescriptor::GetCloudRgb() {
	return this->_cloudRgb;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudDescriptor::GetCloudDownSample() {
	return this->_cloudDownSample;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudDescriptor::GetCloudDownSampleRgb() {
	return this->_cloudDownSampleRgb;
}

pcl::PointCloud<pcl::Normal>::Ptr PointCloudDescriptor::GetNormals() {
	return this->_normals;
}

/* Methods */

void PointCloudDescriptor::LoadPcdFile() {

	std::cout << "[";
	_setmode(_fileno(stdout), _O_U16TEXT);
	std::wcout << L"\u2713";
	_setmode(_fileno(stdout), _O_TEXT);
	std::cout << "]";

	std::cout << " Loading PCD file : " + this->_pcdFileName + " [";

	if (this->_rgb) {
		this->_cloudRgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

		if (pcl::io::loadPCDFile <pcl::PointXYZRGB>(_pcdFileName.c_str(), *_cloudRgb) == -1) {
			exit(-1);
		}

		std::cout << " (RGB) " << this->_cloudRgb->points.size() << " points ]" << std::endl;
	}
	else {
		this->_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile <pcl::PointXYZ>(_pcdFileName.c_str(), *_cloud) == -1) {
			exit(-1);
		}

		std::cout << " " << this->_cloud->points.size() << " points ]" << std::endl;;
	}
}

void PointCloudDescriptor::ComputeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud) {

	std::cout << "[";
	_setmode(_fileno(stdout), _O_U16TEXT);
	std::wcout << L"\u2713";
	_setmode(_fileno(stdout), _O_TEXT);
	std::cout << "]";
	
	std::cout << " Computing Normals" << std::endl;;

	this->_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setInputCloud(_cloud);

	normalEstimation.setRadiusSearch(this->_radiusSearch);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*_normals);
}

void PointCloudDescriptor::ComputeNormalsRgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloudRgb) {

	std::cout << "[";
	_setmode(_fileno(stdout), _O_U16TEXT);
	std::wcout << L"\u2713";
	_setmode(_fileno(stdout), _O_TEXT);
	std::cout << "]";

	std::wcout << " Computing Normals (RGB)" << std::endl;

	this->_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	normalEstimation.setInputCloud(_cloudRgb);

	normalEstimation.setRadiusSearch(this->_radiusSearch);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*_normals);
}

void PointCloudDescriptor::DownSampling(float voxelSize) {

	std::cout << "[";
	_setmode(_fileno(stdout), _O_U16TEXT);
	std::wcout << L"\u2713";
	_setmode(_fileno(stdout), _O_TEXT);
	std::cout << "]";

	if (this->_rgb) {
		this->_cloudDownSampleRgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::VoxelGrid<pcl::PointXYZRGB> sor;
		sor.setInputCloud(this->_cloudRgb);
		sor.setLeafSize(voxelSize, voxelSize, voxelSize);
		sor.filter(*this->_cloudDownSampleRgb);

		std::cout << " Downsampling : [ " << this->_cloudRgb->points.size() << " points -> " << this->_cloudDownSampleRgb->points.size() << " points ]" << std::endl;
	}
	else {
		this->_cloudDownSample = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(this->_cloud);
		sor.setLeafSize(voxelSize, voxelSize, voxelSize);
		sor.filter(*this->_cloudDownSample);

		std::cout << " Downsampling : [ " << this->_cloud->points.size() << " points -> " << this->_cloudDownSample->points.size() << " points ]" << std::endl;
	}
}

void PointCloudDescriptor::ShowPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud) {

	pcl::visualization::CloudViewer viewer("Point Cloud Viewer");

	viewer.showCloud(_cloud);

	while (!viewer.wasStopped())
	{
	}
}

void PointCloudDescriptor::ShowPointCloudRgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloudRgb) {

	pcl::visualization::CloudViewer viewer("Point Cloud Viewer (RGB)");

	viewer.showCloud(_cloudRgb);

	while (!viewer.wasStopped())
	{
	}
}

void PointCloudDescriptor::ComputeSHOTFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud, pcl::PointCloud<pcl::Normal>::Ptr _normals) {

	std::cout << "[";
	_setmode(_fileno(stdout), _O_U16TEXT);
	std::wcout << L"\u2713";
	_setmode(_fileno(stdout), _O_TEXT);
	std::cout << "]";

	std::cout << " Computing SHOT Features" << std::endl;
	
	this->_descriptorSHOT = pcl::PointCloud<pcl::SHOT352>::Ptr(new pcl::PointCloud<pcl::SHOT352>());

	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	shot.setInputCloud(_cloud);
	shot.setInputNormals(_normals);
	shot.setRadiusSearch(this->_radiusSearch);

	shot.compute(*this->_descriptorSHOT);

	for (size_t i = 0; i < this->_descriptorSHOT->points.size(); i++)
	{
		this->_vectDescriptorSHOT.push_back(std::vector<float>());

		for (size_t j = 0; j < this->_descriptorSHOT->points[i].descriptorSize(); j++)
		{
			this->_vectDescriptorSHOT[i].push_back(this->_descriptorSHOT->points[i].descriptor[j]);
		}
	}
}

void PointCloudDescriptor::ComputeSHOTRGBFeature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloudRgb, pcl::PointCloud<pcl::Normal>::Ptr _normals) {

	std::cout << "[";
	_setmode(_fileno(stdout), _O_U16TEXT);
	std::wcout << L"\u2713";
	_setmode(_fileno(stdout), _O_TEXT);
	std::cout << "]";
	
	std::cout << " Computing SHOT RGB Features" << std::endl;
	
	this->_descriptorSHOTRgb = pcl::PointCloud<pcl::SHOT1344>::Ptr(new pcl::PointCloud<pcl::SHOT1344>());

	pcl::SHOTColorEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shot;
	shot.setInputCloud(_cloudRgb);
	shot.setInputNormals(_normals);
	shot.setRadiusSearch(this->_radiusSearch);

	shot.compute(*this->_descriptorSHOTRgb);

	for (size_t i = 0; i < this->_descriptorSHOTRgb->points.size(); i++)
	{
		this->_vectDescriptorSHOTRgb.push_back(std::vector<float>());

		for (size_t j = 0; j < this->_descriptorSHOTRgb->points[i].descriptorSize(); j++)
		{
			this->_vectDescriptorSHOTRgb[i].push_back(this->_descriptorSHOTRgb->points[i].descriptor[j]);
		}
	}
}

void PointCloudDescriptor::ComputeUSCFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud) {

	std::cout << "[";
	_setmode(_fileno(stdout), _O_U16TEXT);
	std::wcout << L"\u2713";
	_setmode(_fileno(stdout), _O_TEXT);
	std::cout << "]";

	std::cout << " Computing USC Features" << std::endl;
	
	this->_descriptorUSC = pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr(new pcl::PointCloud<pcl::UniqueShapeContext1960>());

	pcl::UniqueShapeContext<pcl::PointXYZ, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> usc;
	usc.setInputCloud(_cloud);
	usc.setRadiusSearch(this->_radiusSearch);
	usc.setMinimalRadius(this->_radiusSearch / (this->_radiusSearch * 200));
	usc.setPointDensityRadius(this->_radiusSearch / (this->_radiusSearch * 100));
	usc.setLocalRadius(this->_radiusSearch);

	usc.compute(*this->_descriptorUSC);

	for (size_t i = 0; i < this->_descriptorUSC->points.size(); i++)
	{
		this->_vectDescriptorUSC.push_back(std::vector<float>());

		for (size_t j = 0; j < this->_descriptorUSC->points[i].descriptorSize(); j++)
		{
			this->_vectDescriptorUSC[i].push_back(this->_descriptorUSC->points[i].descriptor[j]);
		}
	}
}

void PointCloudDescriptor::ExportDescriptorNpy(std::vector<std::vector<float>> inputVector, std::string npyFileName) {

	std::cout << "[";
	_setmode(_fileno(stdout), _O_U16TEXT);
	std::wcout << L"\u2713";
	_setmode(_fileno(stdout), _O_TEXT);
	std::cout << "]";

	std::cout << " Export Feature Vector " + npyFileName << std::endl;
	
	std::vector<float> outputVector;

	for (size_t i = 0; i < inputVector.size(); i++)
	{
		for (size_t j = 0; j < inputVector[i].size(); j++)
		{
			outputVector.push_back(inputVector[i][j]);
		}
	}

	cnpy::npy_save(npyFileName, &outputVector[0], { inputVector.size(), inputVector[0].size() }, "w");
}