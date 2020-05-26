#include<iostream>

// load PointCloudDescriptor class
#include "PointCloudDescriptor.h"

using namespace std;

void main(int argc, char** argv) {
	cout << "-- Start --" << endl;

	PointCloudDescriptor* pointCloudDescriptor;

	std::string descriptor_type, pcd_in_file, npy_out_file;
	double radius_search;

	if (argc >= 4 && argc <= 6) {
		descriptor_type = argv[1];
		pcd_in_file = argv[2];
		npy_out_file = argv[3];
		radius_search = std::stod(argv[4]);

		switch (std::stoi(descriptor_type))
		{
		case 1:
			pointCloudDescriptor = new PointCloudDescriptor(pcd_in_file, false, radius_search);

			if (argc == 6) {
				pointCloudDescriptor->DownSampling(std::stof(argv[5]));
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDownSample = pointCloudDescriptor->GetCloudDownSample();
				pointCloudDescriptor->ComputeNormals(cloudDownSample);
				pointCloudDescriptor->ComputeSHOTFeature(cloudDownSample, pointCloudDescriptor->GetNormals());
				pointCloudDescriptor->ExportDescriptorNpy(pointCloudDescriptor->GetVectDescriptorSHOT(), npy_out_file);
			}
			else {
				pointCloudDescriptor->ComputeNormals(pointCloudDescriptor->GetCloud());
				pointCloudDescriptor->ComputeSHOTFeature(pointCloudDescriptor->GetCloud(), pointCloudDescriptor->GetNormals());
				pointCloudDescriptor->ExportDescriptorNpy(pointCloudDescriptor->GetVectDescriptorSHOT(), npy_out_file); 
			}

			break;
		case 2:
			pointCloudDescriptor = new PointCloudDescriptor(pcd_in_file, true, radius_search);

			if (argc == 6) {
				pointCloudDescriptor->DownSampling(std::stof(argv[5]));
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDownSampleRgb = pointCloudDescriptor->GetCloudDownSampleRgb();
				pointCloudDescriptor->ComputeNormalsRgb(cloudDownSampleRgb);
				pointCloudDescriptor->ComputeSHOTRGBFeature(cloudDownSampleRgb, pointCloudDescriptor->GetNormals());
				pointCloudDescriptor->ExportDescriptorNpy(pointCloudDescriptor->GetVectDescriptorSHOTRgb(), npy_out_file);
			}
			else {
				pointCloudDescriptor->ComputeNormalsRgb(pointCloudDescriptor->GetCloudRgb());
				pointCloudDescriptor->ComputeSHOTRGBFeature(pointCloudDescriptor->GetCloudRgb(), pointCloudDescriptor->GetNormals());
				pointCloudDescriptor->ExportDescriptorNpy(pointCloudDescriptor->GetVectDescriptorSHOTRgb(), npy_out_file);
			}

			break;
		case 3:
			pointCloudDescriptor = new PointCloudDescriptor(pcd_in_file, false, radius_search);

			if (argc == 6) {
				pointCloudDescriptor->DownSampling(std::stof(argv[5]));
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDownSample = pointCloudDescriptor->GetCloudDownSample();
				pointCloudDescriptor->ComputeUSCFeature(cloudDownSample);
				pointCloudDescriptor->ExportDescriptorNpy(pointCloudDescriptor->GetVectDescriptorUSC(), npy_out_file);
			}
			else {
				pointCloudDescriptor->ComputeUSCFeature(pointCloudDescriptor->GetCloud());
				pointCloudDescriptor->ExportDescriptorNpy(pointCloudDescriptor->GetVectDescriptorUSC(), npy_out_file);
			}
			break;
		default:
			cout << "Error : invalid descriptor ! SHOT -> 1 | SHOTRgb -> 2 | USC -> 3" << endl;
			exit(1);
		}
	}
	else {
		cout << "Error : invalid number of arguments !" << endl;
		cout << "Arg 1 : Descriptor type (int : 1, 2 or 3) -> SHOT -> 1 | SHOTRgb -> 2 | USC -> 3" << endl;
		cout << "Arg 2 : Input PCD file (string) -> filename.pcd" << endl;
		cout << "Arg 3 : Output NPY file (string) -> filename.npy" << endl;
		cout << "Arg 4 : Radius search value (float) -> 0.05 (5 cm)" << endl;
		cout << "Arg 5 (Opt) : Downsampling value (float) -> 0.05 (5 cm)" << endl;
		exit(1);
	}

	cout << "-- End --" << endl;
}