#pragma once
#include "Toolbox.h"

using namespace std;


class SkeletonConfidenceRatingMethod
{
public:
	SkeletonConfidenceRatingMethod(string filepath, string pcpath);
	~SkeletonConfidenceRatingMethod();

	void skeletonMeasurement(double ratio);

	bool readPointCloud(string path) {
		if (!Toolbox::readPointCloud(path, pc_pts)) {
			cout << "Read File ERROR!" << endl;
			return false;
		}
		cout << "Read Skeleton successfully!" << endl;
		return true;
	}
	bool readSkeleton(string path) {
		if (!Toolbox::readPointCloud(path, skeleton_pts)) {
			cout << "Read File ERROR!" << endl;
			return false;
		}
		cout << "Read PointCloud successfully!" << endl;
		return true;
	}
	vector<pair<double, double>> densityMeasurement();
	vector<vector<double>> scanPCMeasurement();

private:
	


private:
	vector<Eigen::Vector3f> skeleton_pts;
	vector<Eigen::Vector3f> pc_pts;
	Toolbox::BoundingBox skeBB;
	string filepath;
};

