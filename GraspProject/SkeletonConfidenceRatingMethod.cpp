#include "SkeletonConfidenceRatingMethod.h"


//****************************************
// class: SkeletonConfidenceRatingMethod
//****************************************

SkeletonConfidenceRatingMethod::SkeletonConfidenceRatingMethod(string skepath, string pcpath)
{
	filepath = skepath;
	readPointCloud(pcpath);
	readSkeleton(skepath);

	skeBB = Toolbox::computerBB(skeleton_pts);
}


SkeletonConfidenceRatingMethod::~SkeletonConfidenceRatingMethod()
{
}



/**
* densityMeasurement
* \brief	对预测的点云进行置信度分析
* \param	ratio	阈值（与boundingbox的比率, 初始0.1%）
* \return	
**/
void SkeletonConfidenceRatingMethod::skeletonMeasurement(double ratio) {

	vector<pair<double, double>> vec_mean_variance = densityMeasurement();
	vector<vector<double>> dis = scanPCMeasurement();

	vector<Eigen::Vector3f> skeleton_results;
	for (int i = 0; i < skeleton_pts.size(); i++) {
		if (vec_mean_variance[i].first < ratio * skeBB.diagonal_dis) {
			if (vec_mean_variance[i].second < 0.0001) {
				if(dis[0][i]-dis[1][i] < 0.0001)
					skeleton_results.push_back(skeleton_pts[i]);
			}
		}
	}


	//保存处理结果
	string s1 = filepath.replace(filepath.length() - 4, 4, "_.ply");
	ofstream out(s1);
	out << "ply" << endl;
	out << "format ascii 1.0" << endl;
	out << "element vertex " << skeleton_results.size() << endl;
	out << "property float x" << endl;
	out << "property float y" << endl;
	out << "property float z" << endl;
	out << "end_header" << endl;

	for (int i = 0; i < skeleton_results.size(); i++)
		out << setprecision(10) << skeleton_results[i].x() << " " << skeleton_results[i].y() << " " << skeleton_results[i].z() << endl;

	out.close();
}



/**
* densityMeasurement
* \brief	通过密度（点之间的距离和均匀程度）对预测的点云进行置信度分析
* \param	
* \return	vec_mean_variance	平均距离和方差
**/
vector<pair<double, double>> SkeletonConfidenceRatingMethod::densityMeasurement() {
	
	vector<vector<pair<int, double>>> vec_neighbors;	//近邻点信息
	bool isneighbprs = Toolbox::computeAnnNeighbors(skeleton_pts, skeleton_pts, 64, vec_neighbors);

	//近邻点
	pair<double, double> mean_variance;//平均数和方差
	vector<pair<double, double>> vec_mean_variance;
	//double testmax1=0, testmax2=0;
	//double testmin1 = 1, testmin2 = 1;
	for (int i = 0; i < vec_neighbors.size(); i++){

		//最近点距离平均
		double sum = 0;
		for (int j = 0; j < vec_neighbors[i].size(); j++) {
			sum += vec_neighbors[i][j].second;
		}
		double mean = sum / vec_neighbors[i].size();

		//最近点距离方差
		double variance = 0.;
		for (int j = 0; j < vec_neighbors[i].size(); j++) {
			variance += pow(vec_neighbors[i][j].second - mean, 2);
		}
		variance = sqrtf(variance / vec_neighbors[i].size());

		mean_variance.first = mean;
		mean_variance.second = variance;
		vec_mean_variance.push_back(mean_variance);
		
		/*if (mean > testmax1) testmax1 = mean;
		if (variance > testmax2) testmax2 = variance;
		if (mean < testmin1) testmin1 = mean;
		if (variance < testmin2) testmin2 = variance;*/
	}

	return vec_mean_variance;
}


/**
* scanPCMeasurement
* \brief	通过计算最近点到扫描点云的距离（）对预测的点云进行置信度分析
* \param	
* \return	骨架点到点云最近点的平均距离和骨架点邻近点到点云最近点的平均距离
**/
vector<vector<double>> SkeletonConfidenceRatingMethod::scanPCMeasurement() {
	vector<vector<double>> meanDistance;

	vector<vector<pair<int, double>>> ske2pc_neighbors;
	int knn = 4;
	bool is = Toolbox::computeAnnNeighbors(pc_pts, skeleton_pts, knn, ske2pc_neighbors);

	vector<vector<pair<int, double>>> ske2ske_neighbors;
	int kn = 8;
	bool iss = Toolbox::computeAnnNeighbors(skeleton_pts, skeleton_pts, kn, ske2ske_neighbors);

	//计算每个骨架点的最近kn个点到点云的距离的平均值
	vector<double> mean_dist_nearestPts;
	for (int i = 0; i < skeleton_pts.size(); i++) {
		double mean = 0.;
		for (int j = 0; j < kn; j++)
		{
			double mean_d = 0.;
			int index = ske2ske_neighbors[i][j].first;
			for (int k = 0; k < knn; k++) {
				mean_d += ske2pc_neighbors[index][k].second;
			}
			mean_d = mean_d / knn;
			mean += mean_d;
		}
		mean = mean / kn;
		mean_dist_nearestPts.push_back(mean);
	}
	
	//计算骨架点到点云的knn最近点的平均距离
	vector<double> mean_dist;
	for (int i = 0; i < skeleton_pts.size(); i++) {
		double mean = 0.;
		for (int j = 0; j < knn; j++) {
			mean += ske2pc_neighbors[i][j].second;
		}
		mean = mean / knn;
		mean_dist.push_back(mean);
	}
	
	meanDistance.push_back(mean_dist);
	meanDistance.push_back(mean_dist_nearestPts);

	return meanDistance;
}