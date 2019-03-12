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
* \brief	��Ԥ��ĵ��ƽ������Ŷȷ���
* \param	ratio	��ֵ����boundingbox�ı���, ��ʼ0.1%��
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


	//���洦����
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
* \brief	ͨ���ܶȣ���֮��ľ���;��ȳ̶ȣ���Ԥ��ĵ��ƽ������Ŷȷ���
* \param	
* \return	vec_mean_variance	ƽ������ͷ���
**/
vector<pair<double, double>> SkeletonConfidenceRatingMethod::densityMeasurement() {
	
	vector<vector<pair<int, double>>> vec_neighbors;	//���ڵ���Ϣ
	bool isneighbprs = Toolbox::computeAnnNeighbors(skeleton_pts, skeleton_pts, 64, vec_neighbors);

	//���ڵ�
	pair<double, double> mean_variance;//ƽ�����ͷ���
	vector<pair<double, double>> vec_mean_variance;
	//double testmax1=0, testmax2=0;
	//double testmin1 = 1, testmin2 = 1;
	for (int i = 0; i < vec_neighbors.size(); i++){

		//��������ƽ��
		double sum = 0;
		for (int j = 0; j < vec_neighbors[i].size(); j++) {
			sum += vec_neighbors[i][j].second;
		}
		double mean = sum / vec_neighbors[i].size();

		//�������뷽��
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
* \brief	ͨ����������㵽ɨ����Ƶľ��루����Ԥ��ĵ��ƽ������Ŷȷ���
* \param	
* \return	�Ǽܵ㵽����������ƽ������͹Ǽܵ��ڽ��㵽����������ƽ������
**/
vector<vector<double>> SkeletonConfidenceRatingMethod::scanPCMeasurement() {
	vector<vector<double>> meanDistance;

	vector<vector<pair<int, double>>> ske2pc_neighbors;
	int knn = 4;
	bool is = Toolbox::computeAnnNeighbors(pc_pts, skeleton_pts, knn, ske2pc_neighbors);

	vector<vector<pair<int, double>>> ske2ske_neighbors;
	int kn = 8;
	bool iss = Toolbox::computeAnnNeighbors(skeleton_pts, skeleton_pts, kn, ske2ske_neighbors);

	//����ÿ���Ǽܵ�����kn���㵽���Ƶľ����ƽ��ֵ
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
	
	//����Ǽܵ㵽���Ƶ�knn������ƽ������
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