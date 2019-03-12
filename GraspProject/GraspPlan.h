#pragma once
#include "Toolbox.h"
#include "Skeleton.h"
#include "LSEllipse.h"
class GraspPlan
{
public:
	GraspPlan(string skepath, string pcpath);
	~GraspPlan();

	struct GraspStrategy
	{
		Eigen::Vector3f position;
		Eigen::Vector3f direction;
		int preshape;
	};

	void graspStrategiesGenerate();
	vector<vector<Eigen::Vector3f>> projectPCPts2SkePlane(Branch m_branch);
	vector<vector<Eigen::Vector3f>> computerNearsetPts2Planes(Branch m_branch, vector<vector<float>>& vec_param);

	vector<vector<Toolbox::PCAEigen>> skeletonPlaneAnalysis();

private:
	bool decidePowerPreshapeInterval(vector<vector<pair<bool, bool>>> &graspInterval);

	bool readL1Skeleton(string skepath);
	bool readL1ScanPC(string pcpath);
	Eigen::Vector3f getTangentLine(Eigen::Vector3f v1, Eigen::Vector3f v2) {
		return v1 - v2;
	}
	bool calculatePreshape(double lam1, double lam2);
	void setThicknessPrecision(float minThickness, float maxThickness);
	void setThicknessPower(float minThickness, float maxThickness);
	bool decidePowerPreshape(float length, float thickness);
	bool decidePrecisionPreshape(float length, float thickness);
	vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> decideGraspPose(vector<vector<Toolbox::PCAEigen>> skePCA);

	vector<Eigen::Vector3f> calculateApproachDirRound(Toolbox::PCAEigen pca, bool endpoint);
	vector<Eigen::Vector3f> calculateApproachDirRectangular(Toolbox::PCAEigen pca, bool endpoint);
	vector<Eigen::Vector3f> calculateApproachesConnectionPoint(const Toolbox::PCAEigen &pca);
	vector<Eigen::Vector3f> calculateApproachesEndpoint(const Toolbox::PCAEigen &pca);

	void generateApproachDir(Toolbox::PCAEigen pca, Eigen::Vector3f currentVertex, vector<Eigen::Vector3f> approachDirs, bool endpoint, vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& vec_mat);
	Eigen::Matrix4f getApproachPose(Eigen::Vector3f position, Eigen::Vector3f approachDir, Eigen::Vector3f dirY);
private:
	vector<Eigen::Vector3f> pc_pts;
	Skeleton skeleton;
	vector<pair<int, pair<bool, bool>>> preshape;	//形状(first)：0  round; 1 rectangle. 抓取(second): 0 power; 1 precision.（true代表合适，false代表不合适）
	vector<vector<int>> skeleton_type;//0 crossing point; 1 connecting point; 2 end point.
	//vector<vector<Toolbox::PCAEigen>> skePCA;//骨架点PCA

	double threshold_shape = 1.2;

	double minThicknessPower = 0.020f;
	double maxThicknessPower = 0.080f;
	double minLengthPower = 0.020f;

	double minThicknessPrecision = 0.001f;
	double maxThicknessPrecision = 0.050f;

	double figureWidth = 0.02;
	double handWidth = 0.15;
};

