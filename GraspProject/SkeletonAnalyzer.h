/**
*	SkeletonAnalyzer
*	\Description 对骨架进行处理，包括读取骨架，点云；映射扫描的点云到骨架点的切面；对相应的映射做PCA分析
*
**/

#pragma once
#include "Skeleton.h"
#include "Toolbox.h"
#include "LSEllipse.h"


class SkeletonAnalyzer
{
public:
	SkeletonAnalyzer(Skeleton ske, vector<Eigen::Vector3f> pc);
	SkeletonAnalyzer(string skepath, string pcpath);
	~SkeletonAnalyzer();

	enum Shape
	{
		round,
		rectangle
	};

	enum GraspType
	{
		Power,
		Precision
	};

public:
	//! 读取骨架
	bool readL1Skeleton(string skepath);

	//! 读取扫描点云
	bool readL1ScanPC(string pcpath);

	//! 得到两个点之间的切线
	Eigen::Vector3f getTangentLine(Eigen::Vector3f v1, Eigen::Vector3f v2) {
		return v1 - v2;
	}

	//! 对每个骨架点对应的映射点做PCA分析
	vector<vector<Toolbox::PCAEigen>> skeletonPlaneAnalysis();

	//! 映射扫描的点云到骨架点的切面(使用骨架点在scan pointCloud上的最近点)
	vector<vector<Eigen::Vector3f>> projectPCPts2SkePlane(Branch m_branch);

	//! 映射扫描的点云到骨架点的切面(使用scan pointCloud到骨架切面上的最近点)
	vector<vector<Eigen::Vector3f>> computerNearsetPts2Planes(Branch& m_branch, vector<vector<float>>& vec_param);

	//!	返回处理好的骨架
	Skeleton getSkeleton() {
		return skeleton;
	}

	//! 返回读取的点云
	vector<Eigen::Vector3f> getPointCloud() {
		return pc_pts;
	}
	
private:
	//! 扫面点云数据点
	vector<Eigen::Vector3f> pc_pts;

	//! 骨架点
	Skeleton skeleton;

	//! 预抓取形状
	vector<pair<int, pair<bool, bool>>> preshape;	//形状(first)：0  round; 1 rectangle. 抓取(second): 0 power; 1 precision.（true代表合适，false代表不合适）

	//! 骨架点的类型
	//vector<vector<int>> skeleton_type;//0 crossing point; 1 connecting point; 2 end point.

	//!	计算接近面的法向
	void calculateApproachPlane(Eigen::Vector3f &pos, Eigen::Vector3f &dir1, Eigen::Vector3f &dir2, Eigen::Vector3f &result);
};

