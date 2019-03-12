/**
*	SkeletonAnalyzer
*	\Description �ԹǼܽ��д���������ȡ�Ǽܣ����ƣ�ӳ��ɨ��ĵ��Ƶ��Ǽܵ�����棻����Ӧ��ӳ����PCA����
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
	//! ��ȡ�Ǽ�
	bool readL1Skeleton(string skepath);

	//! ��ȡɨ�����
	bool readL1ScanPC(string pcpath);

	//! �õ�������֮�������
	Eigen::Vector3f getTangentLine(Eigen::Vector3f v1, Eigen::Vector3f v2) {
		return v1 - v2;
	}

	//! ��ÿ���Ǽܵ��Ӧ��ӳ�����PCA����
	vector<vector<Toolbox::PCAEigen>> skeletonPlaneAnalysis();

	//! ӳ��ɨ��ĵ��Ƶ��Ǽܵ������(ʹ�ùǼܵ���scan pointCloud�ϵ������)
	vector<vector<Eigen::Vector3f>> projectPCPts2SkePlane(Branch m_branch);

	//! ӳ��ɨ��ĵ��Ƶ��Ǽܵ������(ʹ��scan pointCloud���Ǽ������ϵ������)
	vector<vector<Eigen::Vector3f>> computerNearsetPts2Planes(Branch& m_branch, vector<vector<float>>& vec_param);

	//!	���ش���õĹǼ�
	Skeleton getSkeleton() {
		return skeleton;
	}

	//! ���ض�ȡ�ĵ���
	vector<Eigen::Vector3f> getPointCloud() {
		return pc_pts;
	}
	
private:
	//! ɨ��������ݵ�
	vector<Eigen::Vector3f> pc_pts;

	//! �Ǽܵ�
	Skeleton skeleton;

	//! Ԥץȡ��״
	vector<pair<int, pair<bool, bool>>> preshape;	//��״(first)��0  round; 1 rectangle. ץȡ(second): 0 power; 1 precision.��true������ʣ�false�������ʣ�

	//! �Ǽܵ������
	//vector<vector<int>> skeleton_type;//0 crossing point; 1 connecting point; 2 end point.

	//!	����ӽ���ķ���
	void calculateApproachPlane(Eigen::Vector3f &pos, Eigen::Vector3f &dir1, Eigen::Vector3f &dir2, Eigen::Vector3f &result);
};

