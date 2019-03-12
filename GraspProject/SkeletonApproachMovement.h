#pragma once
#include <vector>
#include <map>
#include "StructClass.h"
#include "DeciderGraspPreshape.h"
#include "Skeleton.h"

class SkeletonApproachMovement
{
public:
	SkeletonApproachMovement(Skeleton ske);
	~SkeletonApproachMovement();

	typedef std::pair<Eigen::Vector3f, Eigen::Vector3f> DirAndPos;

public:
	//! ����һ���Ǽܵ��ϵ����п��з��򣬲�������Ӧ����̬
	void generateApproachPoses(Toolbox::Plane Vplane, Eigen::Vector3f currentVertex, vector<Eigen::Vector3f> approachDirs, bool endpoint, vector<Eigen::Vector3f>& vec_poses);

	//! ͨ��λ�ã��ӽ�����Y�᷽�� ������̬
	Eigen::Matrix4f getApproachPose(Eigen::Vector3f position, Eigen::Vector3f approachDir, Eigen::Vector3f dirY);

	//! Բ�ӽ�����
	vector<Eigen::Vector3f> calculateApproachDirRound(Toolbox::PCAEigen pca, bool endpoint);
	
	//!	�����νӽ�����
	vector<Eigen::Vector3f> calculateApproachDirRectangular(Toolbox::PCAEigen pca, bool endpoint);
	
	//!	��ͨ��ӽ�����
	vector<Eigen::Vector3f> calculateApproachesConnectionPoint(const Toolbox::PCAEigen &pca);
	
	//!	ĩ�˵�ӽ�����
	vector<Eigen::Vector3f> calculateApproachesEndpoint(const Toolbox::PCAEigen &pca);

	//!	����ץȡ����
	void setParameters(StructClass::PlanningParameters &p);

protected:
	Skeleton m_skeleton;
	StructClass::PlanningParameters approachMovementParameters;
	DeciderGraspPreshape decider;
};

