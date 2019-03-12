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
	//! 处理一个骨架点上的所有可行方向，并生成相应的姿态
	void generateApproachPoses(Toolbox::Plane Vplane, Eigen::Vector3f currentVertex, vector<Eigen::Vector3f> approachDirs, bool endpoint, vector<Eigen::Vector3f>& vec_poses);

	//! 通过位置，接近方向，Y轴方向 生成姿态
	Eigen::Matrix4f getApproachPose(Eigen::Vector3f position, Eigen::Vector3f approachDir, Eigen::Vector3f dirY);

	//! 圆接近方向
	vector<Eigen::Vector3f> calculateApproachDirRound(Toolbox::PCAEigen pca, bool endpoint);
	
	//!	长方形接近方向
	vector<Eigen::Vector3f> calculateApproachDirRectangular(Toolbox::PCAEigen pca, bool endpoint);
	
	//!	联通点接近方向
	vector<Eigen::Vector3f> calculateApproachesConnectionPoint(const Toolbox::PCAEigen &pca);
	
	//!	末端点接近方向
	vector<Eigen::Vector3f> calculateApproachesEndpoint(const Toolbox::PCAEigen &pca);

	//!	设置抓取参数
	void setParameters(StructClass::PlanningParameters &p);

protected:
	Skeleton m_skeleton;
	StructClass::PlanningParameters approachMovementParameters;
	DeciderGraspPreshape decider;
};

