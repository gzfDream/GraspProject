/****************************************************
**
**	GraspPlanner
**
**	\description	该程序实现了对抓取策略的生成，包括位置，姿态，类型（power/precision）
**		
*****************************************************/
#pragma once
#include "Toolbox.h"
#include "Skeleton.h"
#include "LSEllipse.h"
#include "StructClass.h"
#include "SkeletonApproachMovement.h"
#include "SkeletonAnalyzer.h"
#include "DeciderGraspPreshape.h"
class GraspPlanner
{
public:
	GraspPlanner(string ske, string pc, StructClass::PlanningParameters para);
	GraspPlanner(Skeleton ske, StructClass::PlanningParameters para);
	~GraspPlanner();

	//! 生成抓取策略
	void generateGraspStrategies(vector<vector<vector<StructClass::GraspStrategy>>>& skeletonStragety);

private:
	//! 通过骨架间隔判断抓取类型
	void decideSkeletonGraspType(vector<vector<Toolbox::PCAEigen>> pca, vector<vector<pair<StructClass::GraspType, StructClass::GraspType>>> &ske_preshape);
	
	//! 得到抓取姿态
	vector<Eigen::Vector3f> decideGraspPose(vector<vector<Toolbox::PCAEigen>> skePCA);

	//! 得到抓取方向
	void generateGraspApproach(vector<vector<vector<Eigen::Vector3f>>>& graspApproaches, vector<vector<Toolbox::PCAEigen>> ske_pca);

private:
	SkeletonAnalyzer *skeAnalyzer;
	DeciderGraspPreshape decider;

	Skeleton skeleton;
	vector<Eigen::Vector3f> pc_pts;
	StructClass::PlanningParameters param;
	float figureWidth;
	float handWidth;
	string ske_path;

};

