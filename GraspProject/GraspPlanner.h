/****************************************************
**
**	GraspPlanner
**
**	\description	�ó���ʵ���˶�ץȡ���Ե����ɣ�����λ�ã���̬�����ͣ�power/precision��
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

	//! ����ץȡ����
	void generateGraspStrategies(vector<vector<vector<StructClass::GraspStrategy>>>& skeletonStragety);

private:
	//! ͨ���Ǽܼ���ж�ץȡ����
	void decideSkeletonGraspType(vector<vector<Toolbox::PCAEigen>> pca, vector<vector<pair<StructClass::GraspType, StructClass::GraspType>>> &ske_preshape);
	
	//! �õ�ץȡ��̬
	vector<Eigen::Vector3f> decideGraspPose(vector<vector<Toolbox::PCAEigen>> skePCA);

	//! �õ�ץȡ����
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

