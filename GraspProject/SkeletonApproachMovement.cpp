#include "SkeletonApproachMovement.h"



SkeletonApproachMovement::SkeletonApproachMovement(Skeleton ske)
{
	m_skeleton = ske;
	decider.setThicknessPrecision(approachMovementParameters.minThickness[StructClass::Precision], approachMovementParameters.maxThickness[StructClass::Precision]);
	decider.setThicknessPower(approachMovementParameters.minThickness[StructClass::Power], approachMovementParameters.maxThickness[StructClass::Power]);
}


SkeletonApproachMovement::~SkeletonApproachMovement()
{
}


/**
* generateApproachPoses
* \brief
* \param
* \return
**/
void SkeletonApproachMovement::generateApproachPoses(Toolbox::Plane Vplane, Eigen::Vector3f currentVertex, vector<Eigen::Vector3f> approachDirs, 
	bool endpoint, vector<Eigen::Vector3f>& vec_poses) {

	for (int i = 0; i < approachDirs.size(); i++) {
		Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
		Eigen::Vector3f position = currentVertex;
		Eigen::Vector3f approachDir = approachDirs[i];

		Eigen::Vector3f dirY = Vplane.n;

		if (endpoint)
		{
			Eigen::Vector3f tmp = dirY;
			dirY = approachDir;
			approachDir = tmp * -1;// since in setEEFToApproachPose, the approachDir is multiplied with -1, we need to adapt our input
		}

		if (i % 2 != 0)
		{
			dirY *= -1;
		}

		pose = getApproachPose(position, approachDir, dirY);

		Eigen::Vector3f pose_rpy;
		Toolbox::eigen4f2rpy(pose, pose_rpy);
		vec_poses.push_back(pose_rpy);
	}

	// move away until valid
	//bool ok = moveEEFAway(approachDir, 1.0f, 150);
	//// move away from object (power grasps)
	//if (ok && approachMovementParameters.retreatDistance[getCurrentGraspType()]>0)
	//{
	//	if (verbose)
	//	{
	//		VR_INFO << "Retreating " << approachMovementParameters.retreatDistance[getCurrentGraspType()] << "mm" << endl;
	//	}
	//	VirtualRobot::SceneObjectSetPtr sos = eef_cloned->createSceneObjectSet();
	//	Eigen::Vector3f delta = approachDir * approachMovementParameters.retreatDistance[getCurrentGraspType()];
	//	Eigen::Matrix4f ep = getEEFPose();
	//	updateEEFPose(delta);
	//	if (eef_cloned->getCollisionChecker()->checkCollision(object->getCollisionModel(), sos))
	//	{
	//		if (verbose)
	//		{
	//			VR_INFO << "Retreat pose in collision, restoring original pose" << endl;
	//		}
	//		setEEFPose(ep);
	//		cout << "index: " << index++ << endl;
	//	}
	//}

}



/**
* getApproachPose
* \brief	得到接近的姿态
* \param	position	位置
* \param	approachDir 接近方向
* \param	dirY		Y轴方向
* \return
**/
Eigen::Matrix4f SkeletonApproachMovement::getApproachPose(Eigen::Vector3f position, Eigen::Vector3f approachDir, Eigen::Vector3f dirY) {
	// target pose
	Eigen::Matrix4f poseFinal = Eigen::Matrix4f::Identity();

	// position
	poseFinal.block(0, 3, 3, 1) = position;

	//target orientation
	Eigen::Vector3f z = approachDir;

	z.normalize();
	z *= -1.0f;

	Eigen::Vector3f y = dirY;


	Eigen::Vector3f x;
	x = y.cross(z);
	x.normalize();

	poseFinal.block(0, 0, 3, 1) = x;
	poseFinal.block(0, 1, 3, 1) = y;
	poseFinal.block(0, 2, 3, 1) = z;

	return poseFinal;
}



/**
* calculateApproachDirRound
* \brief	计算接近方向对round
* \param	pca
* \param	endpoint  是否是endpoint
* \return	接近方向的vector
**/
vector<Eigen::Vector3f> SkeletonApproachMovement::calculateApproachDirRound(Toolbox::PCAEigen pca, bool endpoint)
{
	vector<Eigen::Vector3f> approachDirs;
	Eigen::Vector3f a1 = pca.eigenvector1;
	Eigen::Vector3f b1 = pca.eigenvector2;
	Eigen::Vector3f a2 = pca.eigenvector1 * (-1);
	Eigen::Vector3f b2 = pca.eigenvector2 * (-1);

	Eigen::Vector3f a = Toolbox::createMidVector(pca.eigenvector1, pca.eigenvector2);
	Eigen::Vector3f b = Toolbox::createMidVector(pca.eigenvector1  * (-1), pca.eigenvector2);
	Eigen::Vector3f c = Toolbox::createMidVector(pca.eigenvector1  * (-1), pca.eigenvector2 * (-1));
	Eigen::Vector3f d = Toolbox::createMidVector(pca.eigenvector1, pca.eigenvector2 * (-1));

	approachDirs.push_back(a1);
	approachDirs.push_back(b1);
	approachDirs.push_back(a2);
	approachDirs.push_back(b2);

	if (!endpoint)
	{
		approachDirs.push_back(a1);
		approachDirs.push_back(a2);
		approachDirs.push_back(b1);
		approachDirs.push_back(b2);
	}

	approachDirs.push_back(a);
	approachDirs.push_back(b);
	approachDirs.push_back(c);
	approachDirs.push_back(d);

	if (!endpoint)
	{
		approachDirs.push_back(a);
		approachDirs.push_back(b);
		approachDirs.push_back(c);
		approachDirs.push_back(d);
	}

	return approachDirs;
}


/**
* calculateApproachDirRectangular
* \brief	计算接近方向对rectangle
* \param	pca
* \param	endpoint  是否是endpoint
* \return	接近方向的vector
**/
vector<Eigen::Vector3f> SkeletonApproachMovement::calculateApproachDirRectangular(Toolbox::PCAEigen pca, bool endpoint)
{
	vector<Eigen::Vector3f> approachDirs;
	approachDirs.push_back(pca.eigenvector1);
	approachDirs.push_back(pca.eigenvector1);

	Eigen::Vector3f approach = pca.eigenvector1 * (-1);

	if (!endpoint)
	{
		approachDirs.push_back(approach);
		approachDirs.push_back(approach);
	}

	return approachDirs;
}


/**
* calculateApproachesConnectionPoint
* \brief	计算接近方向对connection
* \param	pca
* \param	endpoint  是否是endpoint
* \return	接近方向的vector
**/
vector<Eigen::Vector3f> SkeletonApproachMovement::calculateApproachesConnectionPoint(const Toolbox::PCAEigen &pca)
{
	float ratio = pca.length / pca.thickness;
	vector<Eigen::Vector3f> approachDirs;
	if (ratio < approachMovementParameters.roundThreshold)
	{
		approachDirs = calculateApproachDirRound(pca, false);
	}
	else {
		approachDirs = calculateApproachDirRectangular(pca, false);
	}

	return approachDirs;
}


/**
* calculateApproachesEndpoint
* \brief	计算接近方向对endpoint
* \param	pca
* \param	endpoint  是否是endpoint
* \return	接近方向的vector
**/
vector<Eigen::Vector3f> SkeletonApproachMovement::calculateApproachesEndpoint(const Toolbox::PCAEigen &pca)
{
	float ratio = pca.eigenvalues[0] / pca.eigenvalues[0];
	vector<Eigen::Vector3f> approachDirs;
	if (ratio < approachMovementParameters.roundThreshold)
	{
		approachDirs = calculateApproachDirRound(pca, true);
	}
	else {
		approachDirs = calculateApproachDirRectangular(pca, true);
	}

	return approachDirs;
}


/**
* setParameters
* \brief	设置抓取的参数
* \param	
* \param	
* \return	
**/
void SkeletonApproachMovement::setParameters(StructClass::PlanningParameters &p)
{
	approachMovementParameters = p;
	
	decider.setThicknessPrecision(approachMovementParameters.minThickness[StructClass::Precision], approachMovementParameters.maxThickness[StructClass::Precision]);
	decider.setThicknessPower(approachMovementParameters.minThickness[StructClass::Power], approachMovementParameters.maxThickness[StructClass::Power]);
}
