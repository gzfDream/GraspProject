#pragma once
#include <algorithm>
#include <string>
#include <list>
#include <sstream>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <float.h>
#include <map>
namespace StructClass{

	enum VertexType
	{
		BranchingVertex,
		EndpointVertex,
		ConnectingVertex
	};

	enum GraspType {
		Power,
		Precision,
		m_null
	};

	//! 抓取参数类 (SkeletonApproachMovement)
	struct PlanningParameters {

		PlanningParameters() {
			// ARMAR-III parameters
			interval[Power] = 0.0940f;
			interval[Precision] = 0.0470f;
			minThickness[Power] = 0.0200f;
			maxThickness[Power] = 0.0600f;
			minThickness[Precision] = 0.0001f;
			maxThickness[Precision] = 0.0200f;
			minLength[Power] = 0.020f;
			minLength[Precision] = 0.0010f;
			maxLength[Power] = 0.020f;
			maxLength[Precision] = 0.0300f;
			preshapeName[Power] = "Power Preshape";
			preshapeName[Precision] = "Precision Preshape";
		}

		// pca ratio of first two eigenvalues to identify round vs rectangular objects
		float roundThreshold = 1.3f;

		std::map<GraspType, float> interval;
		std::map<GraspType, float> minThickness;
		std::map<GraspType, float> maxThickness;
		std::map<GraspType, float> minLength;
		std::map<GraspType, float> maxLength;
		std::map<GraspType, float> retreatDistance;
		std::map<GraspType, std::string> preshapeName;
	};

	struct GraspStrategy
	{
		Eigen::Vector3f position = Eigen::Vector3f(0, 0, 0);
		std::pair<GraspType, Eigen::Vector3f> type_pose;
	};
}