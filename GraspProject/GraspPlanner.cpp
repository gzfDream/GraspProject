#include "GraspPlanner.h"


GraspPlanner::GraspPlanner(string ske, string pc, StructClass::PlanningParameters para) {
	ske_path = ske;
	skeAnalyzer = new SkeletonAnalyzer(ske, pc);
	skeleton = skeAnalyzer->getSkeleton();
	pc_pts = skeAnalyzer->getPointCloud();

	param = para;
	figureWidth = param.interval[StructClass::GraspType::Precision];
	handWidth = param.interval[StructClass::GraspType::Power];

	decider.setLengthPower(param.minLength[StructClass::GraspType::Power]);
	decider.setLengthPrecision(param.minLength[StructClass::GraspType::Precision], param.maxLength[StructClass::GraspType::Precision]);
	decider.setThicknessPower(param.minThickness[StructClass::GraspType::Power], param.maxThickness[StructClass::GraspType::Power]);
	decider.setThicknessPrecision(param.minThickness[StructClass::GraspType::Precision], param.maxThickness[StructClass::GraspType::Precision]);
}


GraspPlanner::GraspPlanner(Skeleton ske, StructClass::PlanningParameters para)
{
	skeleton = ske;
	param = para;
	figureWidth = param.interval[StructClass::GraspType::Precision];
	handWidth = param.interval[StructClass::GraspType::Power];
}


GraspPlanner::~GraspPlanner()
{
}


/**
* graspStrategiesGenerate
* \brief	抓取策略生成
* \param
* \return
**/
void GraspPlanner::generateGraspStrategies(vector<vector<vector<StructClass::GraspStrategy>>>& skeletonStragety) {
	
	vector<vector<Toolbox::PCAEigen>> ske_pca = skeAnalyzer->skeletonPlaneAnalysis();	//对骨架进行分析，得到每个点的PCA
	skeleton = skeAnalyzer->getSkeleton();

	vector<vector<pair<StructClass::GraspType, StructClass::GraspType>>> ske_preshape;
	decideSkeletonGraspType(ske_pca, ske_preshape);

	vector<vector<vector<Eigen::Vector3f>>> graspApproaches;
	generateGraspApproach(graspApproaches, ske_pca);
	
	
	for (int i = 0; i < skeleton.branch_num; i++) {
		vector<vector<StructClass::GraspStrategy>> branchStrategy;
		branchStrategy.clear();
		for (int j = 0; j < skeleton.branches[i].curve.size(); j++) {
			vector<StructClass::GraspStrategy> vertexStrategy;
			vertexStrategy.clear();
			StructClass::GraspStrategy strategy;
			if (ske_preshape[i][j].first == StructClass::GraspType::Power) {
				strategy.position = skeleton.branches[i].curve[j];
				strategy.type_pose.first = StructClass::GraspType::Power;
				for (int k = 0; k < graspApproaches[i][j].size(); k++) {
					strategy.type_pose.second = graspApproaches[i][j][k];
					vertexStrategy.push_back(strategy);
				}
			}
			if (ske_preshape[i][j].second == StructClass::GraspType::Precision) {
				strategy.position = skeleton.branches[i].curve[j];
				strategy.type_pose.first = StructClass::GraspType::Precision;
				for (int k = 0; k < graspApproaches[i][j].size(); k++) {
					strategy.type_pose.second = graspApproaches[i][j][k];
					vertexStrategy.push_back(strategy);
				}
			}
			branchStrategy.push_back(vertexStrategy);
			vertexStrategy.clear();
		}
		skeletonStragety.push_back(branchStrategy);
		branchStrategy.clear();
	}
	
}


/**
* generateGraspApproach
* \brief	得到每个骨架点的抓取姿态
* \param	ske_pca			骨架点的PCA
* \param	graspApproaches	得到的姿态
* \return
**/
void GraspPlanner::generateGraspApproach(vector<vector<vector<Eigen::Vector3f>>>& graspApproaches, vector<vector<Toolbox::PCAEigen>> ske_pca) {
	SkeletonApproachMovement skeletonApproach(skeleton);
	string tempStr = ske_path;
	string s1 = tempStr.replace(tempStr.length() - 5, 5, "_approach0.ply");

	for (int i = 0; i < skeleton.branch_num; i++) {
		vector<vector<Eigen::Vector3f>> branchApproach;
		branchApproach.clear();

		vector<vector<Eigen::Vector3f>> Dirs;

		for (int j = 0; j < skeleton.branches[i].curve.size(); j++) {
			vector<Eigen::Vector3f> vec_pose;
			if (skeleton.branches[i].vertexType[j] == StructClass::VertexType::BranchingVertex) {
				vector<Eigen::Vector3f> vec_dir;
				vec_dir.push_back(Eigen::Vector3f(0, 0, 0));
				Dirs.push_back(vec_dir);

				vec_pose.push_back(Eigen::Vector3f(0,0,0));
			}
			else if (skeleton.branches[i].vertexType[j] == StructClass::VertexType::ConnectingVertex) {
				vector<Eigen::Vector3f> vec_dir = skeletonApproach.calculateApproachesConnectionPoint(ske_pca[i][j]);
				Dirs.push_back(vec_dir);
				skeletonApproach.generateApproachPoses(skeleton.branches[i].vertexPlane[j], skeleton.branches[i].curve[j], vec_dir, false, vec_pose);
			}
			else if (skeleton.branches[i].vertexType[j] == StructClass::VertexType::EndpointVertex) {
				vector<Eigen::Vector3f> vec_dir = skeletonApproach.calculateApproachesEndpoint(ske_pca[i][j]);
				Dirs.push_back(vec_dir);

				skeletonApproach.generateApproachPoses(skeleton.branches[i].vertexPlane[j], skeleton.branches[i].curve[j], vec_dir, true, vec_pose);
			}
			branchApproach.push_back(vec_pose);
		}

		//***********测试输出方向***********
		/*
		string ss = to_string(i) + ".ply";
		string s = s1.replace(s1.length() - 5, 5, ss);
		ofstream out(s1);
		int num = 0;
		for (int x = 0; x < Dirs.size(); x++)
			num += Dirs[x].size();

		out << "ply" << endl;
		out << "format ascii 1.0" << endl;
		out << "comment Author: gzf" << endl;
		out << "element vertex " << num << endl;
		out << "property float x" << endl;
		out << "property float y" << endl;
		out << "property float z" << endl;
		out << "property float nx" << endl;
		out << "property float ny" << endl;
		out << "property float nz" << endl;
		out << "end_header" << endl;
		for (int x = 0; x < Dirs.size(); x++) {
			for (int n = 0; n < Dirs[x].size(); n++) {
				out << skeleton.branches[i].curve[x].x() << " " << skeleton.branches[i].curve[x].y() << " " << skeleton.branches[i].curve[x].z() << " "
					<< Dirs[x][n].x() << " " << Dirs[x][n].y() << " " << Dirs[x][n].z() << endl;
			}
		}

		out.close();
		*/
		//*********************************

		graspApproaches.push_back(branchApproach);
	}
	
	cout << "grasp approach poses got!" << endl;
}


/**
* decideSkeletonGraspType
* \brief	判断每个点的抓取方式（power/precision）
* \param	ske_pca			骨架点的PCA
* \param	ske_preshape	得到的每个骨架点的抓取方式
* \return
**/
void GraspPlanner::decideSkeletonGraspType(vector<vector<Toolbox::PCAEigen>> ske_pca, vector<vector<pair<StructClass::GraspType, StructClass::GraspType>>> &ske_preshape) {
	
	vector<pair<StructClass::GraspType, StructClass::GraspType>> branch_preshape;
	pair<StructClass::GraspType, StructClass::GraspType> p(StructClass::GraspType::m_null, StructClass::GraspType::m_null);//first power; second precision

	for (int i = 0; i < skeleton.branch_num; i++) {
		branch_preshape.clear();
		for (int j = 0; j < skeleton.branches[i].curve.size(); j++) {
			if (skeleton.branches[i].vertexType[j] == StructClass::VertexType::BranchingVertex) {	// 交叉点（不抓）
				p.first = StructClass::GraspType::m_null;
				p.second = StructClass::GraspType::m_null;
			}
			else if (skeleton.branches[i].vertexType[j] == StructClass::VertexType::ConnectingVertex) {	//连接点（①λ1，λ2；②间隔）

				if (decider.calculatePCShape_isRound(ske_pca[i][j].length, ske_pca[i][j].thickness))	//是否是圆的
				{
					if (decider.decidePowerPreshape(ske_pca[i][j].length, ske_pca[i][j].thickness, true)) {
						for (int k = j; k > 0; k--) {
							//判断power grasp
							if (Toolbox::getDistancePoint2Point(skeleton.branches[i].curve[j], skeleton.branches[i].curve[k]) > 0.5*handWidth) {
								for (int m = j; m < skeleton.branches[i].curve.size(); m++) {
									if (Toolbox::getDistancePoint2Point(skeleton.branches[i].curve[j], skeleton.branches[i].curve[m]) > 0.5*handWidth) {
										p.first = StructClass::GraspType::Power;
										break;
									}
								}
								k = 0;
							}
						}
					}

					if (decider.decidePrecisionPreshape(ske_pca[i][j].length, ske_pca[i][j].thickness, true)) {
						for (int k = j; k > 0; k--) {
							//判断 precision grasp
							if (Toolbox::getDistancePoint2Point(skeleton.branches[i].curve[j], skeleton.branches[i].curve[k]) > figureWidth) {
								for (int m = j; m < skeleton.branches[i].curve.size(); m++) {
									if (Toolbox::getDistancePoint2Point(skeleton.branches[i].curve[j], skeleton.branches[i].curve[m]) > figureWidth) {
										p.second = StructClass::GraspType::Precision;
										break;
									}
								}
								k = 0;
							}
						}
					}
				}
				else
				{
					if (decider.decidePowerPreshape(ske_pca[i][j].length, ske_pca[i][j].thickness, false)) {
						for (int k = j; k > 0; k--) {
							//判断power grasp
							if (Toolbox::getDistancePoint2Point(skeleton.branches[i].curve[j], skeleton.branches[i].curve[k]) > 0.5*handWidth) {
								for (int m = j; m < skeleton.branches[i].curve.size(); m++) {
									if (Toolbox::getDistancePoint2Point(skeleton.branches[i].curve[j], skeleton.branches[i].curve[m]) > 0.5*handWidth) {
										p.first = StructClass::GraspType::Power;
										break;
									}
								}
								k = 0;
							}
						}
					}

					if (decider.decidePrecisionPreshape(ske_pca[i][j].length, ske_pca[i][j].thickness, false)) {
						for (int k = j; k > 0; k--) {
							//判断 precision grasp
							if (Toolbox::getDistancePoint2Point(skeleton.branches[i].curve[j], skeleton.branches[i].curve[k]) > figureWidth) {
								for (int m = j; m < skeleton.branches[i].curve.size(); m++) {
									if (Toolbox::getDistancePoint2Point(skeleton.branches[i].curve[j], skeleton.branches[i].curve[m]) > figureWidth) {
										p.second = StructClass::GraspType::Precision;
										break;
									}
								}
								k = 0;
							}
						}
					}
				}

				
			}
			else if (skeleton.branches[i].vertexType[j] == StructClass::VertexType::EndpointVertex) {	//末端点（①λ1，λ2）
				if (decider.decidePrecisionPreshape(ske_pca[i][j].length, ske_pca[i][j].thickness, true)) 
					p.second = StructClass::GraspType::Precision;
				if (decider.decidePowerPreshape(ske_pca[i][j].length, ske_pca[i][j].thickness, true))
					p.first = StructClass::GraspType::Power;
			}
			branch_preshape.push_back(p);
		}
		ske_preshape.push_back(branch_preshape);
	}

	//***********输出测试抓取方式***********
	/*
	string tempStr = ske_path;
	string s1 = tempStr.replace(tempStr.length() - 5, 5, "_grasp_type.ply");
	ofstream out(s1);
	int num = 0;
	for (int x = 0; x < ske_preshape.size(); x++)
		num += ske_preshape[x].size();

	out << "ply" << endl;
	out << "format ascii 1.0" << endl;
	out << "comment Author: gzf" << endl;
	out << "element vertex " << num <<endl;
	out << "property float x" << endl;
	out << "property float y" << endl;
	out << "property float z" << endl;
	out << "property uchar red" << endl; 
	out << "property uchar green" << endl;
	out << "property uchar blue" << endl;
	out << "end_header" << endl;
	
	for (int i = 0; i < ske_preshape.size(); i++)
	{
		for (int j = 0; j < ske_preshape[i].size(); j++)
		{
			if (ske_preshape[i][j].first == StructClass::GraspType::Power && ske_preshape[i][j].second == StructClass::GraspType::Precision) {
				out << skeleton.branches[i].curve[j].x() << " " << skeleton.branches[i].curve[j].y() << " " << skeleton.branches[i].curve[j].z() << " "
					<< 255 << " " << 255 << " " << 255 << endl;
			}
			if (ske_preshape[i][j].first == StructClass::GraspType::Power && ske_preshape[i][j].second == StructClass::GraspType::m_null) {
				out << skeleton.branches[i].curve[j].x() << " " << skeleton.branches[i].curve[j].y() << " " << skeleton.branches[i].curve[j].z() << " "
					<< 255 << " " << 0 << " " << 0 << endl;
			}
			if (ske_preshape[i][j].first == StructClass::GraspType::m_null && ske_preshape[i][j].second == StructClass::GraspType::Precision) {
				out << skeleton.branches[i].curve[j].x() << " " << skeleton.branches[i].curve[j].y() << " " << skeleton.branches[i].curve[j].z() << " "
					<< 0 << " " << 0 << " " << 255 << endl;
			}
			if (ske_preshape[i][j].first == StructClass::GraspType::m_null && ske_preshape[i][j].second == StructClass::GraspType::m_null) {
				out << skeleton.branches[i].curve[j].x() << " " << skeleton.branches[i].curve[j].y() << " " << skeleton.branches[i].curve[j].z() << " "
					<< 0 << " " << 0 << " " << 0 << endl;
			}
		}
	}
	out.close();
	*/
	//*******************************************************

	cout << "grasp shaoe got!" << endl;
}



/**
* decideGraspPose
* \brief	得到一个点的抓取姿态（RPY）
* \param	skePCA		骨架点的PCA
* \return	抓取姿态	RPY
**/
vector<Eigen::Vector3f> GraspPlanner::decideGraspPose(vector<vector<Toolbox::PCAEigen>> skePCA) {
	vector<Eigen::Vector3f> graspPose;
	SkeletonApproachMovement sketeton_movement(skeleton);


	for (int i = 0; i < skeleton.branch_num; i++) {
		vector<Eigen::Vector3f> gp;
		for (int j = 0; j < skeleton.branches[i].curve.size(); j++) {
			if (skeleton.branches[i].vertexType[j] == StructClass::VertexType::BranchingVertex) {		//crossing
				gp.push_back(Eigen::Vector3f(0, 0, 0));
			}
			if (skeleton.branches[i].vertexType[j] == StructClass::VertexType::ConnectingVertex) {		//connection
				gp = sketeton_movement.calculateApproachesConnectionPoint(skePCA[i][j]);
				sketeton_movement.generateApproachPoses(skeleton.branches[i].vertexPlane[j], skeleton.branches[i].curve[j], gp, false, graspPose);
			}
			if (skeleton.branches[i].vertexType[j] == StructClass::VertexType::EndpointVertex) {		//end
				gp = sketeton_movement.calculateApproachesEndpoint(skePCA[i][j]);
				sketeton_movement.generateApproachPoses(skeleton.branches[i].vertexPlane[j], skeleton.branches[i].curve[j], gp, true, graspPose);
			}
		}
		
	}

	return graspPose;
}

