#include "SkeletonAnalyzer.h"


SkeletonAnalyzer::SkeletonAnalyzer(Skeleton ske, vector<Eigen::Vector3f> pc){
	skeleton = ske;
	pc_pts = pc;
}

SkeletonAnalyzer::SkeletonAnalyzer(string skepath, string pcpath)
{
	readL1Skeleton(skepath);
	readL1ScanPC(pcpath);
}


SkeletonAnalyzer::~SkeletonAnalyzer()
{
}

/**
* skeletonPlaneAnalysis
* \brief	对每个骨架点对应的映射点做PCA分析
* \param
* \return	每个骨架点对应的映射点
**/
vector<vector<Toolbox::PCAEigen>> SkeletonAnalyzer::skeletonPlaneAnalysis() {
	vector<vector<Toolbox::PCAEigen>> skeleton_pca;
	for (int i = 0; i < skeleton.branch_num; i++) {
		vector<vector<Eigen::Vector3f>> project_branch;
		vector<Toolbox::PCAEigen> branch_pca;
		Toolbox::PCAEigen pca;

		//project_branch = projectPCPts2SkePlane(skeleton.branches[i]);

		vector<vector<float>> vec_param;
		project_branch = computerNearsetPts2Planes(skeleton.branches[i], vec_param);

		for (int j = 0; j < project_branch.size(); j++) {
			if (skeleton.branches[i].vertexType[j] == StructClass::VertexType::BranchingVertex) {
				branch_pca.push_back(pca);
			}
			else {
				pca = Toolbox::computerPCA(project_branch[j]);
				if (vec_param[j][2] >= vec_param[j][3]) {
					pca.length = vec_param[j][2];
					pca.thickness = vec_param[j][3];
				}
				else {
					pca.length = vec_param[j][3];
					pca.thickness = vec_param[j][2];
				}
				branch_pca.push_back(pca);
			}
		}
		skeleton_pca.push_back(branch_pca);
		branch_pca.clear();
	}

	cout << "got PCA analysis" << endl;

	//输出测试PCA
	/*ofstream out("F:\\Dropbox\\Dropbox\\RoboticGrasp\\code\\testdata\\bottle\\bottle_pca.txt");
	for (int i = 0; i < skeleton_pca.size(); i++)
	{
		for (int j = 0; j < skeleton_pca[i].size(); j++)
		{
			out << skeleton.branches[i].curve[j].x() << " "<< skeleton.branches[i].curve[j].y() << " " << skeleton.branches[i].curve[j].z() << " " 
				<< skeleton_pca[i][j].eigenvector1.x() << " " << skeleton_pca[i][j].eigenvector1.y() << " " << skeleton_pca[i][j].eigenvector1.z() << endl;
			out << skeleton.branches[i].curve[j].x() << " " << skeleton.branches[i].curve[j].y() << " " << skeleton.branches[i].curve[j].z() << " "
				<< skeleton_pca[i][j].eigenvector2.x() << " " << skeleton_pca[i][j].eigenvector2.y() << " " << skeleton_pca[i][j].eigenvector2.z() << endl;
			out << skeleton.branches[i].curve[j].x() << " " << skeleton.branches[i].curve[j].y() << " " << skeleton.branches[i].curve[j].z() << " "
				<< skeleton_pca[i][j].eigenvector3.x() << " " << skeleton_pca[i][j].eigenvector3.y() << " " << skeleton_pca[i][j].eigenvector3.z() << endl;
		}	
	}
	out.close();
	*/

	return skeleton_pca;
}


/**
* computerNearsetPts2Planes
* \brief	映射扫描的点云到骨架点的切面(使用scan pointCloud到骨架切面上的最近点)
* \param
* \return	每个骨架点对应的映射点
**/
vector<vector<Eigen::Vector3f>> SkeletonAnalyzer::computerNearsetPts2Planes(Branch& m_branch, vector<vector<float>>& vec_param) {
	vector<vector<Eigen::Vector3f>> vec_projectPts;

	vector<Eigen::Vector3f> projectPts;
	vector<float> param;
	projectPts.clear();
	param.clear();

	//寻找最近点
	vector<vector<pair<int, double>>> nearestPts;
	Toolbox::computeRadiusNeighbors(pc_pts, m_branch.curve, 256, 0.3, nearestPts);

	//寻找点到平面最近的点
	int ptsNum = m_branch.getSize();
	vector<pair<int, float>> nearestpts_branch;
	for (int j = 0; j < ptsNum; j++) {
		if (m_branch.vertexType[j] == StructClass::VertexType::BranchingVertex) {
			vec_projectPts.push_back(projectPts);
			vec_param.push_back(param);

			Toolbox::Plane skeplane(m_branch.curve[j], Eigen::Vector3f(0,0,0));
			m_branch.vertexPlane.push_back(skeplane);
		}
		else {
			Eigen::Vector3f planeNormal;
			if(m_branch.vertexType[j] == StructClass::VertexType::EndpointVertex) {
				if(j-1<0)
					planeNormal = m_branch.curve[j] - m_branch.curve[j + 1];
				else
					planeNormal = m_branch.curve[j - 1] - m_branch.curve[j];

				planeNormal.normalize();
			}
			/*else if (m_branch.vertexType[j] == StructClass::VertexType::BranchingVertex) {
				planeNormal = m_branch.curve[j] - m_branch.curve[j + 1];
				planeNormal.normalize();
			}*/
			else {
				calculateApproachPlane(m_branch.curve[j], m_branch.curve[j - 1], m_branch.curve[j + 1], planeNormal);
				//planeNormal = m_branch.curve[j + 1] - m_branch.curve[j - 1];
				//planeNormal.normalize();
			}
			Toolbox::Plane skeplane(m_branch.curve[j], planeNormal);
			m_branch.vertexPlane.push_back(skeplane);

			vector<pair<int, float>> near2plane;//点到平面的距离
			for (int k = 0; k < nearestPts[j].size(); k++) {
				if (nearestPts[j][k].first != NULL) {
					pair<int, float> p(nearestPts[j][k].first, Toolbox::getDistancePointPlane(pc_pts[nearestPts[j][k].first], skeplane));
					near2plane.push_back(p);
				}
			}

			vector<pair<int, float>> plane = Toolbox::GetLeastNumbers_Solution(near2plane, 64);//寻找距离最小的64个点
			for (int k = 0; k < plane.size(); k++) {
				projectPts.push_back(Toolbox::projectPointToPlane(pc_pts[plane[k].first], skeplane));
			}

			vec_projectPts.push_back(projectPts);

			//测试输出
			/*
			//骨架点的最近点
			ofstream out0("F:\\Dropbox\\Dropbox\\RoboticGrasp\\code\\testdata\\airplane\\test.txt");
			out0 << m_branch.curve[j].x() << " " << m_branch.curve[j].y() << " " << m_branch.curve[j].z() << endl;
			for (int i = 0; i < 256; i++)
				out0 << setprecision(10) << pc_pts[nearestPts[j][i].first].x() << " " << pc_pts[nearestPts[j][i].first].y() << " " << pc_pts[nearestPts[j][i].first].z() << endl;
			out0.close();

			//测试最近点
			ofstream out("F:\\Dropbox\\Dropbox\\RoboticGrasp\\code\\testdata\\airplane\\test_nearest.txt");
			for (int i = 0; i < plane.size(); i++)
				out << setprecision(10) << pc_pts[plane[i].first].x() << " " << pc_pts[plane[i].first].y() << " " << pc_pts[plane[i].first].z() << endl;
			out.close();

			//测试映射到平面上的点
			ofstream out1("F:\\Dropbox\\Dropbox\\RoboticGrasp\\code\\testdata\\airplane\\test_preject.txt");
			for (int i = 0; i < projectPts.size(); i++)
				out1 << setprecision(10) << projectPts[i].x() << " " << projectPts[i].y() << " " << projectPts[i].z() << endl;
			out1.close();
			*/

			//拟合椭圆
			//*****************----------------*********************
			Eigen::Matrix4f mat;
			Toolbox::quat2eigen4f(Toolbox::getRotation(skeplane.n, Eigen::Vector3f(0., 0., 1.)), mat);

			vector<Eigen::Vector2f> v2;
			for (int k = 0; k < projectPts.size(); k++) {
				Eigen::Vector4f v4 = mat * Eigen::Vector4f(projectPts[k].x(), projectPts[k].y(), projectPts[k].z(), 1);
				projectPts[k] = Eigen::Vector3f(v4.x(), v4.y(), v4.z());

				v2.push_back(Eigen::Vector2f(v4.x(), v4.y()));
			}


			LSEllipse ellipse;
			param = ellipse.fittingEllipse_mine(v2);
			vec_param.push_back(param);
			//*****************----------------*********************

			projectPts.clear();
			param.clear();
		}
	}

	cout << "project to plane end!" << endl;
	return vec_projectPts;
}


/**
* projectPCPts2SkePlane
* \brief	映射扫描的点云到骨架点的切面(使用骨架点在scan pointCloud上的最近点)
* \param
* \return	每个骨架点对应的映射点
**/
vector<vector<Eigen::Vector3f>> SkeletonAnalyzer::projectPCPts2SkePlane(Branch m_branch) {
	vector<Eigen::Vector3f> projectPts;
	vector<vector<Eigen::Vector3f>> vec_projectPts;
	vec_projectPts.push_back(projectPts);
	projectPts.clear();

	vector<vector<pair<int, double>>> neighbors;
	Toolbox::computeAnnNeighbors(pc_pts, m_branch.curve, 64, neighbors);

	//测试输出最近点
	for (int i = 1; i < m_branch.getSize(); i++) {
		ofstream out("F:\\Dropbox\\Dropbox\\RoboticGrasp\\code\\testdata\\bottle\\bottle1_nearest.txt");
		for (int j = 0; j < 64; j++)
			out << setprecision(10) << pc_pts[neighbors[i][j].first].x() << " " << pc_pts[neighbors[i][j].first].y() << " " << pc_pts[neighbors[i][j].first].z() << endl;
		out.close();
	}

	int ptsNum = m_branch.getSize();
	for (int j = 1; j < ptsNum; j++) {
		Eigen::Vector3f planeNormal;
		if (j == ptsNum - 1)
			planeNormal = m_branch.curve[ptsNum - 1] - m_branch.curve[ptsNum - 2];
		else
			planeNormal = m_branch.curve[j + 1] - m_branch.curve[j - 1];
		Toolbox::Plane skeplane(m_branch.curve[j], planeNormal);

		for (int k = 0; k < neighbors[j].size(); k++) {
			projectPts.push_back(Toolbox::projectPointToPlane(pc_pts[neighbors[j][k].first], skeplane));
		}
		vec_projectPts.push_back(projectPts);

		ofstream out("F:\\Dropbox\\Dropbox\\RoboticGrasp\\code\\testdata\\bottle\\bottle1.txt");
		for (int i = 0; i < projectPts.size(); i++)
			out << setprecision(10) << projectPts[i].x() << " " << projectPts[i].y() << " " << projectPts[i].z() << endl;
		out.close();

		projectPts.clear();
	}

	cout << "project to plane end!" << endl;
	return vec_projectPts;
}


/**
* readL1Skeleton
* \brief	读取从L1 skeleton得到的骨架信息（分段存储）每个branch均从交叉点出发
* \param	skepath
* \return	skeleton
**/
bool SkeletonAnalyzer::readL1Skeleton(string skepath) {
	Branch m_branch;
	skeleton.clear();

	std::ifstream in(skepath);

	if (!in.good())
	{
		cout << "ERROR: loading txt:(" << skepath << ") file is not good" << "\n";
		return false;
	}

	float f1, f2, f3;
	int index = -1;
	char buffer[256], str[255];
	Eigen::Vector3f v;
	while (!in.getline(buffer, 255).eof()) {
		buffer[255] = '\0';

		sscanf_s(buffer, "%s", str, 255);

		string sstr = str;
		// reading a vertex
		if (sstr == "CNN") {
			index++;
			skeleton.branches.push_back(m_branch);
		}
		else if (sstr == "EN")
			break;

		if (sscanf(buffer, "%f %f %f %f %f %f", &f1, &f2, &f3, &f1, &f2, &f3) == 3) {
			v.x() = f1;
			v.y() = f2;
			v.z() = f3;
			skeleton.branches[index].pushBackCVertex(v);
		}
	}

	skeleton.generateBranchSampleMap();

	for (int i = 0; i < skeleton.branch_num; i++) {
		bool reve = true;
		for (int j = 0; j < skeleton.branch_num; j++) {
			if (i != j) {
				if (skeleton.branches[i].getHead() == skeleton.branches[j].getTail() || skeleton.branches[i].getHead() == skeleton.branches[j].getHead()) {
					reve = false;
					break;
				}
			}
		}
		if (reve) skeleton.branches[i].reverseBranch();
	}

	//设置骨架点的分割类型
	for (int i = 0; i < skeleton.branch_num; i++) {
		skeleton.branches[i].vertexType.clear();
		if(skeleton.branch_num == 1)
			skeleton.branches[i].vertexType.push_back(StructClass::VertexType::EndpointVertex);
		else
			skeleton.branches[i].vertexType.push_back(StructClass::VertexType::BranchingVertex);

		for (int k = 1; k < skeleton.branches[i].curve.size() - 1; k++) {
			skeleton.branches[i].vertexType.push_back(StructClass::VertexType::ConnectingVertex);
		}

		bool isendpoint = true;
		for (int j = 0; j < skeleton.branch_num; j++) {
			if (i != j) {
				if (skeleton.branches[i].getTail() == skeleton.branches[j].getTail() || skeleton.branches[i].getTail() == skeleton.branches[j].getHead()) {
					skeleton.branches[i].vertexType.push_back(StructClass::VertexType::BranchingVertex);
					isendpoint = false;
					break;
				}
			}
		}
		if(isendpoint)
			skeleton.branches[i].vertexType.push_back(StructClass::VertexType::EndpointVertex);
	}

	cout << "skeleton read successfully!" << endl;
	return true;
}


/**
* readL1ScanPC
* \brief	读取扫描的点云信息
* \param	pcpath	路径
* \return	vector<Eigen::Vector3f> pc_pts
**/
bool SkeletonAnalyzer::readL1ScanPC(string pcpath) {
	return Toolbox::readPointCloud(pcpath, pc_pts);
}


/**
* calculateApproachPlane
* \brief	计算接近面的法向
* \param	pos		位置
* \param	dir1， dir2	相邻点
* \return	result	面的法向
**/
void SkeletonAnalyzer::calculateApproachPlane(Eigen::Vector3f &pos, Eigen::Vector3f &dir1, Eigen::Vector3f &dir2, Eigen::Vector3f &result)
{

	Eigen::Vector3f d1 = dir1 - pos;
	if (d1.norm() > 0) {
		d1.normalize();
	}
	
	Eigen::Vector3f d2 = dir2 - pos;
	d2.normalize();

	Toolbox::Plane plane1;
	Toolbox::Plane plane2;
	plane1.p = pos;
	plane2.p = pos;
	plane1.n = d1;
	plane2.n = d2;


	Eigen::Vector3f mid((d1[0] + d2[0]) / 2.f, (d1[1] + d2[1]) / 2.f, (d1[2] + d2[2]) / 2.f);

	if (mid.norm() < 0.001f)
	{
		result = d1;
	}

	Toolbox::Line line = Toolbox::intersectPlanes(plane1, plane2);

	result = (line.d).cross(mid);
	result.normalize();
}