#include "GraspPlan.h"



GraspPlan::GraspPlan(string skepath, string pcpath)
{
	readL1Skeleton(skepath);
	readL1ScanPC(pcpath);
}


GraspPlan::~GraspPlan()
{
}


/**
* readL1Skeleton
* \brief	��ȡ��L1 skeleton�õ��ĹǼ���Ϣ���ֶδ洢��ÿ��branch���ӽ�������
* \param	skepath
* \return	skeleton
**/
bool GraspPlan::readL1Skeleton(string skepath) {
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
			if (i != j ) {
				if (skeleton.branches[i].getHead() == skeleton.branches[j].getTail() || skeleton.branches[i].getHead() == skeleton.branches[j].getHead()) {
					reve = false;
					break;
				}
			}
		}
		if (reve) skeleton.branches[i].reverseBranch();
	}

	for (int i = 0; i < skeleton.branch_num; i++) {
		vector<int> vec;
		vec.push_back(0);
		for (int k = 1; k < skeleton.branches[k].curve.size()-1; k++) {
			vec.push_back(1);
		}
		
		for (int j = 0; j < skeleton.branch_num; j++) {
			if (i != j) {
				if (skeleton.branches[i].getTail() == skeleton.branches[j].getTail() || skeleton.branches[i].getTail() == skeleton.branches[j].getHead()) {
					vec.push_back(0);
				}
				else{
					vec.push_back(2);
				}
			}
		}
		skeleton_type.push_back(vec);
		vec.clear();
	}

	cout << "skeleton read successfully!" << endl;
	return true;
}


/**
* readL1ScanPC
* \brief	��ȡɨ��ĵ�����Ϣ
* \param	pcpath	·��
* \return	vector<Eigen::Vector3f> pc_pts
**/
bool GraspPlan::readL1ScanPC(string pcpath) {
	return Toolbox::readPointCloud(pcpath, pc_pts);
}


/**
* computerNearsetPts2Planes
* \brief	ӳ��ɨ��ĵ��Ƶ��Ǽܵ������(ʹ��scan pointCloud���Ǽ������ϵ������)
* \param
* \return	ÿ���Ǽܵ��Ӧ��ӳ���
**/
vector<vector<Eigen::Vector3f>> GraspPlan::computerNearsetPts2Planes(Branch m_branch, vector<vector<float>>& vec_param) {
	vector<Eigen::Vector3f> projectPts;
	vector<vector<Eigen::Vector3f>> vec_projectPts;
	vec_projectPts.push_back(projectPts);
	vector<float> param;
	vec_param.push_back(param);
	projectPts.clear();
	param.clear();

	//Ѱ�������
	vector<vector<pair<int, double>>> nearestPts;
	Toolbox::computeRadiusNeighbors(pc_pts, m_branch.curve, 256, 0.3, nearestPts);

	//Ѱ�ҵ㵽ƽ������ĵ�
	int ptsNum = m_branch.getSize();
	vector<pair<int, float>> nearestpts_branch;
	for (int j = 1; j < ptsNum; j++) {

		Eigen::Vector3f planeNormal;
		if (j == ptsNum - 1)
			planeNormal = m_branch.curve[ptsNum - 1] - m_branch.curve[ptsNum - 2];
		else
			planeNormal = m_branch.curve[j + 1] - m_branch.curve[j - 1];
		Toolbox::Plane skeplane(m_branch.curve[j], planeNormal);

		vector<pair<int, float>> near2plane;//�㵽ƽ��ľ���
		for (int k = 0; k < nearestPts[j].size(); k++) {
			if (nearestPts[j][k].first != NULL) {
				pair<int, float> p(nearestPts[j][k].first, Toolbox::getDistancePointPlane(pc_pts[nearestPts[j][k].first], skeplane));
				near2plane.push_back(p);
			}
		}

		vector<pair<int, float>> plane = Toolbox::GetLeastNumbers_Solution(near2plane, 64);//Ѱ�Ҿ�����С��64����
		for (int k = 0; k < plane.size(); k++) {
			projectPts.push_back(Toolbox::projectPointToPlane(pc_pts[plane[k].first], skeplane));
		}

		vec_projectPts.push_back(projectPts);

		//�Ǽܵ�������
		ofstream out0("F:\\Dropbox\\Dropbox\\RoboticGrasp\\code\\testdata\\airplane\\test.txt");
		out0 << m_branch.curve[j].x() << " " << m_branch.curve[j].y() << " " << m_branch.curve[j].z() << endl;
		for (int i = 0; i < 256; i++)
			out0 << setprecision(10) << pc_pts[nearestPts[j][i].first].x() << " " << pc_pts[nearestPts[j][i].first].y() << " " << pc_pts[nearestPts[j][i].first].z() << endl;
		out0.close();

		//���������
		ofstream out("F:\\Dropbox\\Dropbox\\RoboticGrasp\\code\\testdata\\airplane\\test_nearest.txt");
		for (int i = 0; i < plane.size(); i++)
			out << setprecision(10) << pc_pts[plane[i].first].x() << " " << pc_pts[plane[i].first].y() << " " << pc_pts[plane[i].first].z() << endl;
		out.close();

		//����ӳ�䵽ƽ���ϵĵ�
		ofstream out1("F:\\Dropbox\\Dropbox\\RoboticGrasp\\code\\testdata\\airplane\\test_preject.txt");
		for (int i = 0; i < projectPts.size(); i++)
			out1 << setprecision(10) << projectPts[i].x() << " " << projectPts[i].y() << " " << projectPts[i].z() << endl;
		out1.close();
		
		//�����Բ
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

	cout << "project to plane end!" << endl;
	return vec_projectPts;
}


/**
* projectPCPts2SkePlane
* \brief	ӳ��ɨ��ĵ��Ƶ��Ǽܵ������(ʹ�ùǼܵ���scan pointCloud�ϵ������)
* \param	
* \return	ÿ���Ǽܵ��Ӧ��ӳ���
**/
vector<vector<Eigen::Vector3f>> GraspPlan::projectPCPts2SkePlane(Branch m_branch) {
	vector<Eigen::Vector3f> projectPts;
	vector<vector<Eigen::Vector3f>> vec_projectPts;
	vec_projectPts.push_back(projectPts);
	projectPts.clear();

	vector<vector<pair<int, double>>> neighbors;
	Toolbox::computeAnnNeighbors(pc_pts, m_branch.curve, 64, neighbors);

	//������������
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
* skeletonPlaneAnalysis
* \brief	��ÿ���Ǽܵ��Ӧ��ӳ�����PCA����
* \param
* \return	ÿ���Ǽܵ��Ӧ��ӳ���
**/
vector<vector<Toolbox::PCAEigen>> GraspPlan::skeletonPlaneAnalysis() {
	vector<vector<Toolbox::PCAEigen>> skeleton_pca;
	for (int i = 0; i < skeleton.branch_num; i++) {
		vector<vector<Eigen::Vector3f>> project_branch;
		vector<Toolbox::PCAEigen> branch_pca;
		Toolbox::PCAEigen pca;
		branch_pca.push_back(pca);
		//project_branch = projectPCPts2SkePlane(skeleton.branches[i]);
		
		vector<vector<float>> vec_param;
		project_branch = computerNearsetPts2Planes(skeleton.branches[i], vec_param);

		for (int  j= 1; j < project_branch.size(); j++){
			pca = Toolbox::computerPCA(project_branch[j]);
			if (vec_param[j][2] >= vec_param[j][3]) {
				pca.length = vec_param[j][2];
				pca.thickness = vec_param[j][3];
			}
			else{
				pca.length = vec_param[j][3];
				pca.thickness = vec_param[j][2];
			}
				
			branch_pca.push_back(pca);
		}
		skeleton_pca.push_back(branch_pca);
		branch_pca.clear();
	}

	cout << "got PCA analysis" << endl;
	return skeleton_pca;
}



/**
* graspStrategiesGenerate
* \brief	ץȡ��������
* \param
* \return
**/
void GraspPlan::graspStrategiesGenerate() {
	vector<vector<Toolbox::PCAEigen>> skePCA = skeletonPlaneAnalysis();
	vector<vector<pair<bool, bool>>> graspInterval;
	decidePowerPreshapeInterval(graspInterval);

	//vector<pair<int, pair<bool,bool>>> preshape;	//��״(first)��0  round; 1 rectangle. ץȡ(second): 0 power; 1 precision.��true������ʣ�false�������ʣ�
	vector<vector<pair<int, pair<bool, bool>>>> vec_preshape;
	for (int i = 0; i < skePCA.size(); i++){ //i �Ǽ�branch
		preshape.push_back(pair<int, pair<bool, bool>>(-1, pair<bool, bool>(false, false)));
		for (int j = 1; j < skePCA[i].size(); j++) {
			pair<int, pair<int, int>> p;
			//if (calculatePreshape(skePCA[i][j].eigenvalues.x(), skePCA[i][j].eigenvalues.y()))
			if (calculatePreshape(skePCA[i][j].length, skePCA[i][j].thickness))
				p.first = 0;
			else 
				p.first = 1;

			if (decidePowerPreshape(skePCA[i][j].length, skePCA[i][j].thickness) && graspInterval[i][j].first)
				p.second.first = true;
			else
				p.second.first = false;

			if (decidePrecisionPreshape(skePCA[i][j].length, skePCA[i][j].thickness) && graspInterval[i][j].second)
				p.second.second = true;
			else
				p.second.second = false;

			preshape.push_back(p);
		}
		vec_preshape.push_back(preshape);
		preshape.clear();
	}
}



/**
* calculatePreshape
* \brief	�жϵ�����״
* \param	����ֵ lam1��lam2
* \return	round ����true��rectangle ����false.
**/
bool GraspPlan::calculatePreshape(double lam1, double lam2) {
	if (lam2 != 0)
		if ((lam1 / lam2) <= threshold_shape)
			return true;
		else
			return false;
}


/**
* setThicknessPrecision
* \brief	����precision�ĺ�Ȳ���
* \param	��С�������
* \return	
**/
void GraspPlan::setThicknessPrecision(float minThickness, float maxThickness){
	minThicknessPrecision = minThickness;
	maxThicknessPrecision = maxThickness;
}


/**
* setThicknessPower
* \brief	����power�ĺ�Ȳ���
* \param	��С�������
* \return	
**/
void GraspPlan::setThicknessPower(float minThickness, float maxThickness){
	minThicknessPower = minThickness;
	maxThicknessPower = maxThickness;
}


/**
* decidePowerPreshape
* \brief	ȷ���Ƿ����powerץȡ
* \param	��״�Ŀ�ͺ�
* \return	�ʺϷ���true
**/
bool GraspPlan::decidePowerPreshape(float length, float thickness){
	if ((thickness >= minThicknessPower && thickness <= maxThicknessPower) || length >= minLengthPower)
		return true;

	return false;
}


/**
* decidePowerPreshape
* \brief	ȷ���Ƿ����precisionץȡ
* \param	��״�Ŀ�ͺ�
* \return	�ʺϷ���true
**/
bool GraspPlan::decidePrecisionPreshape(float length, float thickness){
	if (thickness > minThicknessPrecision && thickness < maxThicknessPrecision /*&& length > minLengthPrecision && length < maxLengthPrecision*/)
		return true;

	return false;
}


/**
* decidePowerPreshapeInterval
* \brief	ͨ���Ǽܼ���ж�ץȡ����//ץȡ: 0 power; 1 precision.��true������ʣ�false�������ʣ�
* \param	skeleton
* \return	
**/
bool GraspPlan::decidePowerPreshapeInterval(vector<vector<pair<bool, bool>>> &graspInterval) {
	for (int i = 0; i < skeleton.branch_num; i++) {
		vector<pair<bool, bool>> grasp_branch;
		pair<bool, bool> p(false, false);
		grasp_branch.push_back(p);

		for (int j = 1; j < skeleton.branches[i].curve.size(); j++) {
			for (int k = j; k > 0; k--) {
				//�ж�precision grasp
				if (Toolbox::getDistancePoint2Point(skeleton.branches[i].curve[j], skeleton.branches[i].curve[k]) > figureWidth) {
					for (int m = j; m < skeleton.branches[i].curve.size(); m++) {
						if (Toolbox::getDistancePoint2Point(skeleton.branches[i].curve[j], skeleton.branches[i].curve[m]) > figureWidth) {
							p.second = true;
							break;
						}
					}
					k = 0;
				}	

				//�ж�power grasp
				if (Toolbox::getDistancePoint2Point(skeleton.branches[i].curve[j], skeleton.branches[i].curve[k]) > 0.5*handWidth) {
					for (int m = j; m < skeleton.branches[i].curve.size(); m++) {
						if (Toolbox::getDistancePoint2Point(skeleton.branches[i].curve[j], skeleton.branches[i].curve[m]) > 0.5*handWidth) {
							p.first = true;
							break;
						}
					}
					k = 0;
				}
			}
			grasp_branch.push_back(p);
		}
		graspInterval.push_back(grasp_branch);
		grasp_branch.clear();
	}

	return true;
}


/**
* decideGraspPose
* \brief	ͨ���Ǽܼ���ж�ץȡ����//ץȡ: 0 power; 1 precision.��true������ʣ�false�������ʣ�
* \param	skeleton
* \return
**/
vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> GraspPlan::decideGraspPose(vector<vector<Toolbox::PCAEigen>> skePCA) {
	vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> graspPose;
	for (int i = 0; i < skeleton.branch_num; i++) {
		vector<vector<Eigen::Vector3f>> branchpose;
		vector<Eigen::Vector3f> gp;
		for (int j = 0; j < skeleton.branches[i].curve.size(); j++) {
			if (skeleton_type[i][j] == 0) {		//crossing
				gp.push_back(Eigen::Vector3f(0,0,0));
			}
			if (skeleton_type[i][j] == 1) {		//connection
				gp = calculateApproachesConnectionPoint(skePCA[i][j]);
				generateApproachDir(skePCA[i][j], skeleton.branches[i].curve[j], gp, false, graspPose);
			}
			if (skeleton_type[i][j] == 2) {		//end
				gp = calculateApproachesEndpoint(skePCA[i][j]);
				generateApproachDir(skePCA[i][j], skeleton.branches[i].curve[j], gp, true, graspPose);
			}
			
		}
		
	}

	return graspPose;
}


/**
* generateApproachDir
* \brief	
* \param	
* \return
**/
void GraspPlan::generateApproachDir(Toolbox::PCAEigen pca, Eigen::Vector3f currentVertex, vector<Eigen::Vector3f> approachDirs, bool endpoint, vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& vec_mat) {
	
	for (int i = 0; i < approachDirs.size(); i++){
		Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
		Eigen::Vector3f position = currentVertex;
		Eigen::Vector3f approachDir = approachDirs[i];

		Eigen::Vector3f dirY = pca.eigenvector3;

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

		vec_mat.push_back(pose);
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
* \brief	�õ��ӽ�����̬
* \param	position	λ��	
* \param	approachDir �ӽ�����
* \param	dirY		Y�᷽��
* \return
**/
Eigen::Matrix4f GraspPlan::getApproachPose(Eigen::Vector3f position, Eigen::Vector3f approachDir, Eigen::Vector3f dirY) {
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
* \brief	����ӽ������round
* \param	pca
* \param	endpoint  �Ƿ���endpoint
* \return	�ӽ������vector
**/
vector<Eigen::Vector3f> GraspPlan::calculateApproachDirRound(Toolbox::PCAEigen pca, bool endpoint)
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
* \brief	����ӽ������rectangle
* \param	pca
* \param	endpoint  �Ƿ���endpoint
* \return	�ӽ������vector
**/
vector<Eigen::Vector3f> GraspPlan::calculateApproachDirRectangular(Toolbox::PCAEigen pca, bool endpoint)
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
* \brief	����ӽ������connection
* \param	pca
* \param	endpoint  �Ƿ���endpoint
* \return	�ӽ������vector
**/
vector<Eigen::Vector3f> GraspPlan::calculateApproachesConnectionPoint(const Toolbox::PCAEigen &pca)
{
	float ratio = pca.eigenvalues[0] / pca.eigenvalues[1];
	vector<Eigen::Vector3f> approachDirs;
	if (ratio < threshold_shape)
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
* \brief	����ӽ������endpoint
* \param	pca
* \param	endpoint  �Ƿ���endpoint
* \return	�ӽ������vector
**/
vector<Eigen::Vector3f> GraspPlan::calculateApproachesEndpoint(const Toolbox::PCAEigen &pca)
{
	float ratio = pca.eigenvalues[0] / pca.eigenvalues[0];
	vector<Eigen::Vector3f> approachDirs;
	if (ratio < threshold_shape)
	{
		approachDirs = calculateApproachDirRound(pca, true);

	}
	else {
		approachDirs = calculateApproachDirRectangular(pca, true);
	}

	return approachDirs;
}


