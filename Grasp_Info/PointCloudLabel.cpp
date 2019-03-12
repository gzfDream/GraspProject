#include "PointCloudLabel.h"


PointCloudLabel::PointCloudLabel()
{
}


PointCloudLabel::~PointCloudLabel()
{
}

//======��ʾ��ɫ======
int color_pc[36] = {
	127.0, 76.0, 51.0,   // color - brown
	241.0, 94.0, 13.0,   // color - orange
	0.0, 167.0, 30.0,      // color green
	229.0, 51.0,  51.0, // color - red
	51.0,  51.0,  229.0,  // color - blue
	229.0, 229.0, 127.0,  // color - yellow
	178.0, 51.0,  229.0,  // color - purple
	51.0, 229.0, 229.0,  // color - light blue
	155.0, 162.0, 237.0, // color - purple
	229.0, 51.0, 153.0,  // color - pink
	153.0, 153.0, 178.0,  // color - gray
	229.0, 153.0, 127.0   // color - light pink
};

/*
* getFiles
* @brief	����ļ�·���������ļ�
* @param	path ·��
* @param	files ÿ���ļ���·��	(eg. C:\Users\WUQP\Desktop\test_devided\data1.txt)
* @ownname	�洢�ļ�������	(eg. data1.txt)
*/
void PointCloudLabel::getFiles(string path, vector<string>& files, vector<string> &ownname)
{
	//�ļ����  
	long   hFile = 0;
	//�ļ���Ϣ  
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//�����Ŀ¼,����֮  
			//�������,�����б�  
			if ((fileinfo.attrib &  _A_SUBDIR))
			{  /*
			   if(strcmp(fileinfo.name,".") != 0  &&  strcmp(fileinfo.name,"..") != 0)
			   getFiles( p.assign(path).append("\\").append(fileinfo.name), files, ownname ); */
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
				ownname.push_back(fileinfo.name);
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}


/*
* readPCTXT
* @brief	��ȡ�����ļ�����ʽ .txt��
* @param	filename	�ļ�·��
* @param	vec_pc	�����ȡ���
* @return	��ȡ�ɹ�����true�����򷵻�false
*/
bool PointCloudLabel::readPCTXT(string filename, vector<CPoint3D>& vec_pc, vector<CPoint3D>& vec_pc_normal) {
	std::ifstream in(filename);

	if (!in.good())
	{
		cout << "ERROR: loading txt:(" << filename << ") file is not good" << "\n";
		return false;
	}

	float f1, f2, f3, f4, f5, f6, f7;
	char buffer[256], str[255];;
	CPoint3D v;
	while (!in.getline(buffer, 255).eof()) {
		buffer[255] = '\0';

		sscanf_s(buffer, "%s", str, 255);

		// reading a vertex  
		if (sscanf(buffer, "%f %f %f %f %f %f %f", &f1, &f2, &f3, &f4, &f5, &f6, &f7) == 7)
		{
			v.x = f1;
			v.y = f2;
			v.z = f3;
			vec_pc.push_back(v);
			v.x = f4;
			v.y = f5;
			v.z = f6;
			vec_pc_normal.push_back(v);
		}
	}

	return true;
}


/*
* readPCXYZ
* @brief	��ȡ�����ļ�����ʽ .xyz .ply��
* @param	filename	�ļ�·��
* @param	vec_pc	�����ȡ���
* @return	��ȡ�ɹ�����true�����򷵻�false
*/
bool PointCloudLabel::readPCXYZ(string filename, vector<CPoint3D>& vec_pc, vector<CPoint3D>& vec_pc_normal) {
	std::ifstream in(filename);

	if (!in.good())
	{
		cout << "ERROR: loading txt:(" << filename << ") file is not good" << "\n";
		return false;
	}

	float f1, f2, f3, f4, f5, f6;
	char buffer[256], str[255];;
	CPoint3D v;
	while (!in.getline(buffer, 255).eof()) {
		buffer[255] = '\0';

		sscanf_s(buffer, "%s", str, 255);

		// reading a vertex  
		if (sscanf(buffer, "%f %f %f %f %f %f", &f1, &f2, &f3, &f4, &f5, &f6) == 6)
		{
			v.x = f1;
			v.y = f2;
			v.z = f3;
			vec_pc.push_back(v);
			v.x = f4;
			v.y = f5;
			v.z = f6;
			vec_pc_normal.push_back(v);
		}
	}

	return true;
}


/*
* searchPoints
* @brief	Ѱ�ҵ�����mesh�ϵ������
* @param	data	mesh������Ϣ
* @param	query	���ƶ�����Ϣ
* @param	vec_nearest	��������
*/
void PointCloudLabel::searchPoints(vector<CPoint3D> data, vector<CPoint3D> query, vector<int>& vec_nearest) {
	int dim = 3;		//dimension
	int maxPts = 50000; //maximum number of data points
	int K_per = 1;		//number of near neighbors
	double eps = 0;		//error bound
	double radius = 1;	//��ѯ�뾶

	int					nPts;					// actual number of data points
	ANNpointArray		dataPts;				// data points
	ANNpoint			dataPt;					// data point
	ANNpoint			queryPt;				// query point
	ANNidxArray			nnIdx;					// near neighbor indices
	ANNdistArray		dists;					// near neighbor distances
	ANNkd_tree*			kdTree;					// search structure


	dataPt = annAllocPt(dim);
	queryPt = annAllocPt(dim);					// allocate query point
	dataPts = annAllocPts(maxPts, dim);			// allocate data points
	nnIdx = new ANNidx[K_per];					// allocate near neigh indices
	dists = new ANNdist[K_per];					// allocate near neighbor dists

	nPts = data.size();									// read data points

	for (int i = 0; i < data.size(); i++) {
		dataPts[i][0] = data[i].x;
		dataPts[i][1] = data[i].y;
		dataPts[i][2] = data[i].z;
	}

	kdTree = new ANNkd_tree(		// build search structure
		dataPts,					// the data points
		nPts,						// number of points
		dim);						// dimension of space

	for (int i = 0; i < query.size(); i++) {
		queryPt[0] = query[i].x;
		queryPt[1] = query[i].y;
		queryPt[2] = query[i].z;

		kdTree->annkSearch(						// search
			queryPt,								// query point
			K_per,								// number of near neighbors
			nnIdx,							// nearest neighbors (returned)
			dists,							// distance (returned)
			eps);							// error bound

		vec_nearest.push_back(nnIdx[0]);
	}

	delete[] nnIdx;							// clean things up
	delete[] dists;
	annClose();									// done with ANN
}


/*
* normal_test
* @brief	���߲��ԣ��Ƚ�ץȡ����͵㷨��ĽǶ�
* @param	dir		ץȡ����
* @param	nor		�㷨��
* @return	���߽Ƕ��Ƿ��� �� ֮��
*/
bool normal_test(CPoint3D dir, CPoint3D nor) {
	double beta = 60;//�Ƕ�
	dir.Normalize();
	nor.Normalize();

	double n = dir.x*nor.x + dir.y*nor.y + dir.z*nor.z;
	if (n >= cos(beta / 180))
		return true;
	return false;
}


void PointCloudLabel::getDataSet(string path) {
	GraspXml graspData(path);
	graspData.main_process();
	
	vector<GraspXml::GraspPlan> grasp_plan= graspData.getGraspPlan();
	vector<GraspXml::MeshSkeleton> mesh_skeleton = graspData.getMeshSkeleton();

	string filename = path+"\\pointCloud\\airplane0.1024.ply";
	vector<CPoint3D> pc_pts;
	vector<CPoint3D> pc_pts_normal;
	readPCXYZ(filename, pc_pts, pc_pts_normal);

	vector<CPoint3D> mesh_pts;
	for (int i = 0; i < mesh_skeleton.size(); i++) {
		mesh_pts.push_back(mesh_skeleton[i].mesh_pt/1000);
	}

	vector<int> nearest_index;
	searchPoints(mesh_pts, pc_pts, nearest_index);

	vector<CPoint3D> position;
	vector<CPoint3D> dir;
	for (int i = 0; i < pc_pts.size(); i++) {
		vector<int> vec_temp = mesh_skeleton[nearest_index[i]].plan_index;
		if (!vec_temp.empty()) {
			int is_normal = 0;
			for (int j = 0; j < vec_temp.size(); j++) {
				if (normal_test(grasp_plan[vec_temp[j]].approachDir, pc_pts_normal[i])) {
					position.push_back(grasp_plan[vec_temp[j]].position);
					dir.push_back(grasp_plan[vec_temp[j]].approachDir);
					is_normal = 1;
					break;
				}
			}
			if (!is_normal) {
				position.push_back(CPoint3D(0, 0, 0));
				dir.push_back(CPoint3D(0, 0, 0));
			}
		}
		else {
			position.push_back(CPoint3D(0, 0, 0));
			dir.push_back(CPoint3D(0, 0, 0));
		}
			
	}
}
