#include "Toolbox.h"


Toolbox::Toolbox()
{
}


Toolbox::~Toolbox()
{
}


/*
* readPointCloudNormal
* @brief	读取点云文件(带法向)（格式 .xyz .ply）
* @param	filename	文件路径
* @param	pc_pts	保存读取点
* @param	pc_normal	保存读取点法向
* @return	读取成功返回true，否则返回false
*/
/*
bool Toolbox::readPointCloudNormal(string filename, vector<CPoint3D>& pc_pts, vector<CPoint3D>& pc_normal) {
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
			pc_pts.push_back(v);
			v.x = f4;
			v.y = f5;
			v.z = f6;
			pc_normal.push_back(v);
		}
	}

	return true;
}
*/

bool Toolbox::readPointCloudNormal(string filename, vector<Eigen::Vector3f>& pc_pts, vector<Eigen::Vector3f>& pc_normal) {
	std::ifstream in(filename);

	if (!in.good())
	{
		cout << "ERROR: loading txt:(" << filename << ") file is not good" << "\n";
		return false;
	}

	float f1, f2, f3, f4, f5, f6;
	char buffer[256], str[255];;
	Eigen::Vector3f v;
	while (!in.getline(buffer, 255).eof()) {
		buffer[255] = '\0';

		sscanf_s(buffer, "%s", str, 255);

		// reading a vertex  
		if (sscanf(buffer, "%f %f %f %f %f %f", &f1, &f2, &f3, &f4, &f5, &f6) == 6)
		{
			v.x() = f1;
			v.y() = f2;
			v.z() = f3;
			pc_pts.push_back(v);
			v.x() = f4;
			v.y() = f5;
			v.z() = f6;
			pc_normal.push_back(v);
		}
	}

	return true;
}

/*
* readPointCloud
* @brief	读取点云文件(不带法向)（格式 .xyz .ply）
* @param	filename	文件路径
* @param	pc_pts	保存读取点
* @return	读取成功返回true，否则返回false
*/
/*
bool Toolbox::readPointCloud(string filename, vector<CPoint3D>& pc_pts) {
	std::ifstream in(filename);

	if (!in.good())
	{
		cout << "ERROR: loading txt:(" << filename << ") file is not good" << "\n";
		return false;
	}

	float f1, f2, f3;
	char buffer[256], str[255];;
	CPoint3D v;
	while (!in.getline(buffer, 255).eof()) {
		buffer[255] = '\0';

		sscanf_s(buffer, "%s", str, 255);

		// reading a vertex  
		if (sscanf(buffer, "%f %f %f %f %f %f", &f1, &f2, &f3) == 3)
		{
			v.x = f1;
			v.y = f2;
			v.z = f3;
			pc_pts.push_back(v);
		}
	}

	return true;
}
*/
bool Toolbox::readPointCloud(string filename, vector<Eigen::Vector3f>& pc_pts) {
	std::ifstream in(filename);

	if (!in.good())
	{
		cout << "ERROR: loading txt:(" << filename << ") file is not good" << "\n";
		return false;
	}

	float f1, f2, f3;
	char buffer[256], str[255];;
	Eigen::Vector3f v;
	while (!in.getline(buffer, 255).eof()) {
		buffer[255] = '\0';

		sscanf_s(buffer, "%s", str, 255);

		// reading a vertex  
		if (sscanf_s(buffer, "%f %f %f %f %f %f", &f1, &f2, &f3) == 3)
		{
			v.x() = f1;
			v.y() = f2;
			v.z() = f3;
			pc_pts.push_back(v);
		}
	}
	in.close();
	cout << "read pointcloud successfully!" << endl;
	return true;
}

/*
* getFiles
* @brief	获得文件路径下所有文件
* @param	path 路径
* @param	files 每个文件的路径
* @ownname	存储文件的名称	(eg. data1.txt)
*/
void Toolbox::getFiles(string path, vector<string>& files, vector<string> &ownname)
{
	//文件句柄  
	long   hFile = 0;
	//文件信息  
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之  
			//如果不是,加入列表  
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
* getDistancePointPlane
* @brief	获得点到平面的距离
* @param	point 点
* @param	plane 平面
* @return   距离
*/
float Toolbox::getDistancePointPlane(const Eigen::Vector3f& point, const Plane& plane)
{
	return plane.n.dot(point - plane.p);
}


/*
* projectPointToPlane
* @brief	获得点到平面的投影
* @param	point 点
* @param	plane 平面
* @return   投影点
*/
Eigen::Vector3f Toolbox::projectPointToPlane(const Eigen::Vector3f& point, const Plane& plane)
{
	float distance = getDistancePointPlane(point, plane);
	return point - distance * plane.n;
}


/*
* computeAnnNeighbors
* @brief	寻找点的近邻点,最近的knn个点
* @param	datapts		被搜索顶点信息
* @param	querypts	搜索顶点信息
* @param	neighbors	邻近点序号及距离
*/
bool Toolbox::computeAnnNeighbors(vector<Eigen::Vector3f> datapts, vector<Eigen::Vector3f> querypts, int knn, vector<vector<pair<int, double>>>& neighbors) {
	int dim = 3;		//dimension
	int maxPts = knn+50000; //maximum number of data points
	int K_per = knn+1;		//number of near neighbors
	double eps = 0;		//error bound
	//double radius = 1;	//查询半径

	if (datapts.size() >= maxPts)
	{
		cout << "Too many data" << endl;
		return false;
	}
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

	nPts = datapts.size();									// read data points

	for (int i = 0; i < datapts.size(); i++) {
		dataPts[i][0] = datapts[i].x();
		dataPts[i][1] = datapts[i].y();
		dataPts[i][2] = datapts[i].z();
	}

	kdTree = new ANNkd_tree(		// build search structure
		dataPts,					// the data points
		nPts,						// number of points
		dim);						// dimension of space

	for (int i = 0; i < querypts.size(); i++) {
		queryPt[0] = querypts[i].x();
		queryPt[1] = querypts[i].y();
		queryPt[2] = querypts[i].z();

		kdTree->annkSearch(						// search
			queryPt,								// query point
			K_per,								// number of near neighbors
			nnIdx,							// nearest neighbors (returned)
			dists,							// distance (returned)
			eps);							// error bound

		pair<int, double> p;
		vector<pair<int, double>> vec_p;
		for (int j = 1; j < K_per; j++)
		{
			p.first = nnIdx[j];
			p.second = dists[j];
			vec_p.push_back(p);
		}
		neighbors.push_back(vec_p);
		vec_p.clear();
	}

	delete[] nnIdx;							// clean things up
	delete[] dists;
	annClose();									// done with ANN

	return true;
}


/*
* computeRadiusNeighbors
* @brief	寻找点的近邻点，半径radius内的点
* @param	datapts		被搜索顶点信息
* @param	querypts	搜索顶点信息
* @param	neighbors	邻近点序号及距离
*/
bool Toolbox::computeRadiusNeighbors(vector<Eigen::Vector3f> datapts, vector<Eigen::Vector3f> querypts, int knn, double radius, vector<vector<pair<int, double>>>& neighbors) {
	int dim = 3;		//dimension
	int maxPts = 50000; //maximum number of data points
	int K_per = knn + 1;		//number of near neighbors
	double eps = 0;		//error bound
						//double radius = 1;	//查询半径

	if (datapts.size() >= maxPts)
	{
		cout << "Too many data" << endl;
		return false;
	}
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

	nPts = datapts.size();									// read data points

	for (int i = 0; i < datapts.size(); i++) {
		dataPts[i][0] = datapts[i].x();
		dataPts[i][1] = datapts[i].y();
		dataPts[i][2] = datapts[i].z();
	}

	kdTree = new ANNkd_tree(		// build search structure
		dataPts,					// the data points
		nPts,						// number of points
		dim);						// dimension of space

	for (int i = 0; i < querypts.size(); i++) {
		queryPt[0] = querypts[i].x();
		queryPt[1] = querypts[i].y();
		queryPt[2] = querypts[i].z();
		kdTree->annkFRSearch(
			queryPt,
			radius,
			K_per,
			nnIdx,
			dists,
			eps
		);

		pair<int, double> p;
		vector<pair<int, double>> vec_p;
		for (int j = 1; j < K_per; j++)
		{
			p.first = nnIdx[j];
			p.second = dists[j];
			vec_p.push_back(p);
		}
		neighbors.push_back(vec_p);
		vec_p.clear();
	}

	delete[] nnIdx;							// clean things up
	delete[] dists;
	annClose();									// done with ANN

	return true;
}


/*
* computerBB
* @brief	计算boundingbox
* @param	pc_pts		点集
* @return	BoundingBox	点集的boundingbox
*/
Toolbox::BoundingBox Toolbox::computerBB(vector<Eigen::Vector3f> pc_pts) {
	Eigen::Vector3f minBB;
	Eigen::Vector3f maxBB;
	vector<float> vec_x;
	vector<float> vec_y;
	vector<float> vec_z;
	for (int i = 0; i < pc_pts.size(); i++) {
		vec_x.push_back(pc_pts[i].x());
		vec_y.push_back(pc_pts[i].y());
		vec_z.push_back(pc_pts[i].z());
	}

	std::vector<float>::iterator biggest_x = std::max_element(std::begin(vec_x), std::end(vec_x));
	maxBB.x() = *biggest_x;
	std::vector<float>::iterator biggest_y = std::max_element(std::begin(vec_y), std::end(vec_y));
	maxBB.y() = *biggest_y;
	std::vector<float>::iterator biggest_z = std::max_element(std::begin(vec_z), std::end(vec_z));
	maxBB.z() = *biggest_z;

	std::vector<float>::iterator smallest_x = std::min_element(std::begin(vec_x), std::end(vec_x));
	minBB.x() = *smallest_x;
	std::vector<float>::iterator smallest_y = std::min_element(std::begin(vec_y), std::end(vec_y));
	minBB.y() = *smallest_y;
	std::vector<float>::iterator smallest_z = std::min_element(std::begin(vec_z), std::end(vec_z));
	minBB.z() = *smallest_z;

	Toolbox::BoundingBox bb;
	bb.maxBB = maxBB;
	bb.minBB = minBB;
	bb.diagonal_dis = sqrtf(pow(maxBB.x()-minBB.x(), 2) + pow(maxBB.y() - minBB.y(), 2) + pow(maxBB.z() - minBB.z(), 2));

	return bb;
}


/*
* computerPCA
* @brief	计算PCA
* @param	pc_pts		点集
* @return	PCAEigen
*/
Toolbox::PCAEigen Toolbox::computerPCA(vector<Eigen::Vector3f> pts) {
	Toolbox::PCAEigen pca;

	if (pts.empty()) return pca;

	int nrPoints = (int)pts.size() - 1;

	Eigen::Vector3f mean(0.f, 0.f, 0.f);

	for (size_t i = 0; i < pts.size(); i++)
	{
		mean += pts[i];
	}

	mean /= nrPoints;

	Eigen::MatrixXf matrix(pts.size(), 3);
	for (size_t i = 0; i < pts.size(); i++)
	{
		pts[i] -= mean;
		matrix.block(i, 0, 1, 3) = pts[i].transpose();
	}

	Eigen::JacobiSVD<Eigen::MatrixXf> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Vector3f eigenvalues = svd.singularValues();
	
	pca.eigenvector1 = svd.matrixV().col(0);
	pca.eigenvector2 = svd.matrixV().col(1);
	pca.eigenvector3 = svd.matrixV().col(2);
	pca.eigenvalues = eigenvalues;

	pca.length = ((pca.eigenvector1 * pca.eigenvalues[0]) / std::sqrt(nrPoints - 1)).norm();
	pca.thickness = ((pca.eigenvector2 * pca.eigenvalues[1]) / std::sqrt(nrPoints - 1)).norm();

	return pca;
}


/*
* GetLeastNumbers_Solution
* @brief	计算最小的k个数
* @param	input		点集
* @return	最小的k个数
*/
vector<pair<int, float>> Toolbox::GetLeastNumbers_Solution(vector<pair<int, float>> input, int k) {
	vector<pair<int, float>> abs_input;
	for (int i = 0; i < input.size(); i++){
		abs_input.push_back(pair<int, float>(input[i].first, fabs(input[i].second)));
	}
	vector<pair<int, float>> result;
	if (k > input.size())
	{
		result.resize(0);
		return result;
	}
	//先排序（快速排序）
	int start = 0;
	int end = input.size() - 1;
	quick_sort(abs_input, start, end);

	//在输出前k个数
	for (int i = 0; i < k; i++)
	{
		result.push_back(abs_input[i]);
	}
	return result;
}

//快速排序
void Toolbox::quick_sort(vector<pair<int, float>> &input, int start, int end)
{
	int left = start;
	int right = end;
	int index = left;
	int temp = input[left].second;
	while (left < right)
	{
		while (left < right && input[right].second >= temp)
		{
			right--;
		}
		input[left] = input[right];
		index = right;

		while (left < right && input[left].second <= temp)
		{
			left++;
		}
		input[right] = input[left];
		index = left;

		input[index].second = temp;
	}

	if (left - start > 1)
	{
		quick_sort(input, start, left - 1);
	}
	if (end - right > 1)
	{
		quick_sort(input, right + 1, end);
	}
}


/*
* getRotation
* @brief	旋转变换从from到to的变换四元数
* @param	
* @return	
*/
Eigen::Quaterniond Toolbox::getRotation(const Eigen::Vector3f& from, const Eigen::Vector3f& to)
{
	Eigen::Vector3f fromN = from;
	fromN.normalize();
	Eigen::Vector3f toN = to;
	toN.normalize();

	// rotation of the plane
	float d = fromN.dot(toN);
	Eigen::Vector3f crossvec = fromN.cross(toN);
	float crosslen = crossvec.norm();
	Eigen::Quaterniond  quat;

	if (crosslen == 0.0f)
	{
		// Parallel vectors
		if (d > 0.0f)
		{
			// same direction->nothing to do
		}
		else
		{
			// parallel and pointing in opposite direction
			// crossing with x axis.
			Eigen::Vector3f t = fromN.cross(Eigen::Vector3f(1.0f, 0.0f, 0.0f));

			// no->cross with y axis.
			if (t.norm() == 0.0f)
			{
				t = fromN.cross(Eigen::Vector3f(0.0f, 1.0f, 0.0f));
			}

			t.normalize();
			quat.x() = t[0];
			quat.y() = t[1];
			quat.z() = t[2];
			quat.w() = 0.0f;
		}
	}
	else
	{
		// not parallel
		crossvec.normalize();
		// The fabs() wrapping is to avoid problems when d overflows
		crossvec *= (float)sqrt(0.5f * fabs(1.0f - d));
		quat.x() = crossvec[0];
		quat.y() = crossvec[1];
		quat.z() = crossvec[2];
		quat.w() = (float)sqrt(0.5 * fabs(1.0 + d));
	}

	return quat;
}


/*
* quat2eigen4f
* @brief	四元数变成变换矩阵
* @param
* @return
*/
void Toolbox::quat2eigen4f(const Eigen::Quaterniond q, Eigen::Matrix4f& m)
{
	m = quat2eigen4f(q.x(), q.y(), q.z(), q.w());
}

Eigen::Matrix4f Toolbox::quat2eigen4f(float x, float y, float z, float w)
{
	Eigen::Matrix4f m;
	m.setIdentity();
	Eigen::Quaternionf q(w, x, y, z);
	Eigen::Matrix3f m3;
	m3 = q.toRotationMatrix();
	m.block(0, 0, 3, 3) = m3;
	return m;
}


/*
* getDistancePoint2Point
* @brief	计算两点之间的距离
* @param
* @return
*/
float Toolbox::getDistancePoint2Point(Eigen::Vector3f from, Eigen::Vector3f to) {
	return sqrtf(pow(from.x() - to.x(), 2) + pow(from.y() - to.y(), 2) + pow(from.z() - to.z(), 2));
}


/*
* getRotationMat
* @brief	得到（0: x,1: y,2: z）旋转矩阵
* @param    弧度值， 旋转轴
* @return
*/
Eigen::Matrix4f Toolbox::getRotationMat(float radian, string axis) {
	Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
	if (axis == "x") {
		mat << 1, 0, 0, 0,
			0, cosf(radian), -sinf(radian), 0,
			0, sinf(radian), cosf(radian), 0,
			0, 0, 0, 1;
	}
	else if (axis == "y") {
		mat << cosf(radian), 0, sinf(radian), 0,
			0, 1, 0, 0,
			-sinf(radian), 0, cosf(radian), 0,
			0, 0, 0, 1;
	}
	else if (axis == "z") {
		mat << cosf(radian), -sinf(radian), 0, 0,
			sinf(radian), cosf(radian), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
	}
	else{
		cout << "axis error!" << endl;
	}

	return mat;
}


/*
* getRotationMat
* @brief	得到（0: x,1: y,2: z）旋转矩阵
* @param
* @return
*/
void Toolbox::eigen4f2rpy(const Eigen::Matrix4f& m, float x[6])
{
	//Eigen::Matrix4f A = mat.transpose();
	float alpha, beta, gamma;
	beta = atan2(-m(2, 0), sqrtf(m(0, 0) * m(0, 0) + m(1, 0) * m(1, 0)));

	if (fabs(beta - (float)M_PI * 0.5f) < 1e-10)
	{
		alpha = 0;
		gamma = atan2(m(0, 1), m(1, 1));
	}
	else if (fabs(beta + (float)M_PI * 0.5f) < 1e-10)
	{
		alpha = 0;
		gamma = -atan2(m(0, 1), m(1, 1));
	}
	else
	{
		float cb = 1.0f / cosf(beta);
		alpha = atan2(m(1, 0) * cb, m(0, 0) * cb);
		gamma = atan2(m(2, 1) * cb, m(2, 2) * cb);
	}

	x[0] = m(0, 3);
	x[1] = m(1, 3);
	x[2] = m(2, 3);
	x[3] = gamma;
	x[4] = beta;
	x[5] = alpha;
}

void Toolbox::eigen4f2rpy(const Eigen::Matrix4f& m, Eigen::Vector3f& storeRPY)
{
	float x[6];
	eigen4f2rpy(m, x);
	storeRPY(0) = x[3];
	storeRPY(1) = x[4];
	storeRPY(2) = x[5];
}


/*
* createMidVector
* @brief	得到两个向量中间的向量
* @param
* @return
*/
Eigen::Vector3f Toolbox::createMidVector(const Eigen::Vector3f &vec1, const Eigen::Vector3f &vec2)
{
	Eigen::Vector3f d1 = vec1;
	Eigen::Vector3f d2 = vec2;
	d1.normalize();
	d2.normalize();


	Eigen::Vector3f mid((d1[0] + d2[0]) / 2.f, (d1[1] + d2[1]) / 2.f, (d1[2] + d2[2]) / 2.f);
	mid.normalize();
	return mid;
}


/*
* posrpy2eigen4
* @brief	rpy转到mat(包括位置)
* @param
* @return
*/
void Toolbox::posrpy2eigen4f(const float x[6], Eigen::Matrix4f& m)
{
	rpy2eigen4f(x[3], x[4], x[5], m);
	m(0, 3) = x[0];
	m(1, 3) = x[1];
	m(2, 3) = x[2];
}


/*
* posrpy2eigen4f
* @brief	rpy转到mat(包括位置)
* @param	
* @return
*/
void Toolbox::posrpy2eigen4f(const Eigen::Vector3f& pos, const Eigen::Vector3f& rpy, Eigen::Matrix4f& m)
{
	float x[6];

	for (int i = 0; i < 3; i++)
	{
		x[i] = pos(i);
		x[i + 3] = rpy(i);
	}

	posrpy2eigen4f(x, m);
}


/*
* rpy2eigen4f
* @brief	rpy转到mat(无位置)
* @param	
* @return
*/
void Toolbox::rpy2eigen4f(float r, float p, float y, Eigen::Matrix4f& m)
{
	float salpha, calpha, sbeta, cbeta, sgamma, cgamma;

	sgamma = sinf(r);
	cgamma = cosf(r);
	sbeta = sinf(p);
	cbeta = cosf(p);
	salpha = sinf(y);
	calpha = cosf(y);

	m(0, 0) = (float)(calpha * cbeta);
	m(0, 1) = (float)(calpha * sbeta * sgamma - salpha * cgamma);
	m(0, 2) = (float)(calpha * sbeta * cgamma + salpha * sgamma);
	m(0, 3) = 0; //x

	m(1, 0) = (float)(salpha * cbeta);
	m(1, 1) = (float)(salpha * sbeta * sgamma + calpha * cgamma);
	m(1, 2) = (float)(salpha * sbeta * cgamma - calpha * sgamma);
	m(1, 3) = 0; //y

	m(2, 0) = (float)-sbeta;
	m(2, 1) = (float)(cbeta * sgamma);
	m(2, 2) = (float)(cbeta * cgamma);
	m(2, 3) = 0; //z

	m(3, 0) = 0;
	m(3, 1) = 0;
	m(3, 2) = 0;
	m(3, 3) = 1.0f;
}



Toolbox::Line Toolbox::intersectPlanes(const Plane& p1, const Plane& p2)
{
	// algorithm taken from http://paulbourke.net/geometry/planeplane/

	float dotpr = p1.n.dot(p2.n);

	if (fabs(dotpr - 1.0f) < 1.e-6)
	{
		// planes are parallel
		Toolbox::Line l(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0));
		return l;
	}

	Eigen::Vector3f dir = p1.n.cross(p2.n);

	float d1 = p1.n.dot(p1.p);
	float d2 = p2.n.dot(p2.p);

	float det = p1.n.dot(p1.n) * p2.n.dot(p2.n) - p1.n.dot(p2.n) * p1.n.dot(p2.n);

	if (fabs(det) < 1e-9)
	{
		return Toolbox::Line(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0));
	}

	float c1 = (d1 * (p2.n.dot(p2.n)) - d2 * (p1.n.dot(p2.n))) / det;
	float c2 = (d2 * (p1.n.dot(p1.n)) - d1 * (p1.n.dot(p2.n))) / det;
	Eigen::Vector3f pos;
	pos = c1 * p1.n + c2 * p2.n;
	Toolbox::Line l(pos, dir);
	// move pos so that we have a position that is next to plane point p1.p
	pos = nearestPointOnLine(l, p1.p);

	return Toolbox::Line(pos, dir);
}