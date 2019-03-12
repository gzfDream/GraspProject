/**
* ����ͨ�÷���������
*
* @author gzf
* @update 2018-11-21
**/
#pragma once
#include <cstdlib>						// C standard library
#include <cstdio>						// C I/O (for sscanf)
#include <cstring>						// string manipulation
#include <fstream>						// file I/O
#include <ANN/ANN.h>					// ANN declarations
#include <io.h>
#include <string>
#include <iomanip>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
//#include "include\Mesh\Point3D.h"

using namespace std;
class Toolbox
{
public:
	Toolbox();
	~Toolbox();

	//Plane
	struct Plane
	{
		Plane(){
			p = Eigen::Vector3f::Zero();
			n = Eigen::Vector3f::UnitZ();
		}

		Plane(const Eigen::Vector3f& point, const Eigen::Vector3f& normal){
			p = point;
			n = normal;
			n.normalize();
		}

		Plane(const Plane& plane){
			this->p = plane.p;
			this->n = plane.n;
		}

		Eigen::Vector3f p;  // point
		Eigen::Vector3f n;  // normal (unit length)
	};

	//BoundingBox
	struct BoundingBox
	{
		// the bounding box is defined via min and max values
		Eigen::Vector3f minBB;
		Eigen::Vector3f maxBB;
		float diagonal_dis;
	};

	//PCA
	struct PCAEigen {
		PCAEigen() {
			eigenvector1 = Eigen::Vector3f::Zero();
			eigenvector2 = Eigen::Vector3f::Zero();
			eigenvector3 = Eigen::Vector3f::Zero();
			eigenvalues = Eigen::Vector3f::Zero();
			length = 0;
			thickness = 0;
		}

		PCAEigen(Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f v3, Eigen::Vector3f v4) {
			eigenvector1 = v1;
			eigenvector2 = v2;
			eigenvector3 = v3;
			eigenvalues = v4;
		}
		Eigen::Vector3f eigenvector1;
		Eigen::Vector3f eigenvector2;
		Eigen::Vector3f eigenvector3;
		Eigen::Vector3f eigenvalues;
		float length, thickness;	//������ʾ��Ⱥͺ��

	};

	//Line
	template<typename VectorT>
	struct BaseLine
	{
		BaseLine() {}

		BaseLine(const VectorT& point, const VectorT& dir)
		{
			p = point;
			d = dir;

			if (d.norm() > 1e-9)
			{
				d.normalize();
			}
		}

		BaseLine(const BaseLine<VectorT>& line)
			: p(line.p)
			, d(line.d)
		{
		}

		bool isValid() const
		{
			return d.norm() > 1e-9;
		}

		VectorT p;  // point
		VectorT d;  // direction (unit length)
	};

	typedef BaseLine<Eigen::Vector3f> Line;


	//! Get the projected point in 3D
	static Eigen::Vector3f projectPointToPlane(const Eigen::Vector3f& point, const Plane& plane);

	//! �㵽ƽ��ľ���
	static float getDistancePointPlane(const Eigen::Vector3f& point, const Plane& plane);

	//! ����boundingbox
	static BoundingBox computerBB(vector<Eigen::Vector3f> pc_pts);

	//! ����PCA
	static PCAEigen computerPCA(vector<Eigen::Vector3f> pts);

	//! ������С��K����
	static vector<pair<int, float>> GetLeastNumbers_Solution(vector<pair<int, float>> input, int k);

	//! ��������
	static void quick_sort(vector<pair<int, float>> &input, int start, int end);

	//! ��������ļ����е��ļ���
	static void getFiles(string path, vector<string>& files, vector<string> &ownname);


	//static bool readPointCloudNormal(string filename, vector<CPoint3D>& pc_pts, vector<CPoint3D>& pc_normal);
	//static bool readPointCloud(string filename, vector<CPoint3D>& pc_pts);

	//! ��ȡ������ĵ���
	static bool readPointCloudNormal(string filename, vector<Eigen::Vector3f>& pc_pts, vector<Eigen::Vector3f>& pc_normal);

	//! ��ȡ����
	static bool readPointCloud(string filename, vector<Eigen::Vector3f>& pc_pts);

	//! ����knn�������
	static bool computeAnnNeighbors(vector<Eigen::Vector3f> datapts, vector<Eigen::Vector3f> querypts, int knn, vector<vector<pair<int, double>>>& neighbors);

	//! �ڰ뾶radius�м���knn�������
	static bool computeRadiusNeighbors(vector<Eigen::Vector3f> datapts, vector<Eigen::Vector3f> querypts, int knn, double radius, vector<vector<pair<int, double>>>& neighbors);

	//! ������������֮��ı任��ϵ
	static Eigen::Quaterniond getRotation(const Eigen::Vector3f& from, const Eigen::Vector3f& to);

	//! ��Ԫ��ת���任����
	static void quat2eigen4f(const Eigen::Quaterniond q, Eigen::Matrix4f& m);

	//! ��Ԫ��ת���任����
	static Eigen::Matrix4f quat2eigen4f(float x, float y, float z, float w);

	//! ��������֮��ľ���
	static float getDistancePoint2Point(Eigen::Vector3f from, Eigen::Vector3f to);

	//! �õ���ת����
	static Eigen::Matrix4f getRotationMat(float radian, string axis);

	//! mat4f ת�� ryp
	static void eigen4f2rpy(const Eigen::Matrix4f& m, float x[6]);
	static void eigen4f2rpy(const Eigen::Matrix4f& m, Eigen::Vector3f& storeRPY);

	//! rpyת��mat
	static void posrpy2eigen4f(const float x[6], Eigen::Matrix4f& m);

	//! rpyת��mat
	static void posrpy2eigen4f(const Eigen::Vector3f& pos, const Eigen::Vector3f& rpy, Eigen::Matrix4f& m);

	//!	rpyת��mat(��λ��)
	static void rpy2eigen4f(float r, float p, float y, Eigen::Matrix4f& m);

	//! �õ����������м������
	static Eigen::Vector3f createMidVector(const Eigen::Vector3f &vec1, const Eigen::Vector3f &vec2);

	//!
	static Toolbox::Line intersectPlanes(const Plane& p1, const Plane& p2);

	//! Returns nearest point to p on line l
	template<typename VectorT>
	inline static VectorT nearestPointOnLine(const BaseLine<VectorT>& l, const VectorT& p)
	{
		if (!l.isValid())
		{
			return VectorT::Zero();
		}

		VectorT lp = p - l.p;

		float lambda = l.d.dot(lp);

		VectorT res = l.p + lambda * l.d;
		return res;
	}
};

