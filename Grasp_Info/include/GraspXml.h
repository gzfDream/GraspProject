#pragma once
#include <ANN/ANN.h>
#include "tinyxml\tinystr.h"
#include "tinyxml\tinyxml.h"
#include <iostream>
#include <string>
#include <fstream>	
#include <vector>
#include <iomanip>
/**
* 实现读取	simox-cgal生成的相关xml文件：mesh，skeleton，segmentation，contact，position
* 然后存入结构体中 MeshSkeleton, GraspPlan
*
*
* @author   gzf
* @update   2018-11-15
*
**/

#include "Mesh/Point3D.h"
#include "Mesh/BaseModel.h"

using namespace std;

class GraspXml
{
public:
	GraspXml(string file_path);
	~GraspXml();
	struct MeshSkeleton {
		CPoint3D mesh_pt;			// mesh顶点
									//CPoint3D mesh_pt_normal;	// mesh顶点法向
		CPoint3D skeleton_pt;		// mesh顶点对应的骨架点
		int seg_type;				// 分类 (0 Endpoint；1 Connect；2 Branch)
		vector<int> plan_index;				// 对应的抓取plan
	};

	struct GraspPlan {
		int index;						//序号
		CPoint3D position;				//抓取中心点（GCP）位置，也就是骨架点位置
		CPoint3D approachDir;			//抓取方向
										//float mat[4][4] = { 0. };		// 旋转矩阵
		vector<CPoint3D> vec_contact;	//接触点
		string preshape;					// 抓取方式 (1 power; 2 precision; 0 NULL)
	};
public:
	//主函数
	void main_process();
	
	// 返回mesh顶点和骨架点，MeshSkeleton
	vector<MeshSkeleton> getMeshSkeleton() {
		return vec_mesh_skeleton;
	}

	// 返回抓取，GraspPlan
	vector<GraspPlan> getGraspPlan() {
		return vec_grasp_plan;
	}

private:
	vector<CPoint3D> readMesh(string path);
	vector<pair<CPoint3D, vector<int>>> readSkeleton(string path);
	vector<vector<int>> readSegment(string path);
	// readPlan();
	vector<pair<vector<CPoint3D>, string>> readContact(string path);
	vector<pair<CPoint3D, CPoint3D>> readPosition(string path);

private:
	string filepath;
	vector<MeshSkeleton> vec_mesh_skeleton;
	vector<GraspPlan> vec_grasp_plan;
};

