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
* ʵ�ֶ�ȡ	simox-cgal���ɵ����xml�ļ���mesh��skeleton��segmentation��contact��position
* Ȼ�����ṹ���� MeshSkeleton, GraspPlan
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
		CPoint3D mesh_pt;			// mesh����
									//CPoint3D mesh_pt_normal;	// mesh���㷨��
		CPoint3D skeleton_pt;		// mesh�����Ӧ�ĹǼܵ�
		int seg_type;				// ���� (0 Endpoint��1 Connect��2 Branch)
		vector<int> plan_index;				// ��Ӧ��ץȡplan
	};

	struct GraspPlan {
		int index;						//���
		CPoint3D position;				//ץȡ���ĵ㣨GCP��λ�ã�Ҳ���ǹǼܵ�λ��
		CPoint3D approachDir;			//ץȡ����
										//float mat[4][4] = { 0. };		// ��ת����
		vector<CPoint3D> vec_contact;	//�Ӵ���
		string preshape;					// ץȡ��ʽ (1 power; 2 precision; 0 NULL)
	};
public:
	//������
	void main_process();
	
	// ����mesh����͹Ǽܵ㣬MeshSkeleton
	vector<MeshSkeleton> getMeshSkeleton() {
		return vec_mesh_skeleton;
	}

	// ����ץȡ��GraspPlan
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

