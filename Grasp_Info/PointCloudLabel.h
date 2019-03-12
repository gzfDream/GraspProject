/**
* 实现读取扫描的点云文件，然后根据对应的模型获得抓取信息，实现给扫描点云点添加label。
*
*
* @author   gzf
* @update   2018-11-15
*
**/
#pragma once
#include <cstdlib>						// C standard library
#include <cstdio>						// C I/O (for sscanf)
#include <cstring>						// string manipulation
#include <fstream>						// file I/O
#include <io.h>
#include <string>
#include <iomanip>

#include <ANN/ANN.h>					// ANN declarations
#include <iostream>
#include <string>
#include <vector>
#include "include/Mesh/Point3D.h"
#include "include/Mesh/BaseModel.h"
#include "include\GraspXml.h"
#include <Eigen\Dense>
#include "Eigen/Eigen"
#include <Eigen/StdVector>

using namespace std;


class PointCloudLabel
{
public:
	PointCloudLabel();
	~PointCloudLabel();

public:
	void getDataSet(string path);
private:
	void getFiles(string path, vector<string>& files, vector<string> &ownname);
	bool readPCTXT(string filename, vector<CPoint3D>& vec_pc, vector<CPoint3D>& vec_pc_normal);
	bool readPCXYZ(string filename, vector<CPoint3D>& vec_pc, vector<CPoint3D>& vec_pc_normal);
	void searchPoints(vector<CPoint3D> data, vector<CPoint3D> query, vector<int>& vec_nearest);

};

