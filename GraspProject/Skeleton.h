#pragma once
#include "Toolbox.h"
#include "StructClass.h"
class Branch
{
public:
	Branch() :back_up_head(Eigen::Vector3f(-10, -10, -10)), back_up_tail(Eigen::Vector3f(-10, -10, -10)), branch_id(0) {}
	~Branch() {}

	Branch(const Branch& b)
	{
		curve = b.curve;
		back_up_head = b.back_up_head;
		back_up_tail = b.back_up_tail;
		branch_id = b.branch_id;
		vertexType = b.vertexType;
		vertexPlane = b.vertexPlane;
	}

	void pushBackCVertex(Eigen::Vector3f& new_v);
	Eigen::Vector3f getVertexOfIndex(int index) { return curve[index]; }
	int getSize();
	bool isEmpty();
	void reverseBranch();

	Eigen::Vector3f getHead() { return curve[0]; }
	Eigen::Vector3f getTail() { return curve[curve.size() - 1]; }

public:
	vector<Eigen::Vector3f> curve;
	Eigen::Vector3f back_up_head;
	Eigen::Vector3f back_up_tail;
	int branch_id;

	vector<StructClass::VertexType> vertexType;
	vector<Toolbox::Plane> vertexPlane;
};



class RecordItem
{
public:
	RecordItem() {}
	RecordItem(int i, int j, int k = 0) :branch_i(i), node_j(j), sample_index(k) {}
	int branch_i;
	int node_j;
	int sample_index;
};



class Skeleton
{
public:
	Skeleton() { size = 0; branch_num = 0; }
	Skeleton(const Skeleton& s)
	{
		branches = s.branches;
		size = s.size;
		branch_sample_map = s.branch_sample_map;
		chosen_branches = s.chosen_branches;
		branch_num = s.branch_num;
	}

	bool isEmpty() { return branches.empty(); }
	void generateBranchSampleMap();


public:
	vector<Branch> branches;
	vector<RecordItem> branch_sample_map;
	vector<int> chosen_branches;

	int size;
	int branch_num;

public:
	void clear()
	{
		branches.clear();
		branch_sample_map.clear();
		size = 0;
	}
};
