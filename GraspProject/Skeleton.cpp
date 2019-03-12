#include "Skeleton.h"


//****************************************
//            class: Branch
//****************************************

/**
* pushBackCVertex
* \brief	添加节点到分支
* \param	new_v	点
* \return
**/
void Branch::pushBackCVertex(Eigen::Vector3f& new_v)
{
	curve.push_back(new_v);
	if (!curve.empty()) {
		back_up_head = curve[0];
		back_up_tail = curve[curve.size() - 1];
	}
}

/**
*
* \brief	得到分支的点的数量
* \param
* \return	点的数量
**/
int Branch::getSize()
{
	return curve.size();
}

/**
* isEmpty
* \brief	判断分支是否为空
* \return	空返回True
**/
bool Branch::isEmpty()
{
	return curve.empty();
}

/**
* reverseBranch
* \brief	翻转分支
* \return	
**/
void Branch::Branch::reverseBranch()
{
	std::reverse(curve.begin(), curve.end());
	Eigen::Vector3f temp = back_up_head;
	back_up_head = back_up_tail;
	back_up_tail = temp;
}


//****************************************
//            class: Skeleton
//****************************************

/**
* generateBranchSampleMap
* \brief
* \param
* \return
**/
void Skeleton::generateBranchSampleMap()
{
	vector<Branch>::iterator iter;
	for (iter = branches.begin(); iter != branches.end();)
	{
		if (iter->isEmpty())
		{
			iter = branches.erase(iter);
		}
		else
		{
			++iter;
		}
	}

	branch_sample_map.clear();
	int cnt = 0;
	for (int i = 0; i < branches.size(); i++)
	{
		branches[i].branch_id = i;
		vector<Eigen::Vector3f>& curve = branches[i].curve;
		if (curve.size() <= 1)
		{
			for (int j = 0; j < curve.size(); j++)
			{
				branch_sample_map.push_back(RecordItem(i, j, 0));
				cnt++;
			}
		}
		else
		{
			int j = 0;
			for (; j < curve.size() - 1; j++)
			{
				branch_sample_map.push_back(RecordItem(i, j, 0));
				cnt++;
			}
			branch_sample_map.push_back(RecordItem(i, j, 0));

			cnt++;
		}

	}
	size = cnt;
	branch_num = branches.size();
	chosen_branches.clear();
}


