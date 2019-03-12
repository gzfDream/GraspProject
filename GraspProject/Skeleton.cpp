#include "Skeleton.h"


//****************************************
//            class: Branch
//****************************************

/**
* pushBackCVertex
* \brief	��ӽڵ㵽��֧
* \param	new_v	��
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
* \brief	�õ���֧�ĵ������
* \param
* \return	�������
**/
int Branch::getSize()
{
	return curve.size();
}

/**
* isEmpty
* \brief	�жϷ�֧�Ƿ�Ϊ��
* \return	�շ���True
**/
bool Branch::isEmpty()
{
	return curve.empty();
}

/**
* reverseBranch
* \brief	��ת��֧
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


