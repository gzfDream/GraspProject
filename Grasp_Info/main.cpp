/****************************************
**功能：读取抓取信息，建立数据集
**基于文章：Planning High-Quality Grasps using Mean Curvature Object Skeletons
*****************************************/
#include "include\GraspXml.h"
#include "PointCloudLabel.h"
int main() {
	PointCloudLabel pcLabel;
	pcLabel.getDataSet("H:\\Grasp\\dataSet\\original_data\\model\\airplane0");
	return 0;
}