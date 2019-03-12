#include "SkeletonConfidenceRatingMethod.h"
#include "GraspPlan.h"
#include "LSEllipse.h"
#include "GraspPlanner.h"
int main() {
	string predict_ske = "F:\\Dropbox\\Dropbox\\RoboticGrasp\\code\\testdata\\airplane\\20\\airplane20_98.ply";
	string scan_pc = "F:\\Dropbox\\Dropbox\\RoboticGrasp\\code\\testdata\\airplane\\20\\airplane20_98_0.ply";
	string result_ske = "F:\\Dropbox\\Dropbox\\RoboticGrasp\\code\\testdata\\airplane\\20\\airplane.skel";
	//SkeletonConfidenceRatingMethod ske(predict_ske, scan_pc);
	//ske.skeletonMeasurement(0.0001);

	GraspPlan grasping(result_ske, scan_pc);
	grasping.graspStrategiesGenerate();

	StructClass::PlanningParameters param;
	param.interval[StructClass::GraspType::Power] = 0.0940f;
	param.interval[StructClass::GraspType::Precision] = 0.0170f;
	
	param.minThickness[StructClass::GraspType::Power] = 0.005f;
	param.maxThickness[StructClass::GraspType::Power] = 0.300f;
	param.minThickness[StructClass::GraspType::Precision] = 0.0001f;
	param.maxThickness[StructClass::GraspType::Precision] = 0.200f;
	
	param.minLength[StructClass::GraspType::Power] = 0.065f;
	param.maxLength[StructClass::GraspType::Power] = 0.4f;
	param.minLength[StructClass::GraspType::Precision] = 0.00013f;
	param.maxLength[StructClass::GraspType::Precision] = 0.06f;

	param.roundThreshold = 1.3;

	GraspPlanner planner(result_ske, scan_pc, param);
	vector<vector<vector<StructClass::GraspStrategy>>> skeletonStragety;
	planner.generateGraspStrategies(skeletonStragety);

	
	// ‰≥ˆ≤‚ ‘
	/*
	string s1 = result_ske.replace(result_ske.length() - 5, 5, "_grasp_strategies.ply");
	ofstream out(s1);
	int num = 0;
	for (int i = 0; i <  skeletonStragety.size(); i++)
		for (int j = 0; j < skeletonStragety[i].size(); j++)
			num += skeletonStragety[i][j].size();

	out << "ply" << endl;
	out << "format ascii 1.0" << endl;
	out << "comment Author: gzf" << endl;
	out << "element vertex " << num << endl;
	out << "property float x" << endl;
	out << "property float y" << endl;
	out << "property float z" << endl;
	out << "property uchar red" << endl;
	out << "property uchar green" << endl;
	out << "property uchar blue" << endl;
	out << "property float nx" << endl;
	out << "property float ny" << endl;
	out << "property float nz" << endl;
	out << "end_header" << endl;
	
	for (int i = 0; i < skeletonStragety.size(); i++) {
		for (int j = 0; j < skeletonStragety[i].size(); j++){
			for (int k = 0; k < skeletonStragety[i][j].size(); k++){
				out << skeletonStragety[i][j][k].position.x() << "" << skeletonStragety[i][j][k].position.y() << "" << skeletonStragety[i][j][k].position.z() << "";
				if (skeletonStragety[i][j][k].type_pose.first == StructClass::GraspType::Power) 
					out << 255 << " " << 0 << " " << 0 << " ";
				
				if (skeletonStragety[i][j][k].type_pose.first == StructClass::GraspType::Precision) 
					out << 0 << " " << 255 << " " << 0 << " ";
				
				if (skeletonStragety[i][j][k].type_pose.first == StructClass::GraspType::m_null) 
					out << 0 << " " << 0 << " " << 255 << " ";
				
				out << skeletonStragety[i][j][k].type_pose.second.x() << " " << skeletonStragety[i][j][k].type_pose.second.y() << " " << skeletonStragety[i][j][k].type_pose.second.z() << endl;
			}
		}
	}

	out.close();
	*/

	ofstream out(result_ske.replace(result_ske.length() - 5, 5, "_grasp_strategies.txt"));
	for (int i = 0; i < skeletonStragety.size(); i++) {
		for (int j = 0; j < skeletonStragety[i].size(); j++) {
			for (int k = 0; k < skeletonStragety[i][j].size(); k++) {
				Eigen::Matrix4f mat;
				Toolbox::posrpy2eigen4f(skeletonStragety[i][j][k].position, skeletonStragety[i][j][k].type_pose.second, mat);
				out << mat << endl;
			}
		}
	}
	out.close();

	system("pause");
	
	return 0;
}
