#include "GraspXml.h"

GraspXml::GraspXml(string file_path)
{
	filepath = file_path;
}

GraspXml::~GraspXml()
{
	
}

int color_[36] = {
	127.0, 76.0, 51.0,   // color - brown
	241.0, 94.0, 13.0,   // color - orange
	0.0, 167.0, 30.0,      // color green
	229.0, 51.0,  51.0, // color - red
	51.0,  51.0,  229.0,  // color - blue
	229.0, 229.0, 127.0,  // color - yellow
	178.0, 51.0,  229.0,  // color - purple
	51.0, 229.0, 229.0,  // color - light blue
	155.0, 162.0, 237.0, // color - purple
	229.0, 51.0, 153.0,  // color - pink
	153.0, 153.0, 178.0,  // color - gray
	229.0, 153.0, 127.0   // color - light pink
};


/*
* readMesh
* @brief	读取xml，获得mesh顶点信息
* @param	
* @return	mesh_pts	mesh顶点信息
*/
vector<CPoint3D> GraspXml::readMesh(string path) {
	vector<CPoint3D> mesh_pts;

	TiXmlDocument* meshDocument = new TiXmlDocument();
	if (!meshDocument->LoadFile(path.c_str()))
	{
		cout << "无法加载xml文件！" << endl;
		cin.get();
	}
	TiXmlElement* RootElement = meshDocument->RootElement();		//根目录
	TiXmlElement* NextElement = RootElement->FirstChildElement();		//根目录下的第一个节点层

	while (NextElement != NULL)		//判断有没有读完
	{
		if (NextElement->ValueTStr() == "Vertices")		//读到object节点
		{
			TiXmlElement* VertElement = NextElement->FirstChildElement();
			while (VertElement != NULL) {
				if (VertElement->ValueTStr() == "Vertex") {
					TiXmlElement*  PointElement = VertElement->FirstChildElement();
					if (PointElement->ValueTStr() == "Point") {
						CPoint3D pt;

						PointElement->QueryDoubleAttribute("x", &pt.x);
						PointElement->QueryDoubleAttribute("y", &pt.y);
						PointElement->QueryDoubleAttribute("z", &pt.z);
						mesh_pts.push_back(pt);
					}
				}
				VertElement = VertElement->NextSiblingElement();
			}
		}
		NextElement = NextElement->NextSiblingElement();
	}
	delete meshDocument;

	cout << "mesh读取完成！" << endl;
	return mesh_pts;
}


/*
* readSkeleton
* @brief	读取xml，获得skeleton顶点信息，以及每个骨架点关联的mesh顶点序号
* @param
* @return	skeleton_info	skeleton顶点信息，以及对应mesh顶点
*/
vector<pair<CPoint3D, vector<int>>> GraspXml::readSkeleton(string path) {
	vector<pair<CPoint3D, vector<int>>> skeleton_info;  //骨架信息（骨架点以及关联mesh点）

	TiXmlDocument* SkeDocument = new TiXmlDocument();
	if (!SkeDocument->LoadFile(path.c_str()))
	{
		cout << "无法加载xml文件！" << endl;
		cin.get();
	}
	TiXmlElement* RootElement = SkeDocument->RootElement();		//根目录
	TiXmlElement* NextElement = RootElement->FirstChildElement();		//根目录下的第一个节点层

	while (NextElement != NULL)		//判断有没有读完
	{
		if (NextElement->ValueTStr() == "Vertices")		//读到object节点
		{
			TiXmlElement* VertElement = NextElement->FirstChildElement();
			while (VertElement != NULL) {
				pair<CPoint3D, vector<int>> pair_pts_index;
				if (VertElement->ValueTStr() == "Point") {
					vector<int> mesh_index;
					TiXmlElement* PointElement = VertElement->FirstChildElement();
					while (PointElement != NULL) {
						if (PointElement->ValueTStr() == "Coordinate") {
							double x,y,z;
							PointElement->QueryDoubleAttribute("x", &x);
							PointElement->QueryDoubleAttribute("y", &y);
							PointElement->QueryDoubleAttribute("z", &z);
							CPoint3D skeleton_pt(x,y,z);
							pair_pts_index.first = skeleton_pt;
						}

						if (PointElement->ValueTStr() == "IndexToMesh") {
							TiXmlElement* IndexElement = PointElement->FirstChildElement();
							while (IndexElement != NULL) {
								int index;
								IndexElement->QueryIntAttribute("index", &index);
								mesh_index.push_back(index);
								IndexElement = IndexElement->NextSiblingElement();
							}
						}
						PointElement = PointElement->NextSiblingElement();
					}
					pair_pts_index.second = mesh_index;
					mesh_index.clear();
				}
				skeleton_info.push_back(pair_pts_index);
				VertElement = VertElement->NextSiblingElement();
			}
		}
		NextElement = NextElement->NextSiblingElement();
	}
	delete SkeDocument;

	cout << "骨架点读取完成！" << endl;
	return skeleton_info;
}


/*
* readPosition
* @brief	读取position.xml  获得抓取位置以及方向
* @param
* @return	grasp_position	抓取位置和抓取方向
*/
vector<pair<CPoint3D, CPoint3D>>  GraspXml::readPosition(string path) {
	vector<pair<CPoint3D, CPoint3D>> grasp_position;

	TiXmlDocument* pose_Document = new TiXmlDocument();
	if (!pose_Document->LoadFile(path.c_str()))
	{
		cout << "无法加载xml文件！" << endl;
		cin.get();
	}
	CPoint3D dir_pt;
	CPoint3D position_pt;
	pair<CPoint3D, CPoint3D> pair_position;

	TiXmlElement* RootElement = pose_Document->RootElement();		//根目录
	TiXmlElement* NextElement = RootElement->FirstChildElement();		//根目录下的第一个节点层

	while (NextElement != NULL)		//判断有没有读完
	{
		if (NextElement->ValueTStr() == "Approach_pose")		//读到object节点
		{
			TiXmlElement* poseElement = NextElement->FirstChildElement();
			while (poseElement != NULL)
			{
				if (poseElement->ValueTStr() == "position") {
					TiXmlElement*  PointElement = poseElement->FirstChildElement();
					if (PointElement->ValueTStr() == "Node") {
						PointElement->QueryDoubleAttribute("x", &position_pt.x);
						PointElement->QueryDoubleAttribute("y", &position_pt.y);
						PointElement->QueryDoubleAttribute("z", &position_pt.z);
						pair_position.first = position_pt;
					}
				}
				if (poseElement->ValueTStr() == "approachDir") {
					TiXmlElement*  PointElement = poseElement->FirstChildElement();
					if (PointElement->ValueTStr() == "Node") {
						PointElement->QueryDoubleAttribute("x", &dir_pt.x);
						PointElement->QueryDoubleAttribute("y", &dir_pt.y);
						PointElement->QueryDoubleAttribute("z", &dir_pt.z);
						pair_position.second = dir_pt;
					}
				}
				poseElement = poseElement->NextSiblingElement();
			}
			grasp_position.push_back(pair_position);
		}
		NextElement = NextElement->NextSiblingElement();
	}
	delete pose_Document;
	cout << "抓取位置信息读取完成！" << endl;
	return grasp_position;
}


/*
* readSegment
* @brief	读取segmentation.xml  读取skeleton点和mesh顶点的分割信息
* @param
* @return	返回骨架点的序号分类结果 (0 Endpoint；1 Connect；2 Branch)
*/
vector<vector<int>> GraspXml::readSegment(string path) {
	
	vector<vector<int>> vec_seg;
	vector<int> Endpoint, Branch, Connect;
	TiXmlDocument* Segment_Document = new TiXmlDocument();
	if (!Segment_Document->LoadFile(path.c_str()))
	{
		cout << "无法加载xml文件！" << endl;
		cin.get();
	}

	TiXmlElement* RootElement = Segment_Document->RootElement();		//根目录
	TiXmlElement* NextElement = RootElement->FirstChildElement();		//根目录下的第一个节点层

	while (NextElement != NULL)		//判断有没有读完
	{
		if (NextElement->ValueTStr() == "SkeletonPart")		//读到object节点
		{
			TiXmlElement* SegElement = NextElement->FirstChildElement();
			while (SegElement != NULL) {
				if (SegElement->ValueTStr() == "SkeletonSegment") {
					TiXmlElement*  PointElement = SegElement->FirstChildElement();
					while (PointElement != NULL)
					{
						if (PointElement->ValueTStr() == "SkeletonPoint") {
							int num;
							int value1 = -1, value2 = -1;
							PointElement->QueryIntAttribute("vertex", &num);
							//cout << num << endl;
							TiXmlElement*  EndpointElement = PointElement->FirstChildElement();
							while (EndpointElement != NULL) {
								if (EndpointElement->ValueTStr() == "Endpoint")
									EndpointElement->QueryIntAttribute("value", &value1);
								if (EndpointElement->ValueTStr() == "Branch")
									EndpointElement->QueryIntAttribute("value", &value2);

								EndpointElement = EndpointElement->NextSiblingElement();
							}
							if (value1 == 0 && value2 == 0)
								Connect.push_back(num);
							if (value1 == 1 && value2 == 0)
								Endpoint.push_back(num);
							if (value1 == 0 && value2 == 1)
								Branch.push_back(num);
						}
						PointElement = PointElement->NextSiblingElement();
					}
				}
				SegElement = SegElement->NextSiblingElement();
			}
		}
		NextElement = NextElement->NextSiblingElement();
	}

	vec_seg.push_back(Endpoint);
	vec_seg.push_back(Connect);
	vec_seg.push_back(Branch);

	delete Segment_Document;
	cout << "分割信息读取完成！" << endl;
	return vec_seg;
}


/*
* readContact
* @brief	读取contact.xml  读取抓取接触点信息
* @param
* @return	contact_preshape 返回接触点以及抓取方式（power/precision）
*/
vector<pair<vector<CPoint3D>, string>> GraspXml::readContact(string path) {
	//================读取connect.txt========================
	vector<pair<vector<CPoint3D>, string>> contact_preshape;
	vector<vector<CPoint3D>> contact_pts;
	vector<CPoint3D> contact;
	vector<string> vec_preshape;

	TiXmlDocument* Document = new TiXmlDocument();
	if (!Document->LoadFile(path.c_str()))
	{
		cout << "无法加载xml文件！" << endl;
		cin.get();
	}
	TiXmlElement* RootElement = Document->RootElement();		//根目录
	TiXmlElement* NextElement = RootElement->FirstChildElement();		//根目录下的第一个节点层

	while (NextElement != NULL)		//判断有没有读完
	{
		if (NextElement->ValueTStr() == "graspType")		//读到object节点
		{
			vec_preshape.push_back(NextElement->ToElement()->Attribute("type"));
			TiXmlElement* VertElement = NextElement->FirstChildElement();
			while (VertElement != NULL) {
				if (VertElement->ValueTStr() == "ValidPrecision") {
					TiXmlElement*  PointElement = VertElement->FirstChildElement();
					if (PointElement->ValueTStr() == "contacts") {
						TiXmlElement*  NoteElement = PointElement->FirstChildElement();
						while (NoteElement != NULL)
						{
							if (NoteElement->ValueTStr() == "Node") {
								CPoint3D pt;

								NoteElement->QueryDoubleAttribute("x", &pt.x);
								NoteElement->QueryDoubleAttribute("y", &pt.y);
								NoteElement->QueryDoubleAttribute("z", &pt.z);
								contact.push_back(pt);
							}
							NoteElement = NoteElement->NextSiblingElement();
						}
					}
				}
				VertElement = VertElement->NextSiblingElement();
			}
		}
		NextElement = NextElement->NextSiblingElement();
		contact_pts.push_back(contact);
		contact.clear();
	}
	pair<vector<CPoint3D>, string> pair_contact;
	if (contact_pts.size() == vec_preshape.size())
		for (int i = 0; i < contact_pts.size(); i++) {
			
			pair_contact.first = contact_pts[i];
			pair_contact.second = vec_preshape[i];
			contact_preshape.push_back(pair_contact);
		}

	delete Document;
	cout << "接触点读取完成！" << endl;
	return contact_preshape;
}


/*
* main_process
* @brief  主处理函数
*/
void GraspXml::main_process() {
	//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
	//*			分别读取相关文件			   *
	//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
	vector<CPoint3D> mesh = readMesh(filepath+"\\obj\\CGALSkeleton\\mesh\\CGALMesh.xml");
	vector<pair<CPoint3D, vector<int>>> skeleton = readSkeleton(filepath + "\\obj\\CGALSkeleton\\skeleton\\CGALSkeleton.xml");
	vector<vector<int>> segment = readSegment(filepath + "\\obj\\CGALSkeleton\\segmentation\\CGALSegmentation.xml");
	// readPlan();
	vector<pair<vector<CPoint3D>, string>> contact = readContact(filepath+"\\obj\\contact.xml");
	vector<pair<CPoint3D, CPoint3D>> position = readPosition(filepath + "\\obj\\position.xml");


	//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
	//*					整理数据			   *
	//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
	for (int i = 0; i < contact.size(); i++) {
		GraspPlan grasp_plan;
		grasp_plan.approachDir = position[i].second;
		grasp_plan.position = position[i].first;
		grasp_plan.preshape = contact[i].second;
		grasp_plan.vec_contact = contact[i].first;
		grasp_plan.index = i;

		vec_grasp_plan.push_back(grasp_plan);
	}

	for (int i = 0; i < mesh.size(); i++) {
		MeshSkeleton mesh_skeleton;
		mesh_skeleton.mesh_pt = mesh[i];
		for (int j = 0; j < skeleton.size(); j++) {
			int num = std::count(skeleton[j].second.begin(), skeleton[j].second.end(), i);
			if (num) {
				mesh_skeleton.skeleton_pt = skeleton[j].first;
				for (int x = 0; x < segment.size(); x++) {
					if (std::count(segment[x].begin(), segment[x].end(), j)) {
						mesh_skeleton.seg_type = x;
						break;
					}
				}
				break;
			}
		}

		//找的mesh顶点对应的抓取方式，可能不存在，index为空
		vector<int> index;
		for (int i = 0; i < vec_grasp_plan.size(); i++) {
			if (vec_grasp_plan[i].position == mesh_skeleton.skeleton_pt)
				index.push_back(vec_grasp_plan[i].index);
		}
		mesh_skeleton.plan_index = index;

		vec_mesh_skeleton.push_back(mesh_skeleton);
	}
}



