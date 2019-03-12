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
* @brief	��ȡxml�����mesh������Ϣ
* @param	
* @return	mesh_pts	mesh������Ϣ
*/
vector<CPoint3D> GraspXml::readMesh(string path) {
	vector<CPoint3D> mesh_pts;

	TiXmlDocument* meshDocument = new TiXmlDocument();
	if (!meshDocument->LoadFile(path.c_str()))
	{
		cout << "�޷�����xml�ļ���" << endl;
		cin.get();
	}
	TiXmlElement* RootElement = meshDocument->RootElement();		//��Ŀ¼
	TiXmlElement* NextElement = RootElement->FirstChildElement();		//��Ŀ¼�µĵ�һ���ڵ��

	while (NextElement != NULL)		//�ж���û�ж���
	{
		if (NextElement->ValueTStr() == "Vertices")		//����object�ڵ�
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

	cout << "mesh��ȡ��ɣ�" << endl;
	return mesh_pts;
}


/*
* readSkeleton
* @brief	��ȡxml�����skeleton������Ϣ���Լ�ÿ���Ǽܵ������mesh�������
* @param
* @return	skeleton_info	skeleton������Ϣ���Լ���Ӧmesh����
*/
vector<pair<CPoint3D, vector<int>>> GraspXml::readSkeleton(string path) {
	vector<pair<CPoint3D, vector<int>>> skeleton_info;  //�Ǽ���Ϣ���Ǽܵ��Լ�����mesh�㣩

	TiXmlDocument* SkeDocument = new TiXmlDocument();
	if (!SkeDocument->LoadFile(path.c_str()))
	{
		cout << "�޷�����xml�ļ���" << endl;
		cin.get();
	}
	TiXmlElement* RootElement = SkeDocument->RootElement();		//��Ŀ¼
	TiXmlElement* NextElement = RootElement->FirstChildElement();		//��Ŀ¼�µĵ�һ���ڵ��

	while (NextElement != NULL)		//�ж���û�ж���
	{
		if (NextElement->ValueTStr() == "Vertices")		//����object�ڵ�
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

	cout << "�Ǽܵ��ȡ��ɣ�" << endl;
	return skeleton_info;
}


/*
* readPosition
* @brief	��ȡposition.xml  ���ץȡλ���Լ�����
* @param
* @return	grasp_position	ץȡλ�ú�ץȡ����
*/
vector<pair<CPoint3D, CPoint3D>>  GraspXml::readPosition(string path) {
	vector<pair<CPoint3D, CPoint3D>> grasp_position;

	TiXmlDocument* pose_Document = new TiXmlDocument();
	if (!pose_Document->LoadFile(path.c_str()))
	{
		cout << "�޷�����xml�ļ���" << endl;
		cin.get();
	}
	CPoint3D dir_pt;
	CPoint3D position_pt;
	pair<CPoint3D, CPoint3D> pair_position;

	TiXmlElement* RootElement = pose_Document->RootElement();		//��Ŀ¼
	TiXmlElement* NextElement = RootElement->FirstChildElement();		//��Ŀ¼�µĵ�һ���ڵ��

	while (NextElement != NULL)		//�ж���û�ж���
	{
		if (NextElement->ValueTStr() == "Approach_pose")		//����object�ڵ�
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
	cout << "ץȡλ����Ϣ��ȡ��ɣ�" << endl;
	return grasp_position;
}


/*
* readSegment
* @brief	��ȡsegmentation.xml  ��ȡskeleton���mesh����ķָ���Ϣ
* @param
* @return	���عǼܵ����ŷ����� (0 Endpoint��1 Connect��2 Branch)
*/
vector<vector<int>> GraspXml::readSegment(string path) {
	
	vector<vector<int>> vec_seg;
	vector<int> Endpoint, Branch, Connect;
	TiXmlDocument* Segment_Document = new TiXmlDocument();
	if (!Segment_Document->LoadFile(path.c_str()))
	{
		cout << "�޷�����xml�ļ���" << endl;
		cin.get();
	}

	TiXmlElement* RootElement = Segment_Document->RootElement();		//��Ŀ¼
	TiXmlElement* NextElement = RootElement->FirstChildElement();		//��Ŀ¼�µĵ�һ���ڵ��

	while (NextElement != NULL)		//�ж���û�ж���
	{
		if (NextElement->ValueTStr() == "SkeletonPart")		//����object�ڵ�
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
	cout << "�ָ���Ϣ��ȡ��ɣ�" << endl;
	return vec_seg;
}


/*
* readContact
* @brief	��ȡcontact.xml  ��ȡץȡ�Ӵ�����Ϣ
* @param
* @return	contact_preshape ���ؽӴ����Լ�ץȡ��ʽ��power/precision��
*/
vector<pair<vector<CPoint3D>, string>> GraspXml::readContact(string path) {
	//================��ȡconnect.txt========================
	vector<pair<vector<CPoint3D>, string>> contact_preshape;
	vector<vector<CPoint3D>> contact_pts;
	vector<CPoint3D> contact;
	vector<string> vec_preshape;

	TiXmlDocument* Document = new TiXmlDocument();
	if (!Document->LoadFile(path.c_str()))
	{
		cout << "�޷�����xml�ļ���" << endl;
		cin.get();
	}
	TiXmlElement* RootElement = Document->RootElement();		//��Ŀ¼
	TiXmlElement* NextElement = RootElement->FirstChildElement();		//��Ŀ¼�µĵ�һ���ڵ��

	while (NextElement != NULL)		//�ж���û�ж���
	{
		if (NextElement->ValueTStr() == "graspType")		//����object�ڵ�
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
	cout << "�Ӵ����ȡ��ɣ�" << endl;
	return contact_preshape;
}


/*
* main_process
* @brief  ��������
*/
void GraspXml::main_process() {
	//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
	//*			�ֱ��ȡ����ļ�			   *
	//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
	vector<CPoint3D> mesh = readMesh(filepath+"\\obj\\CGALSkeleton\\mesh\\CGALMesh.xml");
	vector<pair<CPoint3D, vector<int>>> skeleton = readSkeleton(filepath + "\\obj\\CGALSkeleton\\skeleton\\CGALSkeleton.xml");
	vector<vector<int>> segment = readSegment(filepath + "\\obj\\CGALSkeleton\\segmentation\\CGALSegmentation.xml");
	// readPlan();
	vector<pair<vector<CPoint3D>, string>> contact = readContact(filepath+"\\obj\\contact.xml");
	vector<pair<CPoint3D, CPoint3D>> position = readPosition(filepath + "\\obj\\position.xml");


	//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
	//*					��������			   *
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

		//�ҵ�mesh�����Ӧ��ץȡ��ʽ�����ܲ����ڣ�indexΪ��
		vector<int> index;
		for (int i = 0; i < vec_grasp_plan.size(); i++) {
			if (vec_grasp_plan[i].position == mesh_skeleton.skeleton_pt)
				index.push_back(vec_grasp_plan[i].index);
		}
		mesh_skeleton.plan_index = index;

		vec_mesh_skeleton.push_back(mesh_skeleton);
	}
}



