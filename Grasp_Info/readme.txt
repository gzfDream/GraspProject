Class 

## GraspXml ##
实现读取	simox-cgal生成的相关xml文件：mesh，skeleton，segmentation，contact，position
然后存入结构体中
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


## *** *** *** *** *** *** *** *** *** *** *** *** *** *** *** ##

## PointCloudLabel ##
实现读取扫描的点云文件，然后根据对应的模型获得抓取信息，实现给扫描点云点添加label。