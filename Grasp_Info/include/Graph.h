#ifndef _GRAPH_H
#define _GRAPH_H

#include <iostream>
#include <string.h>
#include <vector>
#include "Mesh\Point3D.h"

using namespace::std;

///////////////////////////////////////////////////////////////////////////////////////////
//通用图结构
template <typename T>
class graph {
public:
	bool is_empty()const;
	bool is_full()const;

	int get_numvertices()const;    //当前顶点数
	int get_numedges()const;       //当前边数
public:
	virtual bool insert_vertex(const T&) = 0;            //插入顶点
	virtual bool insert_edge(const T&, const T&) = 0;    //插入边

	virtual bool remove_vertex(const T&) = 0;            //删除定点
	virtual bool remove_edge(const T&, const T&) = 0;    //删除边

	virtual int get_firstneighbor(const T&)const = 0;    //得到第一个邻接顶点
	virtual int get_nextneighbor(const T&, const T&)const = 0;    //某邻接顶点的下一个邻接顶点

	virtual void print_graph()const = 0;
	virtual int get_vertex_index(const T&)const = 0;     //得到顶点序号
protected:
	static const int VERTICES_DEFAULT_SIZE = 10;         //默认图顶点数
	int max_vertices;
	int num_vertices;
	int num_edges;
};

template <typename T>
bool graph<T>::is_empty()const
{
	return num_edges == 0;
}

template <typename T>
bool graph<T>::is_full()const
{
	return num_vertices >= max_vertices
		|| num_edges >= max_vertices * (max_vertices - 1) / 2;    //判满，分为顶点满和边满
}

template <typename T>
int graph<T>::get_numvertices()const
{
	return num_vertices;
}

template <typename T>
int graph<T>::get_numedges()const
{
	return num_edges;
}

///////////////////////////////////////////////////////////////////////////////////////////

#define VERTICES_DEFAULT_SIZE graph<T>::VERTICES_DEFAULT_SIZE //派生类继承父类，若父类为模板，派生类不可直接调用父类保护成员
#define num_vertices          graph<T>::num_vertices            //以宏来代替每个位置写作用域的做法
#define num_edges             graph<T>::num_edges
#define max_vertices          graph<T>::max_vertices //另一种方法派生类内定义自己成员函数，可用using graph<T>::num_edges，在类外则不可

///////////////////////////////////////////////////////////////////////////////////////////
//图的邻接矩阵表示法
template <typename T>
class graph_mtx : public graph<T> {
public:
	graph_mtx(int);
	graph_mtx(int(*)[4], const int);         //矩阵构造

	~graph_mtx();
public:
	bool insert_vertex(const T&);
	bool insert_edge(const T&, const T&);

	bool remove_vertex(const T&);
	bool remove_edge(const T&, const T&);

	int get_firstneighbor(const T&)const;
	int get_nextneighbor(const T&, const T&)const;

	int get_vertex_index(const T&)const;
	void print_graph()const;
private:
	T * vertices_list;                        //顶点线性表
	int **edge;                              //内部矩阵
};

template <typename T>
graph_mtx<T>::graph_mtx(int sz/* = VERTICES_DEFAULT_SIZE*/)
{
	max_vertices = sz > VERTICES_DEFAULT_SIZE ? sz
		: VERTICES_DEFAULT_SIZE;
	vertices_list = new T[max_vertices];

	edge = new int*[max_vertices];                    //动态申请二维数组
	for (int i = 0; i<max_vertices; ++i) {
		edge[i] = new int[max_vertices];
		memset(edge[i], 0, sizeof(int)*max_vertices); //清空图
	}

	num_vertices = 0;
	num_edges = 0;
}

template <typename T>
graph_mtx<T>::graph_mtx(int(*mt)[4], const int arr_size)
{
	int edge_count = 0;
	max_vertices = arr_size > VERTICES_DEFAULT_SIZE ? arr_size
		: VERTICES_DEFAULT_SIZE;
	vertices_list = new T[max_vertices];

	edge = new int*[max_vertices];
	for (int i = 0; i<max_vertices; ++i) {
		edge[i] = new int[max_vertices];
		memset(edge[i], 0, sizeof(int)*max_vertices);
	}

	for (int i = 0; i<arr_size; ++i)
		for (int j = 0; j<arr_size; ++j) {
			edge[i][j] = mt[i][j];
			if (mt[i][j] != 0)
				++edge_count;
		}

	num_vertices = arr_size;
	num_edges = edge_count / 2;
}

template <typename T>
graph_mtx<T>::~graph_mtx()
{
	for (int i = 0; i<max_vertices; ++i)
		delete[]edge[i];                     //分别析构，再总析构

	delete edge;
	delete[]vertices_list;
}

template <typename T>
bool graph_mtx<T>::insert_vertex(const T& vert)
{
	if (this->is_full())                       //派生类函数调用父类函数，用this或加作用域
		return false;
	vertices_list[num_vertices++] = vert;
	return true;
}

template <typename T>
bool graph_mtx<T>::insert_edge(const T& vert1, const T& vert2)
{
	if (this->is_full())                       //判满
		return false;

	int index_v1 = get_vertex_index(vert1);   //得到顶点序号
	int index_v2 = get_vertex_index(vert2);

	if (index_v1 == -1 || index_v2 == -1)
		return false;

	edge[index_v1][index_v2] = edge[index_v2][index_v1] = 1;    //无向图
	++num_edges;

	return true;
}

template <typename T>
bool graph_mtx<T>::remove_vertex(const T& vert)          //用最后一个节点直接覆盖删除的节点，转化为删除最后一个节点
{
	int edge_count = 0;
	int index = get_vertex_index(vert);

	if (index == -1)
		return false;

	for (int i = 0; i<num_vertices; ++i) {
		if (edge[index][i] != 0)
			++edge_count;
	}

	if (index != num_vertices - 1) {
		vertices_list[index] = vertices_list[num_vertices - 1];

		for (int i = 0; i<num_vertices; ++i) {                     //此处两个循环不可合并，因为要分别执行，否则右下角数据出错
			edge[index][i] = edge[num_vertices - 1][i];
		}
		for (int i = 0; i<num_vertices; ++i) {
			edge[i][index] = edge[i][num_vertices - 1];
		}
	}

	for (int i = 0; i<num_vertices; ++i) {
		edge[num_vertices - 1][i] = 0;
		edge[i][num_vertices - 1] = 0;
	}

	--num_vertices;
	num_edges -= edge_count;

	return true;
}

template <typename T>
bool graph_mtx<T>::remove_edge(const T& vert1, const T& vert2)
{
	int index_v1 = get_vertex_index(vert1);
	int index_v2 = get_vertex_index(vert2);

	if (index_v1 == -1 || index_v2 == -1)
		return false;

	edge[index_v1][index_v2] = edge[index_v2][index_v1] = 0;
	--num_edges;

	return true;
}

template <typename T>
int graph_mtx<T>::get_firstneighbor(const T& vert)const
{
	int index = get_vertex_index(vert);

	if (index != -1) {
		for (int i = 0; i<num_vertices; ++i) {
			if (edge[index][i] != 0)
				return i;
		}
	}
	return -1;
}

template <typename T>
int graph_mtx<T>::get_nextneighbor(const T& vert1, const T& vert2)const
{
	int index_v1 = get_vertex_index(vert1);
	int index_v2 = get_vertex_index(vert2);

	if (index_v1 != -1 && index_v2 != -1) {
		for (int i = index_v2 + 1; i<num_vertices; ++i) {
			if (edge[index_v1][i] != 0)
				return i;
		}
	}
	return -1;
}

template <typename T>
int graph_mtx<T>::get_vertex_index(const T& vert)const
{
	for (int i = 0; i<num_vertices; ++i) {
		if (vertices_list[i] == vert)
			return i;
	}
	return -1;
}

template <typename T>
void graph_mtx<T>::print_graph()const
{
	if (this->is_empty()) {
		cout << "nil graph" << endl;                      //空图输出nil
		return;
	}

	for (int i = 0; i<num_vertices; ++i) {
		cout << vertices_list[i] << "  ";
	}
	cout << endl;

	for (int i = 0; i<num_vertices; ++i) {
		for (int j = 0; j<num_vertices; ++j)
			cout << edge[i][j] << "  ";
		cout << vertices_list[i] << endl;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//图的邻接表表示法
template <typename T>
struct node {                                              //节点用来存储与它相连接的顶点
	int dest;
	struct node* link;
	node(int d) : dest(d), link(nullptr) {}
};

template <typename T>
struct vertex {                                            //顶点结构体
	T vert;                                               //顶点
	struct node<T>* adj;                                  //指向描述其他节点的链表
	vertex() : vert(T()), adj(nullptr) {}
};

template <typename T>
class graph_lnk : public graph<T> {
public:
	graph_lnk(int);
	~graph_lnk();
public:
	bool insert_vertex(const T& vert);
	bool insert_edge(const T& vert1, const T& vert2);

	bool remove_vertex(const T& vert);
	bool remove_edge(const T& vert1, const T& vert2);

	int get_firstneighbor(const T& vert)const;
	int get_nextneighbor(const T& vert1, const T& vert2)const;

	void print_graph()const;
	int get_vertex_index(const T& vert)const;

	//深度优先搜索
	void DFSUtil(int v, bool visited[], vector<int>& DFSlist);
	void DFS(int v, vector<int>& DFSlist);

	//根据序号得到节点值
	T get_vert(int v);
	//根据节点值判断是否存在边
	bool isExistEdge(const T& vert1, const T& vert2);
	//vector<pair<int, int>> get_Edges();
	//遍历每一条边，并存储
	vector<pair<int, int>> traversingEdges();

	//得到邻接点的数量
	int get_numAdjacent(int v);
	//合并
	void mergeVert(double gap, vector<CPoint3D>& DFSlist);
	void mergeVert_xml(double gap, vector<CPoint3D>& DFSlist, vector<vector<int>> &p_index);

protected:
	void remove_point_self(const int, const int);
	void modify_point_self(const int, const int, const int);
	//有条件的遍历
	void search(int v, vector<int> is, double gap, vector<CPoint3D>& DFSlist, double dis);
	void search_xml(int v, vector<int> is, double gap,  double dis, vector<vector<int>> &p_index, int &in);

private:
	vertex<T>* vertices_table;
	//vector<pair<int, int>> vec_eages;
	
};

template <typename T>
graph_lnk<T>::graph_lnk(int sz /*= VERTICES_DEFAULT_SIZE*/)
{
	max_vertices = sz > max_vertices ? sz : VERTICES_DEFAULT_SIZE;
	vertices_table = new vertex<T>[max_vertices];                 //不用再清空，结构体构造函数已清空

	num_vertices = 0;
	num_edges = 0;
}

template <typename T>
graph_lnk<T>::~graph_lnk()
{
	for (int i = 0; i<max_vertices; ++i) {
		auto tmp = vertices_table[i].adj;
		auto next_tmp = tmp;

		while (tmp != nullptr) {           //删除节点
			next_tmp = tmp->link;
			delete tmp;
			tmp = next_tmp;
		}
	}
	delete[]vertices_table;            //删除表
}

template <typename T>
bool graph_lnk<T>::insert_vertex(const T& vert)
{
	if (this->is_full())
		return false;

	vertices_table[num_vertices++].vert = vert;

	return true;
}

template <typename T>
bool graph_lnk<T>::insert_edge(const T& vert1, const T& vert2)
{
	if (this->is_full())
		return false;

	int index_v1 = get_vertex_index(vert1);
	int index_v2 = get_vertex_index(vert2);

	if (index_v1 == -1 || index_v2 == -1)
		return false;

	auto tmp = vertices_table[index_v1].adj;
	while (tmp != nullptr) {
		if (tmp->dest == index_v2)
			return false;
		tmp = tmp->link;
	}

	tmp = new node<T>(index_v2);                    //采用前插法，比尾插法效率高，省事
	tmp->link = vertices_table[index_v1].adj;
	vertices_table[index_v1].adj = tmp;

	tmp = new node<T>(index_v1);
	tmp->link = vertices_table[index_v2].adj;
	vertices_table[index_v2].adj = tmp;

	//pair<int, int> p(index_v1, index_v2);
	/*pair p2(index_v2, index_v1);
	if (!std::count(vec_eages.begin(), vec_eages.end(), p)&&!std::count(vec_eages.begin(), vec_eages.end(), p))
	{
		vec_eages.push_back(p);
	}*/
	//vec_eages.push_back(p);
	++num_edges;

	return true;
}

template <typename T>
bool graph_lnk<T>::remove_vertex(const T& vert)
{
	int index = get_vertex_index(vert);
	int edge_count = 0;

	if (index == -1)
		return false;

	auto tmp = vertices_table[index].adj;
	auto next_tmp = tmp;

	while (tmp != nullptr) {
		remove_point_self(index, tmp->dest);       //删除目标顶点指向自己的节点   
		++edge_count;                              //记录边数
		next_tmp = tmp->link;
		delete tmp;
		tmp = next_tmp;
	}

	if (index != num_vertices - 1) {
		vertices_table[index].adj = vertices_table[num_vertices - 1].adj;     //用最后一个顶点覆盖要删除的顶点
		vertices_table[index].vert = vertices_table[num_vertices - 1].vert;
	}

	tmp = vertices_table[index].adj;
	while (tmp != nullptr) {
		modify_point_self(tmp->dest, num_vertices - 1, index);                //调整原指向最后一个顶点的节点，使其指向新的位置
		tmp = tmp->link;
	}

	--num_vertices;
	num_edges -= edge_count;

	return true;
}

template <typename T>
bool graph_lnk<T>::remove_edge(const T& vert1, const T& vert2)
{
	int index_v1 = get_vertex_index(vert1);
	int index_v2 = get_vertex_index(vert2);

	if (index_v1 == -1 || index_v2 == -1)
		return false;

	remove_point_self(index_v1, index_v2);                //相互删除指向自己的节点，相当于删除边
	remove_point_self(index_v2, index_v1);

	return true;
}

template <typename T>
int graph_lnk<T>::get_firstneighbor(const T& vert)const
{
	int index = get_vertex_index(vert);

	if (index != -1 && vertices_table[index].adj != nullptr) {    //第一个结点即为所求
		return vertices_table[index].adj->dest;
	}
	return -1;
}

template <typename T>
int graph_lnk<T>::get_nextneighbor(const T& vert1, const T& vert2)const    //目标结点的下一个邻接节点即为所求
{
	int index_v1 = get_vertex_index(vert1);
	int index_v2 = get_vertex_index(vert2);

	if (index_v1 != -1 && index_v2 != -1) {
		auto tmp = vertices_table[index_v1].adj;

		while (tmp != nullptr && tmp->dest != index_v2)
			tmp = tmp->link;

		if (tmp->dest == index_v2 && tmp->link != nullptr)
			return tmp->link->dest;
	}
	return -1;
}

template <typename T>
void graph_lnk<T>::remove_point_self(const int self_index, const int other_index)       //删除指向自己的结点
{
	auto tmp = vertices_table[other_index].adj;
	auto pre_tmp = tmp;

	while (tmp->dest != self_index) {
		pre_tmp = tmp;
		tmp = tmp->link;
	}

	if (pre_tmp == tmp)
		vertices_table[other_index].adj = tmp->link;
	else
		pre_tmp->link = tmp->link;

	delete tmp;
}

template <typename T>
void graph_lnk<T>::modify_point_self(const int other_index,
	const int old_index, const int new_index)         //调整原指向最后一个顶点的节点，使其指向新的位置
{
	auto tmp = vertices_table[other_index].adj;

	while (tmp->dest != old_index) {
		tmp = tmp->link;
	}
	tmp->dest = new_index;
}

template <typename T>
int graph_lnk<T>::get_vertex_index(const T& vert)const
{
	for (int i = 0; i<num_vertices; ++i) {
		if (vertices_table[i].vert == vert)
			return i;
	}
	return -1;
}

template <typename T>
void graph_lnk<T>::print_graph()const
{
	if (this->is_empty()) {
		cout << "nil graph" << endl;
		return;
	}

	for (int i = 0; i<num_vertices; ++i) {
		cout << vertices_table[i].vert << " : ";

		auto tmp = vertices_table[i].adj;
		while (tmp != nullptr) {
			cout << tmp->dest << "-->";
			tmp = tmp->link;
		}
		cout << "nil" << endl;
	}
}

template <typename T>
void graph_lnk<T>::DFSUtil(int v, bool visited[],vector<int>& DFSlist)
{
	visited[v] = true;
	cout << v << " ";
	DFSlist.push_back(v);
	int w = get_firstneighbor(vertices_table[v].vert);
	while (w!=-1)
	{
		if (!visited[w])
			DFSUtil(w, visited, DFSlist);
		w = get_nextneighbor(vertices_table[v].vert, vertices_table[w].vert);
	}
}

template <typename T>
void graph_lnk<T>::DFS(int v, vector<int>& DFSlist)
{
	int count = num_vertices;
	bool *visited = new bool[count];
	for (int i = 0; i < count; ++i)
		visited[i] = false;
	DFSUtil(v, visited, DFSlist);
}

template <typename T>
T graph_lnk<T>::get_vert(int v) {
	return vertices_table[v].vert;
}

//template <typename T>
//vector<pair<int, int>> graph_lnk<T>::get_Edges() {
//	return vec_eages;
//}

template <typename T>
bool graph_lnk<T>::isExistEdge(const T& vert1, const T& vert2) {
	int i = get_vertex_index(vert1);

	auto tmp = vertices_table[i].adj;
	while (tmp != nullptr) {
		if (tmp->dest == get_vertex_index(vert2))
			return true;

		tmp = tmp->link;
	}
	return false;
}

template <typename T>
vector<pair<int, int>> graph_lnk<T>::traversingEdges() {
	vector<pair<int, int>> vec_edges;
	for (int i = 0; i<num_vertices; ++i) {
		//cout << vertices_table[i].vert << " : ";

		auto tmp = vertices_table[i].adj;
		while (tmp != nullptr) {
			pair<int, int>p1(i, tmp->dest);
			pair<int, int>p2(tmp->dest, i);

			if (!std::count(vec_edges.begin(), vec_edges.end(), p1) && !std::count(vec_edges.begin(), vec_edges.end(), p2))
				vec_edges.push_back(p1);
			
			tmp = tmp->link;
		}
		//cout << "nil" << endl;
	}
	return vec_edges;
}

template <typename T>
int graph_lnk<T>::get_numAdjacent(int v) {
	int m_count = 0;
	auto tmp = vertices_table[v].adj;
	while (tmp != nullptr) {
		m_count++;
		tmp = tmp->link;
	}
	return m_count;
}

template <typename T>
void graph_lnk<T>::search(int v, vector<int> is, double gap, vector<CPoint3D>& DFSlist, double dis) {
	is[v] = true;
	int w = get_firstneighbor(vertices_table[v].vert);
	
	while (w != -1)
	{
		if (!is[w]) {
			dis += Distance(vertices_table[v].vert, vertices_table[w].vert);
			if (dis > gap || get_numAdjacent(w)>2) {
				DFSlist.push_back(vertices_table[w].vert);
				dis = 0;
			}
			search(w, is, gap, DFSlist, dis);
		}
		w = get_nextneighbor(vertices_table[v].vert, vertices_table[w].vert);

	}
}

template <typename T>
void graph_lnk<T>::mergeVert(double gap, vector<CPoint3D>& DFSlist) {
	
	vector<int> is;
	for (int i = 0; i < num_vertices; i++)
		is.push_back(0);

	//图中无环
	double dis = 0;
	for (int i = 0; i < num_vertices; i++)
	{
		if (is[i] == 0 && get_numAdjacent(i) > 2) {
			is[i] = 1;
			search(i, is, gap, DFSlist, dis);
			break;
		}
	}
}

template <typename T>
void graph_lnk<T>::search_xml(int v, vector<int> is, double gap, double dis, vector<vector<int>> &p_index, int &in) {
	is[v] = true;

	int w = get_firstneighbor(vertices_table[v].vert);

	while (w != -1)
	{
		if (!is[w]) {
			dis += Distance(vertices_table[v].vert, vertices_table[w].vert);
			p_index[in].push_back(w);
			if (dis > gap || get_numAdjacent(w)!=2) {
				//DFSlist.push_back(vertices_table[w].vert);
				dis = 0;
				in = in + 1;	
			}

			search_xml(w, is, gap, dis, p_index, in);
		}
		w = get_nextneighbor(vertices_table[v].vert, vertices_table[w].vert);
	}
}

template <typename T>
void graph_lnk<T>::mergeVert_xml(double gap, vector<CPoint3D>& DFSlist, vector<vector<int>> &p_index) {

	vector<int> is;
	for (int i = 0; i < num_vertices; i++)
		is.push_back(0);
	p_index.resize(num_vertices);
	//图中无环
	double dis = 0;
	int in=0;
	for (int i = 0; i < num_vertices; i++)
	{
		if (is[i] == 0 && get_numAdjacent(i) > 2) {
			is[i] = 1;
			p_index[in].push_back(i);
			search_xml(i, is, gap, dis, p_index, in);
			break;
		}
	}

	for (int i = 0; i < p_index.size(); i++)
	{
		if (p_index[i].size() != 0)
		{
			int x = p_index[i].size() / 2;
			DFSlist.push_back(vertices_table[p_index[i][x]].vert);
		}
	}
}

#endif