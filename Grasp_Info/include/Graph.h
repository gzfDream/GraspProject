#ifndef _GRAPH_H
#define _GRAPH_H

#include <iostream>
#include <string.h>
#include <vector>
#include "Mesh\Point3D.h"

using namespace::std;

///////////////////////////////////////////////////////////////////////////////////////////
//ͨ��ͼ�ṹ
template <typename T>
class graph {
public:
	bool is_empty()const;
	bool is_full()const;

	int get_numvertices()const;    //��ǰ������
	int get_numedges()const;       //��ǰ����
public:
	virtual bool insert_vertex(const T&) = 0;            //���붥��
	virtual bool insert_edge(const T&, const T&) = 0;    //�����

	virtual bool remove_vertex(const T&) = 0;            //ɾ������
	virtual bool remove_edge(const T&, const T&) = 0;    //ɾ����

	virtual int get_firstneighbor(const T&)const = 0;    //�õ���һ���ڽӶ���
	virtual int get_nextneighbor(const T&, const T&)const = 0;    //ĳ�ڽӶ������һ���ڽӶ���

	virtual void print_graph()const = 0;
	virtual int get_vertex_index(const T&)const = 0;     //�õ��������
protected:
	static const int VERTICES_DEFAULT_SIZE = 10;         //Ĭ��ͼ������
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
		|| num_edges >= max_vertices * (max_vertices - 1) / 2;    //��������Ϊ�������ͱ���
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

#define VERTICES_DEFAULT_SIZE graph<T>::VERTICES_DEFAULT_SIZE //������̳и��࣬������Ϊģ�壬�����಻��ֱ�ӵ��ø��ౣ����Ա
#define num_vertices          graph<T>::num_vertices            //�Ժ�������ÿ��λ��д�����������
#define num_edges             graph<T>::num_edges
#define max_vertices          graph<T>::max_vertices //��һ�ַ����������ڶ����Լ���Ա����������using graph<T>::num_edges���������򲻿�

///////////////////////////////////////////////////////////////////////////////////////////
//ͼ���ڽӾ����ʾ��
template <typename T>
class graph_mtx : public graph<T> {
public:
	graph_mtx(int);
	graph_mtx(int(*)[4], const int);         //������

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
	T * vertices_list;                        //�������Ա�
	int **edge;                              //�ڲ�����
};

template <typename T>
graph_mtx<T>::graph_mtx(int sz/* = VERTICES_DEFAULT_SIZE*/)
{
	max_vertices = sz > VERTICES_DEFAULT_SIZE ? sz
		: VERTICES_DEFAULT_SIZE;
	vertices_list = new T[max_vertices];

	edge = new int*[max_vertices];                    //��̬�����ά����
	for (int i = 0; i<max_vertices; ++i) {
		edge[i] = new int[max_vertices];
		memset(edge[i], 0, sizeof(int)*max_vertices); //���ͼ
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
		delete[]edge[i];                     //�ֱ���������������

	delete edge;
	delete[]vertices_list;
}

template <typename T>
bool graph_mtx<T>::insert_vertex(const T& vert)
{
	if (this->is_full())                       //�����ຯ�����ø��ຯ������this���������
		return false;
	vertices_list[num_vertices++] = vert;
	return true;
}

template <typename T>
bool graph_mtx<T>::insert_edge(const T& vert1, const T& vert2)
{
	if (this->is_full())                       //����
		return false;

	int index_v1 = get_vertex_index(vert1);   //�õ��������
	int index_v2 = get_vertex_index(vert2);

	if (index_v1 == -1 || index_v2 == -1)
		return false;

	edge[index_v1][index_v2] = edge[index_v2][index_v1] = 1;    //����ͼ
	++num_edges;

	return true;
}

template <typename T>
bool graph_mtx<T>::remove_vertex(const T& vert)          //�����һ���ڵ�ֱ�Ӹ���ɾ���Ľڵ㣬ת��Ϊɾ�����һ���ڵ�
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

		for (int i = 0; i<num_vertices; ++i) {                     //�˴�����ѭ�����ɺϲ�����ΪҪ�ֱ�ִ�У��������½����ݳ���
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
		cout << "nil graph" << endl;                      //��ͼ���nil
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
//ͼ���ڽӱ��ʾ��
template <typename T>
struct node {                                              //�ڵ������洢���������ӵĶ���
	int dest;
	struct node* link;
	node(int d) : dest(d), link(nullptr) {}
};

template <typename T>
struct vertex {                                            //����ṹ��
	T vert;                                               //����
	struct node<T>* adj;                                  //ָ�����������ڵ������
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

	//�����������
	void DFSUtil(int v, bool visited[], vector<int>& DFSlist);
	void DFS(int v, vector<int>& DFSlist);

	//������ŵõ��ڵ�ֵ
	T get_vert(int v);
	//���ݽڵ�ֵ�ж��Ƿ���ڱ�
	bool isExistEdge(const T& vert1, const T& vert2);
	//vector<pair<int, int>> get_Edges();
	//����ÿһ���ߣ����洢
	vector<pair<int, int>> traversingEdges();

	//�õ��ڽӵ������
	int get_numAdjacent(int v);
	//�ϲ�
	void mergeVert(double gap, vector<CPoint3D>& DFSlist);
	void mergeVert_xml(double gap, vector<CPoint3D>& DFSlist, vector<vector<int>> &p_index);

protected:
	void remove_point_self(const int, const int);
	void modify_point_self(const int, const int, const int);
	//�������ı���
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
	vertices_table = new vertex<T>[max_vertices];                 //��������գ��ṹ�幹�캯�������

	num_vertices = 0;
	num_edges = 0;
}

template <typename T>
graph_lnk<T>::~graph_lnk()
{
	for (int i = 0; i<max_vertices; ++i) {
		auto tmp = vertices_table[i].adj;
		auto next_tmp = tmp;

		while (tmp != nullptr) {           //ɾ���ڵ�
			next_tmp = tmp->link;
			delete tmp;
			tmp = next_tmp;
		}
	}
	delete[]vertices_table;            //ɾ����
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

	tmp = new node<T>(index_v2);                    //����ǰ�巨����β�巨Ч�ʸߣ�ʡ��
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
		remove_point_self(index, tmp->dest);       //ɾ��Ŀ�궥��ָ���Լ��Ľڵ�   
		++edge_count;                              //��¼����
		next_tmp = tmp->link;
		delete tmp;
		tmp = next_tmp;
	}

	if (index != num_vertices - 1) {
		vertices_table[index].adj = vertices_table[num_vertices - 1].adj;     //�����һ�����㸲��Ҫɾ���Ķ���
		vertices_table[index].vert = vertices_table[num_vertices - 1].vert;
	}

	tmp = vertices_table[index].adj;
	while (tmp != nullptr) {
		modify_point_self(tmp->dest, num_vertices - 1, index);                //����ԭָ�����һ������Ľڵ㣬ʹ��ָ���µ�λ��
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

	remove_point_self(index_v1, index_v2);                //�໥ɾ��ָ���Լ��Ľڵ㣬�൱��ɾ����
	remove_point_self(index_v2, index_v1);

	return true;
}

template <typename T>
int graph_lnk<T>::get_firstneighbor(const T& vert)const
{
	int index = get_vertex_index(vert);

	if (index != -1 && vertices_table[index].adj != nullptr) {    //��һ����㼴Ϊ����
		return vertices_table[index].adj->dest;
	}
	return -1;
}

template <typename T>
int graph_lnk<T>::get_nextneighbor(const T& vert1, const T& vert2)const    //Ŀ�������һ���ڽӽڵ㼴Ϊ����
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
void graph_lnk<T>::remove_point_self(const int self_index, const int other_index)       //ɾ��ָ���Լ��Ľ��
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
	const int old_index, const int new_index)         //����ԭָ�����һ������Ľڵ㣬ʹ��ָ���µ�λ��
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

	//ͼ���޻�
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
	//ͼ���޻�
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