#pragma once
//#pragma once��һ���Ƚϳ��õ�C/C++��ע��
//ֻҪ��ͷ�ļ����ʼ����������ע��
//���ܹ���֤ͷ�ļ�ֻ������һ�Ρ�

/*
������������Ƕ�����ͼ�Ĳ���
�㷨ʵ�֣�
��1��Kahn�㷨
��2��DFS�㷨
�����ڽӱ�洢ͼ
*/
#include<iostream>
#include<string>
#include<stack>
#include "AlgoNode.h"
using namespace std;
//����
struct ArcNode {
	ArcNode * next; //��һ�������ı�
	int adjvex;   //���满β�����ڶ�����е��±�
};
struct Vnode {
	string data; //��������
	ArcNode * firstarc; //��һ�������ڸö����
};

class Graph_DG {
private:
	int vexnum; //ͼ�Ķ�����
	int edge;   //ͼ�ı���
	int * indegree; //ÿ���ߵ�������
	Vnode * arc; //�ڽӱ�
	int size;
public:
	Graph_DG(AlgoNode* algos, int size);
	~Graph_DG();

	AlgoNode* algos;
	//�������ߵĶ����Ƿ�Ϸ�
	bool check_edge_value(int, int);
	//����һ��ͼ
	void createGraph();
	//��ӡ�ڽӱ�
	void print();
	//������������,Kahn�㷨
	bool topological_sort();
	//������������DFS�㷨
	bool topological_sort_by_dfs();
	void dfs(int n, bool * & visit, stack<string> & result);
};