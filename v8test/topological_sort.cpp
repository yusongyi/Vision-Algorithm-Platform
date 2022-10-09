#include "StdAfx.h"
#include"topological_sort.h"

Graph_DG::Graph_DG(AlgoNode* algos, int size) {

	int vexnum = size;
	int edge = 0;
	for (int i = 0; i < size; i++) {
		AlgoNode item = algos[i];
		for (int j = 0; j < item.outputSize; j++) {

			cout << item.outputs[j]->id << ":" << &item.outputs[j] << endl;
			edge += item.outputs[j]->targetSize;
		}
	}

	this->algos = algos;
	this->size = size;
	this->vexnum = vexnum;
	this->edge = edge;
	this->arc = new Vnode[this->vexnum];
	this->indegree = new int[this->vexnum];
	for (int i = 0; i < this->vexnum; i++) {
		this->indegree[i] = 0;
		this->arc[i].firstarc = NULL;
		this->arc[i].data = algos[i].chName;
	}
}
//�ͷ��ڴ�ռ�
Graph_DG::~Graph_DG() {
	ArcNode * p, *q;
	for (int i = 0; i < this->vexnum; i++) {
		if (this->arc[i].firstarc) {
			p = this->arc[i].firstarc;
			while (p) {
				q = p->next;
				delete p;
				p = q;
			}
		}
	}
	delete[] this->arc;
	delete[] this->indegree;
}
//�ж�����ÿ������ĵıߵ���Ϣ�Ƿ�Ϸ�
//�����1��ʼ���
bool Graph_DG::check_edge_value(int start, int end) {
	if (start<1 || end<1 || start>vexnum || end>vexnum) {
		return false;
	}
	return true;
}
void Graph_DG::createGraph() {
	int count = 0;
	int start, end; 

	//�������нڵ�
	for (int i = 0; i < this->size; i++) {
		AlgoNode item = this->algos[i];

		//�����ڵ����
		for (int j = 0; j < item.outputSize; j++) {
			start = i;

			//�����Ӧ��Щ���룬��ÿһ���ߴ�������
			for (int k = 0; k < item.outputs[j]->targetSize; k++) {
				end = item.outputs[j]->targets[k].pIdx;

				//����һ���µı���
				ArcNode * temp = new ArcNode;
				temp->adjvex = end ;
				temp->next = NULL;
				//�����ǰ����Ļ�û�б�����ʱ��
				if (this->arc[start].firstarc == NULL) {
					this->arc[start].firstarc = temp;
				}
				else {
					ArcNode * now = this->arc[start].firstarc;
					while (now->next) {
						now = now->next;
					}//�ҵ�����������һ�����
					now->next = temp;
				} 
			} 
		}
	}

}
void Graph_DG::print() {
	int count = 0;
	cout << "ͼ���ڽӾ���Ϊ��" << endl;
	//��������������������
	while (count != this->vexnum) {
		//�������Ľ��
		cout << this->arc[count].data << " ";
		ArcNode * temp = this->arc[count].firstarc;
		while (temp) {
			cout << "<" << this->arc[count].data << "," << this->arc[temp->adjvex].data << "> ";
			temp = temp->next;
		}
		cout << "^" << endl;
		++count;
	}
}

bool Graph_DG::topological_sort() {
	cout << "ͼ����������Ϊ��" << endl;
	//ջs���ڱ���ջΪ�յĶ����±�
	stack<int> s;
	int i;
	ArcNode * temp;
	//����ÿ���������ȣ�������indgree������
	for (i = 0; i != this->vexnum; i++) {
		temp = this->arc[i].firstarc;
		while (temp) {
			++this->indegree[temp->adjvex];
			temp = temp->next;
		}

	}

	//�����Ϊ0�Ķ�����ջ
	for (i = 0; i != this->vexnum; i++) {
		if (!indegree[i]) {
			s.push(i);
		}
	}

	AlgoNode* res = new AlgoNode[size];
	int cIdx = 0;
	//count���ڼ�������Ķ������
	int count = 0;
	while (!s.empty()) {//���ջΪ�գ������ѭ��
		i = s.top();
		s.pop();//����ջ��Ԫ�أ�����ջ��Ԫ�س�ջ
		cout << this->arc[i].data << " ";//�����������
		res[cIdx++] = this->algos[i];
		temp = this->arc[i].firstarc;
		while (temp) {
			if (!(--this->indegree[temp->adjvex])) {//�����ȼ��ٵ�Ϊ0������ջ
				s.push(temp->adjvex);
			}
			temp = temp->next;
		}
		++count;
	}
	if (count == this->vexnum) {
		this->algos = res;
		cout << endl;
		return true;
	}
	cout << "��ͼ�л�������������" << endl;
	return false;//˵�����ͼ�л�
}
bool Graph_DG::topological_sort_by_dfs() {
	stack<string> result;
	int i;
	bool * visit = new bool[this->vexnum];
	//��ʼ�����ǵ�visit����
	memset(visit, 0, this->vexnum);
	cout << "����DFS����������Ϊ��" << endl;
	//��ʼִ��DFS�㷨
	for (i = 0; i < this->vexnum; i++) {
		if (!visit[i]) {
			dfs(i, visit, result);
		}
	}
	//����������У���Ϊ����ÿ�ζ����ҵ��˳���Ϊ0�Ķ������ջ�У�
	//�������ʱ��ʵ��Ҫ�����������������ÿ�ζ���������Ϊ0�Ķ���
	for (i = 0; i < this->vexnum; i++) {
		cout << result.top() << " ";
		result.pop();
	}
	cout << endl;
	return true;
}
void Graph_DG::dfs(int n, bool * & visit, stack<string> & result) {

	visit[n] = true;
	ArcNode * temp = this->arc[n].firstarc;
	while (temp) {
		if (!visit[temp->adjvex]) {
			dfs(temp->adjvex, visit, result);
		}
		temp = temp->next;
	}
	//���ڼ��붥�㵽�����е�ʱ������dfs���������˳�֮ʱ��
	//��dfs���������Ǹ��ݹ鷽����
	//����Ҫ��ǰ���㻹���ڱ�ָ����������ʲô���㣬
	//���ͻ�ݹ����dfs�������������˳���
	//��ˣ��˳�dfs��������ζ�ŵ�ǰ����û��ָ����������ı���
	//������ǰ������һ��·���ϵ����һ�����㡣
	//���仰˵��ʵ���Ǵ�ʱ�ö������Ϊ0��
	result.push(this->arc[n].data);

}