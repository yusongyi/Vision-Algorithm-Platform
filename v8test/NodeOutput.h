#pragma once 
#include "string"   
#include <pcl/io/pcd_io.h> 
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>  
#include <pcl/ModelCoefficients.h>

#include "json/json.h"
using namespace std;
 
//�����������
typedef pcl::PointXYZ PointT;

class NodeInput;

class NodeOutput
{
public:
	NodeOutput(void);
	virtual ~NodeOutput(void);
	Json::Value toJson();

	//�ڵ���
	string pid;

	//���
	string id;

	//����
	string name;

	//��������
	//���� = 1,
	// �� = 2,
	// �� = 3,
	// �� = 4
	//�߶� = 5
	//Բ�� = 6
	//���ͼ = 7
	int dataType;

	//�����Ķ������
	NodeInput* targets;

	//�������㷨���̳�ʼ��
	int targetSize;
	int targetIdx;

	/**
	�������
	�㣺line_coeff.values.resize (3);    // We need 3 values
		line_coeff.values[0] = point.x ();
		line_coeff.values[1] = point.y ();
		line_coeff.values[2] = point.z ();

	�ߣ�line_coeff.values.resize (6);    // We need 6 values  (point_on_line, line_direction)
		line_coeff.values[0] = point_on_line.x ();
		line_coeff.values[1] = point_on_line.y ();
		line_coeff.values[2] = point_on_line.z ();
 
		line_coeff.values[3] = line_direction.x ();
		line_coeff.values[4] = line_direction.y ();
		line_coeff.values[5] = line_direction.z ();

	�棺plane_coeff.values.resize (4);    // We need 4 values  coefficients	the model coefficients (a, b, c, d with ax+by+cz+d=0)
		plane_coeff.values[0] = plane_parameters.x ();
		plane_coeff.values[1] = plane_parameters.y ();
		plane_coeff.values[2] = plane_parameters.z ();
		plane_coeff.values[3] = plane_parameters.w ();
 
	**/
	pcl::ModelCoefficients::Ptr coeff ;
	pcl::PointCloud<PointT>::Ptr pointCloudList;

};

