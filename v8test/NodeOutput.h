#pragma once 
#include "string"   
#include <pcl/io/pcd_io.h> 
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>  
#include <pcl/ModelCoefficients.h>

#include "json/json.h"
using namespace std;
 
//定义点云类型
typedef pcl::PointXYZ PointT;

class NodeInput;

class NodeOutput
{
public:
	NodeOutput(void);
	virtual ~NodeOutput(void);
	Json::Value toJson();

	//节点编号
	string pid;

	//编号
	string id;

	//名称
	string name;

	//数据类型
	//点云 = 1,
	// 面 = 2,
	// 线 = 3,
	// 点 = 4
	//线段 = 5
	//圆弧 = 6
	//深度图 = 7
	int dataType;

	//关联的多个输入
	NodeInput* targets;

	//仅用于算法流程初始化
	int targetSize;
	int targetIdx;

	/**
	输出数据
	点：line_coeff.values.resize (3);    // We need 3 values
		line_coeff.values[0] = point.x ();
		line_coeff.values[1] = point.y ();
		line_coeff.values[2] = point.z ();

	线：line_coeff.values.resize (6);    // We need 6 values  (point_on_line, line_direction)
		line_coeff.values[0] = point_on_line.x ();
		line_coeff.values[1] = point_on_line.y ();
		line_coeff.values[2] = point_on_line.z ();
 
		line_coeff.values[3] = line_direction.x ();
		line_coeff.values[4] = line_direction.y ();
		line_coeff.values[5] = line_direction.z ();

	面：plane_coeff.values.resize (4);    // We need 4 values  coefficients	the model coefficients (a, b, c, d with ax+by+cz+d=0)
		plane_coeff.values[0] = plane_parameters.x ();
		plane_coeff.values[1] = plane_parameters.y ();
		plane_coeff.values[2] = plane_parameters.z ();
		plane_coeff.values[3] = plane_parameters.w ();
 
	**/
	pcl::ModelCoefficients::Ptr coeff ;
	pcl::PointCloud<PointT>::Ptr pointCloudList;

};

