#pragma once 
#include "string"   
#include <pcl/io/pcd_io.h> 
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>  
#include <pcl/ModelCoefficients.h>
using namespace std;
 

class NodeInput;

class NodeOutput
{
public:
	NodeOutput(void);
	virtual ~NodeOutput(void);

	//节点编号
	string pid;

	//编号
	string id;

	//名称
	string name;

	//数据类型
	// POINT_CLOUD = 1,
	// PLANE = 2,
	// LINE = 3,
	// POINT = 4
	int dataType;

	//关联的多个输入
	NodeInput* targets;

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

};

