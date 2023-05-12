#include "stdafx.h"
#include "NodeOutput.h"
#include "RangeImage.h"

static RangeImage rangeImage;

NodeOutput::NodeOutput(void)
{
	targetSize = 0;
	targetIdx = 0;
}

NodeOutput::~NodeOutput(void)
{
}

 Json::Value NodeOutput::toJson() {



	Json::Value res;
	res["pid"] = this->pid;
	res["id"] = this->id;
	res["name"] = this->name;
	res["dataType"] = this->dataType;
	//判断是否深度图
	if (this->dataType == 7) {
		//处理成点云
		if (this->pointCloudList) {
			//pcl::PointCloud<PointT>::Ptr cloudP(new pcl::PointCloud<PointT>);
			//for (int i = 1; i < this->coeff->values.size(); i=i+3) {
			//	PointT p;
			//	p.x = this->coeff->values[i];
			//	p.y = this->coeff->values[i+1];
			//	p.z = this->coeff->values[i+2];
			//	cloudP->push_back(p);
			//}
			//点云转深度图
			string imgBase64 = rangeImage.pointsToImage(this->pointCloudList);
			res["coeff"].append(imgBase64);
		}
	}
	else {
		if (this->coeff) {
			for (int i = 0; i < this->coeff->values.size(); i++) {

				res["coeff"].append(this->coeff->values[i]);
			}
		}
	}

	return res;

}
