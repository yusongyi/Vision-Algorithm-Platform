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
	//�ж��Ƿ����ͼ
	if (this->dataType == 7) {
		//����ɵ���
		if (this->pointCloudList) {
			 
			//����ת���ͼ
			unsigned char *  res = rangeImage.pointsToImage(this->pointCloudList);
			int size = 54 + this->pointCloudList->width*this->pointCloudList->height ;

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
