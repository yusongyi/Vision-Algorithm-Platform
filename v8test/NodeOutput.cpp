#include "stdafx.h"
#include "NodeOutput.h"

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
	if(this->coeff){
		for (int i = 0; i < this->coeff->values.size(); i++) {

			res["coeff"].append(this->coeff->values[i]);
		} 
	}
	return res;

}
