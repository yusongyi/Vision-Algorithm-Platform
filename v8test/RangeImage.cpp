#include "stdafx.h" 
#include "RangeImage.h"
#include <iostream>
#include <sstream>
#include<cstdio>
#include <stdio.h>
#include<sys/timeb.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h> //����ת���ͷ�ļ�
#include <pcl/visualization/range_image_visualizer.h> //���ͼ����ӻ�
#include <pcl/visualization/pcl_visualizer.h>//���ƿ��ӻ�
#include <boost/thread/thread.hpp>//���߳�
#include <pcl/io/png_io.h>//�������ͼ��
#include <pcl/visualization/common/float_image_utils.h>//�������ͼ��



typedef pcl::PointXYZ PointT;
static const std::string base64_chars =
"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
"abcdefghijklmnopqrstuvwxyz"
"0123456789+/";


string RangeImage::pointsToImage(pcl::PointCloud<PointT>::Ptr cloud)
{
	//��1��Ϊ�Ƿֱ��ʣ������洴���ĵ��ƴ������ͼ��
	//���ͼ���е�һ�����ض�Ӧ�ĽǶȴ�С1�㣬�Ƕ�ת����
	float angularResolution = (float)(1.0f * (M_PI / 180.0f));
	// 360.0��ת���ȣ�ɨ���ˮƽ�����360��
	float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));
	// 180.0��ת���ȣ�ɨ��Ĵ�ֱ�߶���180��
	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));
	//�ɼ�λ�ã��������ĳ�ʼλ��
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);

	//ѡ���ϵͳ X�������ң�Y�����£�Z����ǰ
	//���ѡ����LASER_FRAME,��X����ǰ��Y������Z������
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

	//noiseLevel�������Ϊ0.05����5cmΪ�뾶��Բ�ڵ����е��ƽ��ֵ���õ������ֵΪ׼
	float noiseLevel = 0.00;

	//minRange����0������Ϊr����ôr�ڵ����е㱻���ԣ�Ϊä��
	float minRange = 0.0f;
	int borderSize = 1;

	//-------------------�������ͼ��------------------------
	pcl::RangeImage::Ptr rangeImage_ptr(new pcl::RangeImage);
	pcl::RangeImage& rangeImage = *rangeImage_ptr;
	rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	//-------------------��ȡ���ͼ����Ϣ------------------------
	std::cout << rangeImage << "\n";
	//����ʱ���
	long long timeA = systemtime();
	string imageName = "RangeImage_"+ longtostring(timeA)+".png";
	//-------------------���ͼ�ı���------------------------
	float* ranges = rangeImage.getRangesArray();
	unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, rangeImage.width, rangeImage.height);
	pcl::io::saveRgbPNGFile(imageName, rgb_image, rangeImage.width, rangeImage.height);

	//-------------------ת��base64------------------------
	std::fstream f;
	f.open(imageName, std::ios::in | std::ios::binary);
	f.seekg(0, std::ios_base::end);     //����ƫ�������ļ���β
	std::streampos sp = f.tellg();      //��ȡ�ļ���С
	int size = sp;
	char* buffer = (char*)malloc(sizeof(char) * size);
	f.seekg(0, std::ios_base::beg);     //����ƫ�������ļ���ͷ
	f.read(buffer, size);                //���ļ����ݶ���buffer
	std::string imgBase64 = base64_encode(buffer, size);
	std::cout << imgBase64 << "\n";

	//�ر���
	f.close();
	//ɾ��ͼƬ
	const char *savePath = imageName.c_str();
	remove(savePath);
	return "data:image/jpeg;base64,"+imgBase64;
}

//ͼƬתbase64
std::string RangeImage::base64_encode(const char * bytes_to_encode, unsigned int in_len)
{
	std::string ret;
	int i = 0;
	int j = 0;
	unsigned char char_array_3[3];
	unsigned char char_array_4[4];

	while (in_len--)
	{
		char_array_3[i++] = *(bytes_to_encode++);
		if (i == 3)
		{
			char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
			char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
			char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
			char_array_4[3] = char_array_3[2] & 0x3f;
			for (i = 0; (i < 4); i++)
			{
				ret += base64_chars[char_array_4[i]];
			}
			i = 0;
		}
	}
	if (i)
	{
		for (j = i; j < 3; j++)
		{
			char_array_3[j] = '\0';
		}

		char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
		char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
		char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
		char_array_4[3] = char_array_3[2] & 0x3f;

		for (j = 0; (j < i + 1); j++)
		{
			ret += base64_chars[char_array_4[j]];
		}

		while ((i++ < 3))
		{
			ret += '=';
		}

	}
	return ret;
}

//����ʱ���
long long RangeImage::systemtime()
{
	timeb t;
	ftime(&t);
	return t.time * 1000 + t.millitm;
}

//longתstring
string RangeImage::longtostring(long long t)
{
	std::string result;
	stringstream ss;
	ss << t;
	ss >> result;
	return result;
}