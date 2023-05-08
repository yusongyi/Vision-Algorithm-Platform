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

	clock_t start, end;
	start = clock(); 

	cloud->width = 1000;
	cloud->height = 1000;
	//����ʱ���
	long long timeA = systemtime();
	string imageName = "RangeImage_"+ longtostring(timeA)+".png";
	//-------------------���ͼ�ı���------------------------

	pcl::io::savePNGFile(imageName, *cloud, "z");

	end = clock();
	cout << "image time1 = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;

	start = clock();
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
	end = clock();
	cout << "base64 time1 = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
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