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

//BMPͼƬͷ�ֽ���  ͷ��Ϣ54�ֽڣ���ɫ��1024�ֽ�
const int HEADER_BYTE_SIZE = 54 + 1024;

typedef pcl::PointXYZ PointT;
 
// ���Ʒ�Χӳ�䵽ͼ��Χ�ĺ���
inline int mapToImageRange(float value, float minValue, float maxValue) {
	return static_cast<int>((value - minValue) / (maxValue - minValue) * 255);
}


void write_bmpheader(unsigned char *bitmap, int offset, int bytes, int value)
{
	int i;
	for (i = 0; i < bytes; i++)
		bitmap[offset + i] = (value >> (i << 3)) & 0xFF;
}

int RangeImage::getImgSize(pcl::PointCloud<PointT>::Ptr cloud) {
	return  cloud->points.size() + HEADER_BYTE_SIZE;
}

unsigned char * RangeImage::pointsToImage(pcl::PointCloud<PointT>::Ptr cloud)
{  
	cloud->width = cloud->points.size()/1000;
	cloud->height = 1000;
	int width = cloud->width;
	int height = cloud->height;
 
	//-------------------���ͼ�ı���------------------------
	 // ��ȡ���Ƶ���С����� Z ֵ
	float minZ =1;
	float maxZ = 4;
 
	// �����Ҷ�ͼ���������� 
	unsigned char * imageData = new unsigned char[width * height];
	int idx = 0;
	// ������ת��Ϊ�Ҷ�ͼ����������
	for (const auto& point : cloud->points) { 
		// ����� Z ֵӳ�䵽 0-255 ��������Ϊ�Ҷ�ֵ
		unsigned char grayscaleValue = static_cast<unsigned char>(mapToImageRange(point.y, minZ, maxZ)); 
		imageData[idx++] = grayscaleValue; 
	}
 

	/*create a bmp format file*/
	int bitmap_x = width ;
	unsigned char *bitmap = (unsigned char*)malloc(sizeof(unsigned char)*height*bitmap_x + HEADER_BYTE_SIZE);

	bitmap[0] = 'B';
	bitmap[1] = 'M';
	write_bmpheader(bitmap, 2, 4, height*bitmap_x + HEADER_BYTE_SIZE); //whole file size
	write_bmpheader(bitmap, 0xA, 4, HEADER_BYTE_SIZE); //offset before bitmap raw data
	write_bmpheader(bitmap, 0xE, 4, 40); //length of bitmap info header
	write_bmpheader(bitmap, 0x12, 4, width); //width
	write_bmpheader(bitmap, 0x16, 4, height); //height
	write_bmpheader(bitmap, 0x1A, 2, 1);
	write_bmpheader(bitmap, 0x1C, 2, 8); //bit per pixel
	write_bmpheader(bitmap, 0x1E, 4, 0); //compression
	write_bmpheader(bitmap, 0x22, 4, height*bitmap_x); //size of bitmap raw data
	for (int i = 0x26; i < 0x36; i++)
		 bitmap[i] = 0;

	int j = 0;
	for (int i = 0; i <= 255; i++) {
		bitmap[54  + (j++)] = i;
		bitmap[54  + (j++)] = i;
		bitmap[54  + (j++)] = i;
		bitmap[54  + (j++)] = 0; 
	}
	int k = HEADER_BYTE_SIZE;
	memcpy(bitmap + k, imageData, height*width); 
	delete[] imageData;
	return  bitmap;
}
 

 