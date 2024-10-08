#include "StdAfx.h"
#include "SmartTool.h"


#include <string>
#include <locale>
#include <codecvt>
//线程暂停
void mySleep(int s)
{
	std::this_thread::sleep_for(std::chrono::duration<double>(s));
}

//获取此目录下所有文件，并根据扩展名过滤
void getFiles(string path, string exd, vector<string>& files)
{
	//cout << "getFiles()" << path<< endl; 
	//文件句柄
	long  hFile = 0;
	//文件信息
	struct _finddata_t fileinfo;
	string pathName, exdName;

	if (0 != strcmp(exd.c_str(), ""))
	{
		exdName = "\\Dll_*." + exd;
	}
	else
	{
		exdName = "\\Dll_*";
	}

	if ((hFile = _findfirst(pathName.assign(path).append(exdName).c_str(), &fileinfo)) != -1)
	{
		do
		{
			//cout << fileinfo.name << endl; 

			//如果是文件夹中仍有文件夹,迭代之
			//如果不是,加入列表
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles(pathName.assign(path).append("\\").append(fileinfo.name), exd, files);
			}
			else
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					files.push_back(pathName.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}
//获取文件的名称
void get_FileBaseName(std::string path, std::string &name)
{
	for (int i = path.size() - 1; i > 0; i--)
	{
		if (path[i] == '\\' || path[i] == '/')
		{
			name = path.substr(i + 1);
			return;
		}
	}
	name = path;
}


string getSuffix(string filename)
{
	string res;
	res = filename.substr(filename.find_last_of('.') + 1);//获取文件后缀
	return  res;
}

void readPointCloud(string filename, pcl::PointCloud<PointT>::Ptr cloud) {
	string suffix = getSuffix(filename);
	if (suffix.compare("pcd") == 0) {
		pcl::io::loadPCDFile(filename, *cloud);
	}
	if (suffix.compare("ply") == 0) {
		pcl::io::loadPLYFile(filename, *cloud);
	}
	if (suffix.compare("las") == 0) {
		std::ifstream ifs(filename, std::ios::in | std::ios::binary); // 打开las文件
		liblas::ReaderFactory f;
		liblas::Reader reader = f.CreateWithStream(ifs); // 读取las文件 
		unsigned long int nbPoints = reader.GetHeader().GetPointRecordsCount();//获取las数据点的个数 

		cloud->width = nbPoints;	//保证与las数据点的个数一致	
		cloud->height = 1;
		cloud->is_dense = false;

		while (reader.ReadNextPoint())
		{

			// 获取las数据的x，y，z信息 
			PointT p;
			p.x = (reader.GetPoint().GetX());
			p.y = (reader.GetPoint().GetY());
			p.z = (reader.GetPoint().GetZ());
			cloud->push_back(p);
		}
	}
}
std::string GBKToUTF8(const std::string& strGBK)
{
	std::string strOutUTF8 = "";
	WCHAR * str1;
	int n = MultiByteToWideChar(CP_ACP, 0, strGBK.c_str(), -1, NULL, 0);
	str1 = new WCHAR[n];
	MultiByteToWideChar(CP_ACP, 0, strGBK.c_str(), -1, str1, n);
	n = WideCharToMultiByte(CP_UTF8, 0, str1, -1, NULL, 0, NULL, NULL);
	char * str2 = new char[n];
	WideCharToMultiByte(CP_UTF8, 0, str1, -1, str2, n, NULL, NULL);
	strOutUTF8 = str2;
	delete[]str1;
	str1 = NULL;
	delete[]str2;
	str2 = NULL;
	return strOutUTF8;
}

std::string UTF8ToGBK(const std::string& strUTF8)
{
	int len = MultiByteToWideChar(CP_UTF8, 0, strUTF8.c_str(), -1, NULL, 0);
	WCHAR * wszGBK = new WCHAR[len + 1];
	memset(wszGBK, 0, len * 2 + 2);
	MultiByteToWideChar(CP_UTF8, 0, (LPCTSTR)strUTF8.c_str(), -1, wszGBK, len);

	len = WideCharToMultiByte(CP_ACP, 0, wszGBK, -1, NULL, 0, NULL, NULL);
	char *szGBK = new char[len + 1];
	memset(szGBK, 0, len + 1);
	WideCharToMultiByte(CP_ACP, 0, wszGBK, -1, szGBK, len, NULL, NULL);
	//strUTF8 = szGBK;
	std::string strTemp(szGBK);
	delete[]szGBK;
	delete[]wszGBK;
	return strTemp;
}
 std::wstring string_to_wstring(const std::string &s)
{
	using default_convert = std::codecvt<wchar_t, char, std::mbstate_t>;
	static std::wstring_convert<default_convert>conv(new default_convert("CHS"));
	return conv.from_bytes(s);
}
 std::string wstring_to_string(const std::wstring &s)
{
	using default_convert = std::codecvt<wchar_t, char, std::mbstate_t>;
	static std::wstring_convert<default_convert>conv(new default_convert("CHS"));
	return conv.to_bytes(s);
}
 std::string ansi_to_utf8(const std::string &s)
{
	static std::wstring_convert<std::codecvt_utf8<wchar_t> > conv;
	return conv.to_bytes(string_to_wstring(s));
}
 std::string utf8_to_ansi(const std::string& s)
{
	static std::wstring_convert<std::codecvt_utf8<wchar_t> > conv;
	return wstring_to_string(conv.from_bytes(s));
}


//显示
void visualization_point(PointCloud<PointXYZ>::Ptr &raw_point, NodeOutput** outputs, int outSize) {
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3d viewer"));
	for (int i = 0; i < outSize; i++) {
		if (outputs[i]->coeff == NULL) {
			continue;
		}
		int v = 0;
		viewer->createViewPort(i/double(outSize), 0.0, (i+1) / double(outSize), 1.0, v);
		if (outputs[i]->dataType == 1 || outputs[i]->dataType == 4) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
			for (int j = 0; j < outputs[i]->coeff->values.size(); j+=3)
			{
				pcl::PointXYZ point;
				point.x = outputs[i]->coeff->values[j];
				point.y = outputs[i]->coeff->values[j+1];
				point.z = outputs[i]->coeff->values[j+2];
				cloud->points.push_back(point);
			}

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud, 255, 0, 0);
			viewer->addPointCloud(cloud, single_color1, "cloud"+i, v);
		}
		if (outputs[i]->dataType == 2) {
			viewer->addPlane(*outputs[i]->coeff, "plan" + i, v);
		}
		if (outputs[i]->dataType == 3) {
			viewer->addLine(*outputs[i]->coeff, "line" + i, v);
		}
	 
	} 
	viewer->spin();
}

//读取
static std::string readConfig(const char* path) {
	FILE* file = fopen(path, "rb");
	if (!file)
		return std::string("");
	fseek(file, 0, SEEK_END);
	long size = ftell(file);
	fseek(file, 0, SEEK_SET);
	std::string text;
	char* buffer = new char[size + 1];
	buffer[size] = 0;
	if (fread(buffer, 1, size, file) == (unsigned long)size)
		text = buffer;
	fclose(file);
	delete[] buffer;
	return text;
}

