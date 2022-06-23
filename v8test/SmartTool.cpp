#include "StdAfx.h"
#include "SmartTool.h"
//�߳���ͣ
void mySleep(int s)
{
	std::this_thread::sleep_for(std::chrono::duration<double>(s));
}

//��ȡ��Ŀ¼�������ļ�����������չ������
void getFiles(string path, string exd, vector<string>& files)
{
	//cout << "getFiles()" << path<< endl; 
	//�ļ����
	long  hFile = 0;
	//�ļ���Ϣ
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

			//������ļ����������ļ���,����֮
			//�������,�����б�
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
//��ȡ�ļ�������
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
	res = filename.substr(filename.find_last_of('.') + 1);//��ȡ�ļ���׺
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
		std::ifstream ifs(filename, std::ios::in | std::ios::binary); // ��las�ļ�
		liblas::ReaderFactory f;
		liblas::Reader reader = f.CreateWithStream(ifs); // ��ȡlas�ļ� 
		unsigned long int nbPoints = reader.GetHeader().GetPointRecordsCount();//��ȡlas���ݵ�ĸ��� 

		cloud->width = nbPoints;	//��֤��las���ݵ�ĸ���һ��	
		cloud->height = 1;
		cloud->is_dense = false;

		while (reader.ReadNextPoint())
		{

			// ��ȡlas���ݵ�x��y��z��Ϣ 
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

//��ʾ
void visualization_point(PointCloud<PointXYZ>::Ptr &raw_point, PointCloud<PointXYZ>::Ptr &sor_cloud
	//, PointCloud<PointXYZ>::Ptr &voxel, PointCloud<PointXYZ>::Ptr &uniform
) {
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3d viewer"));
	int v1(0), v2(0), v3(0), v4(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->addPointCloud(raw_point, "cloud1", v1);


	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->addPointCloud(sor_cloud, "cloud2", v2);


	/*viewer->createViewPort(0.5, 0.0, 0.75, 1.0, v3);
	viewer->addPointCloud(voxel, "cloud3", v3);

	viewer->createViewPort(0.75, 0.0, 1, 1.0, v3);
	viewer->addPointCloud(uniform, "cloud4", v3);*/

	viewer->spin();
}
