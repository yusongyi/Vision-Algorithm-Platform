#include <iostream>
#include <cstdio>
#include <map>
 
using namespace std;
 
//�������溯��ָ���map
map<string, long> mFuncPtr;
 
//����һ������
void* function1()
{
    printf("I am function1.\n");
    return NULL;
}
 
void* function2()
{
    printf("I am function2.\n");
    return NULL;
}
 
 
//������ָ�뱣�浽map��
void saveFunction()
{
    //cout << (long)&function << endl;
    mFuncPtr.insert( make_pair<string, long>("fun1", (long)&function1) );
    mFuncPtr.insert( make_pair<string, long>("fun2", (long)&function2) );
}
 
//�������ֻ�ȡ����
int getFunction(string funcName )
{
    map<string, long>::iterator it = mFuncPtr.find(funcName);
 
    if(it != mFuncPtr.end() )
        return it->second;
    return 0;
}
 
int main(int argc, char *argv[])
{
    saveFunction();
 
    //��������
    typedef void* (*FUNC)();
 
    //��ȡ����
    long fptr = getFunction("fun1");
    //cout << fptr << endl;
    if( fptr != 0 )
    {
        //���ú���fun1
        ((FUNC)fptr)();
    }
 
    //��ȡ����
    fptr = getFunction("fun2");
    //cout << fptr << endl;
    if( fptr != 0 )
    {
        //���ú���fun2
        ((FUNC)fptr)();
    }
}