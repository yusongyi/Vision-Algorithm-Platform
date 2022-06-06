#include <iostream>
#include <cstdio>
#include <map>
 
using namespace std;
 
//用来保存函数指针的map
map<string, long> mFuncPtr;
 
//定义一个函数
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
 
 
//将函数指针保存到map中
void saveFunction()
{
    //cout << (long)&function << endl;
    mFuncPtr.insert( make_pair<string, long>("fun1", (long)&function1) );
    mFuncPtr.insert( make_pair<string, long>("fun2", (long)&function2) );
}
 
//根据名字获取函数
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
 
    //声明类型
    typedef void* (*FUNC)();
 
    //获取函数
    long fptr = getFunction("fun1");
    //cout << fptr << endl;
    if( fptr != 0 )
    {
        //调用函数fun1
        ((FUNC)fptr)();
    }
 
    //获取函数
    fptr = getFunction("fun2");
    //cout << fptr << endl;
    if( fptr != 0 )
    {
        //调用函数fun2
        ((FUNC)fptr)();
    }
}