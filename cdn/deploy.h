#ifndef __ROUTE_H__
#define __ROUTE_H__

#include "lib/lib_io.h"
#include <vector>
#include <cstring>
#include <queue>
#include <iostream>
#include <stack>

#include <string.h>
#include <stdlib.h>
#include <algorithm>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <fstream>

//*********结构体信息************
struct Edge
{
    int from;         //源点
    int to;           //终点
    int flow;         //源点到终点的带宽
    int next;         //下一个从源点出发的边的对应位置
    int cost;         //单位带宽消耗
};
struct ConsumeNode
{
    int next_to;      //消费节点直连服务器位置
    int need_cap;     //消费节点所需要的消费带宽
};
struct Point
{
    std::vector<int> J_end;    //服务器位置
    int cost;           //当前花费
};
struct ServerType
{
    int type;
    int cap;
    int price;
};
int deploy_server(char * graph[MAX_EDGE_NUM], int edge_num, char * filename);
//构图
void pre();
void getGraph();
void newGetGraph(Point p);
void preload();
void addEdge(int from,int to,int cap,int cost);
void addEdge1(int from,int to,int cap);
void addConsumeNode(int num,int nextto,int cap);
void quick_sort(int *a,int *pos,int left, int right);
//主题策略部分
void Predicte_Probability();
bool Init();
//禁忌搜索
bool TabuSearchInit();
void TabuSearch();
//算法部分
int SPFA(int src,int to,Point &p);
int SPFA1(int src,int to,Point &p,int node);
int zuidaliu(Point &p);
int zuidaliu1(Point &p);
int zuidaliutest(Point &p);
int chooseServerType(int flow);
void Simplex(Point p);
//文件输出部分
int findpath(int u, int delta);
void save_file();
int findcosume(int node);   //存储路径用
#endif
