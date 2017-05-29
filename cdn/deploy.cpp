#include "deploy.h"
#include <stdio.h>
#include <vector>
#include <queue>
#include <deque>
#include <stdint.h>

using namespace std;
#define DEBUG       //调试使用

#define MAXCONSUME_NUM 10000    // 最大消费节点数目
#define MAXN 10000      // 最大路由节点数目
#define MAXM 100000000      //最大边的数目
#define MAX_SERVER_TYPE 20  //服务器最大档次数量
#define INF 2147483647

int MY_cost = 0;
int minServerNum = 0;
int limit_type =0;
//*********服务器的规模设置********************************
const int LARGE = 1000;
const int MEDIAN = 250;
const int SMALL = 100;
vector<int> NodeFlow;
//*********全局使用:路由节点数目 边的数目 消费节点数目**********
int ORIGIN_N,ORIGIN_M,CONSUME_NUM,ORIGIN_TYPE;

//*********图的数据*********
Edge *temp_ee;  //临时图
Edge *edge; //主要的图
Edge *e;  //保存路径时的残余网络
int *NodeCost;  //每个节点的服务器架设成本
ConsumeNode *consume; //记录消费节点
vector<ServerType> server_type;   //保存服务器的类别
int head[MAXN],h[MAXN],temp_head[MAXN];              //保存边的索引 head->edge h->e
bool visit[MAXN];int dis[MAXN];      //spfa算法使用的临时变量
int cnt,cnt1;                             //建表时使用计数
int server_num;         //服务器单价,服务器数目
int flow_sum;                        //消费节点需求总流量
int max_server_flow;
int numberColumns;                 //记录初始边的数目
int SRC,SINK;                        //源点位置,汇点位置
int piS,sum=0;                       //spfa算法临时变量 距离和总价
int road_num = 0;                    //找到的路径数目
int *leftFlow;      //剩余流量
//*******新算法*********************
int *Predict_flow,*Predict_cost;
//*******保存的当前最优解****************
Point choice;

//*********文件输出*****************
vector<int> outpath;
vector<int> road;
int file_Index;     //文件索引
char ans[300000];  //路径保存
int save;

//*****服务器位置记录*********
std::vector<std::string> pos_searched;  //已经搜索过的解
vector<int> yes,no;                     //剔除重复解的临时变量
vector<int> yes1,no1;
vector<int> J_pos;                      //记录当前的服务器位置
vector<int> J_pos1;
//********计时用*******************
time_t start,finish;
double totaltime;

//********概率预测使用**********
vector<double> pos_probability;
vector<double> neg_probability;
double pos_probability_sum=0.0;
double neg_probability_sum=0.0;
int *tabuTable;//禁忌队列
//********读取数据时使用(全局)***********
char **Topo;
int Line_num;

//---------------------------------------------------------------------------------------------------------------------------------
//********快速随机数生成器****************
uint32_t x,y,z,w;
double xorshift128(void)
{
    uint32_t t = x ^ (x << 11);
    x = y; y = z; z = w;
    return (double)((w = w ^ (w >> 19) ^ t ^ (t >> 8))>>1);
}
void initSrand()
{
    srand( (unsigned)time( NULL ) );
    x=(rand()%RAND_MAX)<<1;
    y=(rand()%RAND_MAX)<<1;
    z=(rand()%RAND_MAX)<<1;
    w=(rand()%RAND_MAX)<<1;
}
//*********检查是否重复扫描***************
//输入:当前所检查的解
//输出:1 表示解重复 0 表示解不重复
int is_Same(Point p)
{
    std::string res;
    for(int i=0; i<ORIGIN_N; ++i)
    {
        res+=(p.J_end[i]==-1)?'0':'1';
    }
    int len = (int)pos_searched.size();
    for(int i=len-1; i>0; i--)
    {
        if(pos_searched[i].compare(res)==0)
        {
            return 1 ;
        }
    }
    pos_searched.push_back(res);
    return 0;
}

//*******************你要完成的功能总入口***********************************
//**************************************************************************
int deploy_server(char * topo[MAX_EDGE_NUM], int line_num,char * filename)
{
    Line_num = line_num;
    Topo = topo;

    time(&start);//=clock();//开始计时
    initSrand();
    pre();
    preload();
    Predicte_Probability();
    if(TabuSearchInit())
    {
        TabuSearch();
        //保存文件
        save_file();
        MY_cost = choice.cost;
    }
    else   //无解的情况,直接输出NA
    {
        ans[0]='N';
        ans[1]='A';
    }
    //释放申请的内存
    delete []NodeCost;delete []edge;delete []e;delete []temp_ee;delete []consume;

    delete []Predict_flow;delete []Predict_cost;delete []tabuTable;delete []leftFlow;
    //需要输出的内容
    char * topo_file = ans;

    //直接调用输出文件的方法输出到指定文件中(ps请注意格式的正确性，如果有解，第一行只有一个数据；第二行为空；第三行开始才是具体的数据，数据之间用一个空格分隔开)
    write_result(topo_file, filename);
    return MY_cost;
}

void countServerPos(Point p)
{
    //统计当前所有的服务器位置和消费节点位置
    yes.clear();no.clear();
    for(int i=0; i<ORIGIN_N; ++i)
    {
        if(p.J_end[i] != -1)
            yes.push_back(i);
        else
            no.push_back(i);
    }

    yes1.clear();no1.clear();
    for(int i=0; i<ORIGIN_N; ++i)
    {
        if(p.J_end[i] != -1)
            yes1.push_back(i);
        else
            no1.push_back(i);
    }
}

//*************禁忌搜索算法********************
//*******************************************
int checkNet(Point &p)
{
    int num=0;
    for(int i=0; i<CONSUME_NUM; ++i)
    {
        if(edge[ORIGIN_M*4+i*2].flow!=0)
        {
            //cout<< choice.J_end[edge[ORIGIN_M*4+i*4].from] <<" "<<edge[ORIGIN_M*4+i*4].from<<endl;
            p.J_end[edge[ORIGIN_M*4+i*2].from]=1;
            num++;
        }
    }
    return num;
}
Point curChoice;     //当前解
//禁忌搜索算法初始化
bool TabuSearchInit()
{
    vector<int> J;
    int len = (int)server_type.size();
    J.assign(ORIGIN_N,len-1);

    Point p;
    p.J_end =J;
    choice.cost = INF;
    //迭代出一个初始解
    int maxnum = 0;
    while(maxnum < 3)
    {
        newGetGraph(p);
        p.cost = SPFA1(SRC,SINK,p,len-1);
        if(p.cost != INF)
        {
            p.J_end = J_pos;
        }
        else
        {
            return false;
        }
        if(choice.cost > p.cost)
        {
            maxnum = 0;
            choice.J_end = J_pos1;
            choice.cost = p.cost;
        }
        else if(choice.cost <= p.cost)
            maxnum++;
    }
    choice.cost = zuidaliu(choice);
    /*
    int len1=0;
    if(ORIGIN_N < LARGE)
        len1 = CONSUME_NUM*0.5;
    else
        len1 = CONSUME_NUM*0.3;
    //直连边界的服务器
    {
        J.assign(ORIGIN_N,-1);
        int *a = new int[CONSUME_NUM];
        int *pos = new int[CONSUME_NUM];

        for(int i=0; i<CONSUME_NUM; ++i)
        {
            a[i] = consume[i].need_cap;
            pos[i] = consume[i].next_to;
        }
        quick_sort(a, pos, 0,CONSUME_NUM-1 );

        for (int i = 0; i <len1; i++)	//减少初始服务器数量,预设一个较小的点
        {
            J[pos[i]] = 1;
        }
        p.J_end = J;
        zuidaliu(p);    //最大流最小费用算法
        delete []a;
        delete []pos;
    }
    checkNet(p);         //补点,保证初始的服务器有解
    p.cost = zuidaliu(p);   //最大流最小费用算法
    choice = p;*/
    is_Same(choice);

#ifdef DEBUG
    cout<<choice.cost<<endl;
#endif
    return true;
}

//更新禁忌表
void flushTabuTable()
{
    for(int i=0; i<ORIGIN_N; ++i)
    {
        if(tabuTable[i]>0)
            tabuTable[i]--;
    }
}

const int MEDIAN_NUM1 = 100;
const int MEDIAN_NUM2 = 400;

Point GetNextLARGE(Point p)
{
    Point ans;
    do
    {
        ans= p;

        int i;

        if(no.size()==0 && yes.size()==0)
            i = MEDIAN_NUM2;
        else if(no.size() ==0 && yes.size()!=0)
            i = (int)xorshift128() % (1000-MEDIAN_NUM1) + MEDIAN_NUM1;
        else  if(no.size() !=0 && yes.size()==0)
            i = ((int)xorshift128() % (1000-MEDIAN_NUM2+MEDIAN_NUM1)>MEDIAN_NUM1)? MEDIAN_NUM1-1 :MEDIAN_NUM2;
        else
            i = (int)xorshift128() % 1000;

        if(i<MEDIAN_NUM1 )
        {
            if((int)no.size()>0)
            {
                int pos = (int)xorshift128() % (int)no.size();   //选择一个点加入
                ans.J_end[no[pos]] = 1;
                vector<int>::iterator it = no.begin()+pos;
                no.erase(it);

            }
        }
        else if(MEDIAN_NUM1<=i && i<MEDIAN_NUM2 )
        {
            if((int)yes.size()>0)
            {
                int pos = (int)xorshift128() % (int)yes.size();   //选择到一个点删除
                ans.J_end[yes[pos]] = -1;
                vector<int>::iterator it = yes.begin()+pos;
                yes.erase(it);

            }
        }
        else if(MEDIAN_NUM2<=i && i<1000)
        {
            int y = (int)xorshift128() % (int)no1.size();    //选择到一个点加入
            ans.J_end[no1[y]] = 1;
            y = (int)xorshift128() % (int)yes1.size();    //选择到一个点删除
            ans.J_end[yes1[y]] = -1;

        }
    }while(is_Same(ans));
    return ans;
}

int max_time;
const int MAX_TIME=1000;
Point GetNextMEDIAN(Point p)
{
    Point ans;
    double dur =0;
    max_time = 0;
    do
    {
        max_time++;
        if(ORIGIN_N<LARGE && max_time==MAX_TIME)
            break;
        time(&finish);
        dur = (double)(finish-start);
        if(dur > 84.0)
            break;
        ans= p;

        int i;

        if(no.size()==0 && yes.size()==0)
            i = MEDIAN_NUM2;
        else if(no.size() ==0 && yes.size()!=0)
            i = (int)xorshift128() % (1000-MEDIAN_NUM1) + MEDIAN_NUM1;
        else  if(no.size() !=0 && yes.size()==0)
            i = ((int)xorshift128() % (1000-MEDIAN_NUM2+MEDIAN_NUM1)>MEDIAN_NUM1)? MEDIAN_NUM1-1 :MEDIAN_NUM2;
        else
            i = (int)xorshift128() % 1000;

        if(i<MEDIAN_NUM1 )
        {
            if((int)no.size()>0)
            {
                int pos = (int)xorshift128() % (int)no.size();   //选择一个点加入
                ans.J_end[no[pos]] = 1;
                vector<int>::iterator it = no.begin()+pos;
                no.erase(it);

            }
        }
        else if(MEDIAN_NUM1<=i && i<MEDIAN_NUM2 )
        {
            if((int)yes.size()>0)
            {
                int pos = (int)xorshift128() % (int)yes.size();   //选择到一个点删除
                ans.J_end[yes[pos]] = -1;
                vector<int>::iterator it = yes.begin()+pos;
                yes.erase(it);

            }
        }
        else if(MEDIAN_NUM2<=i && i<1000)
        {
            int y = (int)xorshift128() % (int)yes1.size();    //选择到一个点删除
            ans.J_end[yes1[y]] = -1;
            vector<int> dd;
            dd.clear();
            for (int d = temp_head[yes1[y]]; d != -1; d = temp_ee[d].next)
            {
                if(temp_ee[d].to != SINK && temp_ee[d].cost!=0)
                {
                    dd.push_back(temp_ee[d].to);
                }
            }
            int yy = (int)xorshift128() % (int)dd.size();
            if(dd[yy]>=0 && dd[yy]<ORIGIN_N)
                ans.J_end[dd[yy]] = 1;
            else
            {
                yy = (int)xorshift128() % ORIGIN_N;
                ans.J_end[yy] = 1;
            }
        }
    }while(is_Same(ans));
    return ans;
}

//获取一定数量的领域解
Point* getNeighbourhood(Point p, int tempNeighbourhoodNum)
{
    Point *NeighbourhoodPoint = new Point[tempNeighbourhoodNum];

    for(int i=0; i<tempNeighbourhoodNum; ++i)
    {
        Point ans;
        ans = GetNextMEDIAN(p);
        NeighbourhoodPoint[i] = ans;
    }
    return NeighbourhoodPoint;
}
//禁忌搜索主函数
void TabuSearch()
{
    int neighbourhoodNum = 0;
    double stop_time=0;
    if(ORIGIN_N > LARGE)
    {
        neighbourhoodNum = 1;
        stop_time = 70.0;
    }
    else
    {
        neighbourhoodNum = 3;
        stop_time = 84.0;
    }
    curChoice = choice;
    Point curBestChoice = choice;
    Point lastPoint = choice;
    countServerPos(curChoice);
    double dur;

    for (int it = 1; dur <= stop_time ; ++it)
    {
        Point *neighbourChoice;
        neighbourChoice = getNeighbourhood(curChoice,neighbourhoodNum); //获取一定数量的领域解
        if(max_time==MAX_TIME)
        {
            max_time=0;
            curChoice = lastPoint;
            countServerPos(curChoice);
        }
        for(int i=0; i<neighbourhoodNum; ++i)
        {
            neighbourChoice[i].cost = zuidaliu(neighbourChoice[i]); //计算该解的花费

            if(neighbourChoice[i].cost < curChoice.cost)
            {
                lastPoint = curChoice;
                curChoice =  neighbourChoice[i];
                countServerPos(curChoice);
            }
        }
#ifdef  DEBUG
        // cout<<"curChoice:"<<curChoice.cost<<endl;
#endif
        if(curChoice.cost < curBestChoice.cost)
        {
            curBestChoice = curChoice;
#ifdef DEBUG
            cout<<it<<":"<<curBestChoice.cost<<endl;
#endif
        }
        // flushTabuTable(curChoice);      //更新禁忌表
        time(&finish);
        dur = (double)(finish - start);
        delete []neighbourChoice;
    }
    Simplex(curBestChoice);    //解优化
}

//简单无脑大法
void Simplex(Point p)
{
    Point curchoice = p;    //当前最优解
    int sumflow = 0;        //用于统计当前的总流量
    time_t finish;
    //先统计当前服务器的位置
    vector<int> server_node;
    double dur;
    for(int i = 0; i<ORIGIN_N; ++i)
    {
        if(curchoice.J_end[i]>=0)        //只统计
            server_node.push_back(i);
    }
    /* int len = (int)server_node.size();
    int *a = new int[len];
    int *b = new int[len];
    for(int i = 0; i<len; ++i)
    {
        int pos = server_node[i];
        a[i] = *(leftFlow+pos);
        b[i] = i;
    }
    quick_sort(a,b,0,len-1);
    server_node.clear();
    for(int i=0;i<len;++i)
    {
        server_node.push_back(b[i]);
    }
    delete []a;
    delete []b;*/
    while(!server_node.empty())
    {
        time(&finish);
        dur = (double)(finish-start);
        if(dur >= 88)
        {
            return;
        }
        sumflow = 0;
        curchoice = p;
        int pp = (int)xorshift128() % server_node.size();
        int node = server_node[pp];
        server_node.erase(server_node.begin()+pp);

        curchoice.J_end[node]--;
        //统计当前服务器的总容量
        for(int i=0; i<ORIGIN_N; ++i)
        {
            if(curchoice.J_end[i]>=0)
            {
                sumflow += server_type[curchoice.J_end[i]].cap;
            }
        }
        if(sumflow < flow_sum)
            continue;
        else
        {
            Point temp = curchoice;
            temp.cost = zuidaliutest(temp);
            if(temp.cost < choice.cost)
            {
                choice = temp;
#ifdef DEBUG
                cout<<"减少等级:"<<choice.cost<<endl;
                cout<<"servernum:"<<server_num<<endl;
#endif
                Simplex(curchoice);

            }
        }
    }
}

//增加一条边
void addEdge(int from,int to,int cap,int cost)
{
    edge[cnt].from = from;
    edge[cnt].to = to;
    edge[cnt].flow = cap;
    edge[cnt].cost = cost;
    edge[cnt].next = head[from];
    head[from] = cnt++;
}
//增加一条边
void addEdge1(int from,int to,int cap)
{
    e[cnt1].from = from;
    e[cnt1].to = to;
    e[cnt1].flow = cap;
    e[cnt1].next = h[from];
    h[from] = cnt1++;
}
//预先增加边
void add(int from,int to,int cap,int cost)
{
    temp_ee[numberColumns].from = from;
    temp_ee[numberColumns].to = to;
    temp_ee[numberColumns].flow = cap;
    temp_ee[numberColumns].cost = cost;
    temp_ee[numberColumns].next = temp_head[from];
    temp_head[from] = numberColumns++;
}
//增加消费节点
void addConsumeNode(int num,int nextto,int cap)
{
    consume[num].next_to=nextto;
    consume[num].need_cap=cap;
}
//预处理
void pre()
{
    int a1;
    int cur_line=0; //记录当前行
    char *pData = *(Topo);
    sscanf(pData,"%d%d%d",&ORIGIN_N,&ORIGIN_M,&CONSUME_NUM);

    save = 0;
    max_server_flow = 0;    //服务器最大流量
    //读入服务器的种类和价格
    pos_searched.clear();
    server_type.clear();

    for(int i=0; i<=MAX_SERVER_TYPE; ++i)
    {
        cur_line = 3 + i;
        pData = *(Topo+2+i);

        if(*pData == '\n' || (*pData == '\r' && *(pData+1)=='\n'))  //如果改行为空行跳出循环
            break;

        ServerType temp;
        sscanf(pData,"%d%d%d",&temp.type,&temp.cap,&temp.price);

        server_type.push_back(temp);

        if(max_server_flow < temp.cap)
            max_server_flow = temp.cap;

    }

    NodeCost = new int[ORIGIN_N];           //申请空间

    //读入每个节点的服务器架设成本
    for(int i=0; i<ORIGIN_N; ++i)
    {
        pData = *(Topo+cur_line+i);
        sscanf(pData,"%d%d",&a1,NodeCost+i);
    }

    long ave_cost =0;
    for(int i =0; i<ORIGIN_N; ++i)
    {
        ave_cost += NodeCost[i];
    }
    ave_cost /= ORIGIN_N;
    int *a = new int[(int)server_type.size()];
    for(int i=0; i<(int)server_type.size(); ++i)
    {
        a[i] = max_server_flow * 1.0 / server_type[i].cap *(server_type[i].price+ave_cost);
    }
    long b = a[0];
    long pos = 0 ;
    for(int i=1; i<(int)server_type.size(); ++i)
    {
        if(b>a[i])
        {
            b=a[i];
            pos=i;
        }
    }

    limit_type = pos;
    delete []a;
}
void preload()
{
    int a1,a2,a3,a4;
    int cur_line=0; //记录当前行
    char *pData = *(Topo);
    sscanf(pData,"%d%d%d",&ORIGIN_N,&ORIGIN_M,&CONSUME_NUM);

    save = 0;
    max_server_flow = 0;    //服务器最大流量
    //读入服务器的种类和价格
    pos_searched.clear();
    server_type.clear();
#ifdef DEBUG
    cout<<"limit_type:"<<limit_type<<endl;
#endif
     if(ORIGIN_N < LARGE)
       limit_type=8;
     else
       limit_type=6;
    for(int i=0; i<=MAX_SERVER_TYPE; ++i)
    {
        cur_line = 3 + i;
        pData = *(Topo+2+i);

        if(*pData == '\n' || (*pData == '\r' && *(pData+1)=='\n'))  //如果改行为空行跳出循环
            break;

        ServerType temp;
        sscanf(pData,"%d%d%d",&temp.type,&temp.cap,&temp.price);

        if(i <= limit_type)
        {
            server_type.push_back(temp);

            if(max_server_flow < temp.cap)
                max_server_flow = temp.cap;
        }

    }

    NodeCost = new int[ORIGIN_N];           //申请空间
    tabuTable = new int[ORIGIN_N];
    leftFlow = new int[ORIGIN_N];
    for(int i=0;i<ORIGIN_N;++i)
    {
        tabuTable[i]=0;
    }
    //读入每个节点的服务器架设成本
    for(int i=0; i<ORIGIN_N; ++i)
    {
        pData = *(Topo+cur_line+i);
        sscanf(pData,"%d%d",&a1,NodeCost+i);
    }

    cur_line = cur_line + ORIGIN_N + 1; //注意加上空行

    //构建初始的图
    int deriveline = cur_line + ORIGIN_M;
    numberColumns = 0;
    flow_sum = 0;        //流量总和
    SRC = ORIGIN_N;      //源点
    SINK = SRC + 1;      //汇点
    //初始化原图
    temp_ee = new Edge[ORIGIN_M * 2 + CONSUME_NUM];
    edge = new Edge[ORIGIN_M * 4 + CONSUME_NUM *2 + ORIGIN_N*2];
    e = new Edge[ORIGIN_M * 4 + CONSUME_NUM *2 + ORIGIN_N*2];
    consume = new ConsumeNode[CONSUME_NUM];

    cnt=0;
    for (int i = 0; i <= SINK; ++i)
    {
        head[i] = -1;
        temp_head[i] = -1;
    }
    for(int i=cur_line; i<Line_num ; ++i)
    {
        if(i<deriveline)  //路由节点
        {
            pData = *(Topo+i);

            sscanf(pData,"%d%d%d%d",&a1,&a2,&a3,&a4);
            //源数据处理
            add(a1,a2,a3,a4);   //读入数据

            addEdge(a1, a2, a3, a4);
            addEdge(a2, a1, 0, -a4);

            add(a2,a1,a3,a4);   //读入数据

            addEdge(a2, a1, a3, a4);
            addEdge(a1, a2, 0, -a4);
        }
        else if(i>deriveline)  //路由节点                 //消费节点
        {
            pData = *(Topo+i);
            sscanf(pData,"%d%d%d",&a1,&a2,&a3);
            //源数据处理
            addConsumeNode(a1,a2,a3);
            add(a2,SINK,a3,0);
            flow_sum +=  a3;
        }
    }
    minServerNum = flow_sum / max_server_flow;
#ifdef DEBUG
    cout<<"ORIGIN_N:"<<ORIGIN_N<<endl;
    cout<<"flow_sum:"<<flow_sum<<endl;
    cout<<"minServerNum:"<<minServerNum<<endl;
#endif
}

//建图
void getGraph(Point p)
{
    server_num = 0; cnt=0;
    for (int i = 0; i <= SINK; ++i)
    {
        head[i] = -1;
    }
    //读取原图
    for (int i = 0; i < numberColumns; ++i)
    {
        addEdge(temp_ee[i].from, temp_ee[i].to, temp_ee[i].flow, temp_ee[i].cost);
        addEdge(temp_ee[i].to, temp_ee[i].from, 0, -temp_ee[i].cost);
    }
    //建立服务器的边
    for (int i = 0; i < ORIGIN_N; ++i)
    {
        if (p.J_end[i] >= 0)
        {
            addEdge(SRC, i, max_server_flow, 0);    //总是设置成最大的服务器
            addEdge(i, SRC, 0, 0);
            server_num++;
        }
    }
}
void getGraphtest(Point p)
{
    server_num = 0; cnt=0;
    for (int i = 0; i <= SINK; ++i)
    {
        head[i] = -1;
    }
    //读取原图
    for (int i = 0; i < numberColumns; ++i)
    {
        addEdge(temp_ee[i].from, temp_ee[i].to, temp_ee[i].flow, temp_ee[i].cost);
        addEdge(temp_ee[i].to, temp_ee[i].from, 0, -temp_ee[i].cost);
    }
    //建立服务器的边
    for (int i = 0; i < ORIGIN_N; ++i)
    {
        if (p.J_end[i] >= 0)
        {
            int flow = server_type[p.J_end[i]].cap;
            addEdge(SRC, i, flow, 0);    //总是设置成最大的服务器
            addEdge(i, SRC, 0, 0);
            server_num++;
        }
    }
}


//********深度优先搜索(SPFA内部调用)**********
//输入:起点 u 流量 delta
//输出:最小流量
int dfs(int u, int delta)
{
    if (u == SINK){
        sum += piS * delta;
        return delta;
    }
    visit[u] = true;
    int l = delta;
    for (int i = head[u]; i != -1; i = edge[i].next)
    {
        if (edge[i].flow && !edge[i].cost && !visit[edge[i].to])
        {
            int d = dfs(edge[i].to, min(l, edge[i].flow));
            edge[i].flow -= d;
            edge[i ^ 1].flow += d;
            l -= d;
            if (!l)
            {
                return delta;
            }
        }
    }

    return delta - l;
}
//SPFA最小费用最大流(优化)
//输入:起点 目的点
//输出:有解返回当前cost 无解返回INF
int SPFA1(int src,int to,Point &p,int node)
{
    int tempFlow = 0;   //局部流量
    int flow = 0;   //总流量初始化
    int ser_num = 0;
    sum = 0;        //总费用初始化
    piS = 0;        //局部变量
    J_pos.assign(ORIGIN_N,node);
    J_pos1.assign(ORIGIN_N,-1);
    while(1)
    {
        deque<int> que;
        for (int i = 0; i <= SINK; ++i)
            dis[i] = INF;

        dis[to] = 0;
        que.push_back(to);

        //进行一次广度优先搜索 找到一条最短路径
        while (!que.empty())
        {
            int dt, u = que.front();
            que.pop_front();

            for (int i = head[u]; i != -1; i = edge[i].next)
            {
                if (edge[i ^ 1].flow>0 && (dt = dis[u] - edge[i].cost) < dis[edge[i].to])
                {
                    dis[edge[i].to] = dt;
                    if(!que.empty())
                    {
                        if(dis[edge[i].to]>dis[que.front()])
                            que.push_back(edge[i].to);
                        else
                            que.push_front(edge[i].to);
                    }
                    else
                        que.push_front(edge[i].to);
                }
            }
        }
        for (int u = 0; u <= SINK; ++u){
            for (int i = head[u]; i != -1; i = edge[i].next){
                edge[i].cost += dis[edge[i].to] - dis[u];
            }
        }
        piS += dis[src];
        if(dis[src] ==INF)  //找不到适合的路径了,跳出循环
            break;

        //从源点开始重新跑费用流
        do{
            flow += tempFlow;
            for(int i =0; i<= SINK ;++i)
            {
                visit[i] = false;
            }
        }while((tempFlow=dfs(src,INF)));
    }
    if(flow < flow_sum)
    {
        return INF;
    }

    //该处原来有误,加入新的节点后需要判断服务器是否有流量输出
    for (int i = numberColumns; i < numberColumns+server_num; i++)
    {
        int v = edge[2*i].to;       //得到服务器的位置
        int flow = server_type[p.J_end[v]].cap; //得到服务器的总容量

        if (edge[2*i].flow < flow)  //该服务器有流量输出
        {
            int flow_dif = flow - edge[2*i].flow;    //消耗的流量

            int temp_cost = flow_dif * Predict_cost[v]; //多计算的服务器价格
            int type_ch = chooseServerType(flow_dif);   //挑选合适规模的服务器
            int ser_price = server_type[type_ch].price;

            Predict_flow[v] = flow_dif ;
            Predict_cost[v] = (NodeCost[v] + ser_price ) /  Predict_flow[v];   //更新预测单位花费

            if(type_ch == -1)           //当前流量花费找不到合适的服务器
                return INF;

            J_pos[v] = type_ch;       //标记有流量输出的服务器位置(该处可优化选择合适流量的服务器)
            J_pos1[v] = type_ch;
            ser_num++;

            sum -= temp_cost;
            sum += (server_type[type_ch].price + NodeCost[v]);
        }
    }
    server_num = ser_num;   //更新变更后的服务器的边的数目
    return sum;
}
//SPFA最小费用最大流(优化)
//输入:起点 目的点
//输出:有解返回当前cost 无解返回INF
int SPFA(int src,int to,Point &p)
{
    int tempFlow = 0;   //局部流量
    int flow = 0;   //总流量初始化
    int ser_num = 0;
    sum = 0;        //总费用初始化
    piS = 0;        //局部变量
    J_pos.assign(ORIGIN_N,-1);
    while(1)
    {
        deque<int> que;
        for (int i = 0; i <= SINK; ++i)
            dis[i] = INF;

        dis[to] = 0;
        que.push_back(to);

        //进行一次广度优先搜索 找到一条最短路径
        while (!que.empty())
        {
            int dt, u = que.front();
            que.pop_front();

            for (int i = head[u]; i != -1; i = edge[i].next)
            {
                if (edge[i ^ 1].flow>0 && (dt = dis[u] - edge[i].cost) < dis[edge[i].to])
                {
                    dis[edge[i].to] = dt;
                    if(!que.empty())
                    {
                        if(dis[edge[i].to]>dis[que.front()])
                            que.push_back(edge[i].to);
                        else
                            que.push_front(edge[i].to);
                    }
                    else
                        que.push_front(edge[i].to);
                }
            }
        }
        for (int u = 0; u <= SINK; ++u){
            for (int i = head[u]; i != -1; i = edge[i].next){
                edge[i].cost += dis[edge[i].to] - dis[u];
            }
        }
        piS += dis[src];
        if(dis[src] ==INF)  //找不到适合的路径了,跳出循环
            break;

        //从源点开始重新跑费用流
        do{
            flow += tempFlow;
            for(int i =0; i<= SINK ;++i)
            {
                visit[i] = false;
            }
        }while((tempFlow=dfs(src,INF)));
    }
    if(flow < flow_sum)
    {
        return INF;
    }

    //该处原来有误,加入新的节点后需要判断服务器是否有流量输出

    for (int i = numberColumns; i < numberColumns+server_num; i++)
    {
        int v = edge[2*i].to;
        if (edge[2*i].flow < max_server_flow)  //该服务器有流量输出
        {
            int flow_dif = max_server_flow - edge[2*i].flow;    //消耗的流量,挑选合适规模的服务器
            int type_ch = chooseServerType(flow_dif);

            if(type_ch == -1)           //当前流量花费找不到合适的服务器
                return INF;

            J_pos[v] = type_ch;       //标记有流量输出的服务器位置(该处可优化选择合适流量的服务器)
            ser_num++;

            sum += (server_type[type_ch].price + NodeCost[v]);
        }
    }
    server_num = ser_num;   //更新变更后的服务器的边的数目
    return sum;
}
int SPFAtest(int src,int to,Point &p)
{
    int tempFlow = 0;   //局部流量
    int flow = 0;   //总流量初始化
    int ser_num = 0;
    sum = 0;        //总费用初始化
    piS = 0;        //局部变量
    J_pos.assign(ORIGIN_N,-1);
    while(1)
    {
        deque<int> que;
        for (int i = 0; i <= SINK; ++i)
            dis[i] = INF;

        dis[to] = 0;
        que.push_back(to);

        //进行一次广度优先搜索 找到一条最短路径
        while (!que.empty())
        {
            int dt, u = que.front();
            que.pop_front();

            for (int i = head[u]; i != -1; i = edge[i].next)
            {
                if (edge[i ^ 1].flow>0 && (dt = dis[u] - edge[i].cost) < dis[edge[i].to])
                {
                    dis[edge[i].to] = dt;
                    if(!que.empty())
                    {
                        if(dis[edge[i].to]>dis[que.front()])
                            que.push_back(edge[i].to);
                        else
                            que.push_front(edge[i].to);
                    }
                    else
                        que.push_front(edge[i].to);
                }
            }
        }
        for (int u = 0; u <= SINK; ++u){
            for (int i = head[u]; i != -1; i = edge[i].next){
                edge[i].cost += dis[edge[i].to] - dis[u];
            }
        }
        piS += dis[src];
        if(dis[src] ==INF)  //找不到适合的路径了,跳出循环
            break;

        //从源点开始重新跑费用流
        do{
            flow += tempFlow;
            for(int i =0; i<= SINK ;++i)
            {
                visit[i] = false;
            }
        }while((tempFlow=dfs(src,INF)));
    }
    if(flow < flow_sum)
    {
        return INF;
    }

    //该处原来有误,加入新的节点后需要判断服务器是否有流量输出

    for (int i = numberColumns; i < numberColumns+server_num; i++)
    {
        int v = edge[2*i].to;
        int flow = server_type[p.J_end[v]].cap;
        if (edge[2*i].flow < flow)  //该服务器有流量输出
        {
            int flow_dif = flow - edge[2*i].flow;    //消耗的流量,挑选合适规模的服务器
            int type_ch = chooseServerType(flow_dif);

            if(type_ch == -1)           //当前流量花费找不到合适的服务器
                return INF;

            *(leftFlow+v) = server_type[type_ch].cap-flow_dif;
            J_pos[v] = type_ch;       //标记有流量输出的服务器位置(该处可优化选择合适流量的服务器)
            ser_num++;

            sum += (server_type[type_ch].price + NodeCost[v]);
        }
    }
    server_num = ser_num;   //更新变更后的服务器的边的数目
    return sum;
}
//挑选合适的服务器
int chooseServerType(int flow)
{
    int len = (int)server_type.size();
    for(int i=0; i<len; ++i)
    {
        if(server_type[i].cap>=flow)
            return i;
    }
    return -1;
}
//对所有点位放置服务器的概率进行估计
void Predicte_Probability()
{
    int i;
    //double temp;
    std::vector<double> X1(ORIGIN_N,0); //边数
    std::vector<double> X2(ORIGIN_N,0); //流量
    std::vector<double> X3(ORIGIN_N,0); //消费节点需求
    std::vector<double> X4(ORIGIN_N,0); //架设成本
    NodeFlow.assign(ORIGIN_N,0);

    for(int u=0; u<ORIGIN_N; ++u)
    {
        for(int i=head[u]; i!=-1; i=edge[i].next)   //遍历当前点所有的边
        {
            X2[u] += edge[i].flow;    //流量
            X1[u] ++;                 //边数
        }
        NodeFlow[u] = X2[u];
    }
    for(i=0; i<CONSUME_NUM; i++)//遍历CONSUME_NUM个消费节点
    {
        X3[consume[i].next_to]++;//统计节点连接的用户节点数
    }
    for(i=0; i<ORIGIN_N; ++i)
    {
        X4[i] = (double)NodeCost[i];
    }
    //new add:设置初始预定流量计算价格
    Predict_flow = new int[ORIGIN_N];
    Predict_cost = new int[ORIGIN_N];
    for(i=0; i<ORIGIN_N; ++i)
    {
        Predict_flow[i] = (int)(X2[i]*0.5) +  3*(int)X3[i]+1;
        Predict_cost[i] = (NodeCost[i] + server_type[(int)server_type.size()-1].price) / Predict_flow[i];
    }
    /*for(int i=0; i<ORIGIN_N; ++i)
    {
        if(Predict_cost[i]>800)
        {
            tabuTable[i] = 1;
        }
    }
    //对N个节点做归一化
    temp = *(std::max_element(X1.begin(), X1.end()));  //取X1中最大值，做数据归一化
    for(i=0; i<ORIGIN_N; i++)//遍历M条边
    {
        X1[i] = X1[i]/temp;
    }
    temp = *(std::max_element(X2.begin(), X2.end()));  //取X2中最大值，做数据归一化
    for(i=0; i<ORIGIN_N; i++)//遍历M条边
    {
        X2[i] = X2[i]/temp;
    }
    temp = *(std::max_element(X4.begin(), X4.end()));  //取X2中最大值，做数据归一化
    for(i=0; i<ORIGIN_N; i++)//遍历M条边
    {
        X4[i] = X4[i]/temp;
    }
    pos_probability.assign(ORIGIN_N,0);
    neg_probability.assign(ORIGIN_N,0);

    double a =0.2,b=1.0,c=0.5,d=1.0;
    for(i=0; i<ORIGIN_N; i++)//遍历M条边
    {
        pos_probability[i] = a*X1[i] + b*X2[i] + c*X3[i] + d*X4[i];
        pos_probability_sum += pos_probability[i];
    }
    for(i=0; i<ORIGIN_N; i++)//遍历M条边
    {
        neg_probability[i] = 1 / pos_probability[i];
        neg_probability_sum += neg_probability[i];
    }*/
}
//*************快速排序算法(降序排序)********************
//输入:数据数组和位置数组
//返回:无
void quick_sort(int *a,int *pos,int left, int right)
{
    if(left<right)
    {
        int i = left;
        int j = right;
        int x = a[i];
        int temp_pos = pos[i];

        while(i<j)
        {
            while(i<j && a[j]<x)
                j--;
            if(i<j){
                a[i] = a[j];
                pos[i] = pos [j];
                i++;
            }
            while(i<j && a[i]>x)
                i++;
            if(i<j){
                a[j] = a[i];
                pos[j] = pos [i];
                j--;
            }
        }
        a[i] = x;
        pos[i] = temp_pos;
        quick_sort(a, pos, left, i-1);
        quick_sort(a, pos, i+1, right);
    }
}
//由路由节点找到相连的消费节点
//输入:路由节点位置
//返回:消费节点位置
int findcosume(int node)
{
    for(int i=0; i<CONSUME_NUM; ++i)
    {
        if (consume[i].next_to == node)
            return i;
    }
    return 0;
}
//**********深度优先寻找路径***************

int findpath(int u, int delta)
{
    if (u == SINK)      //找到目的点
    {
        ++road_num;
        int i = 0;
        for (; i < (int)road.size() - 1; ++i)
        {
            outpath.push_back(road[i]);
        }
        --i;
        outpath.push_back(findcosume(road[i]));      //终点所连的消费节点
        outpath.push_back(delta);                    //路径流量
        outpath.push_back(choice.J_end[road[0]]);    //服务器硬件档次
        outpath.push_back(-1);

        return delta;
    }
    else {
        int ret = 0;
        for (int i = h[u]; i != -1 && delta; i = e[i].next)
        {
            if (e[i].flow)
            {
                road.push_back(e[i].to);
                int dd = findpath(e[i].to, min(e[i].flow, delta));
                road.pop_back();
                e[i].flow -= dd;
                delta -= dd;
                ret += dd;
            }
        }
        return ret;
    }
}
//**********保存文件****************
void push_back(int x)
{
    if (x < 0)
    {
        ans[file_Index++] = '\n';
    }
    else    //整型转字符串型
    {
        int i = file_Index;
        do
        {
            ans[file_Index++] = x % 10 + '0';
            x /= 10;
        } while (x);
        for (int j = file_Index - 1; j > i; --j, ++i)
        {
            swap(ans[i], ans[j]);
        }
    }
}
//保存文件
void save_file()
{
    choice.cost = zuidaliutest(choice);
    cnt1 = 0;
    for (int i = 0; i <= SINK; ++i)
    {
        h[i] = -1;
    }
    for (int i = numberColumns; i < numberColumns+server_num; i++)
    {
        int v = edge[2*i].to;   //找到对应的服务器的点
        int flow = server_type[choice.J_end[v]].cap;
        if (edge[2*i].flow < flow)
        {
            addEdge1(SRC, v, flow - edge[2*i].flow);
        }
    }
    for (int i = 0; i < numberColumns; i++)
    {
        if (edge[2*i].flow < temp_ee[i].flow)   //
        {
            addEdge1(temp_ee[i].from, temp_ee[i].to, temp_ee[i].flow - edge[2*i].flow);
        }
    }
    //进行初始化工作
    road_num = 0;
    outpath.clear();
    road.clear();
    outpath.push_back(road_num);
    outpath.push_back(-1);
    outpath.push_back(-1);
    findpath(SRC,flow_sum);
    outpath[0]=road_num;
    file_Index = 0;
    for (int i = 0; i < (int)outpath.size(); ++i)
    {
        if (outpath[i] >= 0 && i && outpath[i - 1] >= 0)
        {
            ans[file_Index++] = ' ';
        }
        push_back(outpath[i]);
    }
    ans[file_Index -1] = '\0';
}
void newGetGraph(Point p)
{
    server_num = 0; cnt=0;
    for (int i = 0; i <= SINK; ++i)
    {
        head[i] = -1;
    }
    //读取原图
    for (int i = 0; i < numberColumns; ++i)
    {
        addEdge(temp_ee[i].from, temp_ee[i].to, temp_ee[i].flow, temp_ee[i].cost);
        addEdge(temp_ee[i].to, temp_ee[i].from, 0, -temp_ee[i].cost);
    }
    //建立服务器的边
    for (int i = 0; i < ORIGIN_N; ++i)
    {
        if (p.J_end[i] >= 0)
        {
            int flow = server_type[p.J_end[i]].cap;
            addEdge(SRC, i, flow, Predict_cost[i]);    //总是设置成最大的服务器
            addEdge(i, SRC, 0, -Predict_cost[i]);
            server_num++;
        }
    }
}

int zuidaliu(Point &p)
{
    //clock_t s = clock();
    getGraph(p);
    int cost = SPFA(SRC,SINK,p);
    if(cost != INF)
    {
        p.J_end = J_pos;
        is_Same(p);
    }
    //cout<<(double)(clock()-s)/CLOCKS_PER_SEC*1000<<endl;
    return cost;
}

int zuidaliutest(Point &p)
{
    getGraphtest(p);
    int cost = SPFAtest(SRC,SINK,p);
    if(cost != INF)
    {
        p.J_end = J_pos;
    }
    return cost;
}
