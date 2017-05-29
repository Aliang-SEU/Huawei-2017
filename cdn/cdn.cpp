#include "deploy.h"
#include "lib/lib_io.h"
#include "lib/lib_time.h"
#include "stdio.h"
#include <fstream>

#define AUTOTEST
int main(int argc, char *argv[])
{

    char *topo[MAX_EDGE_NUM];
    int line_num;
#ifdef AUTOTEST
    //char *topo_file = argv[1];
    std::ofstream out("data.txt",std::ios::out);

    for(int i =0; i<2; ++i)   //每个案例测试1
    {
        out<<i<<":"<<std::endl;
        for(int j=0;j<10;++j)
        {
            for(int k =0;k<1;++k)
            {
                 print_time("Begin");
                char filename[500]="";
                sprintf(filename,"/home/hzl/Desktop/case_example/%d/case%d.txt",i,j);
                char *topo_file = filename;

                line_num = read_file(topo, MAX_EDGE_NUM, topo_file);

                printf("line num is :%d \n", line_num);
                if (line_num == 0)
                {
                    printf("Please input valid topo file.\n");
                    return -1;
                }

                // char *result_file = argv[2];

                char *result_file = "/home/hzl/Desktop/result.txt";

                int cost = deploy_server(topo, line_num, result_file);
                out<<cost<<" ";
                release_buff(topo, line_num);
                 print_time("End");
            }
            out<<std::endl;
        }
    }
    out.close();
#endif
#ifndef AUTOTEST
    print_time("Begin");
    //char *topo[MAX_EDGE_NUM];
   // int line_num;
    char *topo_file = "/home/hzl/Desktop/case_example2/1/case0.txt";

    line_num = read_file(topo, MAX_EDGE_NUM, topo_file);

    printf("line num is :%d \n", line_num);
    if (line_num == 0)
    {
        printf("Please input valid topo file.\n");
        return -1;
    }

    // char *result_file = argv[2];

    char *result_file = "/home/hzl/Desktop/result.txt";

    deploy_server(topo, line_num, result_file);

    release_buff(topo, line_num);
#endif
    print_time("End");

    return 0;
}

