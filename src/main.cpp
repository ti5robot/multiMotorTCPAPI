/*多电机，位置接口，状态接口，暂停接口，恢复接口 C++版*/
#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include<sstream>
#include<fstream>
#include<cmath>
#include<iostream>
#include <unistd.h>
#include <cstdlib>
#include <signal.h>
#include "Ti5BASIC.h"
#include "socket_service.h"

using namespace std;

void signalHandler(int signum)
{
    char aaa;
    cout << "Interrupt signal (" << signum << ") received.\n";
    cout<<"signalHandler: Numlen=" << Num_Len << endl;
    brake(Num_Len);  
    cout << "stop!!" << endl;

    // 清理并关闭
    // 终止程序  
    logout();
    exit(signum);

 
}

int main()
{
    signal(SIGINT, signalHandler);

    printf("qqqqq\n");
    
    canidList = new uint8_t[Num_Len];

    for (int i = 0; i < Num_Len; i++)
    {
        canidList[i] = i + 1;
        cout << "canlist=" << static_cast<int>(canidList[i]) << endl;
    }
    
    login();
    
    cout << "登录成功" << endl;
   
    std::string serverIP = "192.168.130.64";
    int port = 12345;
    socket_server(serverIP,port);//server端
    
    deallocate_variable();
    logout();

    return 0;
}

