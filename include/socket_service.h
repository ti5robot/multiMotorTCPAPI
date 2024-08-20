#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <iomanip>
#include <arpa/inet.h>
#include <vector>
#include <thread>

#include "Ti5LOGIC.h"
#include "Ti5BASIC.h"

// 宏定义服务器IP地址
// #define SERVER_IP "192.168.30.5"
// #define SERVER_PORT 12345

void socket_data_analysis(uint8_t *input);

/*
 数据处理函数
 参数：
   values:接收到的数据去掉帧头帧尾后的数据（0xaa，电机个数，0xaa，0x55，）
   reconstructedValues:转换后的角度值
   num_len:电机数（Num_Len固定传这个变量名）
*/
void socket_parse_data(const std::vector<uint8_t>& values, uint16_t* reconstructedValues,int num_len);

/*
 socket连接函数
 参数：
    serverIP:服务器IP地址
    port:服务器端口号
 返回值：
    成功返回0，失败返回1
*/
int socket_server(std::string& serverIP,int port);
