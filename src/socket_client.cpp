#include "socket_client.h"

uint8_t new_arr[15]={0};

// 将字节数组转换回浮点数组
void convert_bytes_to_float(const uint8_t* bytes, float* arr, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        uint16_t value = (bytes[2 * i] << 8) | bytes[2 * i + 1]; // 高8位和低8位组合
        arr[i] = static_cast<int16_t>(value) / 10.0; // 转换回浮点数，并处理负数
    }
}

int init_convert(float originalArr[6]) {
    // 原始浮点数组
    // float originalArr[6] = {-173.5, -206.00, 423.90, -2.20, 2.50, 2.70};
    // float originalArr[6] = {0, 0, 590.20, 0, 0, 0};
    uint8_t bytes[12] = {0};

    // 转换浮点数组为字节数组
    for (size_t i = 0; i < 6; ++i) {
        uint16_t value = static_cast<int16_t>(originalArr[i] * 10); // 将浮点数乘以10并转换为有符号整数
        bytes[2 * i] = (value >> 8) & 0xFF;  // 高8位
        bytes[2 * i + 1] = value & 0xFF;     // 低8位
    }

    // 打印转换后的字节数组
    std::cout << "Converted bytes: ";
    for (size_t i = 0; i < 12; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::endl;
    // uint8_t new_arr[15]={0};
    new_arr[0] = {0xaa};
    new_arr[13] = {0xaa};
    new_arr[14] = {0x55};
    std::cout << "new_arr bytes: ";
    for(int i=1;i<13;i++)
    {
        new_arr[i]=bytes[i-1];
    }
    for(int j=0;j<15;j++)
    {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(new_arr[j]) << " ";
    }
    std::cout << std::endl;
    // 用于存储反向转换后的浮点数组
    float recoveredArr[6] = {0};

    // 反向转换字节数组为浮点数组
    convert_bytes_to_float(bytes, recoveredArr, 6);

    // 打印恢复的浮点数组
    std::cout << "Recovered floats: ";
    for (size_t i = 0; i < 6; ++i) {
        std::cout << std::fixed << std::setprecision(2) << recoveredArr[i] << " ";
    }
    std::cout << std::endl;

    return 0;
}


int socket_client(std::string& serverIP,int port,uint8_t input[15],size_t input_size) {
    // 创建套接字
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Error: Failed to create socket." << std::endl;
        return 1;
    }

    // 设置服务器地址
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    // serverAddr.sin_port = htons(12345); // 服务端端口
    serverAddr.sin_port = htons(port); // 服务端端口
    //inet_pton(AF_INET, "127.0.0.1", &serverAddr.sin_addr); // 服务端IP地址
    
    // 将字符串形式的 IP 地址转换成 in_addr_t 类型
    // if (inet_pton(AF_INET, "192.168.231.128", &serverAddr.sin_addr) <= 0) {
    if (inet_pton(AF_INET, serverIP.c_str(), &serverAddr.sin_addr) <= 0) {
        std::cerr << "Error: Invalid IP address." << std::endl;
        close(clientSocket);
        return 1;
    }

    // 连接到服务器
    if (connect(clientSocket, reinterpret_cast<struct sockaddr*>(&serverAddr), sizeof(serverAddr)) == -1) {
        std::cerr << "Error: Connection failed." << std::endl;
        close(clientSocket);
        return 1;
    }
//while(1){
    // 发送数据
    //uint8_t input[15] = {0xaa,0xf7, 0x43, 0xa4, 0x43, 0x5f, 0xc0, 0x01, 0xc0, 0x48, 0x3d,0x4b,0x34,0xaa,0x55};
    // uint8_t input[15] = {0xaa,0xf9, 0x39, 0xf7, 0xf4, 0x10, 0x8f, 0xff, 0xea, 0x00, 0x19,0x00,0x1b,0xaa,0x55};//-173.50 -206.00 423.90 -2.20 2.50 2.70
    uint8_t input[14] = {0xaa,0x05, 0xf9, 0x39, 0xf7, 0xf4, 0x10, 0x8f, 0xff, 0xea, 0x00, 0x19,0xaa,0x55};
    // if (send(clientSocket, input, sizeof(input), 0) == -1) {
    if (send(clientSocket, input, input_size, 0) == -1) {
        std::cerr << "Error: Failed to send data." << std::endl;
        close(clientSocket);
        return 1;
    }
    
    std::cout << "Data sent successfully." << std::endl;

    // 接收ACK
    char ack[4];
    ssize_t bytesReceived = recv(clientSocket, ack, sizeof(ack), 0);
    if (bytesReceived == -1) {
        std::cerr << "Error: Failed to receive ACK." << std::endl;
    } else if (bytesReceived == 0) {
        std::cerr << "Connection closed by the server." << std::endl;
    } else {
        // 确认ACK内容
        if (std::memcmp(ack, "ACK", 3) == 0) {
            std::cout << "ACK received: " << std::string(ack, 3) << std::endl;
        } else {
            std::cerr << "Unexpected response received." << std::endl;
        }
    }
    
//}
    // 关闭套接字
    close(clientSocket);

    return 0;
}
