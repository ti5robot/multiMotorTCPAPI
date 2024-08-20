#include "socket_service.h"

void socket_data_analysis(uint8_t *input)
{
    float output[6] = {0}; // 用于存储结果的浮点数数组

    int inputIndex = 0;
    for (int i = 0; i < 6; ++i)
    {
        // 如果只剩一个字节，跳出循环
        if (inputIndex >= 12)
            break;

        uint32_t floatBits = 0;

        // 将高8位放到floatBits的高8位
        floatBits |= (input[inputIndex++] << 24);

        // 如果已经处理到最后一个字节，则不取低8位
        if (inputIndex < 12)
        {
            // 将低8位放到floatBits的次高8位
            floatBits |= (input[inputIndex++] << 16);
        }

        // 将uint32_t转换回float
        output[i] = *reinterpret_cast<float *>(&floatBits);
    }

    // 输出结果数组
    for (int i = 0; i < 6; ++i)
    {
        std::cout << std::fixed << std::setprecision(5) << output[i] << " ";
    }
    std::cout << std::endl;
}

void socket_parse_data(const std::vector<uint8_t>& values, uint16_t* reconstructedValues,int num_len)
{
    // float angle[num_len];
    for (int i = 0; i < values.size() && i < num_len*2; i += 2)
    {
        uint8_t highByte = values[i];    // 高八位
        uint8_t lowByte = values[i + 1]; // 低八位

        // 反向将高八位和低八位重新组合为无符号整数
        reconstructedValues[i / 2] = (uint16_t)(highByte << 8) | lowByte;

        // 输出结果
        printf("High Byte: %02X\n", highByte);
        printf("Low Byte: %02X\n", lowByte);
        printf("Reconstructed Value: %d\n", (int16_t)reconstructedValues[i / 2]);
        sprintf(LogInfo, "Byte %d: High Byte: %02X, Low Byte: %02X", i, highByte, lowByte);
        
        // sprintf(LogInfo, "High Byte: %02X", highByte);
        // writeDebugInfoToFile(__func__, LogInfo);
        // sprintf(LogInfo, "High Byte: %02X", lowByte);
        // writeDebugInfoToFile(__func__, LogInfo);

        printf("\n");
    }
    // 输出整个重新组合值数组
    printf("Reconstructed Values Array: \n");
    for (int i = 0; i < num_len; ++i)
    {
        printf("%d ", (int16_t)reconstructedValues[i]);

        printf("Number %d: %.2f\n", i + 1, (int16_t)reconstructedValues[i] / 10.0);
        // angle[i] = (int16_t)reconstructedValues[i] / 10.0;
        // printf("angle %d:%.2f\n", i + 1, angle[i]);

        sprintf(LogInfo, "Number %d: %.2f",i+1, (int16_t)reconstructedValues[i] / 10.0);
        writeDebugInfoToFile(__func__, LogInfo);

        TH.j[i]=(int16_t)reconstructedValues[i] / 10.0;//接收到角度
    }
    printf("\n");
    // move_to_pos();//坐标
    
    move_to_joint();//角度
}

int socket_server(std::string& serverIP,int port)
{
    // allocate_variable(Num_Len); // 分配变量
    

    // 创建套接字
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1)
    {
        std::cerr << "Error: Failed to create socket! " << std::endl;
        writeDebugInfoToFile(__func__, "Error: Failed to create socket! ");
        return 1;
    }

    // 设置地址重用选项，避免端口占用问题
    int reuse = 1;
    if (setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) == -1)
    {
        std::cerr << "Error: Failed to set socket options! " << std::endl;
        writeDebugInfoToFile(__func__, "Error: Failed to set socket options! ");
        close(serverSocket);
        return 1;
    }

    // 设置服务器地址
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port); // 服务端端口
    // serverAddr.sin_addr.s_addr = INADDR_ANY;

    // 将字符串形式的 IP 地址转换成 in_addr_t 类型
    if (inet_pton(AF_INET, serverIP.c_str(), &serverAddr.sin_addr) <= 0)
    {
        std::cerr << "Error: Invalid IP address! " << std::endl;
        writeDebugInfoToFile(__func__, "Error: Invalid IP address! ");
        close(serverSocket);
        return 1;
    }

    // 绑定地址
    if (bind(serverSocket, reinterpret_cast<struct sockaddr *>(&serverAddr), sizeof(serverAddr)) == -1)
    {
        std::cerr << "Error: Failed to bind socket! " << std::endl;
        writeDebugInfoToFile(__func__, "Error: Failed to bind socket! ");
        close(serverSocket);
        return 1;
    }

    // 监听连接
    if (listen(serverSocket, 10) == -1)
    {
        std::cerr << "Error: Failed to listen! " << std::endl;
        writeDebugInfoToFile(__func__, "Error: Failed to listen! ");
        close(serverSocket);
        return 1;
    }

    std::cout << "Server is listening for connections..." << std::endl;
    writeDebugInfoToFile(__func__, "Server is listening for connections...");

    while (true)
    {
        // 接受连接
        struct sockaddr_in clientAddr;
        socklen_t clientAddrSize = sizeof(clientAddr);
        int clientSocket = accept(serverSocket, reinterpret_cast<struct sockaddr *>(&clientAddr), &clientAddrSize);
        if (clientSocket == -1)
        {
            std::cerr << "Error: Failed to accept connection! " << std::endl;
            writeDebugInfoToFile(__func__, "Error: Failed to accept connection! ");
            continue; // 继续等待下一个连接
        }

        std::cout << "Client connected." << std::endl;
        writeDebugInfoToFile(__func__, "Client connected.");

        // 接收数据
        std::vector<uint8_t> input(1024); // 临时使用1024字节缓冲区

        ssize_t bytesReceived = recv(clientSocket, input.data(), input.size(), 0);
        if (bytesReceived == -1)
        {
            std::cerr << "Error: Failed to receive data! " << std::endl;
            writeDebugInfoToFile(__func__, "Error: Failed to receive data! ");
            close(clientSocket);
            continue; // 继续等待下一个连接
        }

        // 检查是否接收到 "start"
        const uint8_t startSequence[] = {115, 116, 97, 114, 116}; // "start" 的 ASCII 编码
        // sprintf(LogInfo, "High Byte: %02X", startSequence);
        // writeDebugInfoToFile(__func__, LogInfo);
        if (bytesReceived >= 5 && std::memcmp(input.data(), startSequence, 5) == 0) {
            std::cout << "hello" << std::endl;
        }

        // 将数据大小调整为实际接收到的字节数
        input.resize(bytesReceived);
        // int num_len = static_cast<int>(input[1]);
        Num_Len = static_cast<int>(input[1]);
        std::cout<<"Num_Len="<<Num_Len<<std::endl;

        // 检查第一个元素是否是0xaa
        if (input[0] == 0xaa)
        {
            // 判断最后两个字节是否是0xaa和0x55
            // cout<<"14: "<<input[13]<<"15: "<<input[14]<<endl;//2024-8-6 debug
            if (input[input.size() - 2] == 0xaa && input[input.size() - 1] == 0x55)
            {
                // 创建新的数组来存储从第3个元素开始到倒数第三个元素的数据
                std::vector<uint8_t> newData(input.begin() + 2, input.end() - 2);
                
                
                // 输出格式化后的数据
                std::cout << "Received data (after removing 0xaa and 0x55): ";
                writeDebugInfoToFile(__func__, "Received data (after removing 0xaa and 0x55)! ");
                for (auto byte : newData) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
                }
                std::cout << std::endl;
                
                

                // 调用data_analysis函数
                // data_analysis(newData);
                uint16_t qqq[6];//存储解析后的数据
                socket_parse_data(newData, qqq,Num_Len);

                // 发送 ACK 给客户端
                const char *ackMessage = "ACK";
                if (send(clientSocket, ackMessage, strlen(ackMessage), 0) == -1)
                {
                    std::cerr << "Error: Failed to send ACK!" << std::endl;
                    writeDebugInfoToFile(__func__, "Error: Failed to send ACK! ");
                }
                else
                {
                    std::cout << "ACK sent to client." << std::endl;
                    writeDebugInfoToFile(__func__, "ACK sent to client.");
                }
                
                // 发送 ACK 给客户端
            }
            else
            {
                std::cerr << "Error: Last two bytes are not 0xaa and 0x55." << std::endl;
                writeDebugInfoToFile(__func__, "Error: Last two bytes are not 0xaa and 0x55! ");
            }
        }
        else
        {
            std::cerr << "Error: First element is not 0xaa." << std::endl;
            writeDebugInfoToFile(__func__, "Error: First element is not 0xaa! ");
        }

        // 关闭客户端套接字
        close(clientSocket);
    }

    // 关闭服务器套接字（这里实际上不会执行到）
    close(serverSocket);

    return 0;
}