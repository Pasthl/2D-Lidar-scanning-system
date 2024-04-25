#include"rplidar.h"	
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#include<fstream>
#include<iostream>
#include<math.h>
#include<windows.h> // for sleep

using namespace sl;
using namespace std;


#define delay(x)   ::Sleep(x)

void LidarStart(ILidarDriver* drv)
{
	//雷达硬件参数配置
    const char* opt_channel_param_first = NULL;
    sl_u32         opt_channel_param_second = 0; // 存储波特率，初始为0；将uint32_t定义为sl_u32,实际上还是表示无符号32位整数
    sl_u32         baudrateArray[2] = { 115200, 256000 }; //存储两种波特率，A2是115200
    
    IChannel* _channel; // 指向 IChannel 类型的指针，用于连接串口

    opt_channel_param_first = "com3"; // 将串口设置为COM3（Win）
    opt_channel_param_second = 115200; // 设置波特率为115200

    //创建实例
    _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second)); // 创建串口通道
    drv->connect(_channel); // 连接串口与雷达
    drv->setMotorSpeed(); // 设置电机速度

    drv->startScan(0, 1);
    delay(8000);

}

void LidarStop(ILidarDriver* drv)
{
    // 关闭雷达并清理资源
    drv->stop();
    drv->setMotorSpeed(0); // 设置雷达停止
    drv->disconnect(); //断开连接，不加这个停不下来

    delete drv; // 释放内存
    drv = NULL;
}