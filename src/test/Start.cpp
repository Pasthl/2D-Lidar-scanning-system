//#include "rplidar.h"   
//#include <stdio.h>
//#include <stdlib.h>
//#include <signal.h>
//#include <string.h>
//#include "sl_lidar.h" 
//#include "sl_lidar_driver.h"
//#include<fstream>
//#include<iostream>
//#include<math.h>
//#include<winsock2.h>
//#include<Eigen/Dense>
//#include<vector>
//#define PI acos(-1)
//#define rotate_theta PI
//#define scale_num  20
//using namespace Eigen;
//using namespace sl;
//using namespace std;
//
//bool ctrl_c_pressed;
//void ctrlc(int)
//{
//    ctrl_c_pressed = true;
//}
//
//#define delay(x)   ::Sleep(x)
//
//int main()
//{
//    //雷达硬件参数配置
//    const char* opt_channel_param_first = NULL;
//    sl_u32         opt_channel_param_second = 0; // 存储波特率，初始为0；将uint32_t定义为sl_u32,实际上还是表示无符号32位整数
//    sl_u32         baudrateArray[2] = { 115200, 256000 }; //存储两种波特率，A2是115200
//    sl_result     op_result;// 存储函数返回结果；同理，仍然是uint32_t,起别名表示它们代表不同的语义
//    IChannel* _channel; // 指向 IChannel 类型的指针，用于连接串口
//
//    opt_channel_param_first = "com3"; // 将串口设置为COM3（Win）
//    opt_channel_param_second = 115200; // 设置波特率为115200
//
//    //创建实例
//    _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second)); // 创建串口通道
//    ILidarDriver* drv = *createLidarDriver(); // 创建雷达驱动实例
//    drv->connect(_channel); // 连接串口与雷达
//    drv->setMotorSpeed(); // 设置电机速度
//
//    signal(SIGINT, ctrlc);
//
//    // 开始扫描
//    drv->startScan(0, 1);
//
//    // 打印扫描结果
//    while (1) {
//        // sl_lidar_response_measurement_node_hq_t是用于存储单个扫描节点数据的结构体
//        // nodes数组存储了多个扫描节点的数据，每个节点数据都是一个sl_lidar_response_measurement_node_hq_t结构体实例，数组大小8192
//        sl_lidar_response_measurement_node_hq_t nodes[8192]; 
//        size_t   count = _countof(nodes); //获取节点数量
//
//        op_result = drv->grabScanDataHq(nodes, count); //见SDK该函数注释，将扫描数据存入nodes数组
//
//        if (SL_IS_OK(op_result)) {
//            drv->ascendScanData(nodes, count); // 将扫描数据按角度0-360升序排列
//            for (int pos = 0; pos < (int)count; ++pos) {
//                //单次扫描数据参数见sl_lidar_response_measurement_node_hq_t定义
//                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
//                    (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ? "S " : "  ", // 是否是标志位（扫描完一轮）
//                    (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
//                    nodes[pos].dist_mm_q2 / 4.0f,
//                    nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT); // nodes[pos].quality的值右移两位，提取质量值中的特定信息位
//            }
//        }
//
//        if (ctrl_c_pressed) {
//            break;
//        }
//    }
//
//    // 关闭雷达并清理资源
//    drv->stop();
//    drv->setMotorSpeed(0); // 设置雷达停止
//    drv->disconnect(); //断开连接，不加这个停不下来
//
//    delete drv; // 释放内存
//    drv = NULL;
//
//    return 0;
//}
