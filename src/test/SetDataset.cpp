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
//#define delay(x)   ::Sleep(x)
//
//int main() {
//    //雷达硬件参数配置
//    const char* opt_channel_param_first = NULL;
//    sl_u32         opt_channel_param_second = 0;
//    sl_u32         baudrateArray[2] = { 115200, 256000 };
//    sl_result     op_result;
//    IChannel* _channel;
//
//    opt_channel_param_first = "com3";
//    opt_channel_param_second = 115200;
//
//    _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
//    ILidarDriver* drv = *createLidarDriver();
//    drv->connect(_channel);
//    drv->setMotorSpeed();
//
//
//    do {
//        // 开始扫描
//        drv->startScan(0, 1);
//
//        delay(8000);
//
//        // 打开CSV文件进行写入
//        ofstream file("scan_data.csv");
//
//        // 检查文件是否成功打开
//        if (!file.is_open()) {
//            cout << "无法打开文件！" << endl;
//            return 1;
//        }
//
//        // 循环写入结构体数据到文件中
//            // 假设nodes数组为存储了多个扫描节点的数据的结构体数组
//        sl_lidar_response_measurement_node_hq_t nodes[8192];
//        size_t count = _countof(nodes); // 获取节点数量
//
//        // 假设_op_result为SDK函数的返回值，用于表示操作结果
//        op_result = drv->grabScanDataHq(nodes, count);
//
//        if (SL_IS_OK(op_result)) {
//            drv->ascendScanData(nodes, count); // 将扫描数据按角度0-360升序排列
//            for (int pos = 0; pos < (int)count; ++pos) {
//                // 写入结构体数据到文件中
//                //file << ((nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ? "S " : "  ") << ","
//                //    << (nodes[pos].angle_z_q14 * 90.f) / 16384.f << ","
//                //    << nodes[pos].dist_mm_q2 / 4.0f << ","
//                //    << (nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << "\n";
//                file << (nodes[pos].angle_z_q14 * 90.f) / 16384.f << ","
//                    << nodes[pos].dist_mm_q2 / 4.0f << "," << "\n";
//            }
//
//            // 关闭文件
//            file.close();
//        }
//    } while (0);
//
//    // 关闭雷达并清理资源
//    drv->stop();
//    drv->setMotorSpeed(0); // 设置雷达停止
//    drv->disconnect();
//
//    delete drv;
//    drv = NULL;
//
//    cout << "扫描数据已成功保存到文件 'scan_data.csv'." << endl;
//
//    return 0;
//}
