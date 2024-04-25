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
//    //�״�Ӳ����������
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
//        // ��ʼɨ��
//        drv->startScan(0, 1);
//
//        delay(8000);
//
//        // ��CSV�ļ�����д��
//        ofstream file("scan_data.csv");
//
//        // ����ļ��Ƿ�ɹ���
//        if (!file.is_open()) {
//            cout << "�޷����ļ���" << endl;
//            return 1;
//        }
//
//        // ѭ��д��ṹ�����ݵ��ļ���
//            // ����nodes����Ϊ�洢�˶��ɨ��ڵ�����ݵĽṹ������
//        sl_lidar_response_measurement_node_hq_t nodes[8192];
//        size_t count = _countof(nodes); // ��ȡ�ڵ�����
//
//        // ����_op_resultΪSDK�����ķ���ֵ�����ڱ�ʾ�������
//        op_result = drv->grabScanDataHq(nodes, count);
//
//        if (SL_IS_OK(op_result)) {
//            drv->ascendScanData(nodes, count); // ��ɨ�����ݰ��Ƕ�0-360��������
//            for (int pos = 0; pos < (int)count; ++pos) {
//                // д��ṹ�����ݵ��ļ���
//                //file << ((nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ? "S " : "  ") << ","
//                //    << (nodes[pos].angle_z_q14 * 90.f) / 16384.f << ","
//                //    << nodes[pos].dist_mm_q2 / 4.0f << ","
//                //    << (nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << "\n";
//                file << (nodes[pos].angle_z_q14 * 90.f) / 16384.f << ","
//                    << nodes[pos].dist_mm_q2 / 4.0f << "," << "\n";
//            }
//
//            // �ر��ļ�
//            file.close();
//        }
//    } while (0);
//
//    // �ر��״ﲢ������Դ
//    drv->stop();
//    drv->setMotorSpeed(0); // �����״�ֹͣ
//    drv->disconnect();
//
//    delete drv;
//    drv = NULL;
//
//    cout << "ɨ�������ѳɹ����浽�ļ� 'scan_data.csv'." << endl;
//
//    return 0;
//}
