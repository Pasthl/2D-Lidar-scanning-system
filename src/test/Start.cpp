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
//    //�״�Ӳ����������
//    const char* opt_channel_param_first = NULL;
//    sl_u32         opt_channel_param_second = 0; // �洢�����ʣ���ʼΪ0����uint32_t����Ϊsl_u32,ʵ���ϻ��Ǳ�ʾ�޷���32λ����
//    sl_u32         baudrateArray[2] = { 115200, 256000 }; //�洢���ֲ����ʣ�A2��115200
//    sl_result     op_result;// �洢�������ؽ����ͬ����Ȼ��uint32_t,�������ʾ���Ǵ���ͬ������
//    IChannel* _channel; // ָ�� IChannel ���͵�ָ�룬�������Ӵ���
//
//    opt_channel_param_first = "com3"; // ����������ΪCOM3��Win��
//    opt_channel_param_second = 115200; // ���ò�����Ϊ115200
//
//    //����ʵ��
//    _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second)); // ��������ͨ��
//    ILidarDriver* drv = *createLidarDriver(); // �����״�����ʵ��
//    drv->connect(_channel); // ���Ӵ������״�
//    drv->setMotorSpeed(); // ���õ���ٶ�
//
//    signal(SIGINT, ctrlc);
//
//    // ��ʼɨ��
//    drv->startScan(0, 1);
//
//    // ��ӡɨ����
//    while (1) {
//        // sl_lidar_response_measurement_node_hq_t�����ڴ洢����ɨ��ڵ����ݵĽṹ��
//        // nodes����洢�˶��ɨ��ڵ�����ݣ�ÿ���ڵ����ݶ���һ��sl_lidar_response_measurement_node_hq_t�ṹ��ʵ���������С8192
//        sl_lidar_response_measurement_node_hq_t nodes[8192]; 
//        size_t   count = _countof(nodes); //��ȡ�ڵ�����
//
//        op_result = drv->grabScanDataHq(nodes, count); //��SDK�ú���ע�ͣ���ɨ�����ݴ���nodes����
//
//        if (SL_IS_OK(op_result)) {
//            drv->ascendScanData(nodes, count); // ��ɨ�����ݰ��Ƕ�0-360��������
//            for (int pos = 0; pos < (int)count; ++pos) {
//                //����ɨ�����ݲ�����sl_lidar_response_measurement_node_hq_t����
//                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
//                    (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ? "S " : "  ", // �Ƿ��Ǳ�־λ��ɨ����һ�֣�
//                    (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
//                    nodes[pos].dist_mm_q2 / 4.0f,
//                    nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT); // nodes[pos].quality��ֵ������λ����ȡ����ֵ�е��ض���Ϣλ
//            }
//        }
//
//        if (ctrl_c_pressed) {
//            break;
//        }
//    }
//
//    // �ر��״ﲢ������Դ
//    drv->stop();
//    drv->setMotorSpeed(0); // �����״�ֹͣ
//    drv->disconnect(); //�Ͽ����ӣ��������ͣ������
//
//    delete drv; // �ͷ��ڴ�
//    drv = NULL;
//
//    return 0;
//}
