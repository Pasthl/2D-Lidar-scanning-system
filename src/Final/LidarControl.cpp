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
	//�״�Ӳ����������
    const char* opt_channel_param_first = NULL;
    sl_u32         opt_channel_param_second = 0; // �洢�����ʣ���ʼΪ0����uint32_t����Ϊsl_u32,ʵ���ϻ��Ǳ�ʾ�޷���32λ����
    sl_u32         baudrateArray[2] = { 115200, 256000 }; //�洢���ֲ����ʣ�A2��115200
    
    IChannel* _channel; // ָ�� IChannel ���͵�ָ�룬�������Ӵ���

    opt_channel_param_first = "com3"; // ����������ΪCOM3��Win��
    opt_channel_param_second = 115200; // ���ò�����Ϊ115200

    //����ʵ��
    _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second)); // ��������ͨ��
    drv->connect(_channel); // ���Ӵ������״�
    drv->setMotorSpeed(); // ���õ���ٶ�

    drv->startScan(0, 1);
    delay(8000);

}

void LidarStop(ILidarDriver* drv)
{
    // �ر��״ﲢ������Դ
    drv->stop();
    drv->setMotorSpeed(0); // �����״�ֹͣ
    drv->disconnect(); //�Ͽ����ӣ��������ͣ������

    delete drv; // �ͷ��ڴ�
    drv = NULL;
}