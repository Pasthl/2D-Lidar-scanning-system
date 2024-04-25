//#include "rplidar.h"   
//#include "sl_lidar.h" 
//#include "sl_lidar_driver.h"
//
//#include <stdio.h>
//#include <stdlib.h>
//#include <signal.h>
//#include <string.h>
//#include <fstream>
//#include <iostream>
//#include <math.h>
//#include <winsock2.h>
//#include <opencv2\opencv.hpp>
//#include <highgui.hpp> 
//#include <vector>
//#include <opencv2/core/utils/logger.hpp>
//
//#include "OpenCV_lidar.h"
//#include "LidarControl.h"
//
//#define PI acos(-1)
//#define rotate_theta PI
//#define scale_num  20
//using namespace cv;
//using namespace sl;
//using namespace std;
//
//#define delay(x)   ::Sleep(x)
//
//int main()
//{
//    utils::logging::setLogLevel(utils::logging::LOG_LEVEL_ERROR); //ֻ������󼶱����־
//
//    ILidarDriver* drv = *createLidarDriver(); // �����״�����ʵ��
//
//    LidarStart(drv);
//
//    // ����һ�� Mat ���� ����չʾ�״�����
//    Mat RadarImage = Mat::zeros(600, 600, CV_8UC3);
//
//    // �������ڲ���ʾͼ��
//    namedWindow("Radar", WINDOW_NORMAL);
//
//    // ��ʼ��������
//    int saveCounter = 0;
//    int numImagesToSave = 20; // �����ͼƬ����
//
//    while (true) {
//        // �����洢ɨ�����ݵ�����
//        sl_lidar_response_measurement_node_hq_t nodes[360 * 2];
//        size_t count = _countof(nodes);
//        sl_result     op_result;// �洢�������ؽ����ͬ����Ȼ��uint32_t,�������ʾ���Ǵ���ͬ������
//
//        // ��ȡ�״�ɨ������
//        op_result = drv->grabScanDataHq(nodes, count);
//
//        if (IS_OK(op_result)) {
//            // ���� LidarImage ����
//            LidarImage lidarImage;
//            // ��ɨ������ת��Ϊʵ�ʾ���ͽǶȣ������浽 LidarImage ����ĳ�Ա������
//            lidarImage.scanData(nodes, count); 
//
//            // ��ͼ���ϻ���ɨ������
//            lidarImage.draw(RadarImage);
//
//            // ��ʾͼ��
//            imshow("Radar", RadarImage);
//
//            // �����Ҫ����ͼƬ
//            if (saveCounter < numImagesToSave) {
//                std::string filename = "E:\\Code\\C++\\Lidarsolu\\bin\\Win32\\pic\\" + std::to_string(saveCounter) + ".jpg";
//                if (cv::imwrite(filename, RadarImage)) {
//                    std::cout << "Saved image: " << filename << std::endl;
//                }
//                else {
//                    std::cerr << "Failed to save image: " << filename << std::endl;
//                }
//                ++saveCounter; // ���Ӽ�����
//            }
//
//            // �ȴ�������������� ESC �����˳�ѭ��
//            char key = waitKey(30);
//            if (key == 27) {
//                break;
//            }
//        }
//    }
//
//    // �ͷ�ͼ����Դ�͹رմ���
//    RadarImage.release();
//    destroyWindow("Radar");
//
//    LidarStop(drv);
//
//    return 0;
//}
