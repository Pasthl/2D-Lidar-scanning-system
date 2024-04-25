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
//    utils::logging::setLogLevel(utils::logging::LOG_LEVEL_ERROR); //只输出错误级别的日志
//
//    ILidarDriver* drv = *createLidarDriver(); // 创建雷达驱动实例
//
//    LidarStart(drv);
//
//    // 创建一个 Mat 对象， 用于展示雷达数据
//    Mat RadarImage = Mat::zeros(600, 600, CV_8UC3);
//
//    // 创建窗口并显示图像
//    namedWindow("Radar", WINDOW_NORMAL);
//
//    // 初始化计数器
//    int saveCounter = 0;
//    int numImagesToSave = 20; // 保存的图片数量
//
//    while (true) {
//        // 声明存储扫描数据的数组
//        sl_lidar_response_measurement_node_hq_t nodes[360 * 2];
//        size_t count = _countof(nodes);
//        sl_result     op_result;// 存储函数返回结果；同理，仍然是uint32_t,起别名表示它们代表不同的语义
//
//        // 获取雷达扫描数据
//        op_result = drv->grabScanDataHq(nodes, count);
//
//        if (IS_OK(op_result)) {
//            // 创建 LidarImage 对象
//            LidarImage lidarImage;
//            // 将扫描数据转换为实际距离和角度，并保存到 LidarImage 对象的成员变量中
//            lidarImage.scanData(nodes, count); 
//
//            // 在图像上绘制扫描数据
//            lidarImage.draw(RadarImage);
//
//            // 显示图像
//            imshow("Radar", RadarImage);
//
//            // 如果需要保存图片
//            if (saveCounter < numImagesToSave) {
//                std::string filename = "E:\\Code\\C++\\Lidarsolu\\bin\\Win32\\pic\\" + std::to_string(saveCounter) + ".jpg";
//                if (cv::imwrite(filename, RadarImage)) {
//                    std::cout << "Saved image: " << filename << std::endl;
//                }
//                else {
//                    std::cerr << "Failed to save image: " << filename << std::endl;
//                }
//                ++saveCounter; // 增加计数器
//            }
//
//            // 等待按键，如果按下 ESC 键则退出循环
//            char key = waitKey(30);
//            if (key == 27) {
//                break;
//            }
//        }
//    }
//
//    // 释放图像资源和关闭窗口
//    RadarImage.release();
//    destroyWindow("Radar");
//
//    LidarStop(drv);
//
//    return 0;
//}
