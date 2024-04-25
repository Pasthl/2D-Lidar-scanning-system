#include "opencv_lidar.h"

#define PI 3.141592653

using namespace std;
using namespace cv;

LidarImage::LidarImage(void) {} //构造函数

LidarImage::~LidarImage(void) {} // 析构函数

void LidarImage::scanData(sl_lidar_response_measurement_node_hq_t* buffer, size_t count) {
    scan_data.clear(); // 清空之前的数据
    for (int pos = 0; pos < (int)count; ++pos) {
        scanDot dot;
        if (!buffer[pos].dist_mm_q2) continue; // 如果距离数据为0，则跳过此点

        float angle = buffer[pos].angle_z_q14 * 90.f / 16384.f; // 转换角度为度
        // 过滤后方的数据，只保留前方数据
        if ((angle > 90) && (angle < 270)) continue;

        dot.angle = angle; // 已经转换为度的角度
        dot.dist = buffer[pos].dist_mm_q2 / 4.0f; // 将距离数据转换为毫米
        scan_data.push_back(dot); // 添加到扫描数据列表
    }
}


void LidarImage::draw(Mat& RadarImage)
{
    // 将图像清零，即设置为全黑
    RadarImage.setTo(Scalar(0, 0, 0));

    // 在图像中心加上一个圆心  
    circle(RadarImage, Point(300,300), 3, Scalar(0, 255, 255), -1, 8, 0);


    double theta, rho;

    for (int i = 0; i < scan_data.size(); i++)
    {
        scanDot dot;
        dot = scan_data[i];
        theta = dot.angle * PI / 180;
        rho = dot.dist;

        int x = (int)(rho * sin(theta) / 20) + 300;
        int y = (int)(-rho * cos(theta) / 20) + 300;

        circle(RadarImage, Point(x, y), 1, Scalar(0, 255, 0), 1, 8, 0);
    }

}                                                                                        