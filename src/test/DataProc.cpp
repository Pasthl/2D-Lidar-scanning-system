#include "opencv_lidar.h"

#define PI 3.141592653

using namespace std;
using namespace cv;

LidarImage::LidarImage(void) {} //���캯��

LidarImage::~LidarImage(void) {} // ��������

void LidarImage::scanData(sl_lidar_response_measurement_node_hq_t* buffer, size_t count) {
    scan_data.clear(); // ���֮ǰ������
    for (int pos = 0; pos < (int)count; ++pos) {
        scanDot dot;
        if (!buffer[pos].dist_mm_q2) continue; // �����������Ϊ0���������˵�

        float angle = buffer[pos].angle_z_q14 * 90.f / 16384.f; // ת���Ƕ�Ϊ��
        // ���˺󷽵����ݣ�ֻ����ǰ������
        if ((angle > 90) && (angle < 270)) continue;

        dot.angle = angle; // �Ѿ�ת��Ϊ�ȵĽǶ�
        dot.dist = buffer[pos].dist_mm_q2 / 4.0f; // ����������ת��Ϊ����
        scan_data.push_back(dot); // ��ӵ�ɨ�������б�
    }
}


void LidarImage::draw(Mat& RadarImage)
{
    // ��ͼ�����㣬������Ϊȫ��
    RadarImage.setTo(Scalar(0, 0, 0));

    // ��ͼ�����ļ���һ��Բ��  
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