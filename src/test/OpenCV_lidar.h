#pragma once
#include <iostream>
#include <cmath>
#include <opencv2\opencv.hpp>
#include <highgui.hpp> 
#include "rplidar.h"

using namespace std;
using namespace cv;

struct scanDot {
    float angle = 0;
    float dist = 0;
};

class LidarImage
{
public:
    LidarImage(void);
    ~LidarImage(void);

    vector<scanDot> scan_data; //保存每扫描一周的雷达数据

    void scanData(sl_lidar_response_measurement_node_hq_t* buffer, size_t count);
    void draw(Mat& RadarImage);
};
