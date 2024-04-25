#pragma once
#include <iostream>
#include <vector>

#include "HalconCpp.h"
#include "HDevThread.h"

//将存储点云数据的结构体在头文件中声明，以便其他cpp文件访问并创建实例
struct LidarInfo
{
	float theta = 0;
	float distance = 0;
	std::vector<float> point_x;
	std::vector<float> point_y;
	float distanceLeft = 0;
	float distanceRight = 0;
};
// LidarInfo Result;

void GetData(sl::ILidarDriver* drv, LidarInfo& Result, HalconCpp::HWindow& window);

void DealImage(HalconCpp::HObject  ho_Image, LidarInfo& Result);

void CalDistance(HalconCpp::HObject  ho_Image, LidarInfo& Result);

void Caldistance1(HalconCpp::HObject  ho_Image, LidarInfo& Result);