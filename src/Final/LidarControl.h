#pragma once
#include"rplidar.h"	
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

using namespace sl;

void LidarStart(ILidarDriver* drv);

void LidarStop(ILidarDriver* drv);