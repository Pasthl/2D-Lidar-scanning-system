#include"rplidar.h"	

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include<fstream>
#include<iostream>
#include<math.h>
#include<winsock2.h>
#include<Eigen/Dense>
#include<vector>
#include<thread>

#include "HalconCpp.h"
#include "HDevThread.h"

#include "ImageProcess.h"

#define PI acos(-1) //PI= 3.14159265352979323846
#define rotate_theta PI
const int scale_num = 20; //图像缩放系数
int timesss = 0;


void GetData(sl::ILidarDriver* drv, LidarInfo& Result, HalconCpp::HWindow& window) //参数为雷达实例的指针和存储数据实例
{

    HalconCpp::HObject  ho_Lidar_image;//halon初始化图像
    HalconCpp::HObject ho_ImageR, ho_ImageG, ho_ImageB;
    int window_height = 10000 / scale_num;//图像的长宽定义
    int window_width = 20000 / scale_num;
    std::vector<float> ori_x; //创建左手坐标系的x坐标数组
    std::vector<float> ori_y;// 创建左手坐标系的y坐标数组

    Eigen::Matrix<float, 4, 4> rotate_matrix;//创建坐标系旋转矩阵
    Eigen::MatrixXf point(1, 4); //创建单个点
    point << 0, 0, 0, 1;
    rotate_matrix << 1, 0, 0, (window_height), //创建旋转矩阵
        0, cos(rotate_theta), -std::sin(rotate_theta), (window_height),
        0, sin(rotate_theta), cos(rotate_theta), 0,
        0, 0, 0, 1;

    HalconCpp::GenImageConst(&ho_Lidar_image, "byte", window_width, window_height);

    // 创建一个全白的红色通道图像
    HalconCpp::GenImageProto(ho_Lidar_image, &ho_ImageR, 255);

    // 创建一个全白的绿色通道图像
    HalconCpp::GenImageProto(ho_Lidar_image, &ho_ImageG, 255);

    // 创建一个全白的蓝色通道图像
    HalconCpp::GenImageProto(ho_Lidar_image, &ho_ImageB, 255);

    // 合成三个通道成一个RGB图像
    HalconCpp::Compose3(ho_ImageR, ho_ImageG, ho_ImageB, &ho_Lidar_image);

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t   count = _countof(nodes);
    sl_result op_result = drv->grabScanDataHq(nodes, count);

    if (SL_IS_OK(op_result))
    {
        drv->ascendScanData(nodes, count);
        //printf("count:%d\n", (int)count);
        for (int pos = 0; pos < (int)count; ++pos)
        {
            // 处理原始数据，参考雷达SDK
            float dis = nodes[pos].dist_mm_q2 / 4.0f; // 将距离数据转换为毫米
            float angle = nodes[pos].angle_z_q14 * 90.f / 16384.f; // 转换角度为度

            // 过滤不符合标准的数据
            if (dis < 50 || dis >20000) continue; // 跳过太远和太近的点
            if ((angle > 90) && (angle < 270)) continue; // 只保留前方数据

            // 将极坐标转换为笛卡尔坐标
            float x_temp = dis / scale_num * sin(angle / 180 * PI); //初始坐标系坐标
            float y_temp = dis / scale_num * cos(angle / 180 * PI);

            ori_x.push_back(x_temp);
            ori_y.push_back(y_temp);

        }
        // 将得到的坐标存储在ori(original)中
        Result.point_x = ori_x;
        Result.point_y = ori_y;

        //旋转坐标系
        for (int j = 0; j < ori_x.size(); j++)
        {
            point(0) = -ori_y[j];
            point(1) = -ori_x[j];

            point = rotate_matrix * point.reshaped(4, 1);

            if ((point(0) < (window_height)) && (point(1) < (window_width))) //检查旋转后的坐标是否在图像的有效区域内（即坐标点必须在图像的尺寸范围之内）
            {
                // 若坐标有效，绘制到hoimage上，点为黑色
                HalconCpp::SetGrayval(ho_Lidar_image, int(point(0)), int(point(1)), 0); //旋转之后坐标系坐标

                //std::string name = ("Lidar_image." + std::to_string(timesss) + ".bmp");
                //HalconCpp::HTuple filename = (HalconCpp::HTuple)(name.c_str());
                //HalconCpp::WriteImage(ho_Lidar_image, "bmp", 0, filename);

                // 在Halcon窗口中显示图像
                window.DispObj(ho_Lidar_image); // 在窗口中显示当前帧图像
                HalconCpp::WriteImage(ho_Lidar_image, "bmp", 0, "Lidar_image.bmp"); // 此处只保存了最后一帧的扫描图像
            }
        }

    }
    DealImage(ho_Lidar_image, Result);//处理图像信息
    CalDistance(ho_Lidar_image, Result);
    timesss++;
    //图像重置
    ori_x.clear();
    ori_y.clear();

}

void DealImage(HalconCpp::HObject  ho_Image, LidarInfo& Result)
{
    /// <summary>
    /// 处理图像 获得障碍物的距离和角度信息 距离为障碍物与雷达中心的距离 角度为障碍物与y轴的角度
    /// </summary>
    /// <returns></returns> 返回结构体 包含距离与角度  通过dealImage().获取
    ///将雷达图像进行处理得到障碍物的距离与角度信息
    /// 
    
    // 定义处理过程中需要的HALCON图像对象
    HalconCpp::HObject   ho_Region, ho_ROI_0, ho_RegionDilation;
    HalconCpp::HObject  ho_ConnectedRegions, ho_SelectedRegions, ho_ObjectSelected;
    HalconCpp::HObject  ho_Circle, ho_Contours, ho_Cross;

    // 定义控制和计算用的变量
    HalconCpp::HTuple  hv_Number, hv_dis_region, hv_angle_min_x;
    HalconCpp::HTuple  hv_angle_min_y, hv_Index, hv_DistanceMin, hv_DistanceMax;
    HalconCpp::HTuple  hv_Row, hv_Column, hv_Num_cross, hv_Angle, hv_Deg;


    HalconCpp::Threshold(ho_Image, &ho_Region, 0, 0); // 只选择黑色的点进行阈值分割
    HalconCpp::GenRectangle1(&ho_ROI_0, 0, 400, 500, 600);
    HalconCpp::Intersection(ho_Region, ho_ROI_0, &ho_Region);

    //滤除杂点
    HalconCpp::DilationCircle(ho_Region, &ho_RegionDilation, 2);
    HalconCpp::Connection(ho_RegionDilation, &ho_ConnectedRegions);
    HalconCpp::SelectShape(ho_ConnectedRegions, &ho_SelectedRegions, "area", "and", 50, 99999);

    //计算不同区域与雷达的最短距离与角度
    HalconCpp::CountObj(ho_SelectedRegions, &hv_Number);
    hv_dis_region = HalconCpp::HTuple();
    hv_angle_min_x = HalconCpp::HTuple();
    hv_angle_min_y = HalconCpp::HTuple();

    //距离可以视为处理后区域内障碍物到雷达中心的最短距离，不一定仅限于前方

    HalconCpp::HTuple end_val15 = hv_Number;
    HalconCpp::HTuple step_val15 = 1;
    for (hv_Index = 1; hv_Index.Continue(end_val15, step_val15); hv_Index += step_val15)
    {
        HalconCpp::SelectObj(ho_SelectedRegions, &ho_ObjectSelected, hv_Index);
        HalconCpp::DistancePr(ho_ObjectSelected, 500, 500, &hv_DistanceMin, &hv_DistanceMax);
        hv_dis_region[hv_Index - 1] = hv_DistanceMin;
        HalconCpp::GenCircleContourXld(&ho_Circle, 500, 500, hv_DistanceMin, 0, 6.28318, "positive",
            1);
        HalconCpp::GenContourRegionXld(ho_ObjectSelected, &ho_Contours, "center");
        HalconCpp::IntersectionCircleContourXld(ho_Contours, 500, 500, hv_DistanceMin, 0, 6.28318,
            "positive", &hv_Row, &hv_Column);
        HalconCpp::GenCrossContourXld(&ho_Cross, hv_Row, hv_Column, 6, 0.785398);

        HalconCpp::TupleLength(hv_Row, &hv_Num_cross);
        if (0 != (int(hv_Num_cross == 0)))
        {
            continue;
        }
        //计算与水平线x轴的夹角
        HalconCpp::AngleLx(500, 500, HalconCpp::HTuple(hv_Row[0]), HalconCpp::HTuple(hv_Column[0]), &hv_Angle);
        HalconCpp::TupleDeg(hv_Angle, &hv_Deg);
        hv_angle_min_x[hv_Index - 1] = hv_Deg;

        //这里的夹角，即与y轴的角度，表示机器人当前前进方向与墙的角度差。具体来说：

        //如果角度为正值，这通常表示机器人需要向左调整其前进方向，以便面向或平行于墙面。
        //如果角度为负值，则表示机器人需要向右调整。
        //计算与y轴正方向的夹角
        if (0 != (int(hv_Deg <= 90)))
        {
            hv_angle_min_y[hv_Index - 1] = 90 - hv_Deg;
        }
        else
        {
            hv_angle_min_y[hv_Index - 1] = -(hv_Deg - 90);
        }

    }
    if (hv_angle_min_y.Length() > 0)
    {
        Result.distance = double(hv_DistanceMin * 20);//将最终结果放到结构体中,1像素=20mm
    }

}

void CalDistance(HalconCpp::HObject  ho_Image, LidarInfo& Result)
{
    // Local iconic variables
    HalconCpp::HObject  ho_Region, ho_Cross, ho_Rectangle1;
    HalconCpp::HObject  ho_ImageReducedLeft, ho_Rectangle2, ho_ImageReducedRight;
    HalconCpp::HObject  ho_RegionDilatedLeft, ho_ConnectedRegionsLeft, ho_SelectedRegionsLeft;
    HalconCpp::HObject  ho_RegionDilatedRight, ho_ConnectedRegionsRight;
    HalconCpp::HObject  ho_SelectedRegionsRight, ho_SkeletonLeft, ho_ContoursLeft;
    HalconCpp::HObject  ho_ContoursSplitLeft, ho_FinalSelectedXLDLeft, ho_FinalLineRegionLeft;
    HalconCpp::HObject  ho_LineLeft, ho_SkeletonRight, ho_ContoursRight;
    HalconCpp::HObject  ho_ContoursSplitRight, ho_FinalSelectedXLDRight;
    HalconCpp::HObject  ho_FinalLineRegionRight, ho_LineRight;

    // Local control variables
    HalconCpp::HTuple  hv_Width, hv_Height, hv_WindowHandle;
    HalconCpp::HTuple  hv_Number, hv_xldLengthLeft, hv_XLDNumber, hv_Row;
    HalconCpp::HTuple  hv_Column, hv_PhiLeft, hv_Length1, hv_Length2, hv_LineRowLeft1;
    HalconCpp::HTuple  hv_LineColLeft1, hv_LineRowLeft2, hv_LineColLeft2;
    HalconCpp::HTuple  hv_DegLeft, hv_xldLengthRight, hv_PhiRight, hv_LineRowRight1;
    HalconCpp::HTuple  hv_LineColRight1, hv_LineRowRight2, hv_LineColRight2;
    HalconCpp::HTuple  hv_DegRight, hv_DistanceMin, hv_DistanceMax, hv_DistanceCenterToLeft;
    HalconCpp::HTuple  hv_DistanceCenterToRight;


    //获取图片的大小
    GetImageSize(ho_Image, &hv_Width, &hv_Height);

    //定义ROI和阈值处理
    Threshold(ho_Image, &ho_Region, 0, 0);
    //对图像应用阈值操作，选出灰度值在128到255之间的像素，创建一个二值图像区域。

    GenCrossContourXld(&ho_Cross, 500, 500, 6, 0.785398);

    GenRectangle1(&ho_Rectangle1, 420, 40, 480, 480);
    ReduceDomain(ho_Region, ho_Rectangle1, &ho_ImageReducedLeft);

    GenRectangle1(&ho_Rectangle2, 420, 520, 480, 920);
    ReduceDomain(ho_Region, ho_Rectangle2, &ho_ImageReducedRight);

    DilationCircle(ho_ImageReducedLeft, &ho_RegionDilatedLeft, 2);
    Connection(ho_RegionDilatedLeft, &ho_ConnectedRegionsLeft);
    SelectShape(ho_ConnectedRegionsLeft, &ho_SelectedRegionsLeft, "area", "and", 10,
        99999);

    DilationCircle(ho_ImageReducedRight, &ho_RegionDilatedRight, 2);
    Connection(ho_RegionDilatedRight, &ho_ConnectedRegionsRight);
    SelectShape(ho_ConnectedRegionsRight, &ho_SelectedRegionsRight, "area", "and",
        10, 99999);

    //对左侧区域进行处理
    Skeleton(ho_SelectedRegionsLeft, &ho_SkeletonLeft);
    GenContoursSkeletonXld(ho_SkeletonLeft, &ho_ContoursLeft, 5, "generalize1");
    //分割线段
    SegmentContoursXld(ho_ContoursLeft, &ho_ContoursSplitLeft, "lines_circles", 10,
        4, 2);
    //筛选角度符合范围的直线 这里用的是弧度
    SelectShapeXld(ho_ContoursSplitLeft, &ho_FinalSelectedXLDLeft, (HalconCpp::HTuple("rect2_phi").Append("rect2_phi")),
        "or", (HalconCpp::HTuple(-1.5707964).Append(0.7853982)), (HalconCpp::HTuple(-0.7853982).Append(1.5707964)));
    CountObj(ho_FinalSelectedXLDLeft, &hv_Number);
    if (0 != (int(hv_Number < 1)))
    {
        return;
    }
    LengthXld(ho_FinalSelectedXLDLeft, &hv_xldLengthLeft);
    //选择筛选后的最长直线做凸包
    SelectShapeXld(ho_FinalSelectedXLDLeft, &ho_FinalSelectedXLDLeft, "contlength",
        "and", hv_xldLengthLeft.TupleMax(), hv_xldLengthLeft.TupleMax());
    CountObj(ho_FinalSelectedXLDLeft, &hv_XLDNumber);

    GenRegionContourXld(ho_FinalSelectedXLDLeft, &ho_FinalLineRegionLeft, "filled");
    //计算包围所选区域的最小矩形。此函数返回矩形的中心位置（Row, Column），矩形的旋转角度 PhiLeft（以弧度为单位），以及矩形的两个边长 Length1 和 Length2
    SmallestRectangle2(ho_FinalLineRegionLeft, &hv_Row, &hv_Column, &hv_PhiLeft, &hv_Length1,
        &hv_Length2);
    GenRectangle2(&ho_Rectangle1, hv_Row, hv_Column, hv_PhiLeft, hv_Length1, hv_Length2);

    //计算直线端点坐标
    //根据矩形的中心点、旋转角度和长度，计算出矩形长边的两个端点坐标。这些坐标将用于生成一条直线。
    hv_LineRowLeft1 = hv_Row - (hv_Length1 * (hv_PhiLeft.TupleSin()));
    hv_LineColLeft1 = hv_Column + (hv_Length1 * (hv_PhiLeft.TupleCos()));
    hv_LineRowLeft2 = hv_Row + (hv_Length1 * (hv_PhiLeft.TupleSin()));
    hv_LineColLeft2 = hv_Column - (hv_Length1 * (hv_PhiLeft.TupleCos()));

    GenRegionLine(&ho_LineLeft, hv_LineRowLeft1, hv_LineColLeft1, hv_LineRowLeft2,
        hv_LineColLeft2);
    TupleDeg(hv_PhiLeft, &hv_DegLeft);

    if (0 != (int(hv_DegLeft >= 90)))
    {
        hv_DegLeft = -(hv_DegLeft - 90);
    }
    else
    {
        hv_DegLeft = 90 - hv_DegLeft;
    }


    //对右侧区域进行处理
    Skeleton(ho_SelectedRegionsRight, &ho_SkeletonRight);
    GenContoursSkeletonXld(ho_SkeletonRight, &ho_ContoursRight, 5, "generalize1");
    //分割线段
    SegmentContoursXld(ho_ContoursRight, &ho_ContoursSplitRight, "lines_circles", 10,
        4, 2);
    //筛选角度符合范围的直线 这里用的是弧度
    SelectShapeXld(ho_ContoursSplitRight, &ho_FinalSelectedXLDRight, (HalconCpp::HTuple("rect2_phi").Append("rect2_phi")),
        "or", (HalconCpp::HTuple(-1.5707964).Append(0.7853982)), (HalconCpp::HTuple(-0.7853982).Append(1.5707964)));
    CountObj(ho_FinalSelectedXLDRight, &hv_Number);
    if (0 != (int(hv_Number < 1)))
    {
        return;
    }
    LengthXld(ho_FinalSelectedXLDRight, &hv_xldLengthRight);
    //选择筛选后的最长直线做凸包
    SelectShapeXld(ho_FinalSelectedXLDRight, &ho_FinalSelectedXLDRight, "contlength",
        "and", hv_xldLengthRight.TupleMax(), hv_xldLengthRight.TupleMax());
    CountObj(ho_FinalSelectedXLDRight, &hv_XLDNumber);

    GenRegionContourXld(ho_FinalSelectedXLDRight, &ho_FinalLineRegionRight, "filled");
    SmallestRectangle2(ho_FinalLineRegionRight, &hv_Row, &hv_Column, &hv_PhiRight,
        &hv_Length1, &hv_Length2);
    GenRectangle2(&ho_Rectangle2, hv_Row, hv_Column, hv_PhiRight, hv_Length1, hv_Length2);

    hv_LineRowRight1 = hv_Row - (hv_Length1 * (hv_PhiRight.TupleSin()));
    hv_LineColRight1 = hv_Column + (hv_Length1 * (hv_PhiRight.TupleCos()));
    hv_LineRowRight2 = hv_Row + (hv_Length1 * (hv_PhiRight.TupleSin()));
    hv_LineColRight2 = hv_Column - (hv_Length1 * (hv_PhiRight.TupleCos()));

    GenRegionLine(&ho_LineRight, hv_LineRowRight1, hv_LineColRight1, hv_LineRowRight2,
        hv_LineColRight2);
    TupleDeg(hv_PhiRight, &hv_DegRight);

    //if (0 != (int(hv_DegRight >= 90)))
    //{
    //    hv_DegRight = -(hv_DegRight - 90);
    //}
    //else
    //{
    //    hv_DegRight = 90 - hv_DegRight;
    //}

    // 调整角度基准，以垂直向上（90度）为0度
    hv_DegRight = 90 - hv_DegRight;

    // 标准化角度，使其在-90到+90度范围内
    if (hv_DegRight > 90) {
        hv_DegRight -= 180;
    }
    else if (hv_DegRight < -90) {
        hv_DegRight += 180;
    }

    //计算两条直线之间的最小和最大距离
    DistanceSs(hv_LineRowLeft1, hv_LineColLeft1, hv_LineRowLeft2, hv_LineColLeft2,
        hv_LineRowRight1, hv_LineColRight1, hv_LineRowRight2, hv_LineColRight2, &hv_DistanceMin,
        &hv_DistanceMax);

    //计算中心点到左侧直线的距离
    DistancePl(500, 500, hv_LineRowLeft1, hv_LineColLeft1, hv_LineRowLeft2, hv_LineColLeft2,
        &hv_DistanceCenterToLeft);

    //计算中心点到右侧直线的距离
    DistancePl(500, 500, hv_LineRowRight1, hv_LineColRight1, hv_LineRowRight2, hv_LineColRight2,
        &hv_DistanceCenterToRight);

    //将结果存入结构体
    Result.distanceLeft = hv_DistanceCenterToLeft * 20;
    Result.distanceRight = hv_DistanceCenterToRight * 20;
    Result.theta = hv_DegRight;
}

