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
const int scale_num = 20; //ͼ������ϵ��
int timesss = 0;


void GetData(sl::ILidarDriver* drv, LidarInfo& Result, HalconCpp::HWindow& window) //����Ϊ�״�ʵ����ָ��ʹ洢����ʵ��
{

    HalconCpp::HObject  ho_Lidar_image;//halon��ʼ��ͼ��
    HalconCpp::HObject ho_ImageR, ho_ImageG, ho_ImageB;
    int window_height = 10000 / scale_num;//ͼ��ĳ�����
    int window_width = 20000 / scale_num;
    std::vector<float> ori_x; //������������ϵ��x��������
    std::vector<float> ori_y;// ������������ϵ��y��������

    Eigen::Matrix<float, 4, 4> rotate_matrix;//��������ϵ��ת����
    Eigen::MatrixXf point(1, 4); //����������
    point << 0, 0, 0, 1;
    rotate_matrix << 1, 0, 0, (window_height), //������ת����
        0, cos(rotate_theta), -std::sin(rotate_theta), (window_height),
        0, sin(rotate_theta), cos(rotate_theta), 0,
        0, 0, 0, 1;

    HalconCpp::GenImageConst(&ho_Lidar_image, "byte", window_width, window_height);

    // ����һ��ȫ�׵ĺ�ɫͨ��ͼ��
    HalconCpp::GenImageProto(ho_Lidar_image, &ho_ImageR, 255);

    // ����һ��ȫ�׵���ɫͨ��ͼ��
    HalconCpp::GenImageProto(ho_Lidar_image, &ho_ImageG, 255);

    // ����һ��ȫ�׵���ɫͨ��ͼ��
    HalconCpp::GenImageProto(ho_Lidar_image, &ho_ImageB, 255);

    // �ϳ�����ͨ����һ��RGBͼ��
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
            // ����ԭʼ���ݣ��ο��״�SDK
            float dis = nodes[pos].dist_mm_q2 / 4.0f; // ����������ת��Ϊ����
            float angle = nodes[pos].angle_z_q14 * 90.f / 16384.f; // ת���Ƕ�Ϊ��

            // ���˲����ϱ�׼������
            if (dis < 50 || dis >20000) continue; // ����̫Զ��̫���ĵ�
            if ((angle > 90) && (angle < 270)) continue; // ֻ����ǰ������

            // ��������ת��Ϊ�ѿ�������
            float x_temp = dis / scale_num * sin(angle / 180 * PI); //��ʼ����ϵ����
            float y_temp = dis / scale_num * cos(angle / 180 * PI);

            ori_x.push_back(x_temp);
            ori_y.push_back(y_temp);

        }
        // ���õ�������洢��ori(original)��
        Result.point_x = ori_x;
        Result.point_y = ori_y;

        //��ת����ϵ
        for (int j = 0; j < ori_x.size(); j++)
        {
            point(0) = -ori_y[j];
            point(1) = -ori_x[j];

            point = rotate_matrix * point.reshaped(4, 1);

            if ((point(0) < (window_height)) && (point(1) < (window_width))) //�����ת��������Ƿ���ͼ�����Ч�����ڣ�������������ͼ��ĳߴ緶Χ֮�ڣ�
            {
                // ��������Ч�����Ƶ�hoimage�ϣ���Ϊ��ɫ
                HalconCpp::SetGrayval(ho_Lidar_image, int(point(0)), int(point(1)), 0); //��ת֮������ϵ����

                //std::string name = ("Lidar_image." + std::to_string(timesss) + ".bmp");
                //HalconCpp::HTuple filename = (HalconCpp::HTuple)(name.c_str());
                //HalconCpp::WriteImage(ho_Lidar_image, "bmp", 0, filename);

                // ��Halcon��������ʾͼ��
                window.DispObj(ho_Lidar_image); // �ڴ�������ʾ��ǰ֡ͼ��
                HalconCpp::WriteImage(ho_Lidar_image, "bmp", 0, "Lidar_image.bmp"); // �˴�ֻ���������һ֡��ɨ��ͼ��
            }
        }

    }
    DealImage(ho_Lidar_image, Result);//����ͼ����Ϣ
    CalDistance(ho_Lidar_image, Result);
    timesss++;
    //ͼ������
    ori_x.clear();
    ori_y.clear();

}

void DealImage(HalconCpp::HObject  ho_Image, LidarInfo& Result)
{
    /// <summary>
    /// ����ͼ�� ����ϰ���ľ���ͽǶ���Ϣ ����Ϊ�ϰ������״����ĵľ��� �Ƕ�Ϊ�ϰ�����y��ĽǶ�
    /// </summary>
    /// <returns></returns> ���ؽṹ�� ����������Ƕ�  ͨ��dealImage().��ȡ
    ///���״�ͼ����д���õ��ϰ���ľ�����Ƕ���Ϣ
    /// 
    
    // ���崦���������Ҫ��HALCONͼ�����
    HalconCpp::HObject   ho_Region, ho_ROI_0, ho_RegionDilation;
    HalconCpp::HObject  ho_ConnectedRegions, ho_SelectedRegions, ho_ObjectSelected;
    HalconCpp::HObject  ho_Circle, ho_Contours, ho_Cross;

    // ������ƺͼ����õı���
    HalconCpp::HTuple  hv_Number, hv_dis_region, hv_angle_min_x;
    HalconCpp::HTuple  hv_angle_min_y, hv_Index, hv_DistanceMin, hv_DistanceMax;
    HalconCpp::HTuple  hv_Row, hv_Column, hv_Num_cross, hv_Angle, hv_Deg;


    HalconCpp::Threshold(ho_Image, &ho_Region, 0, 0); // ֻѡ���ɫ�ĵ������ֵ�ָ�
    HalconCpp::GenRectangle1(&ho_ROI_0, 0, 400, 500, 600);
    HalconCpp::Intersection(ho_Region, ho_ROI_0, &ho_Region);

    //�˳��ӵ�
    HalconCpp::DilationCircle(ho_Region, &ho_RegionDilation, 2);
    HalconCpp::Connection(ho_RegionDilation, &ho_ConnectedRegions);
    HalconCpp::SelectShape(ho_ConnectedRegions, &ho_SelectedRegions, "area", "and", 50, 99999);

    //���㲻ͬ�������״����̾�����Ƕ�
    HalconCpp::CountObj(ho_SelectedRegions, &hv_Number);
    hv_dis_region = HalconCpp::HTuple();
    hv_angle_min_x = HalconCpp::HTuple();
    hv_angle_min_y = HalconCpp::HTuple();

    //���������Ϊ������������ϰ��ﵽ�״����ĵ���̾��룬��һ��������ǰ��

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
        //������ˮƽ��x��ļн�
        HalconCpp::AngleLx(500, 500, HalconCpp::HTuple(hv_Row[0]), HalconCpp::HTuple(hv_Column[0]), &hv_Angle);
        HalconCpp::TupleDeg(hv_Angle, &hv_Deg);
        hv_angle_min_x[hv_Index - 1] = hv_Deg;

        //����ļнǣ�����y��ĽǶȣ���ʾ�����˵�ǰǰ��������ǽ�ĽǶȲ������˵��

        //����Ƕ�Ϊ��ֵ����ͨ����ʾ��������Ҫ���������ǰ�������Ա������ƽ����ǽ�档
        //����Ƕ�Ϊ��ֵ�����ʾ��������Ҫ���ҵ�����
        //������y��������ļн�
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
        Result.distance = double(hv_DistanceMin * 20);//�����ս���ŵ��ṹ����,1����=20mm
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


    //��ȡͼƬ�Ĵ�С
    GetImageSize(ho_Image, &hv_Width, &hv_Height);

    //����ROI����ֵ����
    Threshold(ho_Image, &ho_Region, 0, 0);
    //��ͼ��Ӧ����ֵ������ѡ���Ҷ�ֵ��128��255֮������أ�����һ����ֵͼ������

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

    //�����������д���
    Skeleton(ho_SelectedRegionsLeft, &ho_SkeletonLeft);
    GenContoursSkeletonXld(ho_SkeletonLeft, &ho_ContoursLeft, 5, "generalize1");
    //�ָ��߶�
    SegmentContoursXld(ho_ContoursLeft, &ho_ContoursSplitLeft, "lines_circles", 10,
        4, 2);
    //ɸѡ�Ƕȷ��Ϸ�Χ��ֱ�� �����õ��ǻ���
    SelectShapeXld(ho_ContoursSplitLeft, &ho_FinalSelectedXLDLeft, (HalconCpp::HTuple("rect2_phi").Append("rect2_phi")),
        "or", (HalconCpp::HTuple(-1.5707964).Append(0.7853982)), (HalconCpp::HTuple(-0.7853982).Append(1.5707964)));
    CountObj(ho_FinalSelectedXLDLeft, &hv_Number);
    if (0 != (int(hv_Number < 1)))
    {
        return;
    }
    LengthXld(ho_FinalSelectedXLDLeft, &hv_xldLengthLeft);
    //ѡ��ɸѡ����ֱ����͹��
    SelectShapeXld(ho_FinalSelectedXLDLeft, &ho_FinalSelectedXLDLeft, "contlength",
        "and", hv_xldLengthLeft.TupleMax(), hv_xldLengthLeft.TupleMax());
    CountObj(ho_FinalSelectedXLDLeft, &hv_XLDNumber);

    GenRegionContourXld(ho_FinalSelectedXLDLeft, &ho_FinalLineRegionLeft, "filled");
    //�����Χ��ѡ�������С���Ρ��˺������ؾ��ε�����λ�ã�Row, Column�������ε���ת�Ƕ� PhiLeft���Ի���Ϊ��λ�����Լ����ε������߳� Length1 �� Length2
    SmallestRectangle2(ho_FinalLineRegionLeft, &hv_Row, &hv_Column, &hv_PhiLeft, &hv_Length1,
        &hv_Length2);
    GenRectangle2(&ho_Rectangle1, hv_Row, hv_Column, hv_PhiLeft, hv_Length1, hv_Length2);

    //����ֱ�߶˵�����
    //���ݾ��ε����ĵ㡢��ת�ǶȺͳ��ȣ���������γ��ߵ������˵����ꡣ��Щ���꽫��������һ��ֱ�ߡ�
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


    //���Ҳ�������д���
    Skeleton(ho_SelectedRegionsRight, &ho_SkeletonRight);
    GenContoursSkeletonXld(ho_SkeletonRight, &ho_ContoursRight, 5, "generalize1");
    //�ָ��߶�
    SegmentContoursXld(ho_ContoursRight, &ho_ContoursSplitRight, "lines_circles", 10,
        4, 2);
    //ɸѡ�Ƕȷ��Ϸ�Χ��ֱ�� �����õ��ǻ���
    SelectShapeXld(ho_ContoursSplitRight, &ho_FinalSelectedXLDRight, (HalconCpp::HTuple("rect2_phi").Append("rect2_phi")),
        "or", (HalconCpp::HTuple(-1.5707964).Append(0.7853982)), (HalconCpp::HTuple(-0.7853982).Append(1.5707964)));
    CountObj(ho_FinalSelectedXLDRight, &hv_Number);
    if (0 != (int(hv_Number < 1)))
    {
        return;
    }
    LengthXld(ho_FinalSelectedXLDRight, &hv_xldLengthRight);
    //ѡ��ɸѡ����ֱ����͹��
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

    // �����ǶȻ�׼���Դ�ֱ���ϣ�90�ȣ�Ϊ0��
    hv_DegRight = 90 - hv_DegRight;

    // ��׼���Ƕȣ�ʹ����-90��+90�ȷ�Χ��
    if (hv_DegRight > 90) {
        hv_DegRight -= 180;
    }
    else if (hv_DegRight < -90) {
        hv_DegRight += 180;
    }

    //��������ֱ��֮�����С��������
    DistanceSs(hv_LineRowLeft1, hv_LineColLeft1, hv_LineRowLeft2, hv_LineColLeft2,
        hv_LineRowRight1, hv_LineColRight1, hv_LineRowRight2, hv_LineColRight2, &hv_DistanceMin,
        &hv_DistanceMax);

    //�������ĵ㵽���ֱ�ߵľ���
    DistancePl(500, 500, hv_LineRowLeft1, hv_LineColLeft1, hv_LineRowLeft2, hv_LineColLeft2,
        &hv_DistanceCenterToLeft);

    //�������ĵ㵽�Ҳ�ֱ�ߵľ���
    DistancePl(500, 500, hv_LineRowRight1, hv_LineColRight1, hv_LineRowRight2, hv_LineColRight2,
        &hv_DistanceCenterToRight);

    //���������ṹ��
    Result.distanceLeft = hv_DistanceCenterToLeft * 20;
    Result.distanceRight = hv_DistanceCenterToRight * 20;
    Result.theta = hv_DegRight;
}

