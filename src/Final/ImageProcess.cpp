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

                std::string name = ("Lidar_image." + std::to_string(timesss) + ".bmp");
                HalconCpp::HTuple filename = (HalconCpp::HTuple)(name.c_str());
                HalconCpp::WriteImage(ho_Lidar_image, "bmp", 0, filename);

                // ��Halcon��������ʾͼ��
                window.DispObj(ho_Lidar_image); // �ڴ�������ʾ��ǰ֡ͼ��
                // HalconCpp::WriteImage(ho_Lidar_image, "bmp", 0, "Lidar_image.bmp"); // �˴�ֻ���������һ֡��ɨ��ͼ��
            }
        }

    }
    DealImage(ho_Lidar_image, Result);//����ͼ����Ϣ
    Caldistance1(ho_Lidar_image, Result);
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
        Result.distance = float(hv_DistanceMin * 20);//�����ս���ŵ��ṹ����,1����=20mm
        Result.theta = float(hv_angle_min_y[0]);

    }

}

void CalDistance(HalconCpp::HObject  ho_Image, LidarInfo& Result)
{
    // Local iconic variables
    HalconCpp::HObject  ho_Rectangle1, ho_ImageReducedLeft;
    HalconCpp::HObject  ho_Rectangle2, ho_ImageReducedRight, ho_EdgesLeft;
    HalconCpp::HObject  ho_ContoursLeft, ho_LineLeft, ho_EdgesRight, ho_ContoursRight;
    HalconCpp::HObject  ho_LineRight;

    // Local control variables
    HalconCpp::HTuple  hv_RowBeginLeft, hv_ColBeginLeft, hv_RowEndLeft;
    HalconCpp::HTuple  hv_ColEndLeft, hv_NrLeft, hv_NcLeft, hv_DistLeft;
    HalconCpp::HTuple  hv_RowBeginRight, hv_ColBeginRight, hv_RowEndRight;
    HalconCpp::HTuple  hv_ColEndRight, hv_NrRight, hv_NcRight, hv_DistRight;
    HalconCpp::HTuple  hv_x_radar, hv_y_radar, hv_MinDistanceLeft, hv_MinDistanceRight;
    HalconCpp::HTuple  hv_DistancesLeft, hv_DistancesRight;

    // �������ROI
    GenRectangle1(&ho_Rectangle1, 384, 24, 480, 430);
    ReduceDomain(ho_Image, ho_Rectangle1, &ho_ImageReducedLeft);

    // �����Ҳ�ROI
    GenRectangle1(&ho_Rectangle2, 384, 560, 510, 920);
    ReduceDomain(ho_Image, ho_Rectangle2, &ho_ImageReducedRight);

    // �����������д���
    EdgesSubPix(ho_ImageReducedLeft, &ho_EdgesLeft, "lanser2", 0.5, 20, 40); // ��Ե���
    GenPolygonsXld(ho_EdgesLeft, &ho_ContoursLeft, "ramer", 2); // �ӱ�Ե���ɶ����XLD����
    SplitContoursXld(ho_ContoursLeft, &ho_LineLeft, "polygon", 1, 5); // �������XLD������ֳɵ�������Ƭ��

    // ���Ҳ�������д���
    EdgesSubPix(ho_ImageReducedRight, &ho_EdgesRight, "lanser2", 0.5, 20, 40);
    GenPolygonsXld(ho_EdgesRight, &ho_ContoursRight, "ramer", 2);
    SplitContoursXld(ho_ContoursRight, &ho_LineRight, "polygon", 1, 5);

    // ���ֱ��
    FitLineContourXld(ho_LineLeft, "tukey", -1, 0, 5, 2, &hv_RowBeginLeft, &hv_ColBeginLeft,
        &hv_RowEndLeft, &hv_ColEndLeft, &hv_NrLeft, &hv_NcLeft, &hv_DistLeft);
    FitLineContourXld(ho_LineRight, "tukey", -1, 0, 5, 2, &hv_RowBeginRight, &hv_ColBeginRight,
        &hv_RowEndRight, &hv_ColEndRight, &hv_NrRight, &hv_NcRight, &hv_DistRight);

    // �����״����ĺ���С����
    hv_x_radar = 500;
    hv_y_radar = 500;
    hv_MinDistanceLeft = 0;
    hv_MinDistanceRight = 0;

    // ��ʼ����������Ϊһ���ϴ��ֵ���Ա�������飨��ⲻ��ֱ�ߣ������
    hv_DistancesLeft = HalconCpp::HTuple(9999);
    hv_DistancesRight = HalconCpp::HTuple(9999);

    // ����㵽����ߵľ���
    DistancePl(hv_x_radar, hv_y_radar, hv_RowBeginLeft, hv_ColBeginLeft, hv_RowEndLeft,
        hv_ColEndLeft, &hv_DistancesLeft);
    

    // ����㵽�Ҳ��ߵľ���
    DistancePl(hv_x_radar, hv_y_radar, hv_RowBeginRight, hv_ColBeginRight, hv_RowEndRight,
        hv_ColEndRight, &hv_DistancesRight);
    // TupleMin(hv_DistancesRight, &hv_MinDistanceRight);

    // ������С����ǰ���Ƴ���ʼֵ
    hv_DistancesLeft = hv_DistancesLeft.TupleRemove(0);
    hv_DistancesRight = hv_DistancesRight.TupleRemove(0);

    // ȷ��������һ��Ԫ�غ��ټ�����Сֵ
    if (hv_DistancesLeft.Length() > 0) {
        TupleMin(hv_DistancesLeft, &hv_MinDistanceLeft);
    }
    else {
        // û�м�⵽ֱ�ߣ���������Ϊһ���ض��Ĵ���ֵ���߱���9999
        hv_MinDistanceLeft = HalconCpp::HTuple(9999);
    }

    if (hv_DistancesRight.Length() > 0) {
        TupleMin(hv_DistancesRight, &hv_MinDistanceRight);
    }
    else {
        // û�м�⵽ֱ�ߣ���������Ϊһ���ض��Ĵ���ֵ���߱���9999
        hv_MinDistanceRight = HalconCpp::HTuple(9999);
    }

    Result.distanceLeft = float(hv_MinDistanceLeft * 20);
    Result.distanceRight = float(hv_MinDistanceRight * 20);
}

void Caldistance1(HalconCpp::HObject  ho_Image, LidarInfo& Result)
{

    // Local iconic variables
    HalconCpp::HObject  ho_Region, ho_Cross, ho_Rectangle1;
    HalconCpp::HObject  ho_ImageReducedLeft, ho_RegionDilatedLeft, ho_ConnectedRegionsLeft;
    HalconCpp::HObject  ho_SelectedRegionsLeft, ho_Rectangle2, ho_ImageReducedRight;
    HalconCpp::HObject  ho_RegionDilatedRight, ho_ConnectedRegionsRight;
    HalconCpp::HObject  ho_SelectedRegionsRight, ho_ContoursLeft, ho_SmoothContoursLeft;
    HalconCpp::HObject  ho_PolygonsLeft, ho_LineLeft, ho_ContoursRight;
    HalconCpp::HObject  ho_SmoothContoursRight, ho_PolygonsRight, ho_LineRight;

    // Local control variables
    HalconCpp::HTuple  hv_Width, hv_Height, hv_WindowHandle;
    HalconCpp::HTuple  hv_RowBeginLeft, hv_ColBeginLeft, hv_RowEndLeft;
    HalconCpp::HTuple  hv_ColEndLeft, hv_NrLeft, hv_NcLeft, hv_DistLeft;
    HalconCpp::HTuple  hv_RowBeginRight, hv_ColBeginRight, hv_RowEndRight;
    HalconCpp::HTuple  hv_ColEndRight, hv_NrRight, hv_NcRight, hv_DistRight;
    HalconCpp::HTuple  hv_x_radar, hv_y_radar, hv_MinDistanceLeft, hv_MinDistanceRight;
    HalconCpp::HTuple  hv_DistancesLeft, hv_DistancesRight;


    // ѡȡ�Ҷ�ֵΪ0������ɫ�ĵ�
    Threshold(ho_Image, &ho_Region, 0, 0);

    // �������д���
    GenRectangle1(&ho_Rectangle1, 420, 40, 500, 480); // ���ROI
    ReduceDomain(ho_Region, ho_Rectangle1, &ho_ImageReducedLeft); // ���ƴ�������ROI��
    DilationCircle(ho_ImageReducedLeft, &ho_RegionDilatedLeft, 2); // ����
    Connection(ho_RegionDilatedLeft, &ho_ConnectedRegionsLeft); // ��ͨ
    SelectShape(ho_ConnectedRegionsLeft, &ho_SelectedRegionsLeft, "area", "and", 50,
        99999); // ѡȡ����ֵ��50���ϵ�����

    // ���Ҳ���д���
    GenRectangle1(&ho_Rectangle2, 384, 520, 500, 920);  

    ReduceDomain(ho_Region, ho_Rectangle2, &ho_ImageReducedRight);
    DilationCircle(ho_ImageReducedRight, &ho_RegionDilatedRight, 2);
    Connection(ho_RegionDilatedRight, &ho_ConnectedRegionsRight);
    SelectShape(ho_ConnectedRegionsRight, &ho_SelectedRegionsRight, "area", "and",
        50, 99999);

    // �����������
    GenContourRegionXld(ho_SelectedRegionsLeft, &ho_ContoursLeft, "border"); // ��������XLD����
    SmoothContoursXld(ho_ContoursLeft, &ho_SmoothContoursLeft, 9); // ƽ����������ֹ��̫��ת�۵�
    GenPolygonsXld(ho_SmoothContoursLeft, &ho_PolygonsLeft, "ramer", 2); // �������������ɶ����
    SplitContoursXld(ho_PolygonsLeft, &ho_LineLeft, "polygon", 1, 5); // ��ֶ�����������߶�

    // �����Ҳ�����
    GenContourRegionXld(ho_SelectedRegionsRight, &ho_ContoursRight, "border");
    SmoothContoursXld(ho_ContoursRight, &ho_SmoothContoursRight, 9);
    GenPolygonsXld(ho_SmoothContoursRight, &ho_PolygonsRight, "ramer", 2);
    SplitContoursXld(ho_PolygonsRight, &ho_LineRight, "polygon", 1, 5);

    // ���ֱ��
    FitLineContourXld(ho_LineLeft, "tukey", -1, 0, 5, 2, &hv_RowBeginLeft, &hv_ColBeginLeft,
        &hv_RowEndLeft, &hv_ColEndLeft, &hv_NrLeft, &hv_NcLeft, &hv_DistLeft);
    FitLineContourXld(ho_LineRight, "tukey", -1, 0, 5, 2, &hv_RowBeginRight, &hv_ColBeginRight,
        &hv_RowEndRight, &hv_ColEndRight, &hv_NrRight, &hv_NcRight, &hv_DistRight);


    // �����״����ĺ���̾���
    hv_x_radar = 500;
    hv_y_radar = 500;
    hv_MinDistanceLeft = 0;
    hv_MinDistanceRight = 0;

       // ��ʼ����������Ϊһ���ϴ��ֵ���Ա�������飨��ⲻ��ֱ�ߣ������
    hv_DistancesLeft = HalconCpp::HTuple(9999);
    hv_DistancesRight = HalconCpp::HTuple(9999);

    // ����㵽����ߵľ���
    DistancePl(hv_x_radar, hv_y_radar, hv_RowBeginLeft, hv_ColBeginLeft, hv_RowEndLeft,
        hv_ColEndLeft, &hv_DistancesLeft);


    // ����㵽�Ҳ��ߵľ���
    DistancePl(hv_x_radar, hv_y_radar, hv_RowBeginRight, hv_ColBeginRight, hv_RowEndRight,
        hv_ColEndRight, &hv_DistancesRight);
    // TupleMin(hv_DistancesRight, &hv_MinDistanceRight);

    // ������С����ǰ���Ƴ���ʼֵ
    hv_DistancesLeft = hv_DistancesLeft.TupleRemove(0);
    hv_DistancesRight = hv_DistancesRight.TupleRemove(0);

    // ȷ��������һ��Ԫ�غ��ټ�����Сֵ
    if (hv_DistancesLeft.Length() > 0) {
        TupleMin(hv_DistancesLeft, &hv_MinDistanceLeft);
    }
    else {
        // û�м�⵽ֱ�ߣ���������Ϊһ���ض��Ĵ���ֵ���߱���9999
        hv_MinDistanceLeft = HalconCpp::HTuple(9999);
    }

    if (hv_DistancesRight.Length() > 0) {
        TupleMin(hv_DistancesRight, &hv_MinDistanceRight);
    }
    else {
        // û�м�⵽ֱ�ߣ���������Ϊһ���ض��Ĵ���ֵ���߱���9999
        hv_MinDistanceRight = HalconCpp::HTuple(9999);
    }

    Result.distanceLeft = float(hv_MinDistanceLeft * 20);
    Result.distanceRight = float(hv_MinDistanceRight * 20);

}
