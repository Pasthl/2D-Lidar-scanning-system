//#include<iostream>
//#include<math.h>
//#include<winsock2.h>
//#include<Eigen/Dense>
//#include<vector>
//#include<thread>
//# include "HalconCpp.h"
//# include "HDevThread.h"
//using namespace HalconCpp;
//
//// ���� ResultStruct �ṹ���Ա������ͽǶ���Ϣ
//struct Result {
//    double distance = 0.0;
//    double theta = 0.0;
//};
//
//Result dealImage(HalconCpp::HObject  ho_Image)
//{
//    /// <summary>
//    /// ����ͼ�� ����ϰ���ľ���ͽǶ���Ϣ ����Ϊ�ϰ������״����ĵľ��� �Ƕ�Ϊ�ϰ�����y��ĽǶ�
//    /// </summary>
//    /// <returns></returns> ���ؽṹ�� ����������Ƕ�  ͨ��dealImage().��ȡ
//    ///���״�ͼ����д���õ��ϰ���ľ�����Ƕ���Ϣ
//    HalconCpp::HObject   ho_Region, ho_ROI_0, ho_RegionDilation;
//    HalconCpp::HObject  ho_ConnectedRegions, ho_SelectedRegions, ho_ObjectSelected;
//    HalconCpp::HObject  ho_Circle, ho_Contours, ho_Cross;
//
//    // Local control variables
//    HalconCpp::HTuple  hv_Number, hv_dis_region, hv_angle_min_x;
//    HalconCpp::HTuple  hv_angle_min_y, hv_Index, hv_DistanceMin, hv_DistanceMax;
//    HalconCpp::HTuple  hv_Row, hv_Column, hv_Num_cross, hv_Angle, hv_Deg;
//
//
//
//    HalconCpp::Threshold(ho_Image, &ho_Region, 128, 255);
//    HalconCpp::GenRectangle1(&ho_ROI_0, 0, 400, 500, 600);
//    HalconCpp::Intersection(ho_Region, ho_ROI_0, &ho_Region);
//
//    //�˳��ӵ�
//    HalconCpp::DilationCircle(ho_Region, &ho_RegionDilation, 2);
//    HalconCpp::Connection(ho_RegionDilation, &ho_ConnectedRegions);
//    HalconCpp::SelectShape(ho_ConnectedRegions, &ho_SelectedRegions, "area", "and", 50, 99999);
//
//    //���㲻ͬ�������״����̾�����Ƕ�
//    HalconCpp::CountObj(ho_SelectedRegions, &hv_Number);
//    hv_dis_region = HalconCpp::HTuple();
//    hv_angle_min_x = HalconCpp::HTuple();
//    hv_angle_min_y = HalconCpp::HTuple();
//
//    HalconCpp::HTuple end_val15 = hv_Number;
//    HalconCpp::HTuple step_val15 = 1;
//    for (hv_Index = 1; hv_Index.Continue(end_val15, step_val15); hv_Index += step_val15)
//    {
//        HalconCpp::SelectObj(ho_SelectedRegions, &ho_ObjectSelected, hv_Index);
//        HalconCpp::DistancePr(ho_ObjectSelected, 500, 500, &hv_DistanceMin, &hv_DistanceMax);
//        hv_dis_region[hv_Index - 1] = hv_DistanceMin;
//        HalconCpp::GenCircleContourXld(&ho_Circle, 500, 500, hv_DistanceMin, 0, 6.28318, "positive",
//            1);
//        HalconCpp::GenContourRegionXld(ho_ObjectSelected, &ho_Contours, "center");
//        HalconCpp::IntersectionCircleContourXld(ho_Contours, 500, 500, hv_DistanceMin, 0, 6.28318,
//            "positive", &hv_Row, &hv_Column);
//        HalconCpp::GenCrossContourXld(&ho_Cross, hv_Row, hv_Column, 6, 0.785398);
//
//        HalconCpp::TupleLength(hv_Row, &hv_Num_cross);
//        if (0 != (int(hv_Num_cross == 0)))
//        {
//            continue;
//        }
//        //������ˮƽ��x��ļн�
//        HalconCpp::AngleLx(500, 500, HalconCpp::HTuple(hv_Row[0]), HalconCpp::HTuple(hv_Column[0]), &hv_Angle);
//        HalconCpp::TupleDeg(hv_Angle, &hv_Deg);
//        hv_angle_min_x[hv_Index - 1] = hv_Deg;
//        //������y��������ļн�
//        if (0 != (int(hv_Deg <= 90)))
//        {
//            hv_angle_min_y[hv_Index - 1] = 90 - hv_Deg;
//        }
//        else
//        {
//            hv_angle_min_y[hv_Index - 1] = -(hv_Deg - 90);
//        }
//
//    }
//    // �ڴ��������ӷ��ؽ���Ĳ���
//    Result result;
//    if (hv_angle_min_y.Length() > 0) {
//        result.distance = hv_DistanceMin[0].D() * 20.0; // Assuming hv_DistanceMin is HTuple of distances
//        result.theta = hv_angle_min_y[0].D(); // Assuming hv_angle_min_y is HTuple of angles
//    }
//
//    return result;
//
//}
//
//int main(int argc, char* argv[]) {
//    // ��ȡͼ��
//    HObject ho_Image;
//
//    ReadImage(&ho_Image, "E:\\DOCUMENTS\\lidar\\pic\\pic\\lidar_image.45.bmp");
//
//
//    // ����ͼ��
//    Result result = dealImage(ho_Image);
//
//    // ������
//    std::cout << "Distance: " << result.distance << ", Theta: " << result.theta << std::endl;
//
//    return 0;
//}