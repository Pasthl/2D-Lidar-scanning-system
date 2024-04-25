#include "LidarControl.h"
#include "ImageProcess.h"

sl::ILidarDriver* drv = *sl::createLidarDriver();

int main()
{
	LidarInfo Result;
	int scale_num = 20;
	int window_height = 10000 / scale_num;//ͼ��ĳ�������
	int window_width = 20000 / scale_num;

	HalconCpp::SetSystem("use_window_thread", "true"); // ������һ��Halcon�����ǰ���

	// ����Halcon����
	HalconCpp::HWindow window(0, 0, window_width, window_height, 0, "visible", "Lidar");

	window.SetPart(0, 0, window_height - 1, window_width - 1); // ���ô��ڵ���ʾ����
	window.SetDraw("fill"); // ���û�ͼģʽ
	// ���ô��ڱ���ɫΪ��ɫ
	window.SetWindowParam("background_color", "white");
	window.ClearWindow();  // ����������ݣ�Ӧ�ñ�����ɫ

	LidarStart(drv);
	//����ʱ����i ���԰�i<100��ΪTrue
	int i = 0;
	while (i < 100)
	{
		GetData(drv, Result, window);
		std::cout << "-------------------------------------------------------------------------------" << std::endl;
		std::cout << "����:" << Result.distance << std::endl; //��ȡ���
		std::cout << "�Ƕ�:" << Result.theta << std::endl;
		std::cout << "�������:" << Result.distanceLeft << std::endl;
	    std::cout << "�����Ҳ�:" << Result.distanceRight << std::endl;
		std::cout << "-------------------------------------------------------------------------------" << std::endl;
		i++;
	}
	LidarStop(drv);
	system("pause");
	return 0;
}