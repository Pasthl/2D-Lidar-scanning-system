#include "LidarControl.h"
#include "ImageProcess.h"

sl::ILidarDriver* drv = *sl::createLidarDriver();

int main()
{
	LidarInfo Result;
	int scale_num = 20;
	int window_height = 10000 / scale_num;//图像的长宽定义
	int window_width = 20000 / scale_num;

	HalconCpp::SetSystem("use_window_thread", "true"); // 不加这一行Halcon窗口是白屏

	// 创建Halcon窗口
	HalconCpp::HWindow window(0, 0, window_width, window_height, 0, "visible", "Lidar");

	window.SetPart(0, 0, window_height - 1, window_width - 1); // 设置窗口的显示部分
	window.SetDraw("fill"); // 设置绘图模式
	// 设置窗口背景色为白色
	window.SetWindowParam("background_color", "white");
	window.ClearWindow();  // 清除窗口内容，应用背景颜色

	LidarStart(drv);
	//测试时候有i 可以把i<100改为True
	int i = 0;
	while (i < 100)
	{
		GetData(drv, Result, window);
		std::cout << "-------------------------------------------------------------------------------" << std::endl;
		std::cout << "距离:" << Result.distance << std::endl; //获取结果
		std::cout << "角度:" << Result.theta << std::endl;
		std::cout << "距离左侧:" << Result.distanceLeft << std::endl;
	    std::cout << "距离右侧:" << Result.distanceRight << std::endl;
		std::cout << "-------------------------------------------------------------------------------" << std::endl;
		i++;
	}
	LidarStop(drv);
	system("pause");
	return 0;
}