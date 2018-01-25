#pragma once
#include "CamCapThread.h"
#include"dynamictrackdlg.h"

//#define EXTERNAL
#define SORFWARE
using namespace cv;

CamCapThread::CamCapThread(LRThread flag)
{
	runFlag = 1;
	if (flag == LRThread::L)
	{
		nameTem = "./triger/L/image";
		camera.ConnectToCamera(0);
	}
	else
	{
		nameTem = "./triger/R/image";
		camera.ConnectToCamera(1);
	}


}

CamCapThread::~CamCapThread()
{

}

void CamCapThread::stop()
{
	runFlag = 0;
}

void CamCapThread::run(LRThread flag, vector<Image> &qImg, int NumRetrieveBuffer)
{

	while (runFlag) 
	{
		char imgName[50];
		Image img;
		for (int i = 1; i < NumRetrieveBuffer; i++)
		{
			qImg.clear();
			//使用软触发
#ifdef EXTERNAL
			camera.GrabAPicture(img);
#endif //EXTERNAL
#ifdef SORFWARE
			camera.m_Cam.RetrieveBuffer(&img);
#endif //SORFWARE
			
			//将相机拍摄的Image类型的图片转换为Mat类型
			//cv::Mat image = cv::Mat(img.GetRows(), img.GetCols(), CV_8UC1, (void*)img.GetData());
			//将Mat转化为QPixmap
			//QPixmap pixmapL = Dynamic3DTracking::cvMat2QImage(image);
			
			
			//将外触发下拍摄的图片压入队列
			qImg.push_back(img);
			
		    //此处显示相机软触发拍摄好的图片
		   /*	cv::namedWindow("软触发拍摄好的图片", 2);
			cv::imshow("图片", image);
			waitKey();*/

			//将外触发下拍摄的图片存放到本地磁盘中
			sprintf(imgName,"%s%d%s",nameTem.data(),i,".bmp");
			img.Save(imgName);
			//将图片释放掉，线程等待一段时间。
			img.ReleaseBuffer();
			//QThread::msleep(7);
		}
		stop();
	}
	//这里需不需要手动去断开相机的连接呢？
	//camera.DisconnectCamera();
}








