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
			//ʹ������
#ifdef EXTERNAL
			camera.GrabAPicture(img);
#endif //EXTERNAL
#ifdef SORFWARE
			camera.m_Cam.RetrieveBuffer(&img);
#endif //SORFWARE
			
			//����������Image���͵�ͼƬת��ΪMat����
			//cv::Mat image = cv::Mat(img.GetRows(), img.GetCols(), CV_8UC1, (void*)img.GetData());
			//��Matת��ΪQPixmap
			//QPixmap pixmapL = Dynamic3DTracking::cvMat2QImage(image);
			
			
			//���ⴥ���������ͼƬѹ�����
			qImg.push_back(img);
			
		    //�˴���ʾ�����������õ�ͼƬ
		   /*	cv::namedWindow("��������õ�ͼƬ", 2);
			cv::imshow("ͼƬ", image);
			waitKey();*/

			//���ⴥ���������ͼƬ��ŵ����ش�����
			sprintf(imgName,"%s%d%s",nameTem.data(),i,".bmp");
			img.Save(imgName);
			//��ͼƬ�ͷŵ����̵߳ȴ�һ��ʱ�䡣
			img.ReleaseBuffer();
			//QThread::msleep(7);
		}
		stop();
	}
	//�����費��Ҫ�ֶ�ȥ�Ͽ�����������أ�
	//camera.DisconnectCamera();
}








