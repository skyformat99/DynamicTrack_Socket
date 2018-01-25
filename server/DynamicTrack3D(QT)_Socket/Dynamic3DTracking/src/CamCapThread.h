#pragma once
#include<opencv2/opencv.hpp>
#include<queue>
#include<QThread>

//using namespace cv;

//������ͷ�ļ�
#include"pcamera.h"

//����ö�ٱ���
enum LRThread
{
	L,R
};

//����ⴥ�����㣬�����߳�
class CamCapThread :public QThread
{
	Q_OBJECT

public:
	CamCapThread(LRThread flag);
	~CamCapThread();

	PCamera camera;

	string nameTem;
	bool runFlag;

	
public:
	void stop();
	void run(LRThread flag, vector<Image> &qImg,int NumRetrieveBuffer);
};


