#pragma once
#include<opencv2/opencv.hpp>
#include<queue>
#include<QThread>

//using namespace cv;

//工程内头文件
#include"pcamera.h"

//定义枚举变量
enum LRThread
{
	L,R
};

//相机外触发拍摄，开辟线程
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


