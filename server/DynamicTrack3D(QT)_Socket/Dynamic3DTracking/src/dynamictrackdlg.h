#ifndef DYNAMICTRACKDLG_H
#define DYNAMICTRACKDLG_H

//qt header
#include <QDialog>
#include <QFileDialog>
#include <QTimer>
#include <QMessageBox>
#include <QDebug>

//外部库
#include "GBK.h"
#include "ClpMeasurement.h"
#include"XMLReader.h"
#include "StdAfx.h"
#include "Condensation.h"
#include<stdio.h>
#include "winSocketClient.h"
#include<fstream>

//opencv库
#include <opencv2/opencv.hpp>

//核心代码库
#include "SharedHead.h"
#include "SharedMethod.h"
#include "ui_dynamictrackdlg.h"

#include"stdafx.h"
#include "CamCapThread.h"
#include "dynamic3dtracking.h"

using namespace std;


class DynamicTrackDlg : public QDialog
{
	Q_OBJECT

public:
	DynamicTrackDlg(QWidget *parent = 0);
	~DynamicTrackDlg();

private slots:
    //设置跟踪的图片路径
    void on_btnSetTrackPath_clicked();
    //开始跟踪编码点
    void on_btnToTrack_clicked();
	//停止跟踪编码点
    void on_btnStopTracking_clicked();


public:
	//将Mat转换成QImage的函数
	QImage cvMat2QImage(const cv::Mat& mat);
	//将QImage转换成Mat的函数
	cv::Mat QImage2cvMat(QImage image);

	//实现将TagPoint3f  CodeTagPnts3D的数据转化成QString类型，便于lineEdit输出
	QString TagPoint3f2QString(const TagPoint3f CodeTagPnts3D);

	//添加定时器
	void timerEvent(QTimerEvent *event);

	//变形测量新增的函数，函数名均以DM结尾
	void getMessurePara(const std::string CameraParaPathleft, const std::string CameraParaPathright, const std::string camgroupPara, const std::string LightPenParaPath, const std::string LightpenFeatures);
	bool getCodes3DCoordinateDM(const Mat imgLeft, const Mat imgRight, bool &FirstFrameOkOrNot);

	bool getCodes3DCoordinateDM(const Mat imgLeft, const Mat imgRight);
	//通过传入的参数标志位来判断该图片是否为初始帧

	bool findCodesOriPos2DDM(const Mat imgLeft, const Mat imgRight, vector<pair<unsigned int, Point2f>>  &markerOriLeft, vector<pair<unsigned int, Point2f>> &markerOriRight,
		float &Code_Diameter, int &flag);
	bool TrackALLCodesDM(const Mat imgLeft, const Mat imgRight, vector<pair<Mat, Mat>> &measurementVec,
		vector<pair<Point2f, Point2f>>  &markerCenterVec, int flag_Num, float &Cir_Diameter, vector<unsigned int> Flags_Code, vector<TagPoint3f> &CodeTagPnts3D);
	bool findCircularMarkerDM(const Mat img, vector<pair<unsigned int, Point2f>> &results_center);
	bool findCircularMarkerDM(const Mat img, const cv::Rect Mask, vector<pair<unsigned int, Point2f>> &results_center);
	bool findCircularMarkerDM(const Mat img, const cv::Rect Mask, vector<pair<unsigned int, Point2f>> &results_center, float &Code_Diameter);
	bool findCircularMarkerDM(const Mat img, const cv::Rect Mask, vector<pair<unsigned int, Point2f>> &results_center, int flag);
	bool UpdataPositionDM(const vector<pair<Point2f, Point2f>> markerCenterVec,  //Input
		vector<pair<Mat, Mat>>  &measurementVec);   //Output
	//将检测出的编码点地标志位存放到一个容器中，便于调用
	void getCodeFlags(const vector<pair<unsigned int, Point2f>> markerOriLeft, vector<unsigned int> &CodeFlags);
	bool SetInitialPositionDM(
		const vector<pair<unsigned int, Point2f>>  markerOriLeft, const vector<pair<unsigned int, Point2f>> markerOriRight,
		vector<pair<Mat, Mat>> &measurementVec);
	//////对于编码点进行三维重建。
	bool  CalculateCodeCenter3D(vector<pair<unsigned int, Point2f>> LeftmarkerResults,
		vector<pair<unsigned int, Point2f>> RightmarkerResults, vector<TagPoint3f>& CodeTagPnts3f);

	////运用粒子滤波进行单个编码点的跟踪
	bool TrackCodeTrace(const Mat img, Mat &measurement, Point2f &markerCenterPnt, int flag, float &Cir_Diameter);
	////运用粒子滤波跟踪自定义个数的编码点
	bool TrackCodeTraceALL(const Mat imgLeft, const Mat imgRight, vector<pair<Mat, Mat>> &measurementVec, vector<pair<Point2f, Point2f>>  &markerCenterVec, int flag_Num, float &Cir_Diameter, vector<unsigned int> Flags_Code, vector<TagPoint3f> &CodeTagPnts3D);
	//生成随机数函数
	double Rand(double dMin, double dMax);
	//ended
private:
	Ui::DynamicTrackDlg ui;
	//计时器
	QTimer *timerTrack;

	//左右相机拍摄照片的保存路径(L,R文件夹的上一层路径)
	QString imgPathL2, imgPathR2;

	//定义两个容器存放左右拍摄好的图片
	vector<cv::Mat> vecImgL2, vecImgR2;

	//跟踪图像文件列表
	QStringList imgNameListL, imgNameListR;

	//用于存储当前显示图像pixmap
	QImage *qimageL, *qimageR;

	//判断图片是否为初始帧
	bool FirstImgOrNot;

	//图像的帧数，用于标记正在处理的图像序号
	int imgNumBeDealed;

	//定时器ID
	int m_timerId;  
	
public:
	CamPara camparaleft;
	CamPara campararight;
	CamGroupPara camGroupPara;
	LightPenPara lightpenPara;
	std::vector<TagPoint3f> lightpenfeatures;

	OPERATIONERROR err;

	//粒子滤波使用过程中，需要将检测的编码点中心坐标存放在Mat中
	vector<pair<Mat, Mat>> measurementVec;

	//此处将编码点的标志位数值存放到一个容器中，以便后期调用。
	vector<unsigned int>   Flags_Code;

	//存放初始的编码点位置；
	vector<pair<unsigned int, Point2f>>  markerOriLeft, markerOriRight;

	//成像的图片中编码点直径
	float Cir_Diameter_Code;

	//将编码点的初始位置赋给预判位置
	pair<Mat, Mat> measurementPair;

	//存放检测到的编码点的三维坐标
	vector<TagPoint3f> CodeTagPnts3D;

	//保存每一帧图片中检测出的编码点坐标,以便传给“显示位移”对话框
	vector<pair<int, vector<TagPoint3f>>>  CodePosVec;


	//存放左右编码点的中心坐标
	vector<pair<Point2f, Point2f>>  markerCenterVec;

	//设置编码点的数目
	int FlagNum;

	//设置一个标志位 判断输入图片是否为初始帧
	bool FirstFrameOkOrNot;

};

#endif // DYNAMICTRACKDLG_H
