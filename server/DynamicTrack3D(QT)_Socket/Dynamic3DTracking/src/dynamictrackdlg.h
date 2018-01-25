#ifndef DYNAMICTRACKDLG_H
#define DYNAMICTRACKDLG_H

//qt header
#include <QDialog>
#include <QFileDialog>
#include <QTimer>
#include <QMessageBox>
#include <QDebug>

//�ⲿ��
#include "GBK.h"
#include "ClpMeasurement.h"
#include"XMLReader.h"
#include "StdAfx.h"
#include "Condensation.h"
#include<stdio.h>
#include "winSocketClient.h"
#include<fstream>

//opencv��
#include <opencv2/opencv.hpp>

//���Ĵ����
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
    //���ø��ٵ�ͼƬ·��
    void on_btnSetTrackPath_clicked();
    //��ʼ���ٱ����
    void on_btnToTrack_clicked();
	//ֹͣ���ٱ����
    void on_btnStopTracking_clicked();


public:
	//��Matת����QImage�ĺ���
	QImage cvMat2QImage(const cv::Mat& mat);
	//��QImageת����Mat�ĺ���
	cv::Mat QImage2cvMat(QImage image);

	//ʵ�ֽ�TagPoint3f  CodeTagPnts3D������ת����QString���ͣ�����lineEdit���
	QString TagPoint3f2QString(const TagPoint3f CodeTagPnts3D);

	//��Ӷ�ʱ��
	void timerEvent(QTimerEvent *event);

	//���β��������ĺ���������������DM��β
	void getMessurePara(const std::string CameraParaPathleft, const std::string CameraParaPathright, const std::string camgroupPara, const std::string LightPenParaPath, const std::string LightpenFeatures);
	bool getCodes3DCoordinateDM(const Mat imgLeft, const Mat imgRight, bool &FirstFrameOkOrNot);

	bool getCodes3DCoordinateDM(const Mat imgLeft, const Mat imgRight);
	//ͨ������Ĳ�����־λ���жϸ�ͼƬ�Ƿ�Ϊ��ʼ֡

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
	//�������ı����ر�־λ��ŵ�һ�������У����ڵ���
	void getCodeFlags(const vector<pair<unsigned int, Point2f>> markerOriLeft, vector<unsigned int> &CodeFlags);
	bool SetInitialPositionDM(
		const vector<pair<unsigned int, Point2f>>  markerOriLeft, const vector<pair<unsigned int, Point2f>> markerOriRight,
		vector<pair<Mat, Mat>> &measurementVec);
	//////���ڱ���������ά�ؽ���
	bool  CalculateCodeCenter3D(vector<pair<unsigned int, Point2f>> LeftmarkerResults,
		vector<pair<unsigned int, Point2f>> RightmarkerResults, vector<TagPoint3f>& CodeTagPnts3f);

	////���������˲����е��������ĸ���
	bool TrackCodeTrace(const Mat img, Mat &measurement, Point2f &markerCenterPnt, int flag, float &Cir_Diameter);
	////���������˲������Զ�������ı����
	bool TrackCodeTraceALL(const Mat imgLeft, const Mat imgRight, vector<pair<Mat, Mat>> &measurementVec, vector<pair<Point2f, Point2f>>  &markerCenterVec, int flag_Num, float &Cir_Diameter, vector<unsigned int> Flags_Code, vector<TagPoint3f> &CodeTagPnts3D);
	//�������������
	double Rand(double dMin, double dMax);
	//ended
private:
	Ui::DynamicTrackDlg ui;
	//��ʱ��
	QTimer *timerTrack;

	//�������������Ƭ�ı���·��(L,R�ļ��е���һ��·��)
	QString imgPathL2, imgPathR2;

	//�����������������������õ�ͼƬ
	vector<cv::Mat> vecImgL2, vecImgR2;

	//����ͼ���ļ��б�
	QStringList imgNameListL, imgNameListR;

	//���ڴ洢��ǰ��ʾͼ��pixmap
	QImage *qimageL, *qimageR;

	//�ж�ͼƬ�Ƿ�Ϊ��ʼ֡
	bool FirstImgOrNot;

	//ͼ���֡�������ڱ�����ڴ����ͼ�����
	int imgNumBeDealed;

	//��ʱ��ID
	int m_timerId;  
	
public:
	CamPara camparaleft;
	CamPara campararight;
	CamGroupPara camGroupPara;
	LightPenPara lightpenPara;
	std::vector<TagPoint3f> lightpenfeatures;

	OPERATIONERROR err;

	//�����˲�ʹ�ù����У���Ҫ�����ı����������������Mat��
	vector<pair<Mat, Mat>> measurementVec;

	//�˴��������ı�־λ��ֵ��ŵ�һ�������У��Ա���ڵ��á�
	vector<unsigned int>   Flags_Code;

	//��ų�ʼ�ı����λ�ã�
	vector<pair<unsigned int, Point2f>>  markerOriLeft, markerOriRight;

	//�����ͼƬ�б����ֱ��
	float Cir_Diameter_Code;

	//�������ĳ�ʼλ�ø���Ԥ��λ��
	pair<Mat, Mat> measurementPair;

	//��ż�⵽�ı�������ά����
	vector<TagPoint3f> CodeTagPnts3D;

	//����ÿһ֡ͼƬ�м����ı��������,�Ա㴫������ʾλ�ơ��Ի���
	vector<pair<int, vector<TagPoint3f>>>  CodePosVec;


	//������ұ�������������
	vector<pair<Point2f, Point2f>>  markerCenterVec;

	//���ñ�������Ŀ
	int FlagNum;

	//����һ����־λ �ж�����ͼƬ�Ƿ�Ϊ��ʼ֡
	bool FirstFrameOkOrNot;

};

#endif // DYNAMICTRACKDLG_H
