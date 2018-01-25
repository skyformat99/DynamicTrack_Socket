#pragma once
#include <string>
#include <vector>
#include <atlimage.h>
#include "CoreAlgorithm.h"
class ClpMeasurement
{
public:
	ClpMeasurement(void);
	~ClpMeasurement(void);
public:
	//��������궨��������ʱ궨����
	//��������·��
	//�����������͹�ʱ궨���� 
	void getMessurePara(const std::string CameraParaPathleft ,const std::string CameraParaPathright, const std::string camgroupPara,const std::string LightPenParaPath,const std::string LightpenFeatures);
	void getMeasureData(Mat imageleft,Mat imageright);
	void dataOutput(SProbePosition2& _positionOut);
	////qiyong added
	
	/////����getCodepntsPoseToCam,�����ʼλ�õĹ�����ģ�����������ģ��Լ������������������ϵ��ת��

	//////������ͼ���ŵ����������С�
	bool PutImgsToImgVec(vector<pair<Mat, Mat>>  &m_ImgVec,int m_Num);
	/////Ѱ�ұ�����ڶ�άƽ���ʼλ�á�
	bool findCodeOriginalPosition2D(vector<pair<Mat, Mat>>  m_ImgVec, vector<pair<unsigned int, Point2f>> &markerResultsLeft, vector<pair<unsigned int, Point2f>> &markerResultsRight,int &m_OriNum,float &Cir_Diameter_Code);
	bool findCodeOriginalPosition2D(vector<pair<Mat, Mat>>  m_ImgVec, vector<pair<unsigned int, Point2f>> &markerResultsLeft, vector<pair<unsigned int, Point2f>> &markerResultsRight,int &m_OriNum, float &Cir_Diameter_Code,int flag);
	/////Ѱ�ҹ���ڶ�άƽ���еĳ�ʼλ��

	////���������˲����е�����ʵĸ���

	/////���ڹ�ʽ�����ά�ؽ���

	bool calculateCodesCentroid(const  vector<pair<unsigned int, Point2f>>  markerResults, Point2f &centroidPnt);
	static bool calculateCentroid(const vector<Point2f> centerPnts, Point2f &LightPenPntsCentroid);
	bool calculateLightPenCentroid(const Mat img, const vector<pair<unsigned int, Point2f>>  markerResults, Point2f &LightPenCentroidPnt);
	bool calculateLightPenCentroid(const Mat img, const vector<pair<unsigned int, Point2f>>  markerResults, Point2f &LightPenCentroidPnt,float &CircleDiameter);
	static bool calculateLightPenCentroid(const Mat img, Point2f &LightPenCentroidPnt, float &CircleDiameter);
	static bool calculateLightPenCentroid(const Mat img, Point2f &LightPenCentroidPnt);

	
public:	
	bool dataGet;
	OPERATIONERROR err;
private:
	CamPara camparaleft;
	CamPara campararight;
	CamGroupPara camGroupPara;
	LightPenPara lightpenPara;
	std::vector<TagPoint3f> lightpenfeatures;

	SProbePosition2 _position;
private:
	cv::Mat hBitmap2Mat(HBITMAP hBmp);

public:



};

