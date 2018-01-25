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
	//读入相机标定参数及光笔标定参数
	//输入两个路径
	//输出相机参数和光笔标定参数 
	void getMessurePara(const std::string CameraParaPathleft ,const std::string CameraParaPathright, const std::string camgroupPara,const std::string LightPenParaPath,const std::string LightpenFeatures);
	void getMeasureData(Mat imageleft,Mat imageright);
	void dataOutput(SProbePosition2& _positionOut);
	////qiyong added
	
	/////重载getCodepntsPoseToCam,输出初始位置的光笔中心，各编码点中心，以及编码点相对于相机坐标系的转换

	//////将左右图像存放到关联容器中。
	bool PutImgsToImgVec(vector<pair<Mat, Mat>>  &m_ImgVec,int m_Num);
	/////寻找编码点在二维平面初始位置。
	bool findCodeOriginalPosition2D(vector<pair<Mat, Mat>>  m_ImgVec, vector<pair<unsigned int, Point2f>> &markerResultsLeft, vector<pair<unsigned int, Point2f>> &markerResultsRight,int &m_OriNum,float &Cir_Diameter_Code);
	bool findCodeOriginalPosition2D(vector<pair<Mat, Mat>>  m_ImgVec, vector<pair<unsigned int, Point2f>> &markerResultsLeft, vector<pair<unsigned int, Point2f>> &markerResultsRight,int &m_OriNum, float &Cir_Diameter_Code,int flag);
	/////寻找光笔在二维平面中的初始位置

	////运用粒子滤波进行单个光笔的跟踪

	/////对于光笔进行三维重建。

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

