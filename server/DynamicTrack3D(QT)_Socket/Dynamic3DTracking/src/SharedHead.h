#pragma once
#include "opencv2/opencv.hpp"
#include <string>
#include <vector>
#include "./TinyXML/tinyxml.h"
#include "./TinyXML/tinystr.h"
#include <iostream>

//#include "opencv.hpp"

using namespace std;
using cv::Mat;
using cv::Vec4f;
using cv::Point3f;
using cv::Vec3f;
using cv::Point2f;
using cv::Size;
using cv::Point;
using cv::Scalar;
using cv::RotatedRect;
//28.152,12,7
//26.928 11 8
//37.2 11 5
#define SQUARE_SIZE 25.047899689			//<���̸񷽸�ߴ�
#define WIDTH_CORNER_COUNT 17		//<���߽ǵ����
#define HEIGHT_CORNER_COUNT 8		//<�̱߽ǵ����
#define CAMERA_NUM 2				//<�������������
////chengwei added
#define HEADRADIUS 0.99999f;
//��ʲ�ͷ�뾶�������뾶����


typedef Vec3f TagPoint2f;
typedef Vec4f TagPoint3f;
typedef Point3f SurfaceNormal;

enum PointTag
{
	TAG1,
	TAG2,
	TAG3,
	TAG4,
	TAG5,
	TAG6,
	TAG7,
	TAG8,
	TAG9,
	TAG10,
	TAG11,
	TAG12,
	TAG13,
	TAG14,
	TAG15


};

enum CalibrationMode
{
	SINGLE_CAM,
	DOUBLE_CAM,
	TRIPLE_CAM
};

enum CamPosition
{
	LEFT,
	MID,
	RIGHT
};
//chengwei added
enum
{
    PNP_ITERATIVE = 0,
    PNP_EPNP = 1, // F.Moreno-Noguer, V.Lepetit and P.Fua "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
    PNP_P3P = 2, // X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
	//����������Ҫopencv3.0���ϰ汾��֧��
	PNP_DLS = 3, //Method is based on the paper of Joel A. Hesch and Stergios I. Roumeliotis. ��A Direct Least-Squares (DLS) Method for PnP��.
	PNP_UPNP = 4 //Method is based on the paper of A.Penate-Sanchez, J.Andrade-Cetto, F.Moreno-Noguer. 
	//��Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation��.
	//In this case the function also estimates the parameters f_x and f_y assuming that both have the same value. 
	//Then the cameraMatrix is updated with the estimated focal length.
};
enum CAMBEHAVIOR
{
	CAMCALIBRATON,
	LIGHTPENCALIBRATION,
	MESSURE
};
enum OPERATIONERROR
{
	OPERATION_OK,
	READ_MEASUREMENT_PARA_ERROR,
	RECOGNITION_ERROR,
	RE_ERROR
};
enum MESMODEL
{
	MESSINGLEPOINT,
	MESPLANEFIRST,
	MESPLANESECOND,
	MESCIRCLECYLINDER,
	MESPOINTTOPOINT
};
//chengwei added
struct RT
{
	cv::Mat R; //<3X1
	cv::Mat T; //<3X1
};

struct ImageSaveDir
{
	string calibrateCamDir;
	string calibrateProbeDir;
	string probePntDir;
};

class Mat34 
{
public:
	Mat34()
	{
		for(int i=0;i<3;i++)
		{
			for(int j=0;j<4;j++)
			{
				mat[i][j] = 0;
			}
		}
	}
	double mat[3][4];
};

class CamPara 
{ 
public:
	CamPara()
	{
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
				CameraIntrinsic[i][j]=0;
		CameraIntrinsic[2][2] = 1;
		for(int i=0;i<4;i++) {DistortionCoeffs[i]=0;kcError[i]=0;}
		ReprojectionError[0]=0;ReprojectionError[1]=0;
		fcError[0]=0;fcError[1]=0;
		ccError[0]=0;ccError[1]=0;
	}
	double CameraIntrinsic[3][3];	//<����ڲ���
	double DistortionCoeffs[4];		//<����������
	std::vector<RT> imgRTVec;		//<�궨�������
	Mat parallelCamR;					//<ƽ�������ͼ��R
	double fcError[2];
	double ccError[2];
	double kcError[4];
	double ReprojectionError[2];		//<��ͶӰ����׼��
	std::vector<double> reprojectNormErr;
	double totalReproNormErr;
};

class CamGroupPara
{
public:
	CamGroupPara()
	{
		for(int i=0;i<3;i++)
		{
			left2MidRotVector[i] = 0;
			left2MidTraVector[i] = 0;
			right2MidRotVector[i] = 0;
			right2MidTraVector[i] = 0;
			right2LeftRotVector[i] = 0;
			right2LeftTraVector[i] = 0;
		}
	}

public:
	double left2MidRotVector[3];
	double left2MidTraVector[3];
	double right2MidRotVector[3];
	double right2MidTraVector[3];
	double right2LeftRotVector[3];
	double right2LeftTraVector[3];
};

struct CalibrationData
{
	int imgHeight;								//<ͼ��ĸ�
	int imgWidth;								//<ͼ��Ŀ�
	vector<int> frameNumList;					//<ͼ�������
	//std::vector<int> CornerNumPerFrame;       //<ÿһ֡ͼ��ǵ����
	vector<vector<Point3f>> plane3dPntsVec;		//<�궨����άƽ������
	vector<vector<Point2f>> plane2dPntsVec;		//<�궨���ά����
};
///////////////////////////////////////////////////chengwei added////////////////////////////////////////////////////////////////////
//����ÿ�β����Ĳ�ͷ�������꣨��������ϵ�µģ��Ͳ�������ķ���
struct CenterFeartures
{
	Point3f center1;
	Point3f center2;
	Point3f center3;
	SurfaceNormal feature;//��������ķ���
};
//�����뾶�����ı����������������꣨�ڲ�������ϵ�µģ��Լ���������ķ���
struct MessureResult
{
	Point3f point1;
	Point3f point2;
	Point3f point3;
	SurfaceNormal feature;//��������ķ���
};
struct SProbePosition2
{
	SProbePosition2() :  dX( FLT_MAX ), 
		dY( FLT_MAX ), 
		dZ( FLT_MAX ),
		dZI( FLT_MAX ), 
		dZJ( FLT_MAX ), 
		dZK( FLT_MAX )
	{}

	SProbePosition2( double positionX_, double positionY_, double positionZ_, 
		double directionZI_, double directionZJ_, double directionZK_ ) : 
	dX( positionX_ ), 
		dY( positionY_ ), 
		dZ( positionZ_ ),
		dZI( directionZI_ ), 
		dZJ( directionZJ_ ), 
		dZK( directionZK_ )
	{}

	double dX, dY, dZ;      // probe origin
	double dZI, dZJ, dZK;   // probe z-axis vector
};
//��ʱ궨����
struct LightPenPara
{
	vector<TagPoint3f> FeatursPoints;
	Point3f CenterPoint;
};
//ƽ��ģ��
class Plane
{
public:
	SurfaceNormal normal;
	Point3f orignal;
};
//����ģ��
class CircleCylinder
{
public:
	SurfaceNormal axisNormal;//��λ����
	Point3f orignal;//λ�������ϵ�һ��
	float r;//Բ���뾶
};
class Line
{
public:
	SurfaceNormal normal;//��������
	Point3f orignal;//ֱ����һ��

};

