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
#define SQUARE_SIZE 25.047899689			//<棋盘格方格尺寸
#define WIDTH_CORNER_COUNT 17		//<长边角点个数
#define HEIGHT_CORNER_COUNT 8		//<短边角点个数
#define CAMERA_NUM 2				//<操作的相机数量
////chengwei added
#define HEADRADIUS 0.99999f;
//光笔测头半径，用作半径补偿


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
	//以下两个需要opencv3.0以上版本才支持
	PNP_DLS = 3, //Method is based on the paper of Joel A. Hesch and Stergios I. Roumeliotis. “A Direct Least-Squares (DLS) Method for PnP”.
	PNP_UPNP = 4 //Method is based on the paper of A.Penate-Sanchez, J.Andrade-Cetto, F.Moreno-Noguer. 
	//“Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation”.
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
	double CameraIntrinsic[3][3];	//<相机内参数
	double DistortionCoeffs[4];		//<相机畸变参数
	std::vector<RT> imgRTVec;		//<标定板外参数
	Mat parallelCamR;					//<平行相机视图的R
	double fcError[2];
	double ccError[2];
	double kcError[4];
	double ReprojectionError[2];		//<重投影误差标准差
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
	int imgHeight;								//<图像的高
	int imgWidth;								//<图像的宽
	vector<int> frameNumList;					//<图像的索引
	//std::vector<int> CornerNumPerFrame;       //<每一帧图像角点个数
	vector<vector<Point3f>> plane3dPntsVec;		//<标定板三维平面坐标
	vector<vector<Point2f>> plane2dPntsVec;		//<标定板二维坐标
};
///////////////////////////////////////////////////chengwei added////////////////////////////////////////////////////////////////////
//包括每次测量的侧头中心坐标（测量坐标系下的）和测量表面的法线
struct CenterFeartures
{
	Point3f center1;
	Point3f center2;
	Point3f center3;
	SurfaceNormal feature;//测量表面的法线
};
//经过半径补偿的被测表面三个点的坐标（在测量坐标系下的）以及测量表面的法线
struct MessureResult
{
	Point3f point1;
	Point3f point2;
	Point3f point3;
	SurfaceNormal feature;//测量表面的法线
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
//光笔标定参数
struct LightPenPara
{
	vector<TagPoint3f> FeatursPoints;
	Point3f CenterPoint;
};
//平面模型
class Plane
{
public:
	SurfaceNormal normal;
	Point3f orignal;
};
//柱面模型
class CircleCylinder
{
public:
	SurfaceNormal axisNormal;//单位向量
	Point3f orignal;//位于轴线上的一点
	float r;//圆柱半径
};
class Line
{
public:
	SurfaceNormal normal;//方向向量
	Point3f orignal;//直线上一点

};

