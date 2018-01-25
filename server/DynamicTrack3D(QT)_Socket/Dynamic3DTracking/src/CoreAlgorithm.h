#pragma once
#include "SharedHead.h"
//#include "XMLWriter.h"
#include<iostream>
#include <string>
#include <list>
#include <vector>
#include <map>
#include <stack>
//////

//#define BLUR
#define CANNY
//#define THRESHOL

class CoreAlgorithm
{
public:
	///使用迭代方法进行单个相机的标定
	///@param imgsPath 输入 标定图像的绝对路径
	///@param cData 输入 标定用数据
	///@param camParaSrc 输入 由张正友计算出的相机参数初始值
	///@param camParaDst 输出 迭代后计算后获得的相机参数
	static void iterativeCameraCalibration(const vector<string>& imgsPath,const CalibrationData& cData,
		const CamPara& camParaSrc, CamPara& camParaDst);

	///获得畸变矫正后的图像
	///@param src 输入 标定板原始图像
	///@param dst 输出 标定板畸变校正后的图像
	///@param cameraMatrix 输入 相机内参数
	///@param distCoeffs 输入 相机畸变参数
	///@param R 3x3输入 如果输入单位矩阵，则仅输出畸变校正后的图像
	///					如果输入标定板平行视图的R，则输出矫正后的平行视图图像
	///// 2014.9.29  zhang xu undated
	//// @param  const Mat& new_cameraMatrix, 输入相机新的内参数
	static void undistortImg(const Mat& src, Mat& dst,
		const Mat& cameraMatrix, const vector<double>& distCoeffs, const Mat& R);

	///将某一相机图像坐标系下的标定板坐标转换到另一相机图像图像坐标系下
	///@param src 输入 标定板原始视图上的特征点2d
	///@param dst 输出 标定板平行视图上的特征点2d
	///@param cameraMatrix12 输入 相机内参数
	///@param distCoeffs12 输入 相机畸变参数
	///@param R12 输入 标定板R 3x1
	///@param T12 输入 标定板T 3x1 
	static void undistortPoints2DifferentView(const vector<Point2f>& src, vector<Point2f>& dst,
		const Mat& cameraMatrix1, const Mat& R1, const Mat& T1, const vector<double>& distCoeffs1,
		const Mat& cameraMatrix2, const Mat& R2, const Mat& T2, const vector<double>& distCoeffs2);

	///采用Random算法找出在长轴和短轴上的点,十字交叉点属于长轴
	///长轴以y坐标排序，从十字交叉处开始；短轴按照光笔正方时十字交叉左右排序
	///输入点的要求：1、十字叉形状;2、至少6个点
	///@param pnts 输入 至少6个点
	///@param L_pnts 输出 长轴点
	///@param S_pnts 输出 短轴点
	static bool pointsSort2D_3(vector<Point2f>& pnts,vector<Point2f>& L_pnts,vector<Point2f>& S_pnts);

	///根据光笔的具体形状（十字交叉），给输入的点加上标签
	///输入点条件：1、十字叉形状;2、长短轴加起来至少6个点
	///@param L_pnts 输入 长轴点，至少4个点
	///@param S_pnts 输入 短轴点，至少1个点
	///@param tagPnts 输出 带有标签的光斑点坐标,Vec3f 第一个数为标签，后两个数为点坐标 
	/// 这个函数调用之前，已经使用了函数pointsSort2D_3， 保证了排序是正确的，只是对于其中是否会出现漏点未考虑，也就是说，只要点是完全的，那
	/// 按照pointsSort2D_3的结果进行tag 就是正确的
	static bool getPntsWithTag(const vector<Point2f>& L_pnts,const vector<Point2f>& S_pnts,vector<TagPoint2f>& tagPnts);
	//chengwei added
	//根据检测出的轮廓中心点，找出需要的特征点并标出相应序号
	//输入：检测出的轮廓中心点；每个轮廓拟合椭圆的长轴半径,gamma代表圆斑距离除以圆斑半径
	//输出：带有标签信息，已排好序的特征点	一号点的圆斑区域宽度
	static bool findPntsWithTag(vector<Point2f>& centerPnt,vector<float>& longaxisRadius,vector<TagPoint2f>& tagPnts,float &firstFeaturelength,const double gamma);
	
	///红外LED光斑提取算法
	static void detectLightSpot_LED_3(const Mat imgMat, vector<Point2f>& centerPnt, vector<float>& longaxisRadius);
	static bool detectLightSpot_LED2(const Mat imgMat, vector<Point2f>& centerPnt, vector<float>& longaxisRadius);
	static bool detectLightSpot_LED(const Mat& imgMat, vector<Point2f>& centerPnt, vector<float>& longaxisRadius);
	
	///张正友相机标定方法
	///@param cData 输入 标定数据
	///@param campara 输出 相机标定参数
	static void ZhangCalibrationMethod_CPP( const CalibrationData& cData,CamPara& campara);

	///立体标定方法,相机1为主相机
	///@param camPara12 输入 单个相机标定参数
	///@param CalibrationData12 输入 单个相机标定数据
	///@param rt 输出 两相机相对位姿关系
	static void stereoCalibration(const CamPara& camPara1,const CamPara& camPara2,
		const CalibrationData& cornerDataVec1,const CalibrationData& cornerDataVec2,RT& rt);

	///判断一个int形是否存在于vector中
	static bool isBelong2Vec(vector<int> vec,int index);

	///三维重建
	//// 方案0  这个有问题，必须得重新整理一下
	/// 针对结构光系统，投影机只有一个位置对应信息，例如对应性上只有x坐标，没有y坐标
	/// 因为本系统中恢复三维形状，得到的是投影图像x坐标，宽度方向的，只有一个坐标没办法解算畸变模型，所以本函数计算三维点
	///   投影机假设没有畸变，或者对投影机畸变已经校正考虑了， 摄像机可以有畸变。以后再编码时可以考虑把编码图像编制在矫正后的图像上，不过就现在看来，线性模型精度已经够了。
	///   const vector<Point2f>& pointCam,<in> camera image point
	///   const vector<float>& proImg_x <in> the corresponding image point in the projector image. the model of the  projector is linear,no lens distortion 
	///   vector<Point3f>& point3d <out>  the point cloud in the coordinate of camera
	///   const CamPara& camParaLeft, 摄像机内参数
	///   const CamPara& camParapro， 投影机内参数
	///   const double rotVector[3] ,const double traVector[3] ,  投影机相对与摄像机的外参数
	///   */
	static bool Cal3dPointStrLight(const vector<Point2f>& pointCam,const vector<float>& proImg_x,const CamPara& camParaLeft, const CamPara& camParapro,const double rotVector[3] ,const double traVector[3] ,vector<Point3f>& point3d);

	
	//方案一：
	///// 根据左右相机的对应性信息，内外参数值，对畸变进行校正，然后计算三维点
	/// const vector<Point2f> pointLeft <in>, 左相机的对应点信息，要求x坐标，y坐标都有
	/// const CamPara& camParaLeft  <in>  左相机的内参数，
	/// const vector<Point2f> pointRight <in>  右相机的对应点信息，要求x坐标，y坐标都有
	/// const CamPara& camParaRight <in> 右相机的内参数
	/// const double rotVector[3] <in>  右相机相对左相机的旋转向量
	/// const double traVector[3] <in>  右相机相对左相机的平移向量 
	/// vector<Point3f>& point3d  <out>  左相机坐标系下的三维点
	static bool Cal3dPoint( const vector<Point2f> pointLeft ,const CamPara& camParaLeft 
					,const vector<Point2f> pointRight ,const CamPara& camParaRight 
					,const double rotVector[3] ,const double traVector[3] ,vector<Point3f>& point3d );

	///方案二：opencv的triangulatePoints函数
	//// 根据左右相机的对应性信息，内外参数值，对畸变进行校正，然后计算三维点
	//// const vector<Point2f>& pnts2d1, <in> 左相机的对应点信息，要求x坐标，y坐标都有
	/// const vector<Point2f>& pnts2d2, <in> 右相机的对应点信息，要求x坐标，y坐标都有
	/// const Mat& cameraMatrix1,  <in>  左相机的内参数
	/// const Mat& R1, <in> 左相机相对世界坐标系的旋转矩阵，若世界坐标系建立在摄像机坐标1 上，其为单位矩阵
	/// const Mat& T1, <in> 左相机相对世界坐标系的平移向量，若是世界坐标系建立在摄像机坐标1上，其为0矩阵
	/// const vector<double>& distCoeffs1, <in>
	/// const Mat& cameraMatrix2, <in>  右相机的内参数
	/// const Mat& R2, <in> 右相机相对世界坐标系的旋转矩阵，若世界坐标系建立在摄像机坐标系1上，其为相对于摄像机1的旋转矩阵
	/// const Mat& T2, <in> 右相机相对世界坐标系的平移矩阵，若世界坐标系建立在摄像机坐标系1上，其为相对于摄像机1的平移矩阵
	/// const vector<double>& distCoeffs2, <in>
	///	vector<Point3f>& pnts3d  <out>
	static bool triangulatePnts(const vector<Point2f>& pnts2d1,const vector<Point2f>& pnts2d2,
		const Mat& cameraMatrix1, const Mat& R1, const Mat& T1, const vector<double>& distCoeffs1,
		const Mat& cameraMatrix2, const Mat& R2, const Mat& T2, const vector<double>& distCoeffs2,
		vector<Point3f>& pnts3d);
	static vector<std::string>  getAbsoluteImgPath(const std::string &path, const std::string &extension);
	

	///角点提取算法，按照个数均分，一行一行排序
	///算法存在的问题，如果近距离，标定板倾斜较大，则初始角点不能均分，无法检测角点
	static vector<Point2f> extraCorner(const Mat& img,const vector<Point2f>& mousePnts,const int r,const int c);

	///角点提取算法，按照像素搜索;row和 col为输出
	static bool extraCorner2(const Mat& img, const vector<Point2f>& mousePnts
		,vector<Point2f>& ImageCorner, int &row, int &col);
	
	///圆环中心提取算法，点选进行圆环四周
	///整幅图像中进行圆环提取，无序，需要在提取后重新排序
	static bool extraRingCenter(const Mat& img,const vector<Point2f>& mousePnts,
		vector<Point2f>& ImageCorner,int& row,int& col);

	///圆环中心提取算法，点选四个角圆环中心区域
	///整幅图像中进行圆环提取，无序，需要在提取后重新排序
	static bool extraRingCenter2(const Mat& img,const vector<Point2f>& mousePnts,vector<Point2f>& ImageCorner,int& row,int& col);

	//至少3个点
	//oriPoints,起始位置三维坐标点，既X1
	//terminatePoints，结束位置三维坐标点，既X2
	//Rot，输出的旋转矩阵 3X3 double
	//Tra，输出的平移向量 3X1 double
	//X2=R*X1+T 起始位置点变换到结束位置点
	static bool rigidTransform(const std::vector<Point3f>& oriPoints,
								const std::vector<Point3f>& terminatePoints,
								cv::Mat& Rot,cv::Mat& Tra);

	///测头标定算法
	///@param 输入 光笔图片的R(3X3), T(3X1) double 
	///@param 输出 测头中心在测量坐标系下的三维坐标
	///计算方法SVD，放到一个Mat 3xN
	static bool calculateProbeCenter(const vector<Mat>& R_vec,const vector<Mat>& T_vec,Point3f& center);

	///建立光笔坐标系
	///@param vector<TagPoint3f> tagPnts 输入/输出 测量坐标系下,7个光斑点坐标；输出光笔坐标系下的新坐标,其要求是已经进行了排序了
	///@param Point3f probeCenter 输入 光笔坐标系下,测头中心坐标
	///建立方法：原点在测头中心，长轴5点到1点方向确定Z轴正方向，6点到7点方向确定x轴正方向，y轴方向根据右手法则确定
	///tagPnts  Vec4f,第一个数为tag，后三个数为坐标
	static void createLightPenCoordinate(vector<TagPoint3f>& tagPnts,const Point3f& probeCenter);

	///二次曲面模型，亚像素寻找
	///@param 输入 win 3x3 峰值点位于窗口的中间
	///@param 输出 dx  x方向偏移量
	///@param 输出 dy  y方向偏移量
	static bool quadSubPixel(const Mat& win,float& dx,float& dy);

	///由圆环中心点间距生成模板匹配的模板
	static Mat generateRingTemp(const vector<Point2f>& centers);

	///求出标定板特征点的三维点平面点
	///@param R 3x1的旋转向量 或3x3的旋转矩阵
	static void project2CalibrationBoard(const vector<Point2f>& src, vector<Point3f>& pnts3d,
		const Mat& cameraMatrix, vector<double> distCoeffs, const Mat& R,const Mat& T);

	/// 产生余弦条纹图像，I = bias + range*cos(2*PI*x/T + fai)  或者I = bias + range*cos(2*PI*y/T + fai) 
	// Mat& Img, <in and out> 出入余弦条纹图像
	// float T, <in> 余弦条纹的周期 
	// float fai,<in> 余弦的初始相位
	// float bias, <in> 余弦亮度均值
	// float range,<in> 余弦亮度的幅值
	// float gamma, <in> 余弦亮度条纹的非线性特性gamma值，若是线性的 gamma = 1 （默认值），若是非线性的，不等于1
	// bool, isX <in> 是否对X坐标进行余弦编码， true 是对X， 是对y进行编码
	// bool isGrid <in>  x y坐标是否垂直成正交grid， true 为是，false 表示为六边形排布，类似Lightcrafter的DMD形式
	static bool creatCosImg(Mat& Img, float T, float fai, float bias, float range, float gamma = 1, bool isX =true, bool isGrid = true);

	/// 求最大公约数
	/// 扩展的欧几里德算法，计算a  b 的最大公约数q，以及a b的 逆 x y，则满足  ax+by = q= gcd(a,b)
	/// 算法 采用递归的方法 参考 算法导论 528页 网页 http://baike.baidu.com/view/1478219.htm
	/// int a,<in> 输入整数 a
	/// int b,<in> 输入整数 b
	/// int& x,<in and out>  输出的参数，对应a的系数
	/// int& y, <in and out>  输出的参数，对应b的系数
	/// int& q <in and out>  输出的最大公约数
	static bool gcd(int a, int b, int& x, int& y, int& q);

	/// 计算N步相移的卷绕相位，只针对灰度图像序列
	/// vector<Mat> images   输入相同频率的图像序列
	/// float threshold      阈值，判断是否为合适的有效的计算结果，
	/// Mat& mod_phase,  卷绕相位，输出，浮点数矩阵 float，行列数等于图像行列数
	/// Mat& mean_lus,   均值亮度，输出，浮点数矩阵 float，行列数等于图像行列数
	/// Mat& lus_range,   亮度范围，，输出，浮点数矩阵 float，行列数等于图像行列数
	/// Mat& mask，   当前坐标位置的相位是否有效，1为有效，0为无效；输出，整数矩阵，行列数等于图像行列数
	static bool phaseshift (vector<Mat> images, float threshold, Mat& mod_phase, Mat& mean_lus, Mat& lus_range, Mat& mask);

	/// 根据中国剩余定理 计算卷绕相位的绝对相位，并转化为像素坐标
	/// vector<Mat> mod_phases,<in>  输入多个卷绕相位，float型
	/// vector<float> lamda <in>   输入多个频率的周期值，
	/// float m <in>  所有周期的最大公约数
	/// Mat& cor <out>
	static bool robustCRM(vector<Mat> mod_phases,vector<float> lamda, float m, Mat& cor);

	/// 计算无畸变的图像像素坐标，即输入是图像像素坐标，输出也是像素坐标，两者具有相同的摄像机内外参数，只是第二个没有镜头畸变
	/// const vector<Point2f>& src , <in> 输入图像点坐标
	/// vector<Point2f>& dst ,<out>  输出摄像机图像坐标下的像素坐标
	/// double fc[2] ,<in> 摄像机参数focal lengh
	/// double cc[2] ,<in> 摄像机参数主点位置
	/// double kc[5] ,<in> 摄像机畸变参数  
	/// double alfpha_c  <in>  摄像机参数纵横比例因子
	static bool undistorPixcor(const vector<Point2f>& src ,vector<Point2f>& dst ,double fc[2] ,double cc[2] ,double kc[5] ,double alfpha_c );

	/// 根据摄像机参数计算畸变图像，已知无畸变图像的模型，本函数适用于投影机三维计算时，对应无畸变投影图像的畸变图像，这样利用透镜的畸变
	/// 抵消了这个畸变，直接使用无畸变模型参与三维计算
	/// Mat& img,<in and out>  输出带有畸变的图像
	/// double fc[2] ,<in> 摄像机参数focal lengh
	/// double cc[2] ,<in> 摄像机参数主点位置
	/// double kc[5] ,<in> 摄像机畸变参数  
	/// double alfpha_c  <in>  摄像机参数纵横比例因子
	/// float meanLus,<in>  均值亮度
	/// float rangelus,<in>  幅值亮度
	/// float T,<in> 周期
	/// float fai, <in> 起始相位 
	/// int method =0  默认为0，表示模型是 meanLus + rangelus*cos(2*pi*x/T -fai)
	/// method = 1 Gray code  周期为T， 起始的位置在fai，ranglus<0 表示先 黑后白，反之 先白后黑； 如周期是10，fai等于5，ranglus=1
	/// 则一开就是黑，相当于移动了半个周期

	static bool creatdistorImg(Mat& img,double fc[2] ,double cc[2] ,double kc[5] ,double alfpha_c, float meanLus, float rangelus,float T,float fai, int method =0);


	//// 创建一个圆环图像，改图像保存在正方形的矩阵中，正方形尺寸为dx，外圆环半径为 dx/3, 内圆环半径为  dx/6
	//// Mat& img,<out> 输出图像 double型
	/// int dx <in>  正方形矩阵的尺寸
	static bool creatRingImg(Mat& img,int dx);
    ///圆环中心提取算法，自动
    static bool findRingBoardCenter(const Mat& img, Size patternSize, vector<Point2f>& centers);
	//////////////////chengwei added//////////////////////////////////////////////////////////
	//得到有序号的二维特征点集 基于opencv fittingellipse
	static bool getTagPoint2f(const Mat& img,vector<TagPoint2f>& tagPnts2f, int signIndex = 0);
	static bool getTagPoint2fStrong(const Mat& img, vector<TagPoint2f>& tagPnts2f, Mat cameraMatrixLeft, Mat cameraMatrixRight);
    ////重载getTagPoint2fStrong()函数，锁定ROI区域。
	static bool getTagPoint2fStrong(const Mat& img, const cv::Rect maskLightPen, vector<TagPoint2f>& tagPnts2f, Mat cameraMatrixLeft, Mat cameraMatrixRight);
	/////////*************************
	static Mat DrawEllipse(Mat img, double EllipseCenter_x, double EllipseCenter_y, double EllipseLong_axis, double EllipseShort_axis, double angle);

	////////*******************************

	static Mat CoreAlgorithm::circleImg(int radius, int width);
	//得到有序号的二维特征点集 基于dual fitting ellipse亚像素圆检测
	//static bool getTagPoint2f2(const Mat& img, const Mat imgvector<TagPoint2f>& tagPnts2f);
	//输入已知世界坐标系三维特征点objectPoints
	//输入图像坐标系下的二维图像特征点imagePoints
	//输入相机内参数CameraIntrinsic[3][3],畸变参数DistortionCoeffs[4]
	//输出测量坐标系下的各个特征点的三维坐标PointToCam
	//输出世界坐标系相对于测量坐标系的旋转变量和平移变量rvec,tvec
	//输出旋转变量的三倍标准差，平移向量三倍标准差dpdrot,dpdt
	//标志位Flag 代表内部使用的解决PNP问题的方法
	// PNP_ITERATIVE Iterative method is based on Levenberg-Marquardt optimization. 
	//In this case the function finds such a pose that minimizes reprojection error, 
	//that is the sum of squared distances between the observed projections imagePoints and the projected (using projectPoints() ) objectPoints .
    //PNP_EPNP  F.Moreno-Noguer, V.Lepetit and P.Fua "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
    // PNP_P3P   X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
	//以下两个需要opencv3.0以上版本才支持
	//PNP_DLS   Method is based on the paper of Joel A. Hesch and Stergios I. Roumeliotis. “A Direct Least-Squares (DLS) Method for PnP”.
	//PNP_UPNP  Method is based on the paper of A.Penate-Sanchez, J.Andrade-Cetto, F.Moreno-Noguer. 
	//“Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation”.
	//In this case the function also estimates the parameters f_x and f_y assuming that both have the same value. 
	//Then the cameraMatrix is updated with the estimated focal length.
	static void PnPMethod(vector<Point3f> objectPoints,
                        vector<Point2f>imagePoints,
						Mat cameraMatrix, Mat distCoeffs,
						Mat &PoseR,Mat &PoseT,
						vector<double> &dpdrot,vector<double> &dpdt,vector<Point3f> &PointToCam,int Flag = PNP_DLS);
	//输入已知世界坐标系三维特征点objectPoints
	//输入图像坐标系下的二维图像特征点imagePoints
	//输入相机内参数CameraIntrinsic[3][3],畸变参数DistortionCoeffs[4]
	//输出世界坐标系相对于测量坐标系的旋转变量和平移变量rvec,tvec
	//输出旋转变量的三倍标准差，平移向量三倍标准差dpdrot,dpdt
	//标志位Flag 代表内部使用的解决PNP问题的方法
	// PNP_ITERATIVE Iterative method is based on Levenberg-Marquardt optimization. 
	//In this case the function finds such a pose that minimizes reprojection error, 
	//that is the sum of squared distances between the observed projections imagePoints and the projected (using projectPoints() ) objectPoints .
    //PNP_EPNP  F.Moreno-Noguer, V.Lepetit and P.Fua "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
    // PNP_P3P   X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
	//以下两个需要opencv3.0以上版本才支持
	//PNP_DLS   Method is based on the paper of Joel A. Hesch and Stergios I. Roumeliotis. “A Direct Least-Squares (DLS) Method for PnP”.
	//PNP_UPNP  Method is based on the paper of A.Penate-Sanchez, J.Andrade-Cetto, F.Moreno-Noguer. 
	//“Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation”.
	//In this case the function also estimates the parameters f_x and f_y assuming that both have the same value. 
	//Then the cameraMatrix is updated with the estimated focal length.
	static void PnPMethod(vector<Point3f> objectPoints,
                        vector<Point2f>imagePoints,
						Mat cameraMatrix, Mat distCoeffs,
						Mat &PoseR,Mat &PoseT,
						vector<double> &dpdrot,vector<double> &dpdt,int Flag = PNP_DLS);
	//输入拍摄特征点图像img
	//输入已知世界坐标系三维特征点objectPoints
	//输入图像坐标系下的二维图像特征点imagePoints
	//输入相机内参数CameraIntrinsic[3][3],畸变参数DistortionCoeffs[4]
	//输入迭代次数iterativeTimes，默认为10次 
	//输出测量坐标系下的各个特征点的三维坐标PointToCam
	//输出世界坐标系相对于测量坐标系的旋转变量和平移变量rvec,tvec
	//输出旋转变量的三倍标准差，平移向量三倍标准差dpdrot,dpdt
	//标志位Flag 代表内部使用的解决PNP问题的方法
	// 特征点是否是圆环
	// PNP_ITERATIVE Iterative method is based on Levenberg-Marquardt optimization. 
	//In this case the function finds such a pose that minimizes reprojection error, 
	//that is the sum of squared distances between the observed projections imagePoints and the projected (using projectPoints() ) objectPoints .
    //PNP_EPNP  F.Moreno-Noguer, V.Lepetit and P.Fua "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
    // PNP_P3P   X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
	//以下两个需要opencv3.0以上版本才支持
	//PNP_DLS   Method is based on the paper of Joel A. Hesch and Stergios I. Roumeliotis. “A Direct Least-Squares (DLS) Method for PnP”.
	//PNP_UPNP  Method is based on the paper of A.Penate-Sanchez, J.Andrade-Cetto, F.Moreno-Noguer. 
	//“Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation”.
	//In this case the function also estimates the parameters f_x and f_y assuming that both have the same value. 
	//Then the cameraMatrix is updated with the estimated focal length.
	static bool iterativePnPMethod(Mat &img,vector<Point3f> objectPoints,
                        vector<Point2f>imagePoints,
						Mat cameraMatrix, Mat distCoeffs,
						Mat &PoseR,Mat &PoseT,
						vector<double> &dpdrot,vector<double> &dpdt,vector<Point3f> &PointToCam,int Flag = PNP_DLS ,int iterativeTimes =10);
	static bool iterativePnPMethod(Mat &img,vector<Point3f> objectPoints,
                        vector<Point2f>imagePoints,
						Mat cameraMatrix, Mat distCoeffs,
						Mat &PoseR,Mat &PoseT,
						vector<double> &dpdrot,vector<double> &dpdt,int Flag = PNP_DLS ,int iterativeTimes =10);
	//利用三点法2求表面法线
	static void GetSurfaceNormal(Point3f point1,Point3f point2,Point3f point3,SurfaceNormal &SurfaceNormal);
	//平面拟合
	//输入：obsPoints  平面上的观测点
	//输出：Plane
	static bool PlaneFitting(vector<Point3f> obsPoints,Plane& model);
	//柱面拟合
	//输入 obsPoints 柱面上的观测点
	//输出 柱面模型参数
	static bool CircleCylinderFitting(vector<Point3f> obsPoints,CircleCylinder& models,int errorRank =0);
	//球面拟合
	//输入 obsPoints 柱面上的观测点
	//输出 球面模型参数
	static bool sphereFitting(vector<Point3f> obsPoints,CircleCylinder& models,int errorRank =0);
	//求出平面与直线交点
	//输入：直线参数 平面参数
	//输出：直线与平面的交点
	//拟合直线可以用opencv 中的fitLine函数
	static bool CalPlaneLineIntersectPoint(const Plane _plane,const Line _line,Point3f &IntersectPoint);
	//亚像素圆斑检测
	//输出：输入图像的亚像素检测出的椭圆拟合出来的相关要素
	//输入：图像ROI
	//输入kenelsize必须是大于O的奇数
	//nocation:返回长轴为RotatedRect.width,短轴为RotatedRect.height
	static bool findEllipses(const Mat img,const cv::Rect mask,vector<RotatedRect>& findResults,const double precisionlevel = 0.03,bool multi=false,int kenelsize =5);
	//根据文档《》生成平顶视图的转换矩阵
	//const Mat &PoseR,<in>  3X1
	//const Mat &PoseT,<in>  3X1 
	//Mat &parallelR<in & out>  输入平顶视图相对于摄像机的外参数R，3X3矩阵，输出是文档要求的H1 * inv(H0), 3X3
	static void calculateParallelViewR(const Mat &PoseR,const Mat &PoseT,Mat &parallelR);
	///圆环中心亚像素检测
	///@param img 输入 平行视图图像
	///@param centers 输入和输出
	///@param temp 输入 图像模板
	static bool ringSubPix(const Mat& img,vector<Point2f>& centers,const Mat& temp);
	//输入同心圆的对偶椭圆拟合算子
    //Input:  areaGrads    vector containing the horizontal and vertical image gradient
    //        areaPoses     corresponding coordinates of the image gradient in Ix,Iy
    //Output: dCs          the fitted dual ellipse in the form L'*dC*L = 0 where L is a line on the dual conic L = [a b c]'
    //        precisions   estimated uncertainty on the center of the dual ellipse (in pixel)
    //        angleIncertitudes   angle of the center uncertainty covariance ellipse
    static bool MultiEllipseFitting(vector<vector<cv::Point2d>>areaGrads,
                                    vector<vector<cv::Point2d>>areaPoses,
                                    vector<Mat>& dCs, vector<Mat>& precisions,
                                    vector<double> angleIncertitudes);
	///高级圆环中心亚像素检测,必须首先用ringSubPix函数获取像素级精度圆环中心坐标
    ///@param img 输入 平行视图图像
    ///@param centers 输入和输出
    ///@param mode 模式 0为内圆 1为外圆 2为内外圆平均值 3为内外圆优化平均值
    static bool ringSubPixAdv(const Mat& img, vector<Point2f>& centers, uint mode);
	//用于返回单幅图像中包含有T型靶标的的ROI
	//输入检测的七个靶点特征的二维中心点坐标
	//输入相对于原图的ROI
	static void findTtypeROI(vector<Point2f>& imagePoints,cv::Rect& ROI);
	//计算两点之间的距离
	static double distancePoints2f(const Point2f pnt1, const Point2f pnt2);
	//计算两个三维点之间的距离
	static double distancePoints(const Point3f pnt1,const Point3f pnt2);
	static float distancePlanes(const Plane plane1,const Plane plane2);
	static void SellipseBiasCorrection(const std::vector<Point3f> xw,const double radius,const  std::vector<Point2f> ptl,const CamPara cam_l,RT &poseLeft);
	static vector<Point2f> calculateEllipseBias(double radius, vector<Point3f> Ccenter, Mat Rotation, Mat Translation, Mat Camera_Matrix);
	static vector<Point2f> calculateEllipseBias(double radius, vector<Point3f> Ccenter, Mat Rotation, Mat Translation,const CamPara campara);
	static Mat vector2Mat(const vector<Point2f> pnts);
	static vector<Point2f> Mat2vector(const Mat pnts);
	static bool CoreAlgorithm::findPntsWithTagV(vector<Point2f>& centerPnt,vector<float>& longaxisRadius,vector<TagPoint2f>& tagPnts,float &firstFeaturelength,const double gamma);
	static bool findPntsWithTagVStrong(vector<Point2f> &centerPnt, vector<float>& longaxisRadius, vector<TagPoint2f>& tagPnts, float &firstFeaturelength, const double gamma, Mat cameraMatrix,double angle);
	




	static Mat vector2MatD(const vector<Point2f> pnts);
	static Mat vector32Mat(const vector<Point3f> pnts);
	static bool Cal3dPoint( const vector<Point2f> pointLeft ,const CamPara& camParaLeft
                    ,const vector<Point2f> pointRight ,const CamPara& camParaRight
                    ,const Mat rotVector ,const Mat traVector ,vector<Point3f>& point3d );
	static bool getTagPoint3f(const Mat& img1,const CamPara _campara1, const Mat& img2, const CamPara _campara2,
        const CamGroupPara _camgrouppara,vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& tagPnts3f);

	/////对于初始位置的光笔质心坐标，无需考虑其质心位置的精确检测，并且也无需对八个光笔点进行排序
	static bool  getTagPoint2f(const Mat& img1, const Mat& img2,
		Point2f &CentroidPnts2f_Left, Point2f &CentroidPnts2f_Right, float &CirDia_LP);
	/////按顺序精确检测出光笔的八个圆心。
	static bool findLightPenCircleCenter(const Mat& img, const CamPara _campara1, const CamPara _campara2,
		const cv::Rect maskRoi, Point2f &CentroidPnts2f, float &CirDia_LP, vector<TagPoint2f>& tagPnts2f);

	/////重载getTagPoint3f()函数
	static bool getTagPoint3f(const Mat& img1, const cv::Rect maskLeftLightPen, const CamPara _campara1, const Mat& img2, const cv::Rect maskRightLightPen, const CamPara _campara2,
		const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& tagPnts3f);

	//////重载getTagPoint3f()函数，输出左右图像中的光笔的质心。
	static bool getTagPoint3f(const Mat& img1, const cv::Rect maskLeftLightPen, const CamPara _campara1, const Mat& img2, const cv::Rect maskRightLightPen, const CamPara _campara2,
		const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& tagPnts3f, Point2f &LightPenCentroidPnt_Left, Point2f &LightPenCentroidPnt_Right);

	static bool calculateLPTag2fCentroid(const vector<TagPoint2f> leftTagsPnts2f, Point2f &LP_Centroid);


	static bool ellipseBiasCorrection(const std::vector<Point3f> xw,const double radius,const  std::vector<Point2f> ptl,const std::vector<Point2f> ptr,
						   const CamPara cam_l,const CamPara cam_r,const RT rt,vector<Point3f> &pntsl,int &iters);


	////qiyong added
	static bool detectEllipse(const Mat  &img, vector<RotatedRect> &data);
	static bool findCircularMarker(const Mat img, vector<pair<unsigned int, Point2f>> &markerResults);
	static bool findCircularMarker(const Mat img, vector<pair<unsigned int, Point2f>> &results_center, float &Cir_Diameter);
	static bool findCircularMarker(const Mat img, const cv::Rect Mask, vector<pair<unsigned int, Point2f>> &results_center);


	static double get_distance(Point2f pointO, Point2f pointA);
	/////计算空间一条直线外一点到这条直线的垂足点坐标
	static Point3f GetFootOfPerpendicular(
		const Point3f &pt,     //直线外一点  
		const Point3f &begin,  //直线开始点  
		const Point3f &end);   //直线结束点  
	static void  createCodepntsCoordinateSVD(vector<TagPoint3f> tagPnts, vector<TagPoint3f>& CodepntsCenterToOrigin);
	static void createCodepntsCoordinate(vector<TagPoint3f> tagPnts, vector<TagPoint3f>& CodepntsCenterToOrigin);


	static bool getCodePoint3f(const Mat imgLeft, const cv::Rect maskLeft_First, const cv::Rect maskLeft_Second, const cv::Rect maskLeft_Third, const CamPara _campara1, const Mat imgRight, const cv::Rect maskRight_First, const cv::Rect maskRight_Second, const cv::Rect maskRight_Third, const CamPara _campara2,
		const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& tagPnts3f,Point2f &CodeCenterPntLeft_First, Point2f &CodeCenterPntLeft_Second, Point2f &CodeCenterPntLeft_Third,
		Point2f &CodeCenterPntRight_First, Point2f &CodeCenterPntRight_Second, Point2f &CodeCenterPntRight_Third);

	static bool getCodePoint3f(const Mat imgLeft, cv::Rect maskLeftRoi_First, const Mat imgRight, cv::Rect maskRightRoi_First, const CamPara campara1, const CamPara _campara2, const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& TagPnt_First);
	static bool getCodePoint3f(const Mat& imgLeft, const CamPara _campara1, const Mat& imgRight, const CamPara _campara2,
		const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<pair<unsigned int, Point2f>> &LeftmarkerResults, vector<pair<unsigned int, Point2f>> &RightmarkerResults, vector<TagPoint3f>& tagPnts3f);


	//////重载getCodePoint3f()函数
	static bool getCodePoint3f_First(const Mat imgLeft, cv::Rect maskLeftRoi_First, const Mat imgRight, cv::Rect maskRightRoi_First, const CamPara _campara1, const CamPara _campara2, const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& TagPnt_First, pair<unsigned int, Point2f> &CodeLeft_First, pair<unsigned int, Point2f> &CodeRight_First, int flag_First);
	static bool getCodePoint3f_Second(const Mat imgLeft, cv::Rect maskLeftRoi_Second, const Mat imgRight, cv::Rect maskRightRoi_Second, const CamPara _campara1, const CamPara _campara2, const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& TagPnt_Second, pair<unsigned int, Point2f> &CodeLeft_Second, pair<unsigned int, Point2f> &CodeRight_Second, int flag_Second);
	static bool getCodePoint3f_Third(const Mat imgLeft, cv::Rect maskLeftRoi_Third, const Mat imgRight, cv::Rect maskRightRoi_Third, const CamPara _campara1, const CamPara _campara2, const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& TagPnt_Third, pair<unsigned int, Point2f> &CodeLeft_Third, pair<unsigned int, Point2f> &CodeRight_Third, int flag_Third);

	static bool getCodePoint3f(const Mat imgLeft, const CamPara _campara1, const Mat imgRight, const CamPara _campara2,
		const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& tagPnts3f,
		Point2f &LightPenCentroidPntLeft, Point2f &LightPenCentroidPntRight, float &CircleDiameter,
		vector<pair<unsigned int, Point2f>> &markers_Left, vector<pair<unsigned int, Point2f>> &markers_Right);

	///计算一系列三维点的质心
	///*input:带有序号的三维点；
	///*output:质心坐标；
	static void CalculateCentroid(vector<TagPoint3f> tagPnts, Point3f &CentroidPoint);


	static void TagPoint3fToMat(vector<TagPoint3f> tagPnts,Mat &srcMat);
	static Mat CoreAlgorithm::comMatR(Mat Matrix1, Mat Matrix2, Mat &MatrixCom);
	static Mat CoreAlgorithm::comMatC(Mat Matrix1, Mat Matrix2, Mat &MatrixCom);
	
	///ended
private:

	///移除重复的中心点
	///这里认为，一个位置一定会有两个较接近的中心坐标点
	static void removeRepeatPoints( vector<Point2f>& points );

	/// 对含有畸变的图像像素坐标进行畸变校正，并求取在摄像机坐标下的 normalize 坐标
	/// const vector<Point2f>& src , <in> 输入图像点坐标
	/// vector<Point2f>& dst ,<out>  输出摄像机坐标下的normalized 坐标
	/// double fc[2] ,<in> 摄像机参数focal lengh
	/// double cc[2] ,<in> 摄像机参数主点位置
	/// double kc[5] ,<in> 摄像机畸变参数  
	/// double alfpha_c  <in>  摄像机参数纵横比例因子
	static bool normalizPixel( const vector<Point2f>& src ,vector<Point2f>& dst ,double fc[2] ,double cc[2] ,double kc[5] ,double alfpha_c );
	
	static bool GetCornerAtRow(const Mat& ImageGray,Point2f imgpnt_left, Point2f imgpnt_right, vector<Point2f> &cornerrow);

	///对圆环中心进行排序
	///从左上角开始，按行排序
	static bool sortRingCenter(const vector<Point2f>& mousePnts,vector<Point2f>& center,int& row,int& col);
    ///对圆环中心进行排序
    ///按照XY坐标，均从小到大排列，未考虑标定板颠倒的情况
    static bool sortRingCenter2(vector<Point2f>& center,const int row,const int col);

	///判断pnt3是否在pnt1和pnt2的连线上
	static bool isOnLine(const Point2f& pnt1,const Point2f& pnt2,const Point2f& pnt3);

	///按照x坐标由小到大排序
	static void sortByXMin2Max(vector<Point2f>& pnts);

	///按照y坐标由小到大排序
	static void sortByYMin2Max(vector<Point2f>& pnts);

	///颠倒点的排序
	static void reverse(vector<Point2f>& pnts);

	///返回两点间距离
	static float distance(const Point2f& pnt1,const Point2f& pnt2);

	///计算平行视图标定板图像的R，3x3  标定板的坐标系是如下建立的，行坐标为y，列坐标为x，也就是说x轴向右，y轴向下，z轴指向平面内部
	///标定板坐标系建立在lefttop点，按行排列 部
	///标定板坐标系建立在lefttop点，按行排列 
	///
	//static void calculateParallelViewR(const Mat &PoseR,const Mat &PoseT,Mat &parallelR);
	///
	static void calculateParallelViewR(const vector<RT>& rt_vec, vector<Mat>& parallelR);
		///计算平行视图标定板图像的R，3x3  标定板的坐标系是如下建立的，行坐标为y，列坐标为x，也就是说x轴向右，y轴向下，z轴指向平面内

	///生成平行视图相机的R
	///标定板坐标系建立在lefttop点，按行排列
	static void generateParallelCamR(Mat& R);
	//得到摄像机坐标系下的有序号的三维特征点集 基于opencv fittingellipse
  
	///判断一个点是否在一个轮廓内,返回值大于0则在内部
	static double containsPnt(vector<Point2f> mousePnts, Point2f pnt);
    ///圆度评判，越小，圆度越好
    static float roundness(const Point2f& pnt1,const vector<Point>& circle,const float r);
	//////////////////////////////////chengwei added//////////////////////////////////////////////////////////////////
	//判断三个点中哪个点分别是2，6，7点
	//输出容器中一次存放这1，2，6，7点
	static bool findshortAxisPoints(const Point2f footpointNearPnts1,const Point2f footpointNearPnts2,const Point2f footpointNearPnts3,const Point2f footPoint,vector<TagPoint2f>& TagPoints);
	//对偶椭圆拟合算子
	//Input:  areaGrad    vector containing the horizontal and vertical image gradient
	//        areaPos     corresponding coordinates of the image gradient in Ix,Iy
	//Output: dC          the fitted dual ellipse in the form L'*dC*L = 0 where L is a line on the dual conic L = [a b c]'
	//        precision   estimated uncertainty on the center of the dual ellipse (in pixel)
	//        angleIncertitude   angle of the center uncertainty covariance ellipse
	static bool DualConicFitting(vector<cv::Point2d>areaGrad,vector<cv::Point2d>areaPos,Mat& dC,Mat& precision,double& angleIncertitude);
	//输入一般椭圆参数方程参数，求解标准椭圆参数
	static bool conicaEllipseTostd(const Mat ellipsePara,RotatedRect& result);
	//将三维坐标点转换到新的坐标系下
	//输入：三维坐标点集合，两坐标系之间的旋转矩阵，要求是3X3的矩阵，平移向量
	static void pointPosetranslation(const vector<Point3f> PointSrc,vector<Point3f>& PointDst,Mat R,Mat t);
	
	


};
