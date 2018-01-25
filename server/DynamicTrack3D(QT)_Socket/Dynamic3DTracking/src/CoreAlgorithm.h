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
	///ʹ�õ����������е�������ı궨
	///@param imgsPath ���� �궨ͼ��ľ���·��
	///@param cData ���� �궨������
	///@param camParaSrc ���� �������Ѽ���������������ʼֵ
	///@param camParaDst ��� �����������õ��������
	static void iterativeCameraCalibration(const vector<string>& imgsPath,const CalibrationData& cData,
		const CamPara& camParaSrc, CamPara& camParaDst);

	///��û���������ͼ��
	///@param src ���� �궨��ԭʼͼ��
	///@param dst ��� �궨�����У�����ͼ��
	///@param cameraMatrix ���� ����ڲ���
	///@param distCoeffs ���� ����������
	///@param R 3x3���� ������뵥λ��������������У�����ͼ��
	///					�������궨��ƽ����ͼ��R��������������ƽ����ͼͼ��
	///// 2014.9.29  zhang xu undated
	//// @param  const Mat& new_cameraMatrix, ��������µ��ڲ���
	static void undistortImg(const Mat& src, Mat& dst,
		const Mat& cameraMatrix, const vector<double>& distCoeffs, const Mat& R);

	///��ĳһ���ͼ������ϵ�µı궨������ת������һ���ͼ��ͼ������ϵ��
	///@param src ���� �궨��ԭʼ��ͼ�ϵ�������2d
	///@param dst ��� �궨��ƽ����ͼ�ϵ�������2d
	///@param cameraMatrix12 ���� ����ڲ���
	///@param distCoeffs12 ���� ����������
	///@param R12 ���� �궨��R 3x1
	///@param T12 ���� �궨��T 3x1 
	static void undistortPoints2DifferentView(const vector<Point2f>& src, vector<Point2f>& dst,
		const Mat& cameraMatrix1, const Mat& R1, const Mat& T1, const vector<double>& distCoeffs1,
		const Mat& cameraMatrix2, const Mat& R2, const Mat& T2, const vector<double>& distCoeffs2);

	///����Random�㷨�ҳ��ڳ���Ͷ����ϵĵ�,ʮ�ֽ�������ڳ���
	///������y�������򣬴�ʮ�ֽ��洦��ʼ�����ᰴ�չ������ʱʮ�ֽ�����������
	///������Ҫ��1��ʮ�ֲ���״;2������6����
	///@param pnts ���� ����6����
	///@param L_pnts ��� �����
	///@param S_pnts ��� �����
	static bool pointsSort2D_3(vector<Point2f>& pnts,vector<Point2f>& L_pnts,vector<Point2f>& S_pnts);

	///���ݹ�ʵľ�����״��ʮ�ֽ��棩��������ĵ���ϱ�ǩ
	///�����������1��ʮ�ֲ���״;2�����������������6����
	///@param L_pnts ���� ����㣬����4����
	///@param S_pnts ���� ����㣬����1����
	///@param tagPnts ��� ���б�ǩ�Ĺ�ߵ�����,Vec3f ��һ����Ϊ��ǩ����������Ϊ������ 
	/// �����������֮ǰ���Ѿ�ʹ���˺���pointsSort2D_3�� ��֤����������ȷ�ģ�ֻ�Ƕ��������Ƿ�����©��δ���ǣ�Ҳ����˵��ֻҪ������ȫ�ģ���
	/// ����pointsSort2D_3�Ľ������tag ������ȷ��
	static bool getPntsWithTag(const vector<Point2f>& L_pnts,const vector<Point2f>& S_pnts,vector<TagPoint2f>& tagPnts);
	//chengwei added
	//���ݼ������������ĵ㣬�ҳ���Ҫ�������㲢�����Ӧ���
	//���룺�������������ĵ㣻ÿ�����������Բ�ĳ���뾶,gamma����Բ�߾������Բ�߰뾶
	//��������б�ǩ��Ϣ�����ź����������	һ�ŵ��Բ��������
	static bool findPntsWithTag(vector<Point2f>& centerPnt,vector<float>& longaxisRadius,vector<TagPoint2f>& tagPnts,float &firstFeaturelength,const double gamma);
	
	///����LED�����ȡ�㷨
	static void detectLightSpot_LED_3(const Mat imgMat, vector<Point2f>& centerPnt, vector<float>& longaxisRadius);
	static bool detectLightSpot_LED2(const Mat imgMat, vector<Point2f>& centerPnt, vector<float>& longaxisRadius);
	static bool detectLightSpot_LED(const Mat& imgMat, vector<Point2f>& centerPnt, vector<float>& longaxisRadius);
	
	///����������궨����
	///@param cData ���� �궨����
	///@param campara ��� ����궨����
	static void ZhangCalibrationMethod_CPP( const CalibrationData& cData,CamPara& campara);

	///����궨����,���1Ϊ�����
	///@param camPara12 ���� ��������궨����
	///@param CalibrationData12 ���� ��������궨����
	///@param rt ��� ��������λ�˹�ϵ
	static void stereoCalibration(const CamPara& camPara1,const CamPara& camPara2,
		const CalibrationData& cornerDataVec1,const CalibrationData& cornerDataVec2,RT& rt);

	///�ж�һ��int���Ƿ������vector��
	static bool isBelong2Vec(vector<int> vec,int index);

	///��ά�ؽ�
	//// ����0  ��������⣬�������������һ��
	/// ��Խṹ��ϵͳ��ͶӰ��ֻ��һ��λ�ö�Ӧ��Ϣ�������Ӧ����ֻ��x���꣬û��y����
	/// ��Ϊ��ϵͳ�лָ���ά��״���õ�����ͶӰͼ��x���꣬��ȷ���ģ�ֻ��һ������û�취�������ģ�ͣ����Ա�����������ά��
	///   ͶӰ������û�л��䣬���߶�ͶӰ�������Ѿ�У�������ˣ� ����������л��䡣�Ժ��ٱ���ʱ���Կ��ǰѱ���ͼ������ڽ������ͼ���ϣ����������ڿ���������ģ�;����Ѿ����ˡ�
	///   const vector<Point2f>& pointCam,<in> camera image point
	///   const vector<float>& proImg_x <in> the corresponding image point in the projector image. the model of the  projector is linear,no lens distortion 
	///   vector<Point3f>& point3d <out>  the point cloud in the coordinate of camera
	///   const CamPara& camParaLeft, ������ڲ���
	///   const CamPara& camParapro�� ͶӰ���ڲ���
	///   const double rotVector[3] ,const double traVector[3] ,  ͶӰ�������������������
	///   */
	static bool Cal3dPointStrLight(const vector<Point2f>& pointCam,const vector<float>& proImg_x,const CamPara& camParaLeft, const CamPara& camParapro,const double rotVector[3] ,const double traVector[3] ,vector<Point3f>& point3d);

	
	//����һ��
	///// ������������Ķ�Ӧ����Ϣ���������ֵ���Ի������У����Ȼ�������ά��
	/// const vector<Point2f> pointLeft <in>, ������Ķ�Ӧ����Ϣ��Ҫ��x���꣬y���궼��
	/// const CamPara& camParaLeft  <in>  ��������ڲ�����
	/// const vector<Point2f> pointRight <in>  ������Ķ�Ӧ����Ϣ��Ҫ��x���꣬y���궼��
	/// const CamPara& camParaRight <in> ��������ڲ���
	/// const double rotVector[3] <in>  �����������������ת����
	/// const double traVector[3] <in>  ���������������ƽ������ 
	/// vector<Point3f>& point3d  <out>  ���������ϵ�µ���ά��
	static bool Cal3dPoint( const vector<Point2f> pointLeft ,const CamPara& camParaLeft 
					,const vector<Point2f> pointRight ,const CamPara& camParaRight 
					,const double rotVector[3] ,const double traVector[3] ,vector<Point3f>& point3d );

	///��������opencv��triangulatePoints����
	//// ������������Ķ�Ӧ����Ϣ���������ֵ���Ի������У����Ȼ�������ά��
	//// const vector<Point2f>& pnts2d1, <in> ������Ķ�Ӧ����Ϣ��Ҫ��x���꣬y���궼��
	/// const vector<Point2f>& pnts2d2, <in> ������Ķ�Ӧ����Ϣ��Ҫ��x���꣬y���궼��
	/// const Mat& cameraMatrix1,  <in>  ��������ڲ���
	/// const Mat& R1, <in> ����������������ϵ����ת��������������ϵ���������������1 �ϣ���Ϊ��λ����
	/// const Mat& T1, <in> ����������������ϵ��ƽ��������������������ϵ���������������1�ϣ���Ϊ0����
	/// const vector<double>& distCoeffs1, <in>
	/// const Mat& cameraMatrix2, <in>  ��������ڲ���
	/// const Mat& R2, <in> ����������������ϵ����ת��������������ϵ���������������ϵ1�ϣ���Ϊ����������1����ת����
	/// const Mat& T2, <in> ����������������ϵ��ƽ�ƾ�������������ϵ���������������ϵ1�ϣ���Ϊ����������1��ƽ�ƾ���
	/// const vector<double>& distCoeffs2, <in>
	///	vector<Point3f>& pnts3d  <out>
	static bool triangulatePnts(const vector<Point2f>& pnts2d1,const vector<Point2f>& pnts2d2,
		const Mat& cameraMatrix1, const Mat& R1, const Mat& T1, const vector<double>& distCoeffs1,
		const Mat& cameraMatrix2, const Mat& R2, const Mat& T2, const vector<double>& distCoeffs2,
		vector<Point3f>& pnts3d);
	static vector<std::string>  getAbsoluteImgPath(const std::string &path, const std::string &extension);
	

	///�ǵ���ȡ�㷨�����ո������֣�һ��һ������
	///�㷨���ڵ����⣬��������룬�궨����б�ϴ����ʼ�ǵ㲻�ܾ��֣��޷����ǵ�
	static vector<Point2f> extraCorner(const Mat& img,const vector<Point2f>& mousePnts,const int r,const int c);

	///�ǵ���ȡ�㷨��������������;row�� colΪ���
	static bool extraCorner2(const Mat& img, const vector<Point2f>& mousePnts
		,vector<Point2f>& ImageCorner, int &row, int &col);
	
	///Բ��������ȡ�㷨����ѡ����Բ������
	///����ͼ���н���Բ����ȡ��������Ҫ����ȡ����������
	static bool extraRingCenter(const Mat& img,const vector<Point2f>& mousePnts,
		vector<Point2f>& ImageCorner,int& row,int& col);

	///Բ��������ȡ�㷨����ѡ�ĸ���Բ����������
	///����ͼ���н���Բ����ȡ��������Ҫ����ȡ����������
	static bool extraRingCenter2(const Mat& img,const vector<Point2f>& mousePnts,vector<Point2f>& ImageCorner,int& row,int& col);

	//����3����
	//oriPoints,��ʼλ����ά����㣬��X1
	//terminatePoints������λ����ά����㣬��X2
	//Rot���������ת���� 3X3 double
	//Tra�������ƽ������ 3X1 double
	//X2=R*X1+T ��ʼλ�õ�任������λ�õ�
	static bool rigidTransform(const std::vector<Point3f>& oriPoints,
								const std::vector<Point3f>& terminatePoints,
								cv::Mat& Rot,cv::Mat& Tra);

	///��ͷ�궨�㷨
	///@param ���� ���ͼƬ��R(3X3), T(3X1) double 
	///@param ��� ��ͷ�����ڲ�������ϵ�µ���ά����
	///���㷽��SVD���ŵ�һ��Mat 3xN
	static bool calculateProbeCenter(const vector<Mat>& R_vec,const vector<Mat>& T_vec,Point3f& center);

	///�����������ϵ
	///@param vector<TagPoint3f> tagPnts ����/��� ��������ϵ��,7����ߵ����ꣻ����������ϵ�µ�������,��Ҫ�����Ѿ�������������
	///@param Point3f probeCenter ���� �������ϵ��,��ͷ��������
	///����������ԭ���ڲ�ͷ���ģ�����5�㵽1�㷽��ȷ��Z��������6�㵽7�㷽��ȷ��x��������y�᷽��������ַ���ȷ��
	///tagPnts  Vec4f,��һ����Ϊtag����������Ϊ����
	static void createLightPenCoordinate(vector<TagPoint3f>& tagPnts,const Point3f& probeCenter);

	///��������ģ�ͣ�������Ѱ��
	///@param ���� win 3x3 ��ֵ��λ�ڴ��ڵ��м�
	///@param ��� dx  x����ƫ����
	///@param ��� dy  y����ƫ����
	static bool quadSubPixel(const Mat& win,float& dx,float& dy);

	///��Բ�����ĵ�������ģ��ƥ���ģ��
	static Mat generateRingTemp(const vector<Point2f>& centers);

	///����궨�����������ά��ƽ���
	///@param R 3x1����ת���� ��3x3����ת����
	static void project2CalibrationBoard(const vector<Point2f>& src, vector<Point3f>& pnts3d,
		const Mat& cameraMatrix, vector<double> distCoeffs, const Mat& R,const Mat& T);

	/// ������������ͼ��I = bias + range*cos(2*PI*x/T + fai)  ����I = bias + range*cos(2*PI*y/T + fai) 
	// Mat& Img, <in and out> ������������ͼ��
	// float T, <in> �������Ƶ����� 
	// float fai,<in> ���ҵĳ�ʼ��λ
	// float bias, <in> �������Ⱦ�ֵ
	// float range,<in> �������ȵķ�ֵ
	// float gamma, <in> �����������Ƶķ���������gammaֵ���������Ե� gamma = 1 ��Ĭ��ֵ�������Ƿ����Եģ�������1
	// bool, isX <in> �Ƿ��X����������ұ��룬 true �Ƕ�X�� �Ƕ�y���б���
	// bool isGrid <in>  x y�����Ƿ�ֱ������grid�� true Ϊ�ǣ�false ��ʾΪ�������Ų�������Lightcrafter��DMD��ʽ
	static bool creatCosImg(Mat& Img, float T, float fai, float bias, float range, float gamma = 1, bool isX =true, bool isGrid = true);

	/// �����Լ��
	/// ��չ��ŷ������㷨������a  b �����Լ��q���Լ�a b�� �� x y��������  ax+by = q= gcd(a,b)
	/// �㷨 ���õݹ�ķ��� �ο� �㷨���� 528ҳ ��ҳ http://baike.baidu.com/view/1478219.htm
	/// int a,<in> �������� a
	/// int b,<in> �������� b
	/// int& x,<in and out>  ����Ĳ�������Ӧa��ϵ��
	/// int& y, <in and out>  ����Ĳ�������Ӧb��ϵ��
	/// int& q <in and out>  ��������Լ��
	static bool gcd(int a, int b, int& x, int& y, int& q);

	/// ����N�����Ƶľ�����λ��ֻ��ԻҶ�ͼ������
	/// vector<Mat> images   ������ͬƵ�ʵ�ͼ������
	/// float threshold      ��ֵ���ж��Ƿ�Ϊ���ʵ���Ч�ļ�������
	/// Mat& mod_phase,  ������λ����������������� float������������ͼ��������
	/// Mat& mean_lus,   ��ֵ���ȣ���������������� float������������ͼ��������
	/// Mat& lus_range,   ���ȷ�Χ������������������� float������������ͼ��������
	/// Mat& mask��   ��ǰ����λ�õ���λ�Ƿ���Ч��1Ϊ��Ч��0Ϊ��Ч�������������������������ͼ��������
	static bool phaseshift (vector<Mat> images, float threshold, Mat& mod_phase, Mat& mean_lus, Mat& lus_range, Mat& mask);

	/// �����й�ʣ�ඨ�� ���������λ�ľ�����λ����ת��Ϊ��������
	/// vector<Mat> mod_phases,<in>  ������������λ��float��
	/// vector<float> lamda <in>   ������Ƶ�ʵ�����ֵ��
	/// float m <in>  �������ڵ����Լ��
	/// Mat& cor <out>
	static bool robustCRM(vector<Mat> mod_phases,vector<float> lamda, float m, Mat& cor);

	/// �����޻����ͼ���������꣬��������ͼ���������꣬���Ҳ���������꣬���߾�����ͬ����������������ֻ�ǵڶ���û�о�ͷ����
	/// const vector<Point2f>& src , <in> ����ͼ�������
	/// vector<Point2f>& dst ,<out>  ��������ͼ�������µ���������
	/// double fc[2] ,<in> ���������focal lengh
	/// double cc[2] ,<in> �������������λ��
	/// double kc[5] ,<in> ������������  
	/// double alfpha_c  <in>  ����������ݺ��������
	static bool undistorPixcor(const vector<Point2f>& src ,vector<Point2f>& dst ,double fc[2] ,double cc[2] ,double kc[5] ,double alfpha_c );

	/// ��������������������ͼ����֪�޻���ͼ���ģ�ͣ�������������ͶӰ����ά����ʱ����Ӧ�޻���ͶӰͼ��Ļ���ͼ����������͸���Ļ���
	/// ������������䣬ֱ��ʹ���޻���ģ�Ͳ�����ά����
	/// Mat& img,<in and out>  ������л����ͼ��
	/// double fc[2] ,<in> ���������focal lengh
	/// double cc[2] ,<in> �������������λ��
	/// double kc[5] ,<in> ������������  
	/// double alfpha_c  <in>  ����������ݺ��������
	/// float meanLus,<in>  ��ֵ����
	/// float rangelus,<in>  ��ֵ����
	/// float T,<in> ����
	/// float fai, <in> ��ʼ��λ 
	/// int method =0  Ĭ��Ϊ0����ʾģ���� meanLus + rangelus*cos(2*pi*x/T -fai)
	/// method = 1 Gray code  ����ΪT�� ��ʼ��λ����fai��ranglus<0 ��ʾ�� �ں�ף���֮ �Ȱ׺�ڣ� ��������10��fai����5��ranglus=1
	/// ��һ�����Ǻڣ��൱���ƶ��˰������

	static bool creatdistorImg(Mat& img,double fc[2] ,double cc[2] ,double kc[5] ,double alfpha_c, float meanLus, float rangelus,float T,float fai, int method =0);


	//// ����һ��Բ��ͼ�񣬸�ͼ�񱣴��������εľ����У������γߴ�Ϊdx����Բ���뾶Ϊ dx/3, ��Բ���뾶Ϊ  dx/6
	//// Mat& img,<out> ���ͼ�� double��
	/// int dx <in>  �����ξ���ĳߴ�
	static bool creatRingImg(Mat& img,int dx);
    ///Բ��������ȡ�㷨���Զ�
    static bool findRingBoardCenter(const Mat& img, Size patternSize, vector<Point2f>& centers);
	//////////////////chengwei added//////////////////////////////////////////////////////////
	//�õ�����ŵĶ�ά�����㼯 ����opencv fittingellipse
	static bool getTagPoint2f(const Mat& img,vector<TagPoint2f>& tagPnts2f, int signIndex = 0);
	static bool getTagPoint2fStrong(const Mat& img, vector<TagPoint2f>& tagPnts2f, Mat cameraMatrixLeft, Mat cameraMatrixRight);
    ////����getTagPoint2fStrong()����������ROI����
	static bool getTagPoint2fStrong(const Mat& img, const cv::Rect maskLightPen, vector<TagPoint2f>& tagPnts2f, Mat cameraMatrixLeft, Mat cameraMatrixRight);
	/////////*************************
	static Mat DrawEllipse(Mat img, double EllipseCenter_x, double EllipseCenter_y, double EllipseLong_axis, double EllipseShort_axis, double angle);

	////////*******************************

	static Mat CoreAlgorithm::circleImg(int radius, int width);
	//�õ�����ŵĶ�ά�����㼯 ����dual fitting ellipse������Բ���
	//static bool getTagPoint2f2(const Mat& img, const Mat imgvector<TagPoint2f>& tagPnts2f);
	//������֪��������ϵ��ά������objectPoints
	//����ͼ������ϵ�µĶ�άͼ��������imagePoints
	//��������ڲ���CameraIntrinsic[3][3],�������DistortionCoeffs[4]
	//�����������ϵ�µĸ������������ά����PointToCam
	//�����������ϵ����ڲ�������ϵ����ת������ƽ�Ʊ���rvec,tvec
	//�����ת������������׼�ƽ������������׼��dpdrot,dpdt
	//��־λFlag �����ڲ�ʹ�õĽ��PNP����ķ���
	// PNP_ITERATIVE Iterative method is based on Levenberg-Marquardt optimization. 
	//In this case the function finds such a pose that minimizes reprojection error, 
	//that is the sum of squared distances between the observed projections imagePoints and the projected (using projectPoints() ) objectPoints .
    //PNP_EPNP  F.Moreno-Noguer, V.Lepetit and P.Fua "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
    // PNP_P3P   X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
	//����������Ҫopencv3.0���ϰ汾��֧��
	//PNP_DLS   Method is based on the paper of Joel A. Hesch and Stergios I. Roumeliotis. ��A Direct Least-Squares (DLS) Method for PnP��.
	//PNP_UPNP  Method is based on the paper of A.Penate-Sanchez, J.Andrade-Cetto, F.Moreno-Noguer. 
	//��Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation��.
	//In this case the function also estimates the parameters f_x and f_y assuming that both have the same value. 
	//Then the cameraMatrix is updated with the estimated focal length.
	static void PnPMethod(vector<Point3f> objectPoints,
                        vector<Point2f>imagePoints,
						Mat cameraMatrix, Mat distCoeffs,
						Mat &PoseR,Mat &PoseT,
						vector<double> &dpdrot,vector<double> &dpdt,vector<Point3f> &PointToCam,int Flag = PNP_DLS);
	//������֪��������ϵ��ά������objectPoints
	//����ͼ������ϵ�µĶ�άͼ��������imagePoints
	//��������ڲ���CameraIntrinsic[3][3],�������DistortionCoeffs[4]
	//�����������ϵ����ڲ�������ϵ����ת������ƽ�Ʊ���rvec,tvec
	//�����ת������������׼�ƽ������������׼��dpdrot,dpdt
	//��־λFlag �����ڲ�ʹ�õĽ��PNP����ķ���
	// PNP_ITERATIVE Iterative method is based on Levenberg-Marquardt optimization. 
	//In this case the function finds such a pose that minimizes reprojection error, 
	//that is the sum of squared distances between the observed projections imagePoints and the projected (using projectPoints() ) objectPoints .
    //PNP_EPNP  F.Moreno-Noguer, V.Lepetit and P.Fua "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
    // PNP_P3P   X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
	//����������Ҫopencv3.0���ϰ汾��֧��
	//PNP_DLS   Method is based on the paper of Joel A. Hesch and Stergios I. Roumeliotis. ��A Direct Least-Squares (DLS) Method for PnP��.
	//PNP_UPNP  Method is based on the paper of A.Penate-Sanchez, J.Andrade-Cetto, F.Moreno-Noguer. 
	//��Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation��.
	//In this case the function also estimates the parameters f_x and f_y assuming that both have the same value. 
	//Then the cameraMatrix is updated with the estimated focal length.
	static void PnPMethod(vector<Point3f> objectPoints,
                        vector<Point2f>imagePoints,
						Mat cameraMatrix, Mat distCoeffs,
						Mat &PoseR,Mat &PoseT,
						vector<double> &dpdrot,vector<double> &dpdt,int Flag = PNP_DLS);
	//��������������ͼ��img
	//������֪��������ϵ��ά������objectPoints
	//����ͼ������ϵ�µĶ�άͼ��������imagePoints
	//��������ڲ���CameraIntrinsic[3][3],�������DistortionCoeffs[4]
	//�����������iterativeTimes��Ĭ��Ϊ10�� 
	//�����������ϵ�µĸ������������ά����PointToCam
	//�����������ϵ����ڲ�������ϵ����ת������ƽ�Ʊ���rvec,tvec
	//�����ת������������׼�ƽ������������׼��dpdrot,dpdt
	//��־λFlag �����ڲ�ʹ�õĽ��PNP����ķ���
	// �������Ƿ���Բ��
	// PNP_ITERATIVE Iterative method is based on Levenberg-Marquardt optimization. 
	//In this case the function finds such a pose that minimizes reprojection error, 
	//that is the sum of squared distances between the observed projections imagePoints and the projected (using projectPoints() ) objectPoints .
    //PNP_EPNP  F.Moreno-Noguer, V.Lepetit and P.Fua "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
    // PNP_P3P   X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
	//����������Ҫopencv3.0���ϰ汾��֧��
	//PNP_DLS   Method is based on the paper of Joel A. Hesch and Stergios I. Roumeliotis. ��A Direct Least-Squares (DLS) Method for PnP��.
	//PNP_UPNP  Method is based on the paper of A.Penate-Sanchez, J.Andrade-Cetto, F.Moreno-Noguer. 
	//��Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation��.
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
	//�������㷨2����淨��
	static void GetSurfaceNormal(Point3f point1,Point3f point2,Point3f point3,SurfaceNormal &SurfaceNormal);
	//ƽ�����
	//���룺obsPoints  ƽ���ϵĹ۲��
	//�����Plane
	static bool PlaneFitting(vector<Point3f> obsPoints,Plane& model);
	//�������
	//���� obsPoints �����ϵĹ۲��
	//��� ����ģ�Ͳ���
	static bool CircleCylinderFitting(vector<Point3f> obsPoints,CircleCylinder& models,int errorRank =0);
	//�������
	//���� obsPoints �����ϵĹ۲��
	//��� ����ģ�Ͳ���
	static bool sphereFitting(vector<Point3f> obsPoints,CircleCylinder& models,int errorRank =0);
	//���ƽ����ֱ�߽���
	//���룺ֱ�߲��� ƽ�����
	//�����ֱ����ƽ��Ľ���
	//���ֱ�߿�����opencv �е�fitLine����
	static bool CalPlaneLineIntersectPoint(const Plane _plane,const Line _line,Point3f &IntersectPoint);
	//������Բ�߼��
	//���������ͼ��������ؼ�������Բ��ϳ��������Ҫ��
	//���룺ͼ��ROI
	//����kenelsize�����Ǵ���O������
	//nocation:���س���ΪRotatedRect.width,����ΪRotatedRect.height
	static bool findEllipses(const Mat img,const cv::Rect mask,vector<RotatedRect>& findResults,const double precisionlevel = 0.03,bool multi=false,int kenelsize =5);
	//�����ĵ���������ƽ����ͼ��ת������
	//const Mat &PoseR,<in>  3X1
	//const Mat &PoseT,<in>  3X1 
	//Mat &parallelR<in & out>  ����ƽ����ͼ�����������������R��3X3����������ĵ�Ҫ���H1 * inv(H0), 3X3
	static void calculateParallelViewR(const Mat &PoseR,const Mat &PoseT,Mat &parallelR);
	///Բ�����������ؼ��
	///@param img ���� ƽ����ͼͼ��
	///@param centers ��������
	///@param temp ���� ͼ��ģ��
	static bool ringSubPix(const Mat& img,vector<Point2f>& centers,const Mat& temp);
	//����ͬ��Բ�Ķ�ż��Բ�������
    //Input:  areaGrads    vector containing the horizontal and vertical image gradient
    //        areaPoses     corresponding coordinates of the image gradient in Ix,Iy
    //Output: dCs          the fitted dual ellipse in the form L'*dC*L = 0 where L is a line on the dual conic L = [a b c]'
    //        precisions   estimated uncertainty on the center of the dual ellipse (in pixel)
    //        angleIncertitudes   angle of the center uncertainty covariance ellipse
    static bool MultiEllipseFitting(vector<vector<cv::Point2d>>areaGrads,
                                    vector<vector<cv::Point2d>>areaPoses,
                                    vector<Mat>& dCs, vector<Mat>& precisions,
                                    vector<double> angleIncertitudes);
	///�߼�Բ�����������ؼ��,����������ringSubPix������ȡ���ؼ�����Բ����������
    ///@param img ���� ƽ����ͼͼ��
    ///@param centers ��������
    ///@param mode ģʽ 0Ϊ��Բ 1Ϊ��Բ 2Ϊ����Բƽ��ֵ 3Ϊ����Բ�Ż�ƽ��ֵ
    static bool ringSubPixAdv(const Mat& img, vector<Point2f>& centers, uint mode);
	//���ڷ��ص���ͼ���а�����T�Ͱб�ĵ�ROI
	//��������߸��е������Ķ�ά���ĵ�����
	//���������ԭͼ��ROI
	static void findTtypeROI(vector<Point2f>& imagePoints,cv::Rect& ROI);
	//��������֮��ľ���
	static double distancePoints2f(const Point2f pnt1, const Point2f pnt2);
	//����������ά��֮��ľ���
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

	/////���ڳ�ʼλ�õĹ���������꣬���迼��������λ�õľ�ȷ��⣬����Ҳ����԰˸���ʵ��������
	static bool  getTagPoint2f(const Mat& img1, const Mat& img2,
		Point2f &CentroidPnts2f_Left, Point2f &CentroidPnts2f_Right, float &CirDia_LP);
	/////��˳��ȷ������ʵİ˸�Բ�ġ�
	static bool findLightPenCircleCenter(const Mat& img, const CamPara _campara1, const CamPara _campara2,
		const cv::Rect maskRoi, Point2f &CentroidPnts2f, float &CirDia_LP, vector<TagPoint2f>& tagPnts2f);

	/////����getTagPoint3f()����
	static bool getTagPoint3f(const Mat& img1, const cv::Rect maskLeftLightPen, const CamPara _campara1, const Mat& img2, const cv::Rect maskRightLightPen, const CamPara _campara2,
		const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& tagPnts3f);

	//////����getTagPoint3f()�������������ͼ���еĹ�ʵ����ġ�
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
	/////����ռ�һ��ֱ����һ�㵽����ֱ�ߵĴ��������
	static Point3f GetFootOfPerpendicular(
		const Point3f &pt,     //ֱ����һ��  
		const Point3f &begin,  //ֱ�߿�ʼ��  
		const Point3f &end);   //ֱ�߽�����  
	static void  createCodepntsCoordinateSVD(vector<TagPoint3f> tagPnts, vector<TagPoint3f>& CodepntsCenterToOrigin);
	static void createCodepntsCoordinate(vector<TagPoint3f> tagPnts, vector<TagPoint3f>& CodepntsCenterToOrigin);


	static bool getCodePoint3f(const Mat imgLeft, const cv::Rect maskLeft_First, const cv::Rect maskLeft_Second, const cv::Rect maskLeft_Third, const CamPara _campara1, const Mat imgRight, const cv::Rect maskRight_First, const cv::Rect maskRight_Second, const cv::Rect maskRight_Third, const CamPara _campara2,
		const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& tagPnts3f,Point2f &CodeCenterPntLeft_First, Point2f &CodeCenterPntLeft_Second, Point2f &CodeCenterPntLeft_Third,
		Point2f &CodeCenterPntRight_First, Point2f &CodeCenterPntRight_Second, Point2f &CodeCenterPntRight_Third);

	static bool getCodePoint3f(const Mat imgLeft, cv::Rect maskLeftRoi_First, const Mat imgRight, cv::Rect maskRightRoi_First, const CamPara campara1, const CamPara _campara2, const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& TagPnt_First);
	static bool getCodePoint3f(const Mat& imgLeft, const CamPara _campara1, const Mat& imgRight, const CamPara _campara2,
		const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<pair<unsigned int, Point2f>> &LeftmarkerResults, vector<pair<unsigned int, Point2f>> &RightmarkerResults, vector<TagPoint3f>& tagPnts3f);


	//////����getCodePoint3f()����
	static bool getCodePoint3f_First(const Mat imgLeft, cv::Rect maskLeftRoi_First, const Mat imgRight, cv::Rect maskRightRoi_First, const CamPara _campara1, const CamPara _campara2, const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& TagPnt_First, pair<unsigned int, Point2f> &CodeLeft_First, pair<unsigned int, Point2f> &CodeRight_First, int flag_First);
	static bool getCodePoint3f_Second(const Mat imgLeft, cv::Rect maskLeftRoi_Second, const Mat imgRight, cv::Rect maskRightRoi_Second, const CamPara _campara1, const CamPara _campara2, const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& TagPnt_Second, pair<unsigned int, Point2f> &CodeLeft_Second, pair<unsigned int, Point2f> &CodeRight_Second, int flag_Second);
	static bool getCodePoint3f_Third(const Mat imgLeft, cv::Rect maskLeftRoi_Third, const Mat imgRight, cv::Rect maskRightRoi_Third, const CamPara _campara1, const CamPara _campara2, const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& TagPnt_Third, pair<unsigned int, Point2f> &CodeLeft_Third, pair<unsigned int, Point2f> &CodeRight_Third, int flag_Third);

	static bool getCodePoint3f(const Mat imgLeft, const CamPara _campara1, const Mat imgRight, const CamPara _campara2,
		const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& tagPnts3f,
		Point2f &LightPenCentroidPntLeft, Point2f &LightPenCentroidPntRight, float &CircleDiameter,
		vector<pair<unsigned int, Point2f>> &markers_Left, vector<pair<unsigned int, Point2f>> &markers_Right);

	///����һϵ����ά�������
	///*input:������ŵ���ά�㣻
	///*output:�������ꣻ
	static void CalculateCentroid(vector<TagPoint3f> tagPnts, Point3f &CentroidPoint);


	static void TagPoint3fToMat(vector<TagPoint3f> tagPnts,Mat &srcMat);
	static Mat CoreAlgorithm::comMatR(Mat Matrix1, Mat Matrix2, Mat &MatrixCom);
	static Mat CoreAlgorithm::comMatC(Mat Matrix1, Mat Matrix2, Mat &MatrixCom);
	
	///ended
private:

	///�Ƴ��ظ������ĵ�
	///������Ϊ��һ��λ��һ�����������Ͻӽ������������
	static void removeRepeatPoints( vector<Point2f>& points );

	/// �Ժ��л����ͼ������������л���У��������ȡ������������µ� normalize ����
	/// const vector<Point2f>& src , <in> ����ͼ�������
	/// vector<Point2f>& dst ,<out>  �������������µ�normalized ����
	/// double fc[2] ,<in> ���������focal lengh
	/// double cc[2] ,<in> �������������λ��
	/// double kc[5] ,<in> ������������  
	/// double alfpha_c  <in>  ����������ݺ��������
	static bool normalizPixel( const vector<Point2f>& src ,vector<Point2f>& dst ,double fc[2] ,double cc[2] ,double kc[5] ,double alfpha_c );
	
	static bool GetCornerAtRow(const Mat& ImageGray,Point2f imgpnt_left, Point2f imgpnt_right, vector<Point2f> &cornerrow);

	///��Բ�����Ľ�������
	///�����Ͻǿ�ʼ����������
	static bool sortRingCenter(const vector<Point2f>& mousePnts,vector<Point2f>& center,int& row,int& col);
    ///��Բ�����Ľ�������
    ///����XY���꣬����С�������У�δ���Ǳ궨��ߵ������
    static bool sortRingCenter2(vector<Point2f>& center,const int row,const int col);

	///�ж�pnt3�Ƿ���pnt1��pnt2��������
	static bool isOnLine(const Point2f& pnt1,const Point2f& pnt2,const Point2f& pnt3);

	///����x������С��������
	static void sortByXMin2Max(vector<Point2f>& pnts);

	///����y������С��������
	static void sortByYMin2Max(vector<Point2f>& pnts);

	///�ߵ��������
	static void reverse(vector<Point2f>& pnts);

	///������������
	static float distance(const Point2f& pnt1,const Point2f& pnt2);

	///����ƽ����ͼ�궨��ͼ���R��3x3  �궨�������ϵ�����½����ģ�������Ϊy��������Ϊx��Ҳ����˵x�����ң�y�����£�z��ָ��ƽ���ڲ�
	///�궨������ϵ������lefttop�㣬�������� ��
	///�궨������ϵ������lefttop�㣬�������� 
	///
	//static void calculateParallelViewR(const Mat &PoseR,const Mat &PoseT,Mat &parallelR);
	///
	static void calculateParallelViewR(const vector<RT>& rt_vec, vector<Mat>& parallelR);
		///����ƽ����ͼ�궨��ͼ���R��3x3  �궨�������ϵ�����½����ģ�������Ϊy��������Ϊx��Ҳ����˵x�����ң�y�����£�z��ָ��ƽ����

	///����ƽ����ͼ�����R
	///�궨������ϵ������lefttop�㣬��������
	static void generateParallelCamR(Mat& R);
	//�õ����������ϵ�µ�����ŵ���ά�����㼯 ����opencv fittingellipse
  
	///�ж�һ�����Ƿ���һ��������,����ֵ����0�����ڲ�
	static double containsPnt(vector<Point2f> mousePnts, Point2f pnt);
    ///Բ�����У�ԽС��Բ��Խ��
    static float roundness(const Point2f& pnt1,const vector<Point>& circle,const float r);
	//////////////////////////////////chengwei added//////////////////////////////////////////////////////////////////
	//�ж����������ĸ���ֱ���2��6��7��
	//���������һ�δ����1��2��6��7��
	static bool findshortAxisPoints(const Point2f footpointNearPnts1,const Point2f footpointNearPnts2,const Point2f footpointNearPnts3,const Point2f footPoint,vector<TagPoint2f>& TagPoints);
	//��ż��Բ�������
	//Input:  areaGrad    vector containing the horizontal and vertical image gradient
	//        areaPos     corresponding coordinates of the image gradient in Ix,Iy
	//Output: dC          the fitted dual ellipse in the form L'*dC*L = 0 where L is a line on the dual conic L = [a b c]'
	//        precision   estimated uncertainty on the center of the dual ellipse (in pixel)
	//        angleIncertitude   angle of the center uncertainty covariance ellipse
	static bool DualConicFitting(vector<cv::Point2d>areaGrad,vector<cv::Point2d>areaPos,Mat& dC,Mat& precision,double& angleIncertitude);
	//����һ����Բ�������̲���������׼��Բ����
	static bool conicaEllipseTostd(const Mat ellipsePara,RotatedRect& result);
	//����ά�����ת�����µ�����ϵ��
	//���룺��ά����㼯�ϣ�������ϵ֮�����ת����Ҫ����3X3�ľ���ƽ������
	static void pointPosetranslation(const vector<Point3f> PointSrc,vector<Point3f>& PointDst,Mat R,Mat t);
	
	


};
