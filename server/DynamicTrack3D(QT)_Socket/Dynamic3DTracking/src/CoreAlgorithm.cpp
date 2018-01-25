#include "stdafx.h"
#include "CoreAlgorithm.h"
#include <math.h>
#include <iterator> 
#include"opencv2/opencv.hpp"
#include<string>
#include "SharedMethod.h"
#include "pcamera.h"
#include "ClpMeasurement.h"

//using namespace cv;
#define NO_OBJECT 0
//#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define ELEM(img, r, c) (CV_IMAGE_ELEM(img, unsigned char, r, c))p
#define ONETWO(L, r, c, col) (L[(r) * (col) + c])
void CoreAlgorithm::ZhangCalibrationMethod_CPP( const CalibrationData& cData,CamPara& campara )
{
	if(cData.frameNumList.size()==0)
		return;

	int ImgHeight = cData.imgHeight;                            
	int ImgWidth = cData.imgWidth;
	int imgAmount = cData.frameNumList.size();
	vector<vector<Point3f>> object_points = cData.plane3dPntsVec;
	vector<vector<Point2f>> image_points = cData.plane2dPntsVec;

	cv::Size imageSize(ImgWidth,ImgHeight);
	
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	cv::Mat cam_intr_para(3, 3, CV_64FC1);
	cv::Mat distCoeffs = Mat::zeros(1,4,CV_64FC1);
    if(!(cv::calibrateCamera(object_points,image_points,imageSize,cam_intr_para,distCoeffs,rvecs,tvecs)))
    {
        return;
    }
	//���������ͼƬ�������ֵ
	for (int i=0; i < 3; i++)
	{
		for (int j=0; j < 3; j++)
		{
			campara.CameraIntrinsic[i][j] = cam_intr_para.at<double>(i,j);
		}
	}
	for (int i = 0; i < 4; i++)
	{
		campara.DistortionCoeffs[i] = distCoeffs.at<double>(0,i);
	}
	campara.imgRTVec.clear();
	for(unsigned int i=0;i<rvecs.size();i++)
	{
		RT rt;
		rt.R = rvecs[i];
		rt.T = tvecs[i];
		campara.imgRTVec.push_back(rt);
	}

	vector<vector<Point2f>> error_image_points;
	Mat outpara_covariance(6,6,CV_64FC1);
	Mat inpara_covariance(8,8,CV_64FC1);
	Mat inpara_outpara_covariance(8,6,CV_64FC1);
	//covariance��ǰ8��λ����������ڲ���Э����
	Mat covariance = Mat::zeros(8+6*imgAmount,8+6*imgAmount,CV_64FC1);

	for( int i=0; i<imgAmount;i++ )
	{
		int cornerNum = image_points[i].size();
		vector<Point2f> perframe_imagepnt;
		vector<Point3f> perframe_objectpnt = object_points[i];
		Mat rot_vector_perframe(3,1,CV_64FC1);
		Mat tra_vector_perframe(3,1,CV_64FC1);
		tra_vector_perframe = campara.imgRTVec[i].T;
		rot_vector_perframe = campara.imgRTVec[i].R;

		Mat dpoutpara_perframe(2*cornerNum,6,CV_64FC1);
		Mat dpinpara_perframe(2*cornerNum,8,CV_64FC1);

		//������ͶӰ���͵�����
		//�����С 2Nx(10+<numDistCoeffs>)������numDistCoeffs=4
		//���Դ�СΪ 2N x 14������drot dt df dc ddist��˳���������
		//��c�ӿ��У��ſ˱Ⱦ��󱻷��ڼ����ֿ��Ĳ�����
		Mat jacobian;
		projectPoints(perframe_objectpnt
			,rot_vector_perframe,tra_vector_perframe
			,cam_intr_para,distCoeffs,
			perframe_imagepnt,jacobian);
		//����ͶӰ�õ��ļ���ǵ㱣������
		error_image_points.push_back(perframe_imagepnt);

		//��drot dt�ŵ�doutpara��
		for( int j=0;j<6;j++ )
		{
			jacobian.col(j).copyTo(dpoutpara_perframe.col(j));
		}
		//��df dc ddist�ϲ���dpinpara��
		for( int j=0;j<8;j++ )
		{
			jacobian.col(j+6).copyTo(dpinpara_perframe.col(j));
		}

		//��Э�������
		// outpara_covariance[6X6] = dpoutpara_perframe(2N*6  ת��)��dpoutpara_perframe(2N*6)
		mulTransposed(dpoutpara_perframe, outpara_covariance, 1); 
		// inpara_covariance[8X8] = dpinpara_perframe(2N*8  ת��)��dpinpara_perframe(2N*8)
		mulTransposed(dpinpara_perframe, inpara_covariance, 1);  
		// inpara_outpara_covariance[8X6] = dpinpara_perframe[2NX8 ת��]��dpoutpara_perframe(2N*6)
		gemm(dpinpara_perframe,dpoutpara_perframe,1,0,0,inpara_outpara_covariance,CV_GEMM_A_T);

		//���±�֡��Э�������
		for( int row=0;row<8;row++ )
		{
			for( int col=0;col<8;col++ )
			{
				//�ڲ�����Э����ϵ���
				covariance.at<double>(row,col) += inpara_covariance.at<double>(row,col);
			}
		}
		for (int row = 0; row < 8; row++)
		{
			for (int col = 0; col < 6; col++)
			{
				//���inpara_outpara_covarianceת�ã�ǰ8��λ�ó���
				covariance.at<double>( row , col+8+i*6) = inpara_outpara_covariance.at<double>(row,col);
				covariance.at<double>( col+8+i*6, row) = inpara_outpara_covariance.at<double>(row,col);
			}
		}
		for (int row = 0; row < 6; row++)
		{
			for (int col = 0; col < 6; col++)
			{
				//��ʣ�µ�λ�����outpara_covariance
				covariance.at<double>( row+8+i*6 , col+8+i*6 ) = outpara_covariance.at<double>(row,col);
			}
		}
	}
	//���forѭ���Ժ󣬽���ʼ�ǵ�ͼ���ǵ��������ò�ֵ
	vector<Point2f> subDst;
	vector<double>	absErr;
	int totalPoints=0;
	double totalError=0;
	for(unsigned int i=0;i<image_points.size();i++ )
	{
		vector<Point2f> temp;
		subtract(error_image_points[i],image_points[i],temp);
		copy(temp.begin(),temp.end(),back_inserter(subDst));
		double err;
		int n = image_points[i].size();
		err = norm(error_image_points[i],image_points[i],CV_L2);
		absErr.push_back((float)sqrt(err*err/n));
		totalError += err*err;
		totalPoints += n;
	}
	totalError /= totalPoints;
	Scalar xmean,ymean;
	Scalar xstd_dev,ystd_dev;
	Scalar errormean,errorstd_dev;
//#ifdef DEBUG
//	//�������е�����ƽ��ֵ�ͱ�׼��
//	for (int i = 0; i < subDst.size(); i++)
//	{
//		qDebug() << subDst[i].x << " " <<subDst[i].y;
//	}
//#endif // DEBUG
	meanStdDev(subDst,errormean,errorstd_dev);

	campara.ReprojectionError[0] = errorstd_dev[0];
	campara.ReprojectionError[1] = errorstd_dev[1];
	campara.totalReproNormErr = totalError;
	campara.reprojectNormErr = absErr;
	//����궨������3����׼���������ô��ģ�
	vector<double> para_error;
	Mat inver_converiance(covariance.rows,covariance.cols,CV_64FC1);
	invert(covariance,inver_converiance);
	Mat diag_covariance(inver_converiance.rows,1,CV_64FC1);
	diag_covariance = inver_converiance.diag(0);//ȡ���Խ���

	for( int row=0;row<diag_covariance.rows;row++ )
	{
		//Ϊʲôֻȡval[0]???
		para_error.push_back( 3*errorstd_dev.val[0]*sqrt(abs(diag_covariance.at<double>(row,0))));

	}
	campara.fcError[0]=para_error[0];
	campara.fcError[1]=para_error[1];
	campara.ccError[0]=para_error[2];
	campara.ccError[1]=para_error[3];
	for(int i=0;i<4;i++)
		campara.kcError[i]=para_error[i+4];
}

bool CoreAlgorithm::isBelong2Vec( vector<int> vec,int index )
{
	for(unsigned int i=0;i<vec.size();i++)
	{
		if( index == vec[i])
			return true;
	}
	return false;
}

void CoreAlgorithm::stereoCalibration(const CamPara& camPara1,const CamPara& camPara2,
		const CalibrationData& cData1,const CalibrationData& cData2,RT& rt)
{
	Mat intrinsicPara_left(3,3,CV_64FC1);
	Mat intrinsicPara_right(3,3,CV_64FC1);
	Mat distCoeffs_left(1,4,CV_64FC1);
	Mat distCoeffs_right(1,4,CV_64FC1);

	for( int i=0; i<3;i++ )
	{
		for( int j=0;j<3;j++ )
		{
			intrinsicPara_left.at<double>(i,j) = camPara1.CameraIntrinsic[i][j];
			intrinsicPara_right.at<double>(i,j) = camPara2.CameraIntrinsic[i][j];
		}
	}
	for( int i=0; i<4;i++ )
	{
		distCoeffs_left.at<double>(0,i) =  camPara1.DistortionCoeffs[i];
		distCoeffs_right.at<double>(0,i) = camPara2.DistortionCoeffs[i];
	}

	Mat R,T,E,F,R_vec;
	//opencv3.0���°汾
	//stereoCalibrate(cData1.plane3dPntsVec,cData1.plane2dPntsVec,cData2.plane2dPntsVec,
	//	intrinsicPara_left,distCoeffs_left,	intrinsicPara_right,distCoeffs_right,
	//	Size(cData1.imgWidth,cData1.imgHeight),R,T,E,F,TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6),
	//	CV_CALIB_FIX_INTRINSIC +
	//	/*CV_CALIB_FIX_ASPECT_RATIO +
	//	CV_CALIB_FIX_FOCAL_LENGTH +
	//	CV_CALIB_FIX_PRINCIPAL_POINT +
	//	CV_CALIB_ZERO_TANGENT_DIST +
	//	CV_CALIB_SAME_FOCAL_LENGTH*/
	//	CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
		stereoCalibrate(cData1.plane3dPntsVec,cData1.plane2dPntsVec,cData2.plane2dPntsVec,
		intrinsicPara_left,distCoeffs_left,	intrinsicPara_right,distCoeffs_right,
		Size(cData1.imgWidth,cData1.imgHeight),R,T,E,F,cv::CALIB_FIX_INTRINSIC +
		/*CV_CALIB_FIX_ASPECT_RATIO +
		CV_CALIB_FIX_FOCAL_LENGTH +
		CV_CALIB_FIX_PRINCIPAL_POINT +
		CV_CALIB_ZERO_TANGENT_DIST +
		CV_CALIB_SAME_FOCAL_LENGTH*/
		cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-6));
	Rodrigues(R,R_vec);

	R_vec.copyTo(rt.R);
	T.copyTo(rt.T);
}



void CoreAlgorithm::removeRepeatPoints( vector<Point2f>& points )
{
	vector<Point2f> returnVec;
	for(unsigned int i=0;i<points.size();i++)
	{
		for (unsigned int j=i+1;j<points.size();j++)
		{
			if( (points[i].x-points[j].x)<1 && (points[i].y-points[j].y)<1 )
			{
				returnVec.push_back(Point2f((points[i].x+points[j].x)/2,(points[i].y+points[j].y)/2));
			}
		}
	}
	points = returnVec;
}

void CoreAlgorithm::detectLightSpot_LED_3(const Mat imgMat, vector<Point2f>& centerPnt, vector<float>& longaxisRadius)
{
	
	///////���ñ����ʶ���㷨���ʶ���ͼ�б���������
	////��ʶ��������־������
	vector<pair<unsigned int, Point2f>>  markerResults;
	bool leftResult = CoreAlgorithm::findCircularMarker(imgMat, markerResults);
    ////�����������С��ΧԲ�ν����Χ���õ�Բ���Լ��뾶��
	vector<Point2f>   markerPnts;
	for (size_t i = 0; i < markerResults.size(); i++)
	{
		markerPnts.push_back(markerResults[i].second);
	}
	
	///������С��ΧԲ�ε�Բ��
	Point2f center_minAreaRectImg;
	float radius_minAreaRectImg;
	minEnclosingCircle(markerPnts, center_minAreaRectImg, radius_minAreaRectImg);
	/////����С��ΧԲ���ú�ɫ����Ϳ�ڣ�
	Mat imgMat1;
	imgMat.copyTo(imgMat1);
	circle(imgMat1, center_minAreaRectImg, radius_minAreaRectImg*1.3, Scalar::all(0),-1);

	float width_height_ratio = 4;
	Mat img_threshold;
	//STEP-1:��ֵ��
	//blur(imgMat1,img_threshold,Size(8,8),Point(-1,-1));
	//imgMat1.copyTo(img_threshold);
	//double minvalue,maxvalue;
	//cv::minMaxLoc(img_threshold,&minvalue,&maxvalue);
	//step-3 �������ݶ������ѡ��
	//step-3-1 ���������ݶȾ�����ж�ֵ��
	//int thresholdvalue = (int)(minvalue+maxvalue)/2;
	threshold(imgMat1, img_threshold, 100, 255, cv::THRESH_BINARY);
	//�ڴ����������ﵱ�й�Ȳ�ͬʱ�����õ���ֵҲ��ͬ��ISO1600Ϊ50���ʡ�
	///// ����Ӧ��ֵ��
	/* int block_size = 145;
	cv::Scalar meanvalue = cv::mean(img_threshold);
	int C_threshold = int(-1*(abs(2*meanvalue.val[0])+13));
	adaptiveThreshold(img_threshold,img_threshold,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,block_size,C_threshold);*/
	//CV_ADAPTIVE_THRESH_MEAN_C:ʹ��3x3���������ص�ƽ��ֵ��ȥ5����Ϊ����Ӧ����ֵ
	//�԰�ɫ���������ٸ�ʴ
	//int close_type = MORPH_ELLIPSE;
	//int dilate_size = 1;
	// Mat element = getStructuringElement(close_type,Size(2*dilate_size+1,2*dilate_size+1),Point(-1,-1));
	//erode(img_threshold, img_threshold, element,Point(-1, -1),1);
	//dilate(img_threshold, img_threshold, element,Point(-1, -1),1);//������
	//STEP-2��Ѱ������
	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;
	findContours(img_threshold, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	////////////////////
	////////qiyong added ��������Բ���������
	vector<RotatedRect> data;
	if (!contours.size())
	{
		return;
	}
	else
	{
		///////STEP4:����Բ��϶�ά�㼯
		for (int i = 0; i < contours.size(); i++)
		{
			data.push_back(fitEllipse(contours[i]));
		}

	}
	/////ended
	/////

	//////�Գ��μ����Ķ�������������ɸѡ���ҵ���ʵİ˸��б�Բ����
	
	///////////////////////
	//STEP-3:��Բ���,������,������ɸѡ
	RotatedRect rotatedBox;
	//vector<RotatedRect> rotatedBoxVec;
	//vector<RotatedRect> temp_rotatedBoxVec;
	for (vector<vector<Point>>::iterator itr = contours.begin(); itr != contours.end(); ++itr)
	{
		static int n = 0;
		//����Բ�����ķ���Խ����˲�����
		int distanceToHead = abs(itr->at(itr->size() - 1).x + itr->at(itr->size() - 1).y - itr->at(0).x - itr->at(0).y);
		if (itr->size()<5 || itr->size()>500 || distanceToHead>4)//�˴���Ҫ���ݹ�����Զ��͹�����������Լ�Բ����ʵ��Сȷ������ֵ
			//if(itr->size()<10||itr->size()>500)//�˴���Ҫ���ݹ�����Զ��͹�����������Լ�Բ����ʵ��Сȷ������ֵ
			continue;
		try
		{
			rotatedBox = fitEllipse((*itr));
			//////qiyong added ������Բ
			//cv::ellipse(img_threshold, rotatedBox, Scalar(0, 0, 255), 1);
			//cv::namedWindow("rightResult", WINDOW_NORMAL);
			//cv::imshow("rightResult", img_threshold);
			//cv::waitKey(600);
		}
		catch (...)//�������ʾ�κ��쳣
		{
			continue;
		}
		//temp_rotatedBoxVec.push_back(rotatedBox);
		float height = rotatedBox.size.height;
		float width = rotatedBox.size.width;
		//�������������ж����Ƿ�Ϊ��Բ
		double Ellipselength, ratiolengthsize;
		double PI = 3.1415926;
		if (height > width)
		{
			Ellipselength = PI*width + 2 * (height - width);
		}
		else
		{
			Ellipselength = PI*height + 2 * (width - height);
		}
		ratiolengthsize = Ellipselength / itr->size();
		n++;
		//�����ߵ����״������Ļ��������������Ҫ��һ��
		if ((height > width ? (height / width<width_height_ratio) : (width / height<width_height_ratio)) && (0.9<ratiolengthsize) && (ratiolengthsize<1.3) && height>5 && width>5)
		{
			//rotatedBoxVec.push_back(rotatedBox);
			centerPnt.push_back(rotatedBox.center);
			if (height > width)
			{
				longaxisRadius.push_back(height/4 + width/4);
			}
			else
			{
				longaxisRadius.push_back(height/4 + width/4);
			}
		}

	}
	return;
	//STEP-4 �Լ��������ĵ���й��˺��޳�

}



bool CoreAlgorithm::detectLightSpot_LED2(const Mat img_threshold, vector<Point2f>& centerPnt, vector<float>& longaxisRadius)
{

	float width_height_ratio = 4;

	//STEP-1:��ֵ��

	//�ڴ����������ﵱ�й�Ȳ�ͬʱ�����õ���ֵҲ��ͬ��ISO1600Ϊ50���ʡ�
	//STEP-2��Ѱ������
	vector<vector<cv::Point>> contoursOrigin;
	vector<cv::Vec4i> hierarchy;
	findContours(img_threshold, contoursOrigin, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	////////////////////
	////////qiyong added ��������Բ���������
	vector<RotatedRect> data;
	if (!contoursOrigin.size())
	{
		return false;
	}
	else
	{
		///////STEP4:����Բ��϶�ά�㼯
		for (int i = 0; i < contoursOrigin.size(); i++)
		{
			if (contoursOrigin[i].size()>=5)
			{
				data.push_back(fitEllipse(contoursOrigin[i]));
			}
			
		}
	}
	/////��������Ϻõ���Բ������ɫ���������Ƴ���  
	/////�Ƚ���ͨ����imgMatͼƬת����Ϊ��ͨ����ͼƬ
	//Mat colorImg;
	//cv::cvtColor(img_threshold, colorImg, CV_GRAY2BGR);
	//for (size_t j = 0; j < data.size(); j++)
	//{
	//	cv::ellipse(colorImg, data[j], Scalar(0, 255, 0), 1);
	//	////�����е���ԲԲ��������ֱ��
	//	string Num = to_string(j);
	//	cv::putText(colorImg, Num, Point(data[j].center.x, data[j].center.y), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0));

	//}
	//////��Ȼ�Ѿ�֪������������Բ�����꣬Ҳ�Ѿ���ͼ�е�����Բ�Ķ��Ѿ�֪����
	//////��ôֻҪ�������е�22�����������ͨ��Լ������ɾ����������ˡ�
	/////�Ƚ�����Բ�ĵ�����ŵ�һ��vector������
	vector<Point2f>  CenterPnts;
	for (size_t j = 0; j < data.size(); j++)
	{
		CenterPnts.push_back(data[j].center);
	}

	////����һ�����������������֮����������
	Mat distanceMatrix1 = Mat::zeros(CenterPnts.size(), CenterPnts.size(), CV_64F);
	Mat distanceMatrix = Mat::zeros(CenterPnts.size(), CenterPnts.size(), CV_64F);
	////step-2 ������ԲԲ��֮��ľ��룬����洢�ھ����С�
	for (int i = 0; i < CenterPnts.size(); i++)
	{
		for (size_t j = i + 1; j < CenterPnts.size(); j++)
		{
			distanceMatrix1.at<double>(i,j)= distancePoints2f(CenterPnts[i], CenterPnts[j]);
		}
	}
	cv::add(distanceMatrix1,distanceMatrix1.t(),distanceMatrix);
	vector<pair<int, vector<int>>>  LightPenPntsVec, OneVec, TwoVec, ThreeVec, FourVec;
	for (size_t i = 0; i < distanceMatrix.cols; i++)
	{
		pair<int, vector<int>> tempdata;
		tempdata.first = i;
		for (size_t j = 0; j < distanceMatrix.cols; j++)
		{
			if (distanceMatrix.at<double>(i, j)<40&&distanceMatrix.at<double>(i, j)>0) ////�˴��Ŀ�ȡ��ΧΪ��30~80
				tempdata.second.push_back(j);
		}
		switch (tempdata.second.size())
		{
		case 1:
			OneVec.push_back(tempdata);
			continue;
		case 2:
			TwoVec.push_back(tempdata);
			continue;
		case 3:
			ThreeVec.push_back(tempdata);
			continue;
		case 4:
			FourVec.push_back(tempdata);
			continue;
		default:
			LightPenPntsVec.push_back(tempdata);
			continue;
		}
	}
	vector<vector<cv::Point>>  contours;
	for (size_t k = 0; k < LightPenPntsVec.size(); k++)
	{
		contours.push_back(contoursOrigin[LightPenPntsVec[k].first]);
	}

	/////ended
	/////

	//////�Գ��μ����Ķ�������������ɸѡ���ҵ���ʵİ˸��б�Բ����
    ///////////////////////
	//STEP-3:��Բ���,������,������ɸѡ
	RotatedRect rotatedBox;
	//vector<RotatedRect> rotatedBoxVec;
	//vector<RotatedRect> temp_rotatedBoxVec;
	for (vector<vector<Point>>::iterator itr = contours.begin(); itr != contours.end(); ++itr)
	{
		static int n = 0;
		//����Բ�����ķ���Խ����˲�����
		int distanceToHead = abs(itr->at(itr->size() - 1).x + itr->at(itr->size() - 1).y - itr->at(0).x - itr->at(0).y);
		if (itr->size()<5 || itr->size()>500 || distanceToHead>4)//�˴���Ҫ���ݹ�����Զ��͹�����������Լ�Բ����ʵ��Сȷ������ֵ
			//if(itr->size()<10||itr->size()>500)//�˴���Ҫ���ݹ�����Զ��͹�����������Լ�Բ����ʵ��Сȷ������ֵ
			continue;
		try
		{
			rotatedBox = fitEllipse((*itr));
			
		}
		catch (...)//�������ʾ�κ��쳣
		{
			continue;
		}
		//temp_rotatedBoxVec.push_back(rotatedBox);
		float height = rotatedBox.size.height;
		float width = rotatedBox.size.width;
		//�������������ж����Ƿ�Ϊ��Բ
		double Ellipselength, ratiolengthsize;
		double PI = 3.1415926;
		if (height > width)
		{
			Ellipselength = PI*width + 2 * (height - width);
		}
		else
		{
			Ellipselength = PI*height + 2 * (width - height);
		}
		ratiolengthsize = Ellipselength / itr->size();
		n++;
		//�����ߵ����״������Ļ��������������Ҫ��һ��
		if ((height > width ? (height / width<width_height_ratio) : (width / height<width_height_ratio)) && height>5 && width>5)
		{
			//rotatedBoxVec.push_back(rotatedBox);
			centerPnt.push_back(rotatedBox.center);
			if (height > width)
			{
				longaxisRadius.push_back(height / 4 + width / 4);
			}
			else
			{
				longaxisRadius.push_back(height / 4 + width / 4);
			}
		}
	}
	return true;
	//STEP-4 �Լ��������ĵ���й��˺��޳�
}


bool CoreAlgorithm::detectLightSpot_LED(const Mat& imgMat,vector<Point2f>& centerPnt, vector<float>& longaxisRadius)
{

	float width_height_ratio = 4;
	Mat img_threshold;
	//STEP-1:��ֵ��
	//blur(imgMat,img_threshold,Size(8,8),Point(-1,-1));
	//imgMat.copyTo(img_threshold);
	//double minvalue,maxvalue;
	//cv::minMaxLoc(img_threshold,&minvalue,&maxvalue);
	//step-3 �������ݶ������ѡ��
	//step-3-1 ���������ݶȾ�����ж�ֵ��
	//int thresholdvalue = (int)(minvalue+maxvalue)/2;
	threshold(imgMat, img_threshold, 80, 255, cv::THRESH_BINARY);
	//�ڴ����������ﵱ�й�Ȳ�ͬʱ�����õ���ֵҲ��ͬ��ISO1600Ϊ50���ʡ�
	///// ����Ӧ��ֵ��
	/* int block_size = 145;
	cv::Scalar meanvalue = cv::mean(img_threshold);
	int C_threshold = int(-1*(abs(2*meanvalue.val[0])+13));
	adaptiveThreshold(img_threshold,img_threshold,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,block_size,C_threshold);*/
	//CV_ADAPTIVE_THRESH_MEAN_C:ʹ��3x3���������ص�ƽ��ֵ��ȥ5����Ϊ����Ӧ����ֵ
	//�԰�ɫ���������ٸ�ʴ
	//int close_type = MORPH_ELLIPSE;
	//int dilate_size = 1;
	// Mat element = getStructuringElement(close_type,Size(2*dilate_size+1,2*dilate_size+1),Point(-1,-1));
	//erode(img_threshold, img_threshold, element,Point(-1, -1),1);
	//dilate(img_threshold, img_threshold, element,Point(-1, -1),1);//������
	//STEP-2��Ѱ������
	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;
	findContours(img_threshold, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	if (contours.size() == 8)
	{
		//STEP-3:��Բ���,������,������ɸѡ
		RotatedRect rotatedBox;
		//vector<RotatedRect> rotatedBoxVec;
		//vector<RotatedRect> temp_rotatedBoxVec;
		for (vector<vector<Point>>::iterator itr = contours.begin(); itr != contours.end(); ++itr)
		{
			static int n = 0;
			//����Բ�����ķ���Խ����˲�����
			int distanceToHead = abs(itr->at(itr->size() - 1).x + itr->at(itr->size() - 1).y - itr->at(0).x - itr->at(0).y);
			if (itr->size()<5 || itr->size()>500 || distanceToHead > 4)//�˴���Ҫ���ݹ�����Զ��͹�����������Լ�Բ����ʵ��Сȷ������ֵ
				//if(itr->size()<10||itr->size()>500)//�˴���Ҫ���ݹ�����Զ��͹�����������Լ�Բ����ʵ��Сȷ������ֵ

				continue;

			try
			{ 
				rotatedBox = fitEllipse((*itr));
			}
			catch (...)//�������ʾ�κ��쳣
			{
				continue;
			}
			//temp_rotatedBoxVec.push_back(rotatedBox);
			float height = rotatedBox.size.height;
			float width = rotatedBox.size.width;
			//�������������ж����Ƿ�Ϊ��Բ
			double Ellipselength, ratiolengthsize;
			double PI = 3.1415926;
			if (height > width)
			{
				Ellipselength = PI*width + 2 * (height - width);
			}
			else
			{
				Ellipselength = PI*height + 2 * (width - height);
			}
			ratiolengthsize = Ellipselength / itr->size();
			n++;
			//�����ߵ����״������Ļ��������������Ҫ��һ��
			if ((height > width ? (height / width < width_height_ratio) : (width / height<width_height_ratio)) && (0.9<ratiolengthsize) && (ratiolengthsize<1.3) && height>5 && width>5)
			{
				//rotatedBoxVec.push_back(rotatedBox);
				centerPnt.push_back(rotatedBox.center);
				if (height > width)
				{
					longaxisRadius.push_back(height / 4 + width / 4);
				}
				else
				{
					longaxisRadius.push_back(height / 4 + width / 4);
				}
			}

		}
	}
	else
	{
		if (contours.size() > 8)
		{
			if (!CoreAlgorithm::detectLightSpot_LED2(img_threshold, centerPnt, longaxisRadius))
			{
				return false;
			}
			if (centerPnt.size() != 8)
			{
				return false;
			}
		}
	
	}
	return true;
	//STEP-4 �Լ��������ĵ���й��˺��޳�

}




bool CoreAlgorithm::Cal3dPointStrLight(const vector<Point2f>& pointCam,const vector<float>& proImg_x,const CamPara& camParaLeft, const CamPara& camParapro,const double rotVector[3] ,const double traVector[3] ,vector<Point3f>& point3d)
{

	point3d.clear();
	//// ���򻯡������ͼ������
	vector<Point2f> NormPixCam;
	double CAMfc[2], CAMcc[2], CAMkc[5], CAMalfpha_c;
	CAMfc[0] = camParaLeft.CameraIntrinsic[0][0];
	CAMfc[1] = camParaLeft.CameraIntrinsic[1][1];
	CAMcc[0] = camParaLeft.CameraIntrinsic[0][2];
	CAMcc[1] = camParaLeft.CameraIntrinsic[1][2];
	CAMalfpha_c = camParaLeft.CameraIntrinsic[0][1];
	for(int i=0;i<4;i++)
	{
		CAMkc[i] = camParaLeft.DistortionCoeffs[i];
	}
	CAMkc[4] = 0;
	normalizPixel(pointCam,NormPixCam,CAMfc, CAMcc, CAMkc, CAMalfpha_c);
	
	Mat Kp(3,3,CV_32FC1); 
	Kp.at<float>(0,0) = float(camParapro.CameraIntrinsic[0][0]);	//// ע�⸳ֵ��ʱ�� at ������y����x
	Kp.at<float>(1,0) = float(camParapro.CameraIntrinsic[1][0]);	
	Kp.at<float>(2,0) = float(camParapro.CameraIntrinsic[2][0]);	
	Kp.at<float>(0,1) = float(camParapro.CameraIntrinsic[0][1]);	
	Kp.at<float>(1,1) = float(camParapro.CameraIntrinsic[1][1]);	
	Kp.at<float>(2,1) = float(camParapro.CameraIntrinsic[2][1]);	
	Kp.at<float>(0,2) = float(camParapro.CameraIntrinsic[0][2]);	
	Kp.at<float>(1,2) = float(camParapro.CameraIntrinsic[1][2]);	
	Kp.at<float>(2,2) = float(camParapro.CameraIntrinsic[2][2]);	

	//���Mat���͵�R��T
	Mat R(3,3,CV_32FC1);
	Mat Rvec(3,1,CV_32FC1);
	for(int i=0;i<3;i++)
	{
		Rvec.at<float>(i,0) = float(rotVector[i]);
	}
	Rodrigues(Rvec,R);  

	Mat T(3,1,CV_32FC1);
	for(int i=0;i<3;i++)
	{
		T.at<float>(i,0) =  float(traVector[i]);
	}

	Mat P(3,3,CV_32FC1);
	P = Kp*R;

	Mat Kt(3,1,CV_32FC1);
	//// ��ֵƽ������
	for(int i=0;i<3;i++)
	{
		Kt.at<float>(i,0) =  float(traVector[i]);
	}
	Kt = Kp*Kt; /// �����P����ĵ�����


	///// ������ά����
	for (unsigned int i=0;i<NormPixCam.size();i++)
	{
		/// ����ϵ�����󣬱�����Kp�������أ�;������ʽ�ұ�������������Rvec�����أ�	
		float temp = proImg_x[i] * Kt.at<float>(2,0)-Kt.at<float>(0,0);

		Kp.at<float>(0,0) = 1; Kp.at<float>(0,1) = 0; Kp.at<float>(0,2) = -NormPixCam[i].x;
		Kp.at<float>(1,0) = 0; Kp.at<float>(1,1) = 1; Kp.at<float>(1,2) = -NormPixCam[i].y;

		Kp.at<float>(2,0) = (P.at<float>(0,0) - proImg_x[i] * P.at<float>(2,0))/temp; 
		Kp.at<float>(2,1) = (P.at<float>(0,1) - proImg_x[i] * P.at<float>(2,1))/temp; 
		Kp.at<float>(2,2) = (P.at<float>(0,2) - proImg_x[i] * P.at<float>(2,2))/temp; 

		Rvec.at<float>(0,0) = 0;
		Rvec.at<float>(1,0) = 0;
		Rvec.at<float>(2,0) = 1;		

		Mat inverA(3,3,CV_32FC1);
		Mat point(3,1,CV_32FC1);

		invert(Kp,inverA,cv::DECOMP_SVD);//�������
	
		point = inverA*Rvec;		
		Point3f pnt;
		if (point.at<float>(2,0)<0)
		{
			pnt.x = -point.at<float>(0,0);
			pnt.y = -point.at<float>(1,0);
			pnt.z = -point.at<float>(2,0);
		}
		else
		{
			pnt.x = point.at<float>(0,0);
			pnt.y = point.at<float>(1,0);
			pnt.z = point.at<float>(2,0);
		}		
		point3d.push_back(pnt);
	}
	return true;
}


bool CoreAlgorithm::Cal3dPoint( const vector<Point2f> pointLeft
								  ,const CamPara& camParaLeft 
								  ,const vector<Point2f> pointRight
								  ,const CamPara& camParaRight
								  ,const double rotVector[3]
								  ,const double traVector[3]
								  ,vector<Point3f>& point3d )
{
	if (pointLeft.size() != pointRight.size())
		return false;

	point3d.clear();
	vector<Point2f> NormPixLeft;
	vector<Point2f> NormPixRight;

	double Rfc[2], Rcc[2], Rkc[5], Ralfpha_c=0;
	double Lfc[2], Lcc[2], Lkc[5], Lalfpha_c=0;
	double L2RRotVector[3];
	double L2RTraVector[3];

	for(int i=0;i<3;i++)
	{
		L2RRotVector[i] = rotVector[i];
		L2RTraVector[i] = traVector[i];
	}
	
	Rfc[0] = camParaRight.CameraIntrinsic[0][0];
	Rfc[1] = camParaRight.CameraIntrinsic[1][1];
	Rcc[0] = camParaRight.CameraIntrinsic[0][2];
	Rcc[1] = camParaRight.CameraIntrinsic[1][2];
	for(int i=0;i<4;i++)
	{
		Rkc[i] = camParaRight.DistortionCoeffs[i];
	}
	Rkc[4] = 0;

	Lfc[0] = camParaLeft.CameraIntrinsic[0][0];
	Lfc[1] = camParaLeft.CameraIntrinsic[1][1];
	Lcc[0] = camParaLeft.CameraIntrinsic[0][2];
	Lcc[1] = camParaLeft.CameraIntrinsic[1][2];
	for(int i=0;i<4;i++)
	{
		Lkc[i] = camParaLeft.DistortionCoeffs[i];
	}
	Lkc[4] = 0;

	//// ���򻯡������ͼ������
	normalizPixel(pointLeft,NormPixLeft,Lfc, Lcc, Lkc, Lalfpha_c);
	//// ���򻯡�ͶӰ��ͼ������
	normalizPixel(pointRight,NormPixRight,Rfc, Rcc, Rkc, Ralfpha_c);

	Mat Kp(3,3,CV_32FC1);

	//���Mat���͵�R��T
	Mat R(3,3,CV_32FC1);
	Mat Rvec(3,1,CV_32FC1);
	for(int i=0;i<3;i++)
	{
		Rvec.at<float>(i,0) =  float(rotVector[i]);
	}
	Rodrigues(Rvec,R);   

	Mat T(3,1,CV_32FC1);
	for(int i=0;i<3;i++)
	{
		T.at<float>(i,0) =  float(traVector[i]);
	}

	//������ά����,ʹ��right�����x�����y����
	for (unsigned int i=0;i<NormPixLeft.size();i++)
	{
		//����ϵ�����󣬱�����Kp�����أ�
		float temp = NormPixRight[i].x * T.at<float>(2,0) - T.at<float>(0,0);//���ֵ��ʲô���壿

		Kp.at<float>(0,0) = 1; Kp.at<float>(0,1) = 0; Kp.at<float>(0,2) = -NormPixLeft[i].x;
		Kp.at<float>(1,0) = 0; Kp.at<float>(1,1) = 1; Kp.at<float>(1,2) = -NormPixLeft[i].y;

		Kp.at<float>(2,0) = (R.at<float>(0,0) - NormPixRight[i].x * R.at<float>(2,0))/temp; 
		Kp.at<float>(2,1) = (R.at<float>(0,1) - NormPixRight[i].x * R.at<float>(2,1))/temp; 
		Kp.at<float>(2,2) = (R.at<float>(0,2) - NormPixRight[i].x * R.at<float>(2,2))/temp; 

		Rvec.at<float>(2,0) = 1;		

		Mat inverA(3,3,CV_32FC1);
		Mat point(3,1,CV_32FC1);

		invert(Kp,inverA);//�������

		inverA.col(2).copyTo(point.col(0));
		Point3f pnt;
		if (point.at<float>(2,0)<0)
		{
			pnt.x = -point.at<float>(0,0);
			pnt.y = -point.at<float>(1,0);
			pnt.z = -point.at<float>(2,0);
		}
		else
		{
			pnt.x = point.at<float>(0,0);
			pnt.y = point.at<float>(1,0);
			pnt.z = point.at<float>(2,0);
		}		
		point3d.push_back(pnt);
	}
	return true;
}

bool CoreAlgorithm::normalizPixel( const vector<Point2f>& src 
										,vector<Point2f>& dst 
										,double fc[2] 
										,double cc[2] 
										,double kc[5] 
										,double alfpha_c )
{
	dst.resize(src.size());
    for (unsigned int i=0;i<src.size();i++)
    {
        //cwqͼ���������굽ͼ�����������ת��
        dst[i].x = (float)((src[i].x-cc[0])/fc[0]);
        dst[i].y = (float)((src[i].y-cc[1])/fc[1]);
        dst[i].x = (float)(dst[i].x - alfpha_c/fc[0]*dst[i].y);//dst[i].x = dst[i].x - alfpha_c*dst[i].y;
    }
    vector<Point2f> temp;
    temp = dst;
    double norm2 = kc[0]*kc[0] + kc[1]*kc[1] + kc[2]*kc[2]+ kc[3]*kc[3]+ kc[4]*kc[4];
    if (norm2>0)
    {
        double r2,k_radial,delta_x,delta_y;
        for (unsigned int i=0;i<dst.size();i++)
        {
            for (int j=0;j<400;j++)///����400�����ǻ�����
            {
                r2 = temp[i].x * temp[i].x + temp[i].y * temp[i].y;
                //����
                k_radial = 1 + kc[0]*r2 + kc[1]*r2*r2 + kc[4]*r2*r2*r2;
                //����
                delta_x = 2*kc[2]*temp[i].x*temp[i].y + kc[3]*(r2+2*temp[i].x*temp[i].x);
                delta_y = kc[2]*( r2 + 2*temp[i].y*temp[i].y ) + 2*kc[3]*temp[i].x*temp[i].y;
                //����У����ʽ
                temp[i].x = (dst[i].x-delta_x)/k_radial;
                temp[i].y = (float)((dst[i].y-delta_y)/k_radial);
            }
        }
        dst = temp;
    }
    return true;
}

vector<Point2f> CoreAlgorithm::extraCorner(const Mat& img,const vector<Point2f>& mousePnts,const int r,const int c)
{
	vector<Point2f> ImageCorner;

	Point2f imgpntlefttop = mousePnts[0];
	Point2f imgpntrighttop = mousePnts[1];
	Point2f imgpntrightdown = mousePnts[2];
	Point2f imgpntleftdown = mousePnts[3];

	Point2f rowLength,colLength_l,colLength_r;
	colLength_l.x = imgpntleftdown.x-imgpntlefttop.x;	
	colLength_l.y = imgpntleftdown.y-imgpntlefttop.y;
	colLength_r.x = imgpntrightdown.x-imgpntrighttop.x;
	colLength_r.y = imgpntrightdown.y-imgpntrighttop.y;
	Point2f tempPnt,leftPnt,rightPnt;

	tempPnt = imgpntlefttop;//cwq��ע��tempPnt��������Ҫpush�ĵ�
	leftPnt = imgpntlefttop;
	rightPnt = imgpntrighttop;

	vector<Point2f> col_Pnt;// �нǵ�
	for (int row=1;row<r+1;row=row+1)
	{	
		//col_Pnt.clear();
		for (int col=1;col<c+1;col=col+1)
		{
			//����õ㵽��
			//col_Pnt.push_back(tempPnt);
			ImageCorner.push_back(tempPnt);
			//cwq��ע������߳���x������y����
			rowLength.x = rightPnt.x-leftPnt.x;
			rowLength.y = rightPnt.y-leftPnt.y;
			//cwq��ע���еĳ�ʼ������������ϵ���
			tempPnt.x = int ((float)col/(float)(c-1)*(float)rowLength.x)+leftPnt.x;
			tempPnt.y = int ((float)col/(float)(c-1)*(float)rowLength.y)+leftPnt.y;
		}
		//ImageCorner.push_back(col_Pnt);///�����б���
		if (row<r-1)//����������һ��
		{
			//cwq��ע���ƶ��е����Ҷ˵㵽��һ��
			leftPnt.x = int ((float)row/(float)(r-1)*(float)colLength_l.x)+imgpntlefttop.x;
			leftPnt.y = int ((float)row/(float)(r-1)*(float)colLength_l.y)+imgpntlefttop.y;
			rightPnt.x = int ((float)row/(float)(r-1)*(float)colLength_r.x)+imgpntrighttop.x;
			rightPnt.y = int ((float)row/(float)(r-1)*(float)colLength_r.y)+imgpntrighttop.y;			
		}
		else//cwq��ע����������һ��
		{
			leftPnt = imgpntleftdown;
			rightPnt = imgpntrightdown;
		}
		
		tempPnt = leftPnt;		
	}
	//*********************************************************************************//

	//�����ؽǵ���ȡ
	Mat matImg_gray;
	cvtColor(img, matImg_gray, CV_BGR2GRAY);

	cornerSubPix(matImg_gray,ImageCorner, cvSize(12, 12), cvSize(-1, -1)
		,cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 60, 0.01));

	return ImageCorner;
}

bool CoreAlgorithm::extraCorner2(const Mat& img,const vector<Point2f>& mousePnts,vector<Point2f>& ImageCorner,int &row, int &col)
{
	Point2f imgpntlefttop = mousePnts[0];
	Point2f imgpntrighttop = mousePnts[1];
	Point2f imgpntrightdown = mousePnts[2];
	Point2f imgpntleftdown = mousePnts[3];

	//double dircol,dirrow;
	//double tranXY1[2],tranXY2[2];

	double lengthrow,lengthcol_l,lengthcol_r;
	lengthrow = sqrtf((imgpntrighttop.x - imgpntlefttop.x) * (imgpntrighttop.x - imgpntlefttop.x) + (imgpntrighttop.y - imgpntlefttop.y) * (imgpntrighttop.y - imgpntlefttop.y));
	lengthcol_l = sqrtf((imgpntleftdown.x - imgpntlefttop.x) * (imgpntleftdown.x - imgpntlefttop.x) + (imgpntleftdown.y - imgpntlefttop.y) * (imgpntleftdown.y - imgpntlefttop.y));
	lengthcol_r = sqrtf((imgpntrightdown.x - imgpntrighttop.x) * (imgpntrightdown.x - imgpntrighttop.x) + (imgpntrightdown.y - imgpntrighttop.y) * (imgpntrightdown.y - imgpntrighttop.y));

	Mat ImageGray;
	cvtColor(img, ImageGray, CV_BGR2GRAY);

	row = 0;
	col = 0;
	vector<Point2f> cornercol_l,conercol_r;

	if ( !(GetCornerAtRow(ImageGray,imgpntlefttop, imgpntleftdown, cornercol_l) 
		&& GetCornerAtRow(ImageGray,imgpntrighttop, imgpntrightdown, conercol_r)) )
	{
		return false;
	}

	if (cornercol_l.size() == conercol_r.size())
	{
		row = (int)cornercol_l.size();    // ��ýǵ��з������
	}
	else
	{
		return false;
	}

	for (int i = 0; i < row; i++)
	{
		Point2f templ,tempr;
		templ = cornercol_l[i];
		tempr = conercol_r[i];
		vector<Point2f> cornerrow;
		GetCornerAtRow(ImageGray,templ, tempr, ImageCorner);
		if (i==0)
			col = ImageCorner.size();				// ��ýǵ��з������
	}

	if (ImageCorner.size() != HEIGHT_CORNER_COUNT*WIDTH_CORNER_COUNT)
		return false;

	cornerSubPix(ImageGray,ImageCorner, cvSize(10, 10), cvSize(-1, -1)
		,cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 60, 0.01));

	////Ϊ�����Զ���ȡ�Ľ���˳���������תcorner��˳��
	//vector<Point2f> dst;
	//for (int i = ImageCorner.size()-1; i >=0 ;i--)
	//{
	//	dst.push_back(ImageCorner[i]);
	//}
	//ImageCorner = dst;
	return true;
}

bool CoreAlgorithm::GetCornerAtRow(const Mat& ImageGray, Point2f imgpnt_left, Point2f imgpnt_right, vector<Point2f> &cornerrow)
{
	double threhold;    // ����Ӧȷ����ֵ������ǰ��������ȷ��
	bool signwb;
	int height = ImageGray.rows;
	int width = ImageGray.cols;
	int step = ImageGray.step;
	int chennels = ImageGray.channels();
	uchar* ImageGray_data = (uchar*)ImageGray.data;
	double pixel;

	float dir[2],length;
	dir[0] = imgpnt_right.x - imgpnt_left.x;
	dir[1] = imgpnt_right.y - imgpnt_left.y;
	length = sqrtf(dir[0] * dir[0] + dir[1] * dir[1]);

	//cwq��ֹ�����غ�
	if( 0 == length)
	{
		return false;
	}
	dir[0] = dir[0] / length;//cwq  sin\cos
	dir[1] = dir[1] / length;
	// �趨��ʼ��־
	pixel = 0;
	//cwq ѭ�����25������ֵ��������ƽ����ֵ
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j > -5; j--)
		{
			// ��dir��ֱ�ķ����ϼ�3�������Է������ˡ����⣬ͼ����step�൱��y
			//3/22 cwq����ǳ���ͼ���Ե
			if( (imgpnt_left.y - 10 * dir[1] + 10 * dir[0] + j) < 0  || (imgpnt_left.x - 10 * dir[0] - 10 * dir[1] + i ) < 0 )
			{
				return false;
			}
			pixel = pixel + ImageGray_data[(int)(imgpnt_left.y - 10 * dir[1] + 10 * dir[0] + j) * step + (int)(imgpnt_left.x - 10 * dir[0] - 10 * dir[1] + i) * chennels];
		}
	}
	double pretopleft = pixel / 25;
	pixel = 0;
	for (int i = 0;i < 5; i++)
	{
		for (int j = 0;j < 5; j++)
		{
			// ��dir��ֱ�ķ����ϼ�3�������Է������ˡ����⣬ͼ����step�൱��y
			pixel = pixel + ImageGray_data[(int)(imgpnt_left.y + 10 * dir[1] + 10 * dir[0] + j) * step + (int)(imgpnt_left.x + 10 * dir[0] - 10 * dir[1] + i) * chennels];
		}
	}
	double aftertopleft = pixel / 25;
	threhold = (pretopleft + aftertopleft) / 2;
	if (pretopleft < threhold)
	{
		signwb = FALSE;
	}
	else
	{
		signwb = TRUE;
	}

	// Ѱ�ҽǵ�
	for (int l = 0;l < length + 5; l++)
	{
		Point2f presentpnt;
		//cwq����sin��cos���ɨ��ֱ���ϵ���һ��
		presentpnt.x = float(imgpnt_left.x + l * dir[0]);
		presentpnt.y = float(imgpnt_left.y + l * dir[1]);
		pixel = 0;
		for (int i = 0;i < 5; i++)
		{
			for (int j = 0;j < 5; j++)
			{
				// ��dir��ֱ�ķ����ϼ�3�������Է������ˡ����⣬ͼ����step�൱��y
				pixel = pixel+ImageGray_data[(int)(presentpnt.y + 10 * dir[0] + j) * step + (int)(presentpnt.x - 10 * dir[1] + i) * chennels];
			}
		}
		if (pixel / 25 < threhold && signwb == TRUE)
		{
			signwb = FALSE;
			cornerrow.push_back(presentpnt);
			// �������ֵ���ƣ���Ծһ������
			l = l + 4;
			continue;
		}
		if (pixel / 25 > threhold && signwb == FALSE)
		{
			signwb = TRUE;
			cornerrow.push_back(presentpnt);
			// �������ֵ���ƣ���Ծһ������
			l = l + 5;
		}
	}

	return TRUE;
}



bool CoreAlgorithm::rigidTransform(const std::vector<Point3f>& oriPoints,
					const std::vector<Point3f>& terminatePoints,
					cv::Mat& Rot,cv::Mat& Tra)
{
	//�����������Ŀ�Ƿ���ͬ,�Ҳ�Ϊ0
	if( oriPoints.size()<3 || terminatePoints.size()<3 )
		return FALSE;
	if( oriPoints.size() != terminatePoints.size() )
		return FALSE;
	
	//��õ������
	int pointNum = oriPoints.size();
	vector<Point3f> X1 = oriPoints;
	vector<Point3f> X2 = terminatePoints;

	//����ʼ�����ֹ�������
	Point3f C1(0,0,0),C2(0,0,0);
	for( int i=0; i<pointNum; i++ )
	{
		C1 += X1[i];
		C2 += X2[i];
	}
	C1.x = C1.x/pointNum;	C2.x = C2.x/pointNum;
	C1.y = C1.y/pointNum;	C2.y = C2.y/pointNum;
	C1.z = C1.z/pointNum;	C2.z = C2.z/pointNum;

	//ԭʼ���ȥ���ĵ����꣬����µĵ�
	for( int i=0; i<pointNum; i++ )
	{
		X1[i] -= C1;
		X2[i] -= C2;
	}

	//����N����
	double Sxx, Sxy, Sxz, Syx, Syy, Syz, Szx, Szy, Szz;
	Sxx=Sxy=Sxz=Syx=Syy=Syz=Szx=Szy=Szz=0;

	for( int i=0; i<pointNum; i++ )
	{
		Sxx += X2[i].x * X1[i].x;	
		Sxy += X2[i].x * X1[i].y;	
		Sxz += X2[i].x * X1[i].z;

		Syx += X2[i].y * X1[i].x;
		Syy += X2[i].y * X1[i].y;
		Syz += X2[i].y * X1[i].z;

		Szx += X2[i].z * X1[i].x;
		Szy += X2[i].z * X1[i].y;
		Szz += X2[i].z * X1[i].z;
	}

	//��N����ֵ
	Mat N(4,4,CV_64FC1);
	N.at<double>(0,0) = Sxx + Syy + Szz;
	N.at<double>(0,1) = Syz - Szy;
	N.at<double>(0,2) = Szx - Sxz;
	N.at<double>(0,3) = Sxy - Syx;

	N.at<double>(1,0) = Syz - Szy;
	N.at<double>(1,1) = Sxx - Syy - Szz;
	N.at<double>(1,2) = Sxy + Syx;
	N.at<double>(1,3) = Szx + Sxz;

	N.at<double>(2,0) = Szx - Sxz;
	N.at<double>(2,1) = Sxy + Syx;
	N.at<double>(2,2) = Syy - Sxx - Szz;
	N.at<double>(2,3) = Syz + Szy;

	N.at<double>(3,0) = Sxy - Syx;
	N.at<double>(3,1) = Szx + Sxz;
	N.at<double>(3,2) = Syz + Szy;
	N.at<double>(3,3) = Szz - Sxx - Syy;

	//����N�������������������ֵ
	vector<double> eigenvalues;
	Mat eigenvectors;
	if( !eigen(N,eigenvalues,eigenvectors) )
		return FALSE;

	//�����������ֵ��Ӧ����������
	double maximal_vaule = -1;
	int maximal_index = 0;
	for(unsigned int i=0; i<eigenvalues.size(); i++ )
	{
		if( eigenvalues[i]>maximal_vaule && eigenvalues[i]>0 )
		{
			maximal_vaule = eigenvalues[i];
			maximal_index = i;
		}
	}

	if ( maximal_vaule <= 0 )
	{
		return FALSE;
	}

	//unit quaternion Q
	double Q0,Q1,Q2,Q3;
	Q0 = eigenvectors.at<double>(maximal_index,0);
	Q1 = eigenvectors.at<double>(maximal_index,1);
	Q2 = eigenvectors.at<double>(maximal_index,2);
	Q3 = eigenvectors.at<double>(maximal_index,3);

	//������ת����
	Rot.create(3,3,CV_64FC1);
	Rot.at<double>(0,0) = Q0*Q0 + Q1*Q1 - Q2*Q2 - Q3*Q3;
	Rot.at<double>(0,1) = 2 * (Q1*Q2 - Q0*Q3);
	Rot.at<double>(0,2) = 2 * (Q1*Q3 + Q0*Q2);

	Rot.at<double>(1,0) = 2 * (Q2*Q1 + Q0*Q3);
	Rot.at<double>(1,1) = Q0*Q0 - Q1*Q1 + Q2*Q2 - Q3*Q3 ;
	Rot.at<double>(1,2) = 2 * (Q2*Q3 - Q0*Q1) ;

	Rot.at<double>(2,0) = 2 * (Q3*Q1 - Q0*Q2);
	Rot.at<double>(2,1) = 2 * (Q3*Q2 + Q0*Q1);
	Rot.at<double>(2,2) = Q0*Q0 - Q1*Q1 - Q2*Q2 + Q3*Q3;

	//����ƽ�ƾ���
	Mat C1_mat(3,1,CV_64FC1);
	Mat C2_mat(3,1,CV_64FC1);
	C1_mat.at<double>(0,0) = C1.x;
	C1_mat.at<double>(1,0) = C1.y;
	C1_mat.at<double>(2,0) = C1.z;

	C2_mat.at<double>(0,0) = C2.x;
	C2_mat.at<double>(1,0) = C2.y;
	C2_mat.at<double>(2,0) = C2.z;

 	transpose(Rot,Rot);
	gemm(Rot,C1_mat,-1,C2_mat,1,Tra); //Tra = C2_mat - Rot*C1_mat

	return TRUE;
}

bool CoreAlgorithm::calculateProbeCenter(const vector<Mat>& R_vec, const vector<Mat>& T_vec,Point3f& center)
{
	if (R_vec.size() != T_vec.size()
		|| R_vec.size() == 0)
	{
		return false;
	}
	if(R_vec.size() < 2)
	{
		return false;
	}
	Mat A = Mat::eye(3,3,CV_64F);
	Mat src1(3*(R_vec.size()),3,CV_64F);
	Mat src2(3*(R_vec.size()),1,CV_64F);
	int row = 0;
	for (unsigned int i = 0; i < R_vec.size(); i++)
	{
		Mat subMat1 = A- R_vec[i];
		subMat1.rowRange(0, 3).copyTo(src1.rowRange(row,row+3));
		Mat subMat2 = T_vec[i];
		subMat2.rowRange(0, 3).copyTo(src2.rowRange(row,row+3));
		row += 3;
	}

	Mat dst;
	solve(src1,src2,dst,cv::DECOMP_SVD);
	center = Point3f(dst);
	return true;
}



bool CoreAlgorithm::getPntsWithTag(const vector<Point2f>& L_pnts, const vector<Point2f>& S_pnts,vector<TagPoint2f>& tagPnts)
{
	if (L_pnts.size() < 4 || S_pnts.size()<1)
		return false;

	//STEP-1:�����ֱ�����
	Vec4f L_Axis;//(xx,xy)ǰ����Ϊ��ֱ��ƽ�еĵ�λ������(x0,y0)������Ϊֱ���ϵ�һ����
	float xx, xy, x0, y0;
	fitLine(L_pnts, L_Axis, CV_DIST_L2, 0, 0.01, 0.01);//��С�������
	xx = L_Axis[0]; xy = L_Axis[1]; x0 = L_Axis[2]; y0 = L_Axis[3];

	////// zhang xu added  �ж�ֱ�ߵķ����ǲ���1-5��ķ���
	int l_num = L_pnts.size() - 1;
	float pnt15x,pnt15y;
	pnt15x = L_pnts[l_num].x - L_pnts[0].x;
	pnt15y = L_pnts[l_num].y - L_pnts[0].y;
	float dis = pnt15x*xx + pnt15y*xy;
	if(dis<0)  //// ���L_pnts˳�� ���Ǵ�1��5��˳������
	{
		xx = -xx;
		xy = -xy;
	}
	////// end


	//STEP-2:���ʮ�ֽ���㣨�飩�������㷨���Ͽ��ģ���û�����Ͳ���
	Point2f crossPnt;
	Point2f interceptPnt;//����ֱ����Y��ǵ�
	interceptPnt.x = 0; interceptPnt.y = y0 - (xy/xx) * x0;//y1=y0-(xy/xx)*x0;

	Point2f pt1, pt2, pt3;//pt1,pt2Ϊֱ�������㣬pt3Ϊֱ����һ��
	pt1 = interceptPnt; pt2.x = x0; pt2.y = y0; pt3 = S_pnts[0];
	double dba,dbb;  
	//a = sqr( (x2 - x1)^2 +(y2 - y1)^2 )  
	dba = sqrt((long double)((pt2.x - pt1.x) * (pt2.x - pt1.x) + (pt2.y - pt1.y) * (pt2.y - pt1.y) ));  
	//b = (x2-x1) * (x3-x1) +(y2 -y1) * (y3 -y1)  
	dbb = ((pt2.x - pt1.x) * (pt3.x -pt1.x) + (pt2.y - pt1.y) * (pt3.y - pt1.y) );  
	//a = b / (a*a)  
	dba = dbb / (dba * dba);  
	//x4 = x1 +(x2 - x1)*a  
	crossPnt.x = float(pt1.x + (pt2.x - pt1.x) * dba);  
	//y4 = y1 +(y2 - y1)*a  
	crossPnt.y = float(pt1.y + (pt2.y - pt1.y) * dba);

	//STEP-3:�����ϵĵ㣬��ʮ�ֽ������߾���tag6���ұ߾���tag7
	////// zhang xu added  �жϳ���ֱ�ߵķ��� �� tag6  tag7  �ķ��� ���  �ǲ���1-5��ķ���
	Point2f p0;  //// �����ϵĵ�
	int signShort = 1; ////1��ʾp0ѡ����� tag5�� -1 ��ʾp0ѡ�����tag6
	p0 = S_pnts[0];

	
	//����AB����x1,y1,z1��, ����CD����x2,y2,z2��
	//����AB������CD����y1z2-z1y2��z1x2-x1z2��x1y2-y1x2��
	Vec3f norm_X,norm_Y,norm_Z;
	norm_X[0] = xx;  norm_X[1] = xy; norm_X[2] = 0; 
	norm_Y[0] = p0.x - crossPnt.x;  norm_Y[1] = p0.y - crossPnt.y; norm_Y[2] = 0; 

	norm_Z[0] = norm_X[1] * norm_Y[2] - norm_X[2] * norm_Y[1];
	norm_Z[1] = norm_X[2] * norm_Y[0] - norm_X[0] * norm_Y[2];
	norm_Z[2] = norm_X[0] * norm_Y[1] - norm_X[1] * norm_Y[0];	

	////// end
	for (unsigned int i = 0; i < S_pnts.size(); i++)
	{
		TagPoint2f _tagPnt;

		_tagPnt[1] = S_pnts[i].x;
		_tagPnt[2] = S_pnts[i].y;

		if(norm_Z[2]>0 && i==0) //// ��˺��Z��Ч��Z�᷽����˵��p0 ��tag6�� ������Ϊtag7
		{
			_tagPnt[0] = TAG6;
			tagPnts.push_back(_tagPnt);
		}
		if(norm_Z[2]>0 && i==1) ////��˺��Z��Ч��Z�᷽�� �ڶ�������Tag7
		{
			_tagPnt[0] = TAG7;
			tagPnts.push_back(_tagPnt);
		}
		if(norm_Z[2]<0 && S_pnts.size()==1)   ///// ��˺���z������S_pnts.size()  ֻ��1�������Ȼ��TAG7
		{
			_tagPnt[0] = TAG7;
			tagPnts.push_back(_tagPnt);
		}

		//if (S_pnts[i].x < crossPnt.x)//��ʮ�ֽ������
		//	_tagPnt[0] = TAG6;
		//else//��ʮ�ֽ����ұ�
		//	_tagPnt[0] = TAG7;

	}
	//step-4 

	////// zhangxu added
	float unit_dis;//������ĳ�㵽ʮ�ֽ����ľ���
	unit_dis = sqrt((S_pnts[0].x - crossPnt.x)*(S_pnts[0].x - crossPnt.x)
		+ (S_pnts[0].y - crossPnt.y)*(S_pnts[0].y - crossPnt.y));
	
	//�жϳ���������ʮ�ֽ���㣨ʵ�����У��õ�Ϊtag1��//chengwei changed
	for(unsigned int i = 0; i < L_pnts.size(); i++)
	{
		TagPoint2f _tagPnt;
		_tagPnt[1] = L_pnts[i].x;
		_tagPnt[2] = L_pnts[i].y;

		float _d = sqrt((L_pnts[i].x - crossPnt.x)*(L_pnts[i].x - crossPnt.x)
				+ (L_pnts[i].y - crossPnt.y)*(L_pnts[i].y - crossPnt.y));
		float _k = _d / unit_dis;
		if(_k<0.5)
		{
			_tagPnt[0] = TAG1;//chengwei changed
			tagPnts.push_back(_tagPnt);
			crossPnt = L_pnts[i];
			break;
		}
	}
	//// ���� cross point �γɵ�������  xx  xy ����һ�£�����Ϊ tag1 ���� Ϊ tag3 tag4  tag5
	//���� cross point �γɵ�������  xx  xy �����෴������Ϊ Ϊ tag2 tag3 tag4  tag5 //chengwei changed
	/////////////////chengwei added///////////
	vector<float> _ks;
	vector<int> tags;
	for(int i =0;i<5;i++)
	{
		tags.push_back(i);
	}
	/////////////////chengwei added///////////
	for (unsigned int i = 0; i < L_pnts.size(); i++)
	{
		//TagPoint2f _tagPnt;
		//_tagPnt[1] = L_pnts[i].x;
		//_tagPnt[2] = L_pnts[i].y;

		Point2f pnt2cross; //// �ɵ㵽cross ���ɵĵ�
		pnt2cross.x = crossPnt.x - L_pnts[i].x;
		pnt2cross.y = crossPnt.y - L_pnts[i].y;

		double dis = pnt2cross.x*xx + pnt2cross.y*xy;  /// �����������

		float _d = sqrt((L_pnts[i].x - crossPnt.x)*(L_pnts[i].x - crossPnt.x)
				+ (L_pnts[i].y - crossPnt.y)*(L_pnts[i].y - crossPnt.y));
		float _k = _d / unit_dis;
		_ks.push_back(_k);//chengwei added
	}
	///////////////////////chengwei added//////////////////////////////////
	for(unsigned int i=0;i< _ks.size()-1;i++)
	{
		for (unsigned int j = i+1; j<_ks.size(); j++)
		{
			if (_ks[i] > _ks[j])
			{
				swap(tags[i], tags[j]);
			}
		}
	}

	for(unsigned int i =1;i<tags.size();i++)
	{
		TagPoint2f _tagPnt;
		_tagPnt[0] = float(tags[i]);
		_tagPnt[1] = L_pnts[tags[i]].x;
		_tagPnt[2] = L_pnts[tags[i]].y;
		tagPnts.push_back(_tagPnt);
	}
	///////////////////////chengwei added//////////////////////////////////
//�Ƿ�������������   //chengwei changed
		//if (dis>0 && _k>0.5) ////tag1--��cross  ��������xx  xy ����һ�£�������Ϊ��,��Ϊtag1
		//{
		//	_tagPnt[0] = TAG1;
		//	tagPnts.push_back(_tagPnt);
		//}
		//else ////���� Ϊ tag3 tag4  tag5
		//{


		//	/*if (_k>0.5 && _k<1.5)*/
		//if (_k>0.5 && _k<1.75)
		//	{
		//		_tagPnt[0] = TAG2;
		//		tagPnts.push_back(_tagPnt);
		//	}
		//	/*if (_k>1.5 && _k < 2.5)*/
		//if (_k>1.75 && _k < 3.00)
		//	{
		//		_tagPnt[0] = TAG3;
		//		tagPnts.push_back(_tagPnt);
		//	}
		//	/*if (_k>2.5 && _k < 3.5)*/
		//if (_k>3.00 && _k < 4.25)
		//	{
		//		_tagPnt[0] = TAG4;
		//		tagPnts.push_back(_tagPnt);
		//	}
		//	//chengwei added
		//	/*if (_k>3.5 && _k < 4.5)*/
		//if (_k>4.25 && _k <5.50)
		//	{
		//		_tagPnt[0] = TAG5;
		//		tagPnts.push_back(_tagPnt);
			/*}*/
	//	}
		

	//}
	///// �Ա�Ǻõĵ㣬���ձ�ǵ��������
	for(unsigned int i=0;i<tagPnts.size()-1;i++)
	{
		for (unsigned int j = i+1; j<tagPnts.size(); j++)
		{
			if (tagPnts[i][0] > tagPnts[j][0])
			{
				swap(tagPnts[i], tagPnts[j]);
			}
		}
	}
	return true;
	///// ended
}
bool CoreAlgorithm::findPntsWithTag(vector<Point2f>& centerPnt,vector<float>& longaxisRadius,vector<TagPoint2f>& tagPnts,float &firstFeaturelength,const double gamma)
{
	if(centerPnt.size()<7)
		return false;
	vector<TagPoint2f> TagPnts;
	if(centerPnt.size()!=longaxisRadius.size())
		return false;
	int pntsSize = centerPnt.size();
	Mat distanceMatrix1 = Mat::zeros(pntsSize,pntsSize,CV_64F);
	Mat distanceMatrix = Mat::zeros(pntsSize,pntsSize,CV_64F);
	//step-2 ������Բ����֮��ľ��벢��������Բ����뾶������洢�ھ�����
	for(int i = 0;i<pntsSize;i++)
	{
		for(int j=i+1;j<pntsSize;j++)
		{
			distanceMatrix1.at<double>(i,j) = distancePoints2f(centerPnt[i],centerPnt[j])/(longaxisRadius[i]*gamma);
		}
	}
	Mat distanceMatrix1_T = distanceMatrix1.t();
	cv::add(distanceMatrix1,distanceMatrix1_T,distanceMatrix);
	//step-3 �Ӿ����е��ҳ���������Ԫ��ֵ��0.7-1.3֮�����������ӦԪ������Ӧ���б꼰�б�
	vector<int> pointsIndextemp;
	int rowIndex=0;
	for(int i= 0;i<distanceMatrix.rows;i++)
	{
		rowIndex=i;
		pointsIndextemp.clear();
		int n=0;
		for(int j=0;j<distanceMatrix.cols;j++)
		{
			if(distanceMatrix.at<double>(i,j)>0.7&&1>distanceMatrix.at<double>(i,j)
				&&longaxisRadius[i]/longaxisRadius[j]>0.8&&longaxisRadius[i]/longaxisRadius[j]<1.2)
			{
				n++;
				pointsIndextemp.push_back(j);
			}
		}
		if(n>=3)
		{
			pair<int,vector<int>> possiblefistPoints;
			possiblefistPoints.first=i;
			possiblefistPoints.second=pointsIndextemp;
			//�ж�1267��
			vector<Point2f> footpointNearpoints;
			TagPnts.clear();
			for(unsigned int i = 0;i<possiblefistPoints.second.size();i++)
			{
				footpointNearpoints.push_back(centerPnt[possiblefistPoints.second[i]]);
			}
			if(!(findshortAxisPoints(footpointNearpoints[0],footpointNearpoints[1],footpointNearpoints[2],centerPnt[possiblefistPoints.first],TagPnts)))
			{
				continue;
			}
			firstFeaturelength = longaxisRadius[possiblefistPoints.first];
			if(!TagPnts.size())
			{
				continue;
			}
				//�������12����ȷ���ĳ���ֱ�ߣ���������ϵĵ�
			float dis_threshold = 10;
			vector<Point2f> longAxisPntsVec;
			float a, b;//ֱ�߹�ʽax+by+1=0;a=(y1-y2)/(x1y2-x2y1),b=(x1-x2)/(x2y1-x1y2);
			Point2f pnt1 = Point2f(TagPnts[0].val[1],TagPnts[0].val[2]);
			Point2f pnt2 = Point2f(TagPnts[1].val[1],TagPnts[1].val[2]);
			Mat ZaxisVector = Mat::zeros(3,1,CV_64F);
			Mat axisVectorTemp = Mat::zeros(3,1,CV_64F);
			a = (pnt1.y - pnt2.y) / (pnt1.x*pnt2.y - pnt2.x*pnt1.y);
			b = (pnt1.x - pnt2.x) / (pnt2.x*pnt1.y - pnt1.x*pnt2.y);
			ZaxisVector.at<double>(0,0)= pnt2.x - pnt1.x;
			ZaxisVector.at<double>(1,0)= pnt2.y - pnt1.y;
			for (size_t n = 0; n < centerPnt.size(); n++)
			{
				//�㵽ֱ�ߵľ���d=fabs(a*x+b*y+1)/sqrt(a*a+b*b);

				float dis = float(fabs(a*centerPnt[n].x + b*centerPnt[n].y + 1) / sqrt(a*a + b*b));
				axisVectorTemp.at<double>(0,0)= centerPnt[n].x-pnt1.x;
				axisVectorTemp.at<double>(1,0)= centerPnt[n].y-pnt1.y;
				double tempnum = ZaxisVector.dot(axisVectorTemp);
				if (dis < dis_threshold&&tempnum>0)//�������ҵĲ�׼��ʱ�������ֵҪ�ſ�һ��
				{
					longAxisPntsVec.push_back(centerPnt[n]);
				}
			}
			//step-5 ȷ����3��4��5��
			float unit_dis;//2�㵽1��ľ�����Ϊ�Ƚϵ�λ����
			unit_dis = (float)sqrt(pow((TagPnts[1][1] - TagPnts[0][1]),2)+ pow((TagPnts[1][2] - TagPnts[0][2]),2));
			for (unsigned int i = 0; i < longAxisPntsVec.size(); i++)
			{
				TagPoint2f _tagPnt;
				float dis = (float)sqrt(pow((longAxisPntsVec[i].x - TagPnts[0][1]),2)+pow((longAxisPntsVec[i].y - TagPnts[0][2]),2));
				float _k = dis/ unit_dis;
				if (_k>1.8 && _k<2.2)
					{
						_tagPnt[0] = TAG3;
						_tagPnt[1] =longAxisPntsVec[i].x;
						_tagPnt[2] =longAxisPntsVec[i].y;
						TagPnts.push_back(_tagPnt);
					}	
				if (_k>2.8 && _k < 3.2)
					{
						_tagPnt[0] = TAG4;
						_tagPnt[1] =longAxisPntsVec[i].x;
						_tagPnt[2] =longAxisPntsVec[i].y;
						TagPnts.push_back(_tagPnt);
					}
				if (_k>3.7 && _k < 4.2)
					{
						_tagPnt[0] = TAG5;
						_tagPnt[1] =longAxisPntsVec[i].x;
						_tagPnt[2] =longAxisPntsVec[i].y;
						TagPnts.push_back(_tagPnt);
					}
			}
			if(TagPnts.size()!=7)
			{
				if(rowIndex==distanceMatrix.rows-1)
				{
					return false;
				}
				else
				{
					continue;
				}
			}
			else
			{
				break;
			}
		}
	}
	// step-6�Ա�Ǻõĵ㣬���ձ�ǵ��������
			//��С��������//chengwei added
	if(TagPnts.size()!=7)
	{
		return false;
	}
	for(unsigned int i=0;i<TagPnts.size()-1;i++)
	{
		for (unsigned int j = i+1; j<TagPnts.size(); j++)
		{
			if (TagPnts[i][0] > TagPnts[j][0])
			{
				swap(TagPnts[i], TagPnts[j]);
			}
		}
	}
	tagPnts = TagPnts;
	return true;
}
bool CoreAlgorithm::pointsSort2D_3(vector<Point2f>& pnts,vector<Point2f>& L_pnts,vector<Point2f>& S_pnts)
{
	if( pnts.size()<6 )
		return false;
	L_pnts.clear();
	S_pnts.clear();
	float dis_threshold = 40;
	//float noisePnt_threshold = 100;
	//float dis_threshold = 15;
	float noisePnt_threshold = 100;
	int pntsSize = pnts.size();
	vector<int> longAxisPntsIndexVec;
	vector<int> noisePntsIndexVec;
	//STEP-1 �ҳ����ڳ����ϵĵ�,�ж�ʣ�µĵ��Ƿ���һ��ֱ����
	for(int i=0;i<pntsSize;i++)
	{
		int count;//���ڳ����ϵĵ����
		for (int j = i + 1; j < pntsSize; j++)
		{
			count = 0;	
			noisePntsIndexVec.clear();
			longAxisPntsIndexVec.clear();
			float a, b;//ֱ�߹�ʽax+by+1=0;a=(y1-y2)/(x1y2-x2y1),b=(x1-x2)/(x2y1-x1y2);
			Point2f pnt1 = pnts[i];
			Point2f pnt2 = pnts[j];
			a = (pnt1.y - pnt2.y) / (pnt1.x*pnt2.y - pnt2.x*pnt1.y);
			b = (pnt1.x - pnt2.x) / (pnt2.x*pnt1.y - pnt1.x*pnt2.y);

			for (unsigned int n = 0; n < pnts.size(); n++)
			{
				//�㵽ֱ�ߵľ���d=fabs(a*x+b*y+1)/sqrt(a*a+b*b);
				float dis = fabs(a*pnts[n].x + b*pnts[n].y + 1) / sqrt(a*a + b*b);
				if (dis < dis_threshold)//�������ҵĲ�׼��ʱ�������ֵҪ�ſ�һ��
				{
					count++;
					longAxisPntsIndexVec.push_back(n);
				}
				if (dis>noisePnt_threshold)
					noisePntsIndexVec.push_back(n);
			}
			if (count >=4)
				break;
		}
		if (count >=4)
			break;
	}

	if( longAxisPntsIndexVec.size()==0 )
		return false;

	//STEP-2 �����ϵĵ�ŵ�һ��vector��;�����ϵĵ�ŵ�һ��vector��
	vector<Point2f> longAxisPntsVec;
	vector<Point2f>	shortAxisPntsVec;
	for(unsigned int i=0;i<pnts.size();i++)
	{
		if( isBelong2Vec(longAxisPntsIndexVec,i) )
		{
			longAxisPntsVec.push_back(pnts[i]);
		}
		else
		{
			shortAxisPntsVec.push_back(pnts[i]);
		}
	}
	
	if( shortAxisPntsVec.size()==0 )
		return false;

	//STEP-3 �Գ����ϵĵ㰴��y�����С��������,ð�ݷ�
	sortByYMin2Max(longAxisPntsVec);
	
	//STEP-4 �Զ����ϵ㰴��x�����С��������ð�ݷ�
	sortByXMin2Max(shortAxisPntsVec);

	////// zhangxu  added Ϊ�˱�����ֳ���ƽ��x����������Ҫ���Ѿ��ź�˳��ĵ���У��ж�x���껹��y�����ĸ�����
	float tempx_len,tempy_len;
	int longAxisSize = longAxisPntsVec.size() -1;
	tempx_len = fabs(longAxisPntsVec[longAxisSize].x - longAxisPntsVec[0].x);
	tempy_len = fabs(longAxisPntsVec[longAxisSize].y - longAxisPntsVec[0].y);
	if(tempx_len>tempy_len) ///// ���x����ĳ��ȴ���y����ĳ��ȣ�˵�������ϵĵ�ֲ��Ǻ��ŵģ���ʱ������Ҫ����x���С�������򣬶�����Ҫ�Ӵ�С����
	{
		sortByXMin2Max(longAxisPntsVec);
		sortByYMin2Max(shortAxisPntsVec);
		//// Ȼ��������
		reverse(shortAxisPntsVec);
	}
	///// end


	//STEP-5 �ж��ź���ĳ����ϵ���ĩ�㣬��������еĵ�һ����һ����
		//��ʵߵ���־λ,
	bool isUpSideDown = false;
	
		//y��С�㵽�����е����
	int dis_minY = int(sqrt((shortAxisPntsVec[0].x - longAxisPntsVec[0].x)*(shortAxisPntsVec[0].x - longAxisPntsVec[0].x)+
						(shortAxisPntsVec[0].y - longAxisPntsVec[0].y)*(shortAxisPntsVec[0].y - longAxisPntsVec[0].y)));
		//y���㵽�����е����
	int dis_maxY = int(sqrt((shortAxisPntsVec[0].x - longAxisPntsVec[longAxisSize].x)*(shortAxisPntsVec[0].x - longAxisPntsVec[longAxisSize].x)+
						(shortAxisPntsVec[0].y - longAxisPntsVec[longAxisSize].y)*(shortAxisPntsVec[0].y - longAxisPntsVec[longAxisSize].y)));
		//y������С���Ǹ����������е��Զ�Ļ�����Ϊ��ʵߵ���
	if( dis_minY > dis_maxY )
		isUpSideDown = true;

	//STEP-6 �����ʵߵ�����Զ���ͳ������������
	if( isUpSideDown )
	{
		////// ������ �����ϵĵ� ����ߵ�
		reverse(longAxisPntsVec);
		reverse(shortAxisPntsVec);	
	}

	//STEP-7 ���ź���ĵ����
	pnts.clear();
	L_pnts = longAxisPntsVec; S_pnts = shortAxisPntsVec;
	for(unsigned int i=0;i<longAxisPntsVec.size();i++)
	{
		pnts.push_back(longAxisPntsVec[i]);
	}
	for(unsigned int i=0;i<shortAxisPntsVec.size();i++)
	{
		pnts.push_back(shortAxisPntsVec[i]);
	}
	return true;
}

void CoreAlgorithm::createLightPenCoordinate(vector<TagPoint3f>& tagPnts, const Point3f& probeCenter)
{
	//////  zhangxu added  ��tagPnts ���д�С���������
	for(unsigned int i=0;i<tagPnts.size()-1;i++)
	{
		for (unsigned int j = i+1; j<tagPnts.size(); j++)
		{
			if (tagPnts[i][0] > tagPnts[j][0])
			{
				swap(tagPnts[i], tagPnts[j]);
			}
		}
	}
   //////ended

	vector<Point3f> longAxisPnts;
	//int i;
	for (unsigned int i = 0; i < tagPnts.size();i++)
	{
		if (tagPnts[i][0] <= TAG5)
		{
			longAxisPnts.push_back(Point3f(tagPnts[i][1],tagPnts[i][2],tagPnts[i][3]));
		}		
	}

	///// zhang xu �޸ģ����ݵõ����� tag6  ����tag7 ����Y����ʱ �Ƿ�ȡ��
	double Ydir = 1;
	Point3f  pt1;//Y��ֱ�ߣ�pt0Ϊֱ����һ�㣬pt1Ϊֱ����һ��
	int index = longAxisPnts.size();
	if(tagPnts[index][0]==TAG6)
	{
		Ydir = 1;
		pt1 = Point3f(tagPnts[index][1],tagPnts[index][2],tagPnts[index][3]);
	}
	if(tagPnts[index][0]==TAG7)
	{
		Ydir = -1;
		pt1 = Point3f(tagPnts[index][1],tagPnts[index][2],tagPnts[index][3]);
	}
	////// end

	//STEP-1:�����ĵ㣬����5�㵽1�㷽�����ֱ����ϣ����Z�ᵥλ����
	cv::Vec6f Z_Line;
	fitLine(longAxisPnts, Z_Line, CV_DIST_L2, 0, 0.01, 0.01);//Z��ķ���
	Vec3f norm_Z;	
	norm_Z[0] = Z_Line[0];norm_Z[1] = Z_Line[1];norm_Z[2] = Z_Line[2];
	////// zhangxu  added �����жϷ��߷����Ƿ�Ϊ��1�㵽5��ķ���,5���ȥ1������
	Vec3f norm_15;
	norm_15[0] = longAxisPnts[4].x - longAxisPnts[0].x;
	norm_15[1] = longAxisPnts[4].y - longAxisPnts[0].y;
	norm_15[2] = longAxisPnts[4].z - longAxisPnts[0].z;

	///// ���
	float temp = norm_Z[0]*norm_15[0]  + norm_Z[1]*norm_15[1]  + norm_Z[2]*norm_15[2];
	if(temp<0) //// ������ͬ������Ӹ���
	{
		norm_Z = -1*norm_Z;
	}
	////// end
	float a, b, c;
	////a = Z_Line[0];b = Z_Line[1];c = Z_Line[2];
	a = norm_Z[0];b = norm_Z[1];c = norm_Z[2];

	//STEP-2:�󾭹������ϵĵ�6���Ҵ�ֱ��Z��ĵ�λ���������Y�ᵥλ����
	Vec3f norm_Y;
	Point3f perpendicularFoot;

	Point3f pt0;//pt0Ϊֱ����һ�㣬pt1Ϊֱ����һ��
	pt0 = Point3f(Z_Line[3],Z_Line[4],Z_Line[5]); 

	//pt1 = shortAxisPnts[0];  ////���뱣֤�˵��ǵ�6�㣬�������壬�п��ܲ��ǵ�6��
	///// ��Ϊ���Ѿ���������������shortAxisPnts�ĵ�һ������tagPnts   

	float k, u0, u1, u2;
	u1 = a*pt1.x + b*pt1.y + c*pt1.z;
	u2 = a*pt0.x + b*pt0.y + c*pt0.z;
	u0 = a*a + b*b + c*c;
	k = (u1 - u2) / u0;

	perpendicularFoot.x = k*a + pt0.x;
	perpendicularFoot.y = k*b + pt0.y;
	perpendicularFoot.z = k*c + pt0.z;
	
	float d = sqrt((perpendicularFoot.x - pt1.x)*(perpendicularFoot.x - pt1.x)
		+(perpendicularFoot.y - pt1.y)*(perpendicularFoot.y - pt1.y)
		+(perpendicularFoot.z - pt1.z)*(perpendicularFoot.z - pt1.z));
	norm_Y[0] = (perpendicularFoot.x - pt1.x) / d;
	norm_Y[1] = (perpendicularFoot.y - pt1.y) / d;
	norm_Y[2] = (perpendicularFoot.z - pt1.z) / d;
	///// �ж� ��Ϊtag7����Ӧ�÷���  zhangxu added
	if(Ydir==-1)
	{
		norm_Y = -norm_Y;
	}
	/////// end
	
	//STEP-3:Z ��� Y �����X�ᵥλ����   ʵ���� ����ط�Ӧ���� Y���Z ��X
	//����AB����x1,y1,z1��, ����CD����x2,y2,z2��
	//����AB������CD����y1z2-z1y2��z1x2-x1z2��x1y2-y1x2��
	Vec3f norm_X;
	norm_X[0] = norm_Z[1] * norm_Y[2] - norm_Z[2] * norm_Y[1];
	norm_X[1] = norm_Z[2] * norm_Y[0] - norm_Z[0] * norm_Y[2];
	norm_X[2] = norm_Z[0] * norm_Y[1] - norm_Z[1] * norm_Y[0];
	//////��Ϊ���˳���ˣ���������һ������  zhangxu
	norm_X = -norm_X;

	//STEP-4:������λ��������ΪR����ͷ���ĵ�ΪT
	Mat T(3, 1, CV_32F); Mat R(3, 3, CV_32F);
	T.at<float>(0, 0) = probeCenter.x;
	T.at<float>(1, 0) = probeCenter.y;
	T.at<float>(2, 0) = probeCenter.z;

	R.at<float>(0, 0) = norm_X[0];R.at<float>(1, 0) = norm_X[1];R.at<float>(2, 0) = norm_X[2];
	R.at<float>(0, 1) = norm_Y[0];R.at<float>(1, 1) = norm_Y[1];R.at<float>(2, 1) = norm_Y[2];
	R.at<float>(0, 2) = norm_Z[0];R.at<float>(1, 2) = norm_Z[1];R.at<float>(2, 2) = norm_Z[2];
	
	Mat R_invert;
	invert(R, R_invert);
	//STEP-5:����������ϵ�µĵ�ת�����������ϵ�£�P2 = R*P1 + T
	for (unsigned int i = 0; i < tagPnts.size(); i++)
	{
		Mat pnt(3,1,CV_32F);
		pnt.at<float>(0, 0) = tagPnts[i][1];
		pnt.at<float>(1, 0) = tagPnts[i][2];
		pnt.at<float>(2, 0) = tagPnts[i][3];

		pnt = R_invert*(pnt - T);

		tagPnts[i][1] = pnt.at<float>(0, 0);
		tagPnts[i][2] = pnt.at<float>(1, 0);
		tagPnts[i][3] = pnt.at<float>(2, 0);
	}
}

void CoreAlgorithm::iterativeCameraCalibration(const vector<string>& imgsPath,const CalibrationData& cData,
	const CamPara& camParaSrc, CamPara& camParaDst)
{
	camParaDst = camParaSrc;
    CalibrationData _cData = cData;
    CamPara camPara = camParaSrc;
    generateParallelCamR(camPara.parallelCamR);

    for (unsigned int k = 4; k>0;k--)
    {
        Mat cameraMatrix(3, 3, CV_64F);
        for (unsigned int i = 0; i < 3; i++)
        {
            for (unsigned int j = 0; j < 3; j++)
            {
                cameraMatrix.at<double>(i, j) = camPara.CameraIntrinsic[i][j];
            }
        }

        vector<double> distCoeffs;
        for (int i = 0; i < 4; i++)
            distCoeffs.push_back(camPara.DistortionCoeffs[i]);

        //STEP-1 ����ÿ���궨��ƽ����ͼ��R'����������ת���� 3x3
        vector<Mat> parallelImgR;
        calculateParallelViewR(camPara.imgRTVec, parallelImgR);

        //STEP-2 ������������ת����ƽ����ͼ��
        vector< vector<Point2f> > corner_Parallel;
        for (unsigned int i = 0; i < cData.plane2dPntsVec.size(); i++)
        {
            vector<Point2f> pnts;
			///// Mat T
			////// calculateParallelViewR  �����޸���ƽ����ͼ��t1 Ϊ t0 ��1.5 z��x��y���ֲ���
			Mat t1 = Mat::zeros(3,1,CV_64F);
			camPara.imgRTVec[i].T.copyTo(t1);
			/*t1.at<double>(2,0) = 1.5*t1.at<double>(2,0);*/
			//////
            undistortPoints2DifferentView(cData.plane2dPntsVec[i], pnts,
                cameraMatrix,camPara.imgRTVec[i].R,camPara.imgRTVec[i].T,distCoeffs,
                cameraMatrix,camPara.parallelCamR,t1,vector<double>());
            corner_Parallel.push_back(pnts);
        }

        //STEP-3 ����ƽ����ͼ�ϵ������㣬��ƽ����ͼ�Ͻ��������ؽǵ���
        for (unsigned int i = 0; i < imgsPath.size();i++)  //// just for test (int i = 0; i < imgsPath.size();i++)
        {
            Mat srcimgMat1;
            srcimgMat1 = cv::imread(imgsPath[i],0);
            ////// image is too small
            Mat resultimgMat1(2*srcimgMat1.rows, 2*srcimgMat1.cols, srcimgMat1.type());
            /////

            undistortImg(srcimgMat1, resultimgMat1, cameraMatrix, distCoeffs, parallelImgR[i]);
            Mat ringImg;
            CoreAlgorithm::creatRingImg(ringImg, 708);
            ////end
            ringSubPix(resultimgMat1,corner_Parallel[i],ringImg); ////ringSubPix(resultimgMat1,corner_Parallel[i],Mat());
			//CoreAlgorithm::ringSubPixAdv(resultimgMat1,corner_Parallel[i],3);

            //ringSubPix(resultimgMat1,corner_Parallel[i],Mat());
            int j = 0;
        }

        //STEP-4 ��ƽ����ͼ�ϼ�⵽�������ؽǵ�ת����ԭʼ��ͼ
        for (unsigned int i = 0; i < corner_Parallel.size(); i++)
        {
            vector<Point2f> pnts;
            undistortPoints2DifferentView(corner_Parallel[i], pnts,
                cameraMatrix,camPara.parallelCamR,camPara.imgRTVec[i].T,vector<double>(),
                cameraMatrix,camPara.imgRTVec[i].R,camPara.imgRTVec[i].T,distCoeffs);
            corner_Parallel[i] = (pnts);
        }

        //STEP-5 ʹ�������ѱ궨���궨
        _cData.plane2dPntsVec.clear();
        _cData.plane2dPntsVec = corner_Parallel;

        ZhangCalibrationMethod_CPP(_cData,camParaDst);
        //// added by zhang xu ������ֹ�������������û��ʹ��totalReproNormErr ���٣���ֹͣ����
        if(camParaDst.totalReproNormErr> camPara.totalReproNormErr)
        {
            camParaDst = camPara;
            return;  /// ����
        }
        ////end
        camParaDst.parallelCamR = camPara.parallelCamR;
        camPara = camParaDst;
    }
}

void CoreAlgorithm::undistortPoints2DifferentView(const vector<Point2f>& src,vector<Point2f>& dst,
		const Mat& cameraMatrix1,const Mat& R1,const Mat& T1,const vector<double>& distCoeffs1,
		const Mat& cameraMatrix2,const Mat& R2,const Mat& T2,const vector<double>& distCoeffs2)
{
	//STEP-1 ʹ��cam1�Ĳ���������������άƽ���
	vector<Point3f> pnts3d;
	project2CalibrationBoard(src,pnts3d,cameraMatrix1,distCoeffs1,R1,T1);
	//STEP-2 �����������άƽ�����ͶӰ��ͼ���ϣ�����cam2��R��T
	Mat _R2;
	R2.copyTo(_R2);
	if (R2.cols==3 && R2.rows==3)
	{
		Rodrigues(R2, _R2);
	}
	projectPoints(pnts3d, _R2, T2, cameraMatrix2, distCoeffs2, dst);
}

void CoreAlgorithm::undistortImg(const Mat& src, Mat& dst,
	const Mat& cameraMatrix, const vector<double>& distCoeffs,const Mat& R)
{
	Mat map1, map2;
	Mat new_cameraMatrix;
	cameraMatrix.copyTo(new_cameraMatrix);

	///// just for test
	int row = dst.rows;
	int col = dst.cols;
	/////

	initUndistortRectifyMap(cameraMatrix, distCoeffs, R, new_cameraMatrix, dst.size(), CV_32FC1, map1, map2);
	remap(src,dst,map1,map2,cv::INTER_LINEAR,cv::BORDER_CONSTANT);
}

void CoreAlgorithm::calculateParallelViewR(const vector<RT>& rt_vec, vector<Mat>& parallelR)
{
	for (unsigned int i = 0; i < rt_vec.size(); i++)
	{
		Mat R = rt_vec[i].R;
		Mat T = rt_vec[i].T;
		Mat _R;
		Rodrigues(R, _R);
		Mat H0(3,3,CV_64F);
		_R.copyTo(H0); 
		T.col(0).copyTo(H0.col(2));
		Mat H1 = Mat::zeros(3,3,CV_64F);
		T.col(0).copyTo(H1.col(2));
		//// zhangxu �޸�
		//H1.at<double>(0, 2) = -SQUARE_SIZE*WIDTH_CORNER_COUNT/2;
		//H1.at<double>(1, 2) = -SQUARE_SIZE*HEIGHT_CORNER_COUNT/2;
		/*H1.at<double>(2, 2) = 1.5*H1.at<double>(2, 2);*/
		//////

		///////  ���µı궨������ϵ��  x���£�y���ң�z�궨����ǰ����matlabtool box camera calibrationһ��
		//H1.at<double>(0, 1) = 1;
		//H1.at<double>(1, 0) = 1;
		///////  ���µı궨������ϵ��  x���ң�y���£�z�궨�����
		H1.at<double>(0, 0) = 1;
		H1.at<double>(1, 1) = 1;
		///// end


		Mat H0_invert;
		invert(H0, H0_invert);

		_R = H1*H0_invert;

		parallelR.push_back(_R);
	}
}
//chengwei added

	//�����ĵ���������ƽ����ͼ��ת������
	//const Mat &PoseR,<in>  3X1
	//const Mat &PoseT,<in>  3X1 
	//Mat &parallelR<in & out>  ����ƽ����ͼ�����������������R��3X3����������ĵ�Ҫ���H1 * inv(H0), 3X3
void CoreAlgorithm::calculateParallelViewR(const Mat &PoseR,const Mat &PoseT,Mat &parallelR)
{
	/// �жϾ���ĳߴ磬����ȷ������

	if(parallelR.rows ==0 ||(parallelR.cols == 0))
	{
		parallelR = Mat::eye(3,3,CV_64F);
	}
	///  step  ����H1
	Mat H1(3,3,CV_64F);
	parallelR.copyTo(H1);
	PoseT.col(0).copyTo(H1.col(2));
	/// step ����H0
	Mat temR = Mat::zeros(3,3,CV_64F);
	Rodrigues(PoseR, temR);
	Mat H0(3,3,CV_64F);
	temR.copyTo(H0); 
	PoseT.col(0).copyTo(H0.col(2));
	
	//// zhangxu �޸�
	//H1.at<double>(0, 2) = -SQUARE_SIZE*WIDTH_CORNER_COUNT/2;
	//H1.at<double>(1, 2) = -SQUARE_SIZE*HEIGHT_CORNER_COUNT/2;
	/*H1.at<double>(2, 2) = 1.5*H1.at<double>(2, 2);*/
	//////

	///////  ���µı궨������ϵ��  x���£�y���ң�z�궨����ǰ����matlabtool box camera calibrationһ��
	//H1.at<double>(0, 1) = 1;
	//H1.at<double>(1, 0) = 1;
	///////  ���µı궨������ϵ��  x���ң�y���£�z�궨�����
	//H1.at<double>(0, 0) = 1;
	//H1.at<double>(1, 1) = 1;
	///// end
	Mat H0_invert;
	invert(H0, H0_invert);
	parallelR = H1*H0_invert;
}


void CoreAlgorithm::project2CalibrationBoard(const vector<Point2f>& src, vector<Point3f>& pnts3d,
	const Mat& cameraMatrix, vector<double> distCoeffs, const Mat& R,const Mat& T)
{
	vector<Point2f> pnts2d;
	Mat _R;
	R.copyTo(_R);
	if (R.cols==1 || R.rows==1)
	{
		Rodrigues(R,_R);
	}
	T.col(0).copyTo(_R.col(2));
	invert(_R, _R); 
	undistortPoints(src, pnts2d, cameraMatrix, distCoeffs,_R);
	
	for (unsigned int i = 0; i < pnts2d.size(); i++)
	{
		pnts3d.push_back(Point3f(pnts2d[i].x,pnts2d[i].y,0));
	}
}

bool CoreAlgorithm::creatCosImg(Mat& Img, float T, float fai, float bias, float range, float gamma, bool isX, bool isGrid)
{
	int height = Img.rows;
	int width = Img.cols;
	int step = Img.step;
	int channels = Img.channels();
	uchar* Img_data = (uchar*)Img.data;

	if (height<1 || width <1)
	{
		///// ͼ����Ϊ��
		return false;
	}

	///// ����һЩ����
	double PI = acos(double(-1));
	float A = bias/(bias+range);
	float B = range/(bias+range);
	//////��������ȵķ��������Ӵ�B�ķ���
	float Xmin = pow((A-B),gamma);
	float A2 = (A+B+Xmin)/2;
	float B2 = (A+B-Xmin)/2;

	///  ��gridͼ��  x�� ����ͼ��Img��channel ��ͼ��������  xָ�ľ���col
	if (isGrid && isX)
	{
		for (int col=0;col<width; col++)
		{
			///// ��������
			float norm = float(A2+B2*cos(2*PI*col/T-fai));
			float power = 1/gamma;
			float temp = pow(norm,power);
			float lus = (bias+range)*temp;//// ����ֱ��ʹ��col����pitch ����2PI

			///// ��ֵ��ͼ��
			for (int row=0; row<height; row++)
			{
				for (int i=0;i<channels;i++)
				{
					Img_data[row*step + col*channels + i] = int(lus);
				}
			}
		}
	}
	///  ��gridͼ��  y�� ����ͼ��Img��channel ��ͼ��������  yָ�ľ���row
	if (isGrid && !isX)		
	{
		for (int row = 0;row<height; row++)
		{
			///// ��������
			float norm = float(A2+B2*cos(2*PI*row/T-fai));
			float power = 1/gamma;
			float temp = pow(norm,power);
			float lus = (bias+range)*temp;//// ����ֱ��ʹ��col����pitch ����2PI
			///// ��ֵ��ͼ��
			for (int col=0; col<width; col++)
			{
				for (int i=0;i<channels;i++)
				{
					Img_data[row*step + col*channels + i] = int(lus);
				}
			}
		}

	}

	///  ��diamond ��������ͼ��  x�� ����ͼ��Img��channel ��ͼ��������  xָ�ľ���col
	/// ���� LIghtcragter 4500 page11 ˵�� ��0��������������أ���1������ͻ��������أ�����ɵĹ��ɣ�ż�������ص������궼�Ǵ���
	/// �����������������أ�5.4΢�ף��������еļ����5.4΢�ף������еļ����5.4΢��,
	/// �ܽ��������� ÿ�������ϣ������������ż���и�������߰�����أ��� ż�����������0.5�������е���������
	if (!isGrid && isX)
	{
		for (int col=0;col<width; col++)
		{	
			///// ��ֵ��ͼ��������
			for (int row=1; row<height; row=row+2)
			{
				///// ��������
				float tempcol = float(col);
				float norm = float(A2+B2*cos(2*PI*tempcol/T-fai));
				float power = 1/gamma;
				float temp = pow(norm,power);
				float lus = (bias+range)*temp;//// ����ֱ��ʹ��col����pitch ����2PI

				for (int i=0;i<channels;i++)
				{
					Img_data[row*step + col*channels + i] = int(lus);
				}
			}	

			///// ��ֵ��ͼ��ż����
			for (int row=0; row<height; row=row+2)
			{
				///// ��������
				float tempcol = float(col+0.5);
				float norm = float(A2+B2*cos(2*PI*tempcol/T-fai));
				float power = 1/gamma;
				float temp = pow(norm,power);
				float lus = (bias+range)*temp;//// ����ֱ��ʹ��col����pitch ����2PI

				for (int i=0;i<channels;i++)
				{
					Img_data[row*step + col*channels + i] = int(lus);
				}
			}	
		}
	}

	///  ��diamond ��������ͼ��  y�� ����ͼ��Img��channel ��ͼ��������  xָ�ľ���col
	/// ���� LIghtcragter 4500 page11 ˵�� ��0��������������أ���1������ͻ��������أ�����ɵĹ��ɣ�ż�������ص������궼�Ǵ���
	/// �����������������أ�5.4΢�ף��������еļ����5.4΢�ף������еļ����5.4΢��,
	/// �ܽ��������� ÿ��	�����ϣ����������ż���и�������߰�����أ��� ż���� ��Ӧ�������0.5�������е���������

	if (!isGrid && !isX)		
	{
		for (int row = 0;row<height; row++)
		{
			///// ��ֵ��ͼ��������
			for (int col=1; col<width; col = col+2)
			{
				///// ��������
				float temprow = float(row);
				float norm = float(A2+B2*cos(2*PI*temprow/T-fai));
				float power = 1/gamma;
				float temp = pow(norm,power);
				float lus = (bias+range)*temp;//// ����ֱ��ʹ��col����pitch ����2PI

				for (int i=0;i<channels;i++)
				{
					Img_data[row*step + col*channels + i] = int(lus);
				}
			}
			///// ��ֵ��ͼ��ż����  +0.5
			for (int col=0; col<width; col = col +2)
			{
				///// ��������
				float temprow = float(row+0.5);
				float norm = float(A2+B2*cos(2*PI*temprow/T-fai));
				float power = 1/gamma;
				float temp = pow(norm,power);
				float lus = (bias+range)*temp;//// ����ֱ��ʹ��col����pitch ����2PI

				for (int i=0;i<channels;i++)
				{
					Img_data[row*step + col*channels + i] = int(lus);
				}
			}
		}	
	}
	return true;
}

bool CoreAlgorithm::gcd(int a, int b, int& x, int& y, int& q)
{
	int temp, xt, yt, qt;
	if (b==0)
	{
    		x=1;
    		y=0;
    		q=a;
    		return true;
	}
	else
	{
		temp = a%b;
		gcd (b, temp, xt, yt, qt );
    	x = yt;
    	y = xt-(a/b)*yt;
    	q = qt;
	}

	return true;
}

bool CoreAlgorithm::phaseshift (vector<Mat> images, float threshold, Mat& mod_phase, Mat& mean_lus, Mat& lus_range, Mat& mask)
{
	int frameNum = images.size();  //// ͼƬ����

	if (frameNum<3)
	{
		return false;  /// ͼƬ������������3
	}

	int height = images[0].rows;
	int width = images[0].cols;
	int channels = images[0].channels();
	if (channels>1)
	{
		return false;  /// ������ֻ���ܻҶ�ͼ��
	}

	/*if (height!= mod_phase.rows || height!= mean_lus.rows || height!= lus_range.rows || height!=mask.rows)
	{
		return false;  /// ��������� ������һ��
	}

	if(width != mod_phase.cols || width!= mean_lus.cols || width != lus_range.cols || width != mask.cols  )
	{
		return false;  /// ����Ĳ��� ������һ��
	}*/
	

	///////
	Mat lusin = Mat::zeros(height,width,CV_32FC1);
	Mat lucos = Mat::zeros(height,width,CV_32FC1);
	mean_lus = Mat::zeros(height,width,CV_32FC1);  ////������ֵΪ0
	mod_phase = Mat::zeros(height,width,CV_32FC1);  ////������ֵΪ0
	lus_range = Mat::zeros(height,width,CV_32FC1);  ////������ֵΪ0
	mask =  Mat::zeros(height,width,CV_8UC1); ////������ֵΪ0
	Mat temMat ;  ////������ֵΪ0   = Mat::zeros(height,width,CV_32FC1)

	double PI = acos(double(-1));
	for (int i=0;i<frameNum; i++)
	{
		double fai = 2*PI*i/frameNum;
		images[i].convertTo(temMat,CV_32FC1);
		scaleAdd(temMat, sin(fai), lusin, lusin);
		scaleAdd(temMat, cos(fai), lucos, lucos);
		scaleAdd(temMat,1.0/frameNum, mean_lus, mean_lus);
	}

	for (int row=0;row<height; row++)
	{
		for(int col=0;col<width;col++)
		{
			mod_phase.at<float>(row,col) = atan2(lusin.at<float>(row,col), lucos.at<float>(row,col));
			if (mod_phase.at<float>(row,col)<0)
			{
				mod_phase.at<float>(row,col) = mod_phase.at<float>(row,col)+float(2*PI);
			}		
		}
	}
	multiply(lusin, lusin, lusin);
	multiply(lucos, lucos, lucos);
	add(lusin,lucos,lus_range);
	sqrt(lus_range,lus_range);
	lus_range = 2*lus_range/frameNum;
	
	//// �ж�mask
	for (int row=0;row<height; row++)
	{
		for(int col=0;col<width;col++)
		{
			if (lus_range.at<float>(row,col)>threshold && mean_lus.at<float>(row,col)>threshold)
			{
				mask.at<char>(row,col) = 1;
			}		
		}
	}
	return true;
}

bool CoreAlgorithm::robustCRM(vector<Mat> mod_phases,vector<float> lamda, float m, Mat& cor)
{
	int freNum = mod_phases.size();
	if (freNum!=lamda.size())
	{
		return false; //// Ƶ�ʸ�����һ��
	}
	int height = mod_phases[0].rows;
	int width = mod_phases[0].cols;

	double PI = acos(double(-1));
	
	vector<Mat> q; /// ��������������Լ���Ľ��
	vector<int> taobari1;
	int gamma1 = 1; /// ���������ĳɼ����Ե�һ�������������˵�һ������֮������������ĳ˻�
	///// ��������λת��������
	for(int i=0;i<freNum;i++)
	{
		mod_phases[i] = mod_phases[i]*lamda[i]/2/PI;  /// ������λת��������
		lamda[i] = lamda[i]/m;  //// ת���ɻ��ʵ���		
		////// ���������Ĳ�
		Mat temQ,temQ2;  ///��ʱ����		
		if (i>0)
		{
			////
			gamma1 = int(gamma1*lamda[i]); /// �������������ĳ˻����� tao1
			///// ����ģ��ϵ��
			int temtaoi1,temtao11,temq;
			gcd(int(lamda[0]),int(lamda[i]),temtaoi1,temtao11,temq);
			taobari1.push_back(temtaoi1);
			///// ������������Թ�Լ��m������ģ��ϵ��
			subtract(mod_phases[i], mod_phases[0], temQ); //// ���	
			temQ = temQ/m; 
			temQ.convertTo(temQ2,CV_32SC1);  //// ����ת��������			
			q.push_back(temQ2);  /// ����
		}
	}
	///// 			//// ����bi1
	vector<int> b;
	for (int i=0;i<freNum-1;i++)
	{
		int bi1, b11,q11;
		gcd(int(gamma1/lamda[i+1]), int(lamda[i+1]),bi1,b11,q11);
		b.push_back(bi1);
	}
	////����	 cor
	cor = Mat::zeros(height,width,CV_32FC1);
	for (int row=0;row<height; row++)
	{
		for (int col=0;col<width; col++)
		{
			float y =0;
			int zn = 0;
			for (int i=0;i<freNum-1; i++)
			{
				int epiu, tqt1;
				tqt1 = q[i].at<int>(row,col);
				epiu = tqt1*taobari1[i]%int(lamda[i+1]);
				if (epiu<0)
				{
					epiu = epiu + int(lamda[i+1]);
				}
				zn = zn + epiu*b[i]*int(gamma1)/int(lamda[i+1]); ///��ʽ��25�� ���				
			}
			int tqt1,tqt2,n1,ni1;
			n1 = zn%gamma1;
			if (n1<0)
			{
				n1 = n1+gamma1;
			}
			y = n1*lamda[0]*m + mod_phases[0].at<float>(row,col); 

					
			for (int i=0;i<freNum-1; i++)
			{
				tqt1 = q[i].at<int>(row,col);
				tqt2 = int(lamda[0]);
				ni1 = int((n1*tqt2-tqt1)/lamda[i+1]);  
				float yi1;
				yi1 = ni1*lamda[i+1]*m + mod_phases[i+1].at<float>(row,col); 
				y =y+ yi1;
			}
			cor.at<float>(row,col) = y/freNum;
		}
	}
	return true;
}

bool CoreAlgorithm::undistorPixcor(const vector<Point2f>& src ,vector<Point2f>& dst ,double fc[2] ,double cc[2] ,double kc[5] ,double alfpha_c )
{
	normalizPixel( src, dst,  fc , cc ,kc ,alfpha_c );
	int pntNum = dst.size();
	Point2f temPnt;
	for (int i=0;i<pntNum; i++)
	{
		temPnt.x = float(fc[0]*dst[i].x + alfpha_c*dst[i].y + cc[0]);
		temPnt.y = float(fc[1]*dst[i].y + cc[1]);
		dst[i] = temPnt;
	}
	return true;
}
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
bool CoreAlgorithm::creatdistorImg(Mat& img,double fc[2] ,double cc[2] ,double kc[5] ,double alfpha_c, float meanLus, float rangelus,float T,float fai, int method )
{
	//// �ж�Ҫ�����ͼ���С�Ƿ����0
	int height,width;
	height = img.rows;
	width  = img.cols;
	if (height<1 || width <1)
	{
		return false;  //// ͼ������Ϊ0��������Ч��ͼ��
	}
	//// �ж�ͼ���ͨ�����������3��ͨ���� ����
	int channel = img.channels();
	if (channel>1)
	{
		return false;
	}
	double PI = acos(double(-1));
	//// ��ͼ�������ת���� vector<point2f>
	vector<Point2f> src, dst;
	src.clear();
	dst.clear();
	Point2f tempPnt;
	for (float i=0;i<height; i++)
	{
		for (float j=0;j<width; j++)
		{
			tempPnt.x = j;
			tempPnt.y = i;
			src.push_back(tempPnt);
		}
	}
	///// �����޻������������
	undistorPixcor(src ,dst ,fc ,cc ,kc ,alfpha_c);
	//// �����޻�������������������		
	int intensty;
	///////////////////////////  method =0  ��ʾģ���� meanLus + rangelus*cos(2*pi*x/T -fai)
	double temp;
	if (method == 0)
	{
		for(unsigned int i=0;i<dst.size();i++)
		{
			tempPnt = dst[i];
			temp = meanLus + rangelus*cos(2*PI*tempPnt.x/T -fai);
			for (int j=0;j<channel;j++)
			{
				intensty = int (temp);
				///// ���ݴ����ȶ�ͼ���������
				img.at<uchar>(int(src[i].y),int(src[i].x)) = int(intensty);
			}			
		}
	}
	/////////  method =1 Gray code
	if(method ==1)
	{

	}
	return true;
}

bool CoreAlgorithm::creatRingImg(Mat& img,int dx)
{
	///// ���dx С�ڵ���0���򷵻ش���
	if(dx<=0)
	{
		return false;
	}

	//// ���ɵ�Բ��ͼ��ߴ�Ϊdx���⻷�뾶Ϊ dx/3�� �ڻ��뾶Ϊdx/6
	//// ����Բ֮��Ĳ���Ϊ��0.����������255
	Mat _img = Mat::zeros(dx, dx, CV_8U);  ////Mat _img = Mat::zeros(dx, dx, CV_32F); 
	//// ͼ���С
	int height,width;
	height = _img.rows;
	width  = _img.cols;
	double rmax,rmin,dist,cen_x,cen_y;
	rmax = dx/3;
	rmin = dx/6;
	////// ������C++�о������ʼ����Ϊ0��������1����������size��ȥ1�����Ч�ڣ�size-1��/2
	cen_x = (dx-1)/2;
	cen_y = (dx-1)/2;


	for(int i=0;i<height; i++)
	{
		for(int j=0;j<width;j++)
		{
			 dist  = double(i-cen_y)*double(i-cen_y) + double(j-cen_x)*double(j-cen_x);
			 dist = sqrt(dist);
			 if(dist<rmin || dist>rmax) //// ���ó�Ϊ��ɫ 255
			 {
				 _img.at<uchar>(i,j) = 255;
			 }
			 //// ���򣬾���Ϊ��ɫ���������κβ���
		}
	}
	img = _img.clone();
	return true;
}

void CoreAlgorithm::generateParallelCamR(Mat& R)
{
	
	Mat r = Mat::zeros(3, 3, CV_64F);
		//�������ϵ��
		/*r.at<double>(1, 0) = 1;
		r.at<double>(0, 2) = 1;
		r.at<double>(2, 1) = 1;*/
		//���µı궨������ϵ��  x���ң�y���£�z�궨�����
		r.at<double>(0, 0) = 1;
		r.at<double>(1, 1) = 1;
		r.at<double>(2, 2) = 1;
		///////  ���µı궨������ϵ��  x���£�y���ң�z�궨����ǰ����matlabtool box camera calibrationһ��
	//r.at<double>(0, 1) = 1;
	//r.at<double>(1, 0) = 1;
	//r.at<double>(2, 2) = -1;
	r.copyTo(R);
}

bool CoreAlgorithm::triangulatePnts(const vector<Point2f>& pnts2d1, const vector<Point2f>& pnts2d2,
	const Mat& cameraMatrix1, const Mat& R1, const Mat& T1, const vector<double>& distCoeffs1,
	const Mat& cameraMatrix2, const Mat& R2, const Mat& T2, const vector<double>& distCoeffs2,
	vector<Point3f>& pnts3d)
{
	Mat _R1, _R2;
	R1.copyTo(_R1);
	R2.copyTo(_R2);
	if (R1.rows==1||R1.cols==1)
		Rodrigues(_R1,_R1);
	if (R2.rows==1||R2.cols==1)
		Rodrigues(_R2,_R2);

	Mat RT1(3,4,CV_64F), RT2(3,4,CV_64F);
	_R1.colRange(0,3).copyTo(RT1.colRange(0, 3));
	_R2.colRange(0,3).copyTo(RT2.colRange(0, 3));
	T1.copyTo(RT1.col(3));
	T2.copyTo(RT2.col(3));

	Mat P1, P2;//3x4
	P1 = cameraMatrix1 * RT1;
	P2 = cameraMatrix2 * RT2;

	vector<Point2f> _pnts2d1, _pnts2d2;
	
	undistortPoints2DifferentView(pnts2d1,_pnts2d1,
		cameraMatrix1,R1,T1,distCoeffs1,
		cameraMatrix1,R1,T1,vector<double>());

	undistortPoints2DifferentView(pnts2d2,_pnts2d2,
		cameraMatrix2,R2,T2,distCoeffs2,
		cameraMatrix2,R2,T2,vector<double>());

	Mat pnts4d;
	triangulatePoints(P1,P2,_pnts2d1,_pnts2d2,pnts4d);

	for (int i = 0; i < pnts4d.cols; i++)
	{
		float a = pnts4d.at<float>(3,i);
		pnts3d.push_back(Point3f(pnts4d.at<float>(0,i)/a,
						pnts4d.at<float>(1,i)/a,
						pnts4d.at<float>(2,i)/a));
	}

	return true;
}

bool CoreAlgorithm::quadSubPixel(const Mat& win, float& dx, float& dy)
{
	Mat A(9,6,CV_32F), B(9,1,CV_32F);
	int row = 0;
	for (int y = -1; y <=1 ; y++)
	{
		for (int x = -1; x <= 1; x++,row++)
		{
			A.at<float>(row,0)=float(x*x);		A.at<float>(row,1)=float(y*y);
			A.at<float>(row,2)=float(x);		A.at<float>(row,3)=float(y);
			A.at<float>(row,4)=float(x*y);		A.at<float>(row,5)=1.0;

			B.at<float>(row, 0) = win.at<float>(y + 1, x + 1);
		}
	}

	Mat X(6,1,CV_32F);

	//����A���Ƿ���
	solve(A, B, X,cv::DECOMP_QR);

	Mat H(2,2,CV_32F);
	H.at<float>(0, 0) = X.at<float>(0, 0);
	H.at<float>(0, 1) = float(X.at<float>(1, 0) / 2.0);
	H.at<float>(1, 0) = float(X.at<float>(1, 0) / 2.0);
	H.at<float>(1, 1) = X.at<float>(2, 0);

	vector<double> eigenvalues;
	Mat eigenvectors(2,2,CV_32F);
	eigen(H, eigenvalues, eigenvectors);

	//�������
	/*if (eigenvectors.at<float>(0,0) < 0 && eigenvectors.at<float>(0,1) < 0)
	{
	float a = X.at<float>(0, 0); float b = X.at<float>(1, 0); float c = X.at<float>(2, 0);
	float d = X.at<float>(3, 0); float e = X.at<float>(4, 0); float f = X.at<float>(5, 0);

	dx = -1*(2*b*c - d*e)/(4.0*a*b - e*e);
	dy = -1*(2*a*d - c*e)/(4.0*a*b - e*e);
	}
	else
	{
	dx = 0;
	dy = 0;
	}*/
	//X*V = V*D;
	Mat aa = H*eigenvectors.row(0).t();
	Mat bb = eigenvalues[0] * eigenvectors.row(0).t();

	float a = X.at<float>(0, 0); float b = X.at<float>(1, 0); float c = X.at<float>(2, 0);
	float d = X.at<float>(3, 0); float e = X.at<float>(4, 0); float f = X.at<float>(5, 0);

	dx = float(-1*(2*b*c - d*e)/(4.0*a*b - e*e));
	dy = float(-1*(2*a*d - c*e)/(4.0*a*b - e*e));
	return true;

}

bool CoreAlgorithm::extraRingCenter(const Mat& img, const vector<Point2f>& mousePnts
	, vector<Point2f>& plane2dPnts, int& row, int& col)
{
	  //ת���ɻҶ�ͼ
        Mat ImageGray;
        if(img.channels()==1)
        {
            ImageGray = img;
        }
        else
        {
        cvtColor(img, ImageGray, CV_BGR2GRAY);
        }
        //chengwei changed
		float areaHeight = abs(mousePnts[0].y-mousePnts[2].y);
		unsigned int multiples = int(ceil(areaHeight/300));
        ///// ����Ӧ��ֵ��
        int block_size = 100*multiples+1;
		int C_threshold = 10;
        adaptiveThreshold(ImageGray,ImageGray,255,cv::ADAPTIVE_THRESH_MEAN_C ,cv::THRESH_BINARY,block_size,C_threshold);
        //�԰�ɫ���������ٸ�ʴ
        int close_type = cv::MORPH_ELLIPSE;
        int dilate_size = multiples/3+1;
        Mat element = getStructuringElement(close_type,Size(2*dilate_size+1,2*dilate_size+1),Point(-1,-1));
        dilate(ImageGray, ImageGray, element,Point(-1, -1),1);
        erode(ImageGray, ImageGray, element,Point(-1, -1),1);
        Point2f leftTop = mousePnts[0]; Point2f leftDown = mousePnts[3];
        int scaleSize = 1;
        vector<Point2f> mousePnts_half;
        for (unsigned int i = 0; i < mousePnts.size();i++)
        {
            mousePnts_half.push_back(Point2f(mousePnts[i].x/scaleSize,mousePnts[i].y/scaleSize));
        }
        if (scaleSize!=1)
            resize(ImageGray, ImageGray,Size(ImageGray.cols/scaleSize,ImageGray.rows/scaleSize));

        //�����궨�������ROI������Χ����ȥһȦ
        RotatedRect ROI = minAreaRect(mousePnts_half);
        cv::Rect roi = ROI.boundingRect();

        ///// ���ROI���� ���������Χ����ֵͼ�����ߴ�
        if(roi.x<0)
        {
            roi.x=0;
        }
        if(roi.y<0)
        {
            roi.y=0;
        }
        if(roi.x+roi.width>ImageGray.cols-1)
        {
            roi.width = ImageGray.cols-roi.x -1;
        }
        if(roi.y+roi.height>ImageGray.rows-1)
        {
            roi.height=ImageGray.rows-roi.y-1;
        }
        Mat imgGray_ROI = ImageGray(roi).clone();
        //chengwei tested
//        imshow("window2",imgGray_ROI);
//        waitKey();

        //�����ѡ������Ĳ���Ϊ��ɫ��������ɨ�裬���ٱ���ʱ��
        for (int i = 0; i < imgGray_ROI.rows; i++)
        {
            bool minArea = false;
            for (int j = 0; j < imgGray_ROI.cols;j++)
            {
                int row = i + roi.y;
                int col = j + roi.x;
                if ( containsPnt(mousePnts_half, cv::Point2i(col,row))<0)
                {
                    imgGray_ROI.at<char>(i, j) = char(255);
                }
                else
                {
                    minArea = true;
                    break;
                }
            }
            if (minArea = true)
            {
                for (int j = imgGray_ROI.cols-1; j >=0 ;j--)
                {
                    int row = i + roi.y;
                    int col = j + roi.x;
                    if ( containsPnt(mousePnts_half, cv::Point2i(col,row))<0)
                    {
                        imgGray_ROI.at<char>(i, j) = char(255);
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }
        //// ��Ե���
       Mat img_threshold;
       Canny(imgGray_ROI,img_threshold,50,255,3,true);/// 255,  3
        ///// end test
        //Ѱ������
        vector< vector<Point> > contours;
		findContours(img_threshold,contours,CV_RETR_EXTERNAL ,CV_CHAIN_APPROX_SIMPLE);
  //     Mat contourImg = Mat::zeros(img_threshold.size(),CV_8U);
  //      drawContours(contourImg, contours, -1, Scalar(255, 255, 255));
		//namedWindow("window",2);
		//imshow("window",contourImg);
		//waitKey();

        //��Բ���,������,������ɸѡ
        RotatedRect rotatedBox;
        vector<RotatedRect> rotatedBoxVec;
        vector<RotatedRect> temp_rotatedBoxVec;

        for( vector< vector<Point> >::iterator itr=contours.begin();itr!=contours.end();++itr)
        {
            static int n=0;
            n++;
            if(itr->size()<100||itr->size()>450)
               continue;

            try
            {
                rotatedBox = fitEllipse((*itr));
            }
            catch (...)//�������ʾ�κ��쳣
            {
                continue;
            }
            temp_rotatedBoxVec.push_back(rotatedBox);
            float height = rotatedBox.size.height;
            float width = rotatedBox.size.width;
            //if ( height>width? height/width<1.6 : width/height<1.6 )
                plane2dPnts.push_back(rotatedBox.center);
        }

        for (unsigned int i = 0; i < plane2dPnts.size();i++)
        {
            plane2dPnts[i].x += roi.x;
            plane2dPnts[i].y += roi.y;
        }

        if (!sortRingCenter(mousePnts_half,plane2dPnts, row, col))
            return false;
        for (unsigned int i = 0; i < plane2dPnts.size();i++)
        {
            plane2dPnts[i].x *= scaleSize;
            plane2dPnts[i].y *= scaleSize;
        }
        return true;
	return true;
}

bool CoreAlgorithm::isOnLine(const Point2f& pnt1, const Point2f& pnt2, const Point2f& pnt3)
{
	float dis_threshold = 15;
	float a = (pnt1.y - pnt2.y) / (pnt1.x*pnt2.y - pnt2.x*pnt1.y);
	float b = (pnt1.x - pnt2.x) / (pnt2.x*pnt1.y - pnt1.x*pnt2.y);

	//�㵽ֱ�ߵľ���d=fabs(a*x+b*y+1)/sqrt(a*a+b*b);
	float dis = fabs(a*pnt3.x + b*pnt3.y + 1) / sqrt(a*a + b*b);

	if (dis > dis_threshold)
		return false;

	return true;
}

bool CoreAlgorithm::sortRingCenter(const vector<Point2f>& mousePnts,vector<Point2f>& center,int& row,int& col)
{
	//�ҳ��ĸ����ϵĵ�
	vector<Point2f> cornerRingPnts;
	for (unsigned int i = 0; i < mousePnts.size(); i++)
	{
		Point2f pnt = center[0];
		float minDis = sqrtf((pnt.x-mousePnts[i].x)*(pnt.x-mousePnts[i].x)+
			(pnt.y-mousePnts[i].y)*(pnt.y-mousePnts[i].y));
		for (unsigned int n = 1; n < center.size(); n++)
		{
			float dis = sqrtf((center[n].x-mousePnts[i].x)*(center[n].x-mousePnts[i].x)+
				(center[n].y-mousePnts[i].y)*(center[n].y-mousePnts[i].y));
			if (dis < minDis)
			{
				minDis = dis;
				pnt = center[n];
			}
		}
		cornerRingPnts.push_back(pnt);
	}

	//�ҳ������������ϵĵ�
	Point2f pntlefttop = cornerRingPnts[0];
	Point2f pntrighttop = cornerRingPnts[1];
	Point2f pntrightdown = cornerRingPnts[2];
	Point2f pntleftdown = cornerRingPnts[3];

	vector<Point2f> leftColPnts,rightColPnts;
	for (unsigned int i = 0; i < center.size();i++)
	{
		if (isOnLine(pntlefttop, pntleftdown, center[i]))
		{
			leftColPnts.push_back(center[i]);
		}
		else if (isOnLine(pntrighttop, pntrightdown, center[i]))
		{
			rightColPnts.push_back(center[i]);
		}
	}

	if ( rightColPnts.size()!=leftColPnts.size())
		return false;

	row = rightColPnts.size();

	//�������еĵ㰴��top2down��˳������,�Ȱ���������������
	//�������е��Ȱ���Y�����������жϵ�top��ľ���
	sortByYMin2Max(rightColPnts);
	float dis2Top = distance(rightColPnts[row - 2], pntrighttop);
	float dis2Top2 = distance(rightColPnts[1] , pntrighttop);
	if ( dis2Top < dis2Top2)
		reverse(rightColPnts);

	sortByYMin2Max(leftColPnts);
	dis2Top = distance(leftColPnts[row - 2], pntlefttop);
	dis2Top2 = distance(leftColPnts[1] , pntlefttop);
	if ( dis2Top < dis2Top2)
		reverse(leftColPnts);	

	//�ӵ����һ�п�ʼ��
	vector<Point2f> result;
	int colSize = 0;
	for (unsigned int n = 0; n <leftColPnts.size() ;n++)
	{
		vector<Point2f> rowPnts;
		for (unsigned int i = 0; i < center.size(); i++)
		{
			if (isOnLine(leftColPnts[n], rightColPnts[n], center[i]))
			{
				rowPnts.push_back(center[i]);
			}
		}

		sortByXMin2Max(rowPnts);

		float dis2left = distance(rowPnts[row - 2],leftColPnts[n]);
		float dis2left2 = distance(rowPnts[1], leftColPnts[n]);
		
		if (dis2left < dis2left2)
			reverse(rowPnts);

		if (colSize == 0)
			colSize = rowPnts.size();
		else
		{
			if (colSize != rowPnts.size())
				return false;
		}

		for (unsigned int i = 0; i <rowPnts.size() ;i++)
		{
			result.push_back(rowPnts[i]);
		}
	}

	col = colSize;
	center = result;
	return true;
}
bool CoreAlgorithm::sortRingCenter2(vector<Point2f>& center, const int row, const int col)
{
    //�ҳ��ĸ����ϵĵ�
    Point2f rectAnglePnts[4];
    vector<Point2f> cornerRingPnts;
    minAreaRect(center).points(rectAnglePnts);//����ĵ��ǰ���˳ʱ�������
    for (int i = 0; i < 4; i++)
    {
        Point2f pnt = center[0];
        float minDis = distance(pnt,rectAnglePnts[i]);
        for (unsigned int n = 1; n < center.size(); n++)
        {
            float dis = distance(center[n], rectAnglePnts[i]);
            if (dis < minDis)
            {
                minDis = dis;
                pnt = center[n];
            }
        }
        cornerRingPnts.push_back(pnt);
    }

    //�ҳ����Ͻǵ�
    int dis=100000;
    int leftTopIndex;
    for (int n = 0; n < 4;n++)
    {
        int _dis = int(cornerRingPnts[n].x + cornerRingPnts[n].y);
        if (_dis<dis)
        {
            dis = _dis;
            leftTopIndex = n;
        }
    }

    //�ҳ������������ϵĵ�
    Point2f pntlefttop = cornerRingPnts[leftTopIndex];
    Point2f pntrighttop = cornerRingPnts[(leftTopIndex+1)%4];
    Point2f pntrightdown = cornerRingPnts[(leftTopIndex+2)%4];
    Point2f pntleftdown = cornerRingPnts[(leftTopIndex+3)%4];

    vector<Point2f> leftColPnts,rightColPnts;
    for (unsigned int i = 0; i < center.size();i++)
    {
        if (isOnLine(pntlefttop, pntleftdown, center[i]))
        {
            leftColPnts.push_back(center[i]);
        }
        else if (isOnLine(pntrighttop, pntrightdown, center[i]))
        {
            rightColPnts.push_back(center[i]);
        }
    }

    if ( rightColPnts.size()!=leftColPnts.size() /* || rightColPnts.size()!=row || leftColPnts.size()!=row*/)
        return false;

    //�������е��Ȱ���Y��������
    sortByYMin2Max(rightColPnts);

    sortByYMin2Max(leftColPnts);

    //�ӵ����һ�п�ʼ��
    vector<Point2f> result;
    int colSize = 0;
    for (unsigned int n = 0; n <leftColPnts.size() ;n++)
    {
        vector<Point2f> rowPnts;
        for (unsigned int i = 0; i < center.size(); i++)
        {
            if (isOnLine(leftColPnts[n], rightColPnts[n], center[i]))
            {
                rowPnts.push_back(center[i]);
            }
        }

        sortByXMin2Max(rowPnts);

        /*if (rowPnts.size() != col)
            return false;*/

        if (colSize == 0)
            colSize = rowPnts.size();
        else
        {
            if (colSize != rowPnts.size())
                return false;
        }

        for (unsigned int i = 0; i <rowPnts.size() ;i++)
        {
            result.push_back(rowPnts[i]);
        }
    }

    center = result;
    return true;
}
bool CoreAlgorithm::extraRingCenter2(const Mat& img, const vector<Point2f>& mousePnts,vector<Point2f>& ImageCorner, int& row, int& col)
{
	 //ת���ɻҶ�ͼ
        Mat ImageGray;
        if(img.channels()==1)
        {
            ImageGray = img;
        }
        else
        {
        cvtColor(img, ImageGray, CV_BGR2GRAY);
        }
      // cv::Rect mask = cv::Rect(mousePnts[0],mousePnts[2]);
			//cv::Rect mask =cv::Rect(Point2f(0,0),ImgOrignal.size());
			//step-2 �߾���Բ���
		vector<RotatedRect> findResults;
		vector<Point2f> plane2dPntstemp;
		cv::Rect mask =cv::Rect(mousePnts[0],mousePnts[2]);
		if(!CoreAlgorithm::findEllipses(ImageGray,mask,findResults,5,false))
		{
			return false;
		}
		/*if(!CoreAlgorithm::findEllipses(ImageGray,mask,findResults))
		{
			return false;
		}*/
		for(unsigned int i=0;i<findResults.size();i++)
		{
			if(findResults[i].size.width>90)
				plane2dPntstemp.push_back(findResults[i].center);

		}
		/*for (unsigned int i = 0; i < plane2dPntstemp.size();i++)
        {
            plane2dPntstemp[i].x += mask.x;
            plane2dPntstemp[i].y += mask.y;
        }*/

        if (!sortRingCenter(mousePnts,plane2dPntstemp, row, col))
            return false;
        return true;
}

void CoreAlgorithm::sortByXMin2Max(vector<Point2f>& pnts)
{
	for(unsigned int i=0;i<pnts.size()-1;i++)
	{
		for (unsigned int j = 0; j<pnts.size() - 1; j++)
		{
			if (pnts[j].x>pnts[j + 1].x)
			{
				swap(pnts[j], pnts[j + 1]);
			}
		}
	}
}

void CoreAlgorithm::sortByYMin2Max(vector<Point2f>& pnts)
{
	for(unsigned int i=0;i<pnts.size()-1;i++)
	{
		for (unsigned int j = 0; j<pnts.size() - 1; j++)
		{
			if (pnts[j].y>pnts[j + 1].y)
			{
				swap(pnts[j], pnts[j + 1]);
			}
		}
	}
}

void CoreAlgorithm::reverse(vector<Point2f>& pnts)
{
	vector<Point2f> result;
	for (unsigned int i = pnts.size()-1; i >=0 ; i--)
	{
		result.push_back(pnts[i]);
	}
	pnts = result;
}

bool CoreAlgorithm::ringSubPix(const Mat& img, vector<Point2f>& centers,const Mat& tampl)
{
	//������ļ��
	int count = 0;
	float space = 0;
	float dis_max = distance(centers[0], centers[1]) + 30;
	for (unsigned int i = 0; i < centers.size()-1; i++)
	{
		float _dis = distance(centers[i], centers[i+1]);
		if (_dis < dis_max)
		{
			space += _dis;
			count++;
		}
	}
	space /= count;

	space  = float(cvRound(space));

	//��������ģ��Ϊ�������ľഴ��ģ��
	vector<float> dx_vec, dy_vec;
	Mat _tampl;
	if (tampl.empty())
	{
		_tampl = img(cv::Rect(Point2f(centers[0].x - (space-1) / 2, centers[0].y - (space-1) / 2),
		Point2f(centers[0].x + (space+1) / 2, centers[0].y + (space+1) / 2))).clone();
	}
	else
	{
		///// ��ͼ��ģ��tampl  �������ľ���space����resize
		cv::resize(tampl,_tampl,Size(int(space),int(space)),0,0,cv::INTER_CUBIC );		
	}

	
	//����ģ��ƥ��ROI�����ROI�ı߽���չһ��space�ǲ����ģ�Ӧ����1.5��space
	Mat img_ROI;
	cv::Rect ROI = boundingRect(centers);
	
	ROI.x =int( ROI.x - 1.5*space);   
	ROI.y = int(ROI.y - 1.5*space);
	ROI.width = int(ROI.width + 3 * space); 
	ROI.height = int(ROI.height + 3 * space);
	/// zhangxu added �ж�ROI�Ƿ�Խ��
	if(ROI.x<0)
	{
		ROI.x = 0;
	}
	if(ROI.y<0)
	{
		ROI.y = 0;
	}
	if(ROI.x+ROI.width>img.cols-1)
	{
		ROI.width = img.cols - ROI.x -1;
	}
	if(ROI.y+ROI.height>img.rows-1)
	{
		ROI.height = img.rows - ROI.y -1;
	}
	////// end the test and obtain the ROI
	img_ROI = img(ROI);

	//CV_TM_CCORR_NORMED��CV_TM_CCOEFF��CV_TM_CCOEFF_NORMED��Խ��ƥ���Խ��
	Mat result;
	matchTemplate(img_ROI, _tampl, result, CV_TM_CCOEFF);  ///// ���matchTemplateҪ���������������ı���������һ�µģ�Ҫô�����޷�������8λ��Ҫô���Ǹ�����32λ
	normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, Mat());
	int centerArea_Size = 3;
	
	for (unsigned int n = 0; n < centers.size(); n++)
	{
		//������ĵ���ƥ����resultͼ���е����꣬��Ϊresult �ĳߴ���  (W-w+1) X (H-h+1)������ת��������Ͳ���ʹ����size������
				
		float x = centers[n].x - float(_tampl.cols-1) / 2 - ROI.x;
		float y = centers[n].y - float(_tampl.rows-1) / 2 - ROI.y;
		
		//�ڼ������ĵ㸽���ҳ�ƥ�����ĵ�
		double minVal; double maxVal; Point minLoc; Point maxLoc;
		///����Խ����
         cv::Rect mask_roi = cv::Rect(cv::Point2i(cvRound(x)-centerArea_Size,cvRound(y)-centerArea_Size),
                              cv::Point2i(cvRound(x)+centerArea_Size+1,cvRound(y)+centerArea_Size+1));
         if(mask_roi.x<0)
         {
             mask_roi.x = 0;
         }
         if(mask_roi.y<0)
         {
             mask_roi.y = 0;
         }
         if(mask_roi.x+mask_roi.width>result.cols-1)
         {
             mask_roi.width = result.cols - ROI.x -1;
         }
         if(mask_roi.y+mask_roi.height>result.rows-1)
         {
             mask_roi.height = result.rows - ROI.y -1;
         }

        Mat centerArea = result(mask_roi);
        minMaxLoc( centerArea, &minVal, &maxVal, &minLoc, &maxLoc );

        Point matchLoc = maxLoc;
        ////// �˴���������ؼ����㣬��result�е�����
        matchLoc.x += cvRound(x-centerArea_Size);
        matchLoc.y += cvRound(y-centerArea_Size);

		//���������ؼ������
		Mat win(3,3,CV_32F);
		for (int i = 0; i < 3;i++)
		{
			for (int j = 0; j < 3; j++)
			{
				int col = matchLoc.x+(j-1);
				int row = matchLoc.y+(i-1);
				//// test
				float dd = result.at<float>(row, col);
				win.at<float>(i, j) = result.at<float>(row, col);		
		
			}
		}

		float dx=0, dy=0;
		quadSubPixel(win,dx,dy);
		//// zhang xu added �õ���ʱ�����ؾ��ȼ�������
		centers[n].x = matchLoc.x + float(_tampl.cols-1) / 2 + ROI.x;
		centers[n].y = matchLoc.y + float(_tampl.rows-1) / 2 + ROI.y;
		//// ���������ص�����ƫ����
		centers[n].x += dx;
		centers[n].y += dy;
		////// ���� test dx dy
		if (fabs(dx)>1.5 ||fabs(dy)>1.5)
		{
			int ee;
			ee=2;
		}
		//// end test
	}
	return true;
}

cv::Mat CoreAlgorithm::generateRingTemp(const vector<Point2f>& centers)
{
	Mat tempImg;
	float dis_threshold = 100; 

	int count = 0;
	float space = 0;
	for (unsigned int i = 0; i < centers.size()-1; i++)
	{
		float _dis = distance(centers[i], centers[i+1]);
		if (_dis < dis_threshold)
		{
			space += _dis;
			count++;
		}
	}
	space /= count;

	double R,r;
	R = 0.25*space; r = 0.5*R;//���ݹ̶��ı���

	//R = 27; r = 0.5*R;//test ,����뾶���ԵĻ������ܴ�
	int white_size=7;
	tempImg = Mat(int(2*R+white_size),int(2*R+white_size),CV_8UC1);
	for (int i = 0; i < tempImg.rows;i++)
	{
		for (int j = 0; j < tempImg.cols;j++)
		{
			tempImg.at<uchar>(i, j) = 255;
		}
	}

	Point2f center = Point2f(float((tempImg.cols - 1) / 2),float((tempImg.rows - 1) / 2));
	cv::circle( tempImg, center,int(R), Scalar(0), -1, 8 );
	cv::circle( tempImg, center,int(r), Scalar(255), -1, 8 );

	return tempImg.clone();
}

float CoreAlgorithm::distance(const Point2f& pnt1, const Point2f& pnt2)
{
	return sqrtf(pow(pnt1.x - pnt2.x,2) + pow(pnt1.y - pnt2.y,2));
}

double CoreAlgorithm::containsPnt(vector<Point2f> mousePnts, Point2f pnt)
{
	return pointPolygonTest(mousePnts,pnt,false);
}
float CoreAlgorithm::roundness(const Point2f& pnt1, const vector<Point>& circle, const float r)
{
    float res=0;
    for (unsigned int i = 0; i < circle.size(); i++)
    {
        float dis = distance(pnt1, circle[i]);
        res += pow(abs(dis-r),2);
    }
    float non_roundness = sqrtf(res/circle.size());
    return non_roundness;
}
bool CoreAlgorithm::findRingBoardCenter(const Mat& img, Size patternSize, vector<Point2f>& centers)
{
   
    //ת���ɻҶ�ͼ
        Mat ImageGray;
        if(img.channels()==1)
        {
            ImageGray = img;
        }
        else
        {
        cvtColor(img, ImageGray, CV_BGR2GRAY);
        }
        //chengwei changed

        ////ֱ��ͼ���⻯
        //equalizeHist(ImageGray,ImageGray);

        //// ��ֵ�˲�
        blur(ImageGray,ImageGray,Size(5,5),Point(-1,-1));


    //����Ӧ��ֵ����
    Mat thresh_img;
    int block_size = 501;
    adaptiveThreshold(ImageGray,thresh_img,255,cv::ADAPTIVE_THRESH_MEAN_C ,cv::THRESH_BINARY,block_size,0 );

    int close_type = cv::MORPH_ELLIPSE;
    int dilate_size = 3;
    Mat element = getStructuringElement(close_type,Size(2*dilate_size+1,2*dilate_size+1),Point(-1,-1));
    dilate(thresh_img, thresh_img, element,Point(-1, -1),1);

    //������
    vector<vector<Point>> all_contours;
    vector<int> contoursIndex;
    vector<cv::Vec4i> all_hierarchy;
    Mat temp_LED;
    thresh_img.copyTo(temp_LED);
    findContours(temp_LED,all_contours,all_hierarchy,CV_RETR_EXTERNAL ,CV_CHAIN_APPROX_SIMPLE);

    //��������
    vector<vector<Point>> contours;
    vector<cv::Vec4i> hierarchy;
    for (unsigned int n = 0; n<all_contours.size(); n++)
    {
        double area = contourArea(all_contours[n]);
        //������������ĵ������
       // if(  all_contours[n].size() < 80 || all_contours[n].size() > 700)
		if(  all_contours[n].size() < 80 )
        {continue;}
        //����������������Ȱ��������ظ���
        /*else if(  area < area_threshold_min || area > area_threshold_max )
        {continue;}*/
        else
        {
            RotatedRect rotatedBox;
            rotatedBox = fitEllipse(all_contours[n]);
            float height = rotatedBox.size.height;
            float width = rotatedBox.size.width;
            Point2f center = rotatedBox.center;
            if( height > width ? (height/width<1.5) : (width/height<1.5) )
            {
                //�жϵ�һ���������������Ƿ����丸�����ӽ�
                int k = all_hierarchy[n][2];
                if (k>0)//��k<0�������û��������
                {
                    if(  all_contours[k].size() < 30 || all_contours[k].size() > 350)
                    {continue;}

                    Point2f _center = fitEllipse(all_contours[k]).center;
                    if (abs(_center.x - center.x) < 2 && abs(_center.y - center.y) < 2)
                    {
                        if (roundness(center, all_contours[n], (height + width) / 4) < 5)
                        {
                            centers.push_back(rotatedBox.center);
                            contoursIndex.push_back(n);
                            contours.push_back(all_contours[n]);
                        }
                    }
                }
            }
        }
    }

    if (centers.size()!=patternSize.width * patternSize.height)
    {
        return false;
    }

    return sortRingCenter2(centers,patternSize.height,patternSize.width);
}

bool CoreAlgorithm::getTagPoint2fStrong(const Mat& img, vector<TagPoint2f>& tagPnts2f,Mat cameraMatrixLeft,Mat cameraMatrixRight)
{
	//STEP-1:���ĵ���ȡ
	vector<Point2f> centerPnts;
	vector<float> longaxisRadius;
	Mat ImageGray;
	if (img.channels() == 1)
	{
		ImageGray = img;
	}
	else
	{
		cvtColor(img, ImageGray, CV_BGR2GRAY);
	}
	CoreAlgorithm::detectLightSpot_LED(ImageGray, centerPnts, longaxisRadius);
	////////////////////////////////////////////
	vector<TagPoint2f>  tagPnts;
	 //25,45
	float Featurelength;
	//double t = (double)getTickCount();////��ʼ��ʱ
	double pi = acos(-1);
	double angle;
	double a = 45.0;
	double d = 15.0;
	double gamma = 2 * (a + d) / d;

 	if (!findPntsWithTagV(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma))/////���޶�
	{
		angle = 5.0 / 180.0 * pi;
		if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
		{
			angle = -5.0 / 180.0 * pi;
			if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
			{
				angle = 10.0 / 180.0 * pi;
				if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
				{
					angle = -10.0 / 180.0 * pi;
					if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
					{
						angle = 15.0 / 180.0 * pi;
						if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))/////���޶�
						{
							angle = -15.0 / 180.0 * pi;
							if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
							{
								angle = 20.0 / 180.0 * pi;
								if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))//////���޶�
								{
									angle = -20.0 / 180.0 * pi;
									if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
									{
										angle = 25.0 / 180.0 * pi;
										if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
										{
											angle = -25.0 / 180.0 * pi;
											if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
											{
												angle = 30.0 / 180.0 * pi;
												if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
												{
													angle = -30.0 / 180.0 * pi;
													if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
													{
														angle = 35.0 / 180.0 * pi;
														if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
														{
															angle = -35.0 / 180.0 * pi;
															if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
															{
																angle = 40.0 / 180.0 * pi;
																if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
																{
																	angle = -40.0 / 180.0 * pi;
																	if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
																	{
																		angle = 45.0 / 180.0 * pi;
																		if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
																		{
																			angle = -45.0 / 180.0 * pi;
																			if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
																			{
																				angle = 50.0 / 180.0 * pi;
																				if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
																				{
																					angle = -50.0 / 180.0 * pi;
																					if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
																					{
																						cout << "���δ�ɹ����뱣��ͼƬ" << endl;
																						return false;
																					}

																				}
																			}

																		}

																	}

																}


															}


														}

													}

												}

											}

										}

									}


								}

							}

						}

					}

				}


			}

		}

	}
	if (tagPnts.size() != 8)
		return false;

    //STEP-3 �����������и߾��ȵ�Բ���
	//t = ((double)getTickCount() - t) / getTickFrequency();///////���ں�����β������ʱ�䣬��λΪ����
	//cout << "ʱ��== " << t << endl;////���ʱ��
    for(size_t j=0;j<tagPnts.size();j++)
    {
        cv::Rect ROI = cv::Rect(cv::Point2i((int)abs(tagPnts[j].val[1]-2*Featurelength),(int)abs(tagPnts[j].val[2]-2*Featurelength)),
                cv::Size(int(4*Featurelength),int(4*Featurelength)));
        vector<RotatedRect> findResulttemp;
        if(!CoreAlgorithm::findEllipses(ImageGray,ROI,findResulttemp,0.05))
        {
            return false;
        }
        if(!findResulttemp.size())
            return false;
        if(findResulttemp.size()==1)
        {
            if(findResulttemp[0].size.width>2)
            {
                tagPnts[j].val[1] = findResulttemp[0].center.x;
                tagPnts[j].val[2] = findResulttemp[0].center.y;
            }
            else
                return false;
        }
        else
        {
            RotatedRect tempRotateRect = findResulttemp[0];
            for(size_t k=1;k<findResulttemp.size();k++)
            {
                if(tempRotateRect.size.width<findResulttemp[k].size.width)
                    tempRotateRect = findResulttemp[k];
            }
            tagPnts[j].val[1] = tempRotateRect.center.x;
            tagPnts[j].val[2] = tempRotateRect.center.y;
        }
    }
    tagPnts2f = tagPnts;
    return true;
}

////����getTagPoint2fStrong()����������ROI��
bool CoreAlgorithm::getTagPoint2fStrong(const Mat& img, const cv::Rect maskLightPen, vector<TagPoint2f>& tagPnts2f, Mat cameraMatrixLeft, Mat cameraMatrixRight)
{
	//STEP-1:���ĵ���ȡ
	vector<Point2f> centerPnts;
	vector<float> longaxisRadius;
	Mat ImageGray;
	if (img.channels() == 1)
	{
		ImageGray = img;
	}
	else
	{
		cvtColor(img, ImageGray, CV_BGR2GRAY);
	}
	cv::Rect maskRoi = maskLightPen;
	if (maskRoi.x < 0 || maskRoi.y < 0)
	{
		maskRoi.x = 0;
		maskRoi.y = 0;
	}
	if (maskRoi.x>img.cols || maskRoi.y>img.rows)
	{
		maskRoi.x = img.cols - maskRoi.width;
		maskRoi.y = img.rows - maskRoi.height;
	}
	if ((maskRoi.x + maskRoi.width) > img.cols || (maskRoi.y + maskRoi.height) > img.rows)
	{
		maskRoi.width = img.cols - maskRoi.x;
		maskRoi.height = img.rows - maskRoi.y;
	}
	ImageGray = Mat(ImageGray, maskRoi);

	cv::namedWindow("maskRoi", CV_WINDOW_NORMAL);
	cv::imshow("maskRoi", ImageGray);
	cv::waitKey(10);


	if (!CoreAlgorithm::detectLightSpot_LED(ImageGray, centerPnts, longaxisRadius))
	{
		return false;
	}
	////////////////////////////////////////////
 	vector<TagPoint2f>  tagPnts;
	//25,45
	float Featurelength;
	//double t = (double)getTickCount();////��ʼ��ʱ
	double pi = acos(-1);
	double angle;
	double a = 45.0;
	double d = 15.0;
	double gamma = 2 * (a + d) / d;
	if (!findPntsWithTagV(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma))/////���޶�
	{
		angle = 5.0 / 180.0 * pi;
		if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
		{
			angle = -5.0 / 180.0 * pi;
			if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
			{
				angle = 10.0 / 180.0 * pi;
				if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
				{
					angle = -10.0 / 180.0 * pi;
					if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
					{
						angle = 15.0 / 180.0 * pi;
						if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))/////���޶�
						{
							angle = -15.0 / 180.0 * pi;
							if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
							{
								angle = 20.0 / 180.0 * pi;
								if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))//////���޶�
								{
									angle = -20.0 / 180.0 * pi;
									if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
									{
										angle = 25.0 / 180.0 * pi;
										if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
										{
											angle = -25.0 / 180.0 * pi;
											if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
											{
												angle = 30.0 / 180.0 * pi;
												if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
												{
													angle = -30.0 / 180.0 * pi;
													if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
													{
														angle = 35.0 / 180.0 * pi;
														if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
														{
															angle = -35.0 / 180.0 * pi;
															if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
															{
																angle = 40.0 / 180.0 * pi;
																if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
																{
																	angle = -40.0 / 180.0 * pi;
																	if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
																	{
																		angle = 45.0 / 180.0 * pi;
																		if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
																		{
																			angle = -45.0 / 180.0 * pi;
																			if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
																			{
																				angle = 50.0 / 180.0 * pi;
																				if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
																				{
																					angle = -50.0 / 180.0 * pi;
																					if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
																					{
																						cout << "���δ�ɹ����뱣��ͼƬ" << endl;
																						return false;
																					}

																				}
																			}

																		}

																	}

																}


															}


														}

													}

												}

											}

										}

									}


								}

							}

						}

					}

				}


			}

		}

	}
	if (tagPnts.size() != 8)
		return false;

	//STEP-3 �����������и߾��ȵ�Բ���
	//t = ((double)getTickCount() - t) / getTickFrequency();///////���ں�����β������ʱ�䣬��λΪ����
	//cout << "ʱ��== " << t << endl;////���ʱ��
	for (size_t j = 0; j < tagPnts.size(); j++)
	{
		cv::Rect ROI = cv::Rect(cv::Point2i((int)abs(tagPnts[j].val[1] - 2 * Featurelength), (int)abs(tagPnts[j].val[2] - 2 * Featurelength)),
			cv::Size(int(4 * Featurelength), int(4 * Featurelength)));
		vector<RotatedRect> findResulttemp;
		if (!CoreAlgorithm::findEllipses(ImageGray, ROI, findResulttemp, 0.05))
		{
			return false;
		}
		if (!findResulttemp.size())
			return false;
		if (findResulttemp.size() == 1)
		{
			if (findResulttemp[0].size.width > 2)
			{
				tagPnts[j].val[1] = findResulttemp[0].center.x + maskRoi.x;
				tagPnts[j].val[2] = findResulttemp[0].center.y + maskRoi.y;
			}
			else
				return false;
		}
		else
		{
			RotatedRect tempRotateRect = findResulttemp[0];
			for (size_t k = 1; k < findResulttemp.size(); k++)
			{
				if (tempRotateRect.size.width < findResulttemp[k].size.width)
					tempRotateRect = findResulttemp[k];
			}
			tagPnts[j].val[1] = tempRotateRect.center.x;
			tagPnts[j].val[2] = tempRotateRect.center.y;
		}
	}
	tagPnts2f = tagPnts;

	////������������
	for (unsigned int i = 0; i < tagPnts2f.size() - 1; i++)
	{
		for (unsigned int j = i + 1; j<tagPnts2f.size(); j++)
		{
			if (tagPnts2f[i][0] > tagPnts2f[j][0])
			{
				swap(tagPnts2f[i], tagPnts2f[j]);
			}
		}
	}


	return true;
}






void CoreAlgorithm::GetSurfaceNormal(Point3f point1,Point3f point2,Point3f point3,SurfaceNormal &Normal)
{
	//�õ��ı��淨�߷�������ԭ��
	
	float  w0,w1,w2,v0,v1,v2,nx,ny,nz; 
	w0=point2.x-point1.x;   
	w1=point2.y-point1.y;   
	w2=point2.z-point1.z;
    v0=point3.x-point1.x;  
    v1=point3.y-point1.y; 
    v2=point3.z-point1.z;
    nx=w1*v2-w2*v1; 
    ny=w2*v0-w0*v2; 
    nz=w0*v1-w1*v0; 
	Normal.x = nx;
	Normal.y = ny;
	Normal.z = nz;
}
void CoreAlgorithm::PnPMethod(vector<Point3f> objectPoints,
                        vector<Point2f> imagePoints,
						 Mat cameraMatrix, Mat distCoeffs,
						 Mat &PoseR,Mat &PoseT,
						 vector<double> &dpdrot,vector<double> &dpdt,vector<Point3f> &PointToCam,int Flag)
 {
	 float xsum=0,ysum=0,zsum=0;
	for(size_t h=0;h<objectPoints.size();h++)
	{
		xsum +=objectPoints[h].x;
		ysum +=objectPoints[h].y;
		zsum +=objectPoints[h].z;
	}
	Point3f centroid = Point3f(xsum/objectPoints.size(),ysum/objectPoints.size(),zsum/objectPoints.size());
	vector<Point3f> objectPointsnew;
	objectPointsnew.resize(objectPoints.size());
	for(size_t l=0;l<objectPoints.size();l++)
	{
		objectPointsnew[l] = objectPoints[l]-centroid;
	}
	 PoseR = Mat::zeros(3,1,CV_64F);
	 PoseT = Mat::zeros(3,1,CV_64F);
	 //��һ����ʹ��sovlePnP�õ���ת������ƽ������
	 if(!(solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs,PoseR,PoseT,false,Flag)))
	 {
		 return;
	 }
	 //�ڶ�����ʹ��projectPoints�õ���ͶӰ�Ķ�ά�����㣬�õ��ſɱȾ��󣬽�����ȡǰ���У� ��ת������ƽ���������򹹳�J
     vector<Point2f> imageReprojected;
	 Mat jacobian;
	 Mat dpoutpara(objectPoints.size()*2,6,CV_64FC1);
	 projectPoints(objectPoints,PoseR,PoseT,cameraMatrix,distCoeffs,imageReprojected,jacobian);
	 //��drot dt�ŵ�doutpara��(��ȡǰ����)
	for( int i=0;i<6;i++ )
	{
		jacobian.col(i).copyTo(dpoutpara.col(i));
	}
	 //�������������������ͼ�����꣬�����ͼ�����������������Ϊ����Ͷ������Ϊdelta����2nX1�ľ���
	Mat delta(dpoutpara.rows,1,CV_64FC1);
	for (unsigned int i = 0;i < imagePoints.size();i++)
	{
		delta.at<double>(2*i,0) = fabs(double(imagePoints[i].x-imageReprojected[i].x));
		delta.at<double>(2*i+1,0) =fabs(double(imagePoints[i].y-imageReprojected[i].y));

	}
	//XMLWriter::writeMatData("F:/1.xml",delta);
	 //���Ĳ������ݹ�ʽ���һ������ֵ��6X6�ľ���ȡ�Խ���Ԫ�ؼ�Ϊ���Ϊ6X6�ľ�������ƽ������λ��׼�����3���������Ϊ3����׼������
	Mat covariance_pre;
	Mat dpoutpara_invert;
	Mat covariance;
	double error = invert(dpoutpara,dpoutpara_invert,CV_SVD);
	gemm(dpoutpara_invert,delta,1,0,0,covariance_pre);
	//covariance_pre = dpoutpara_invert*delta;//���־�����˷�ʽҲ�ǿ��е�
	mulTransposed(covariance_pre,covariance,0);
	 //���岽�����3����׼��
	Mat diag_covariance(covariance.rows,1,CV_64FC1);
	diag_covariance = covariance.diag(0);//ȡ���Խ���
	for(int i =0;i<diag_covariance.rows;i++)
	{
		if(i<3)
		{
			dpdrot.push_back(3*sqrt(abs(diag_covariance.at<double>(i,0))));
		}
		else
		{
			dpdt.push_back(3*sqrt(abs(diag_covariance.at<double>(i,0))));
		}
	}
	//������ϵԪԭ��ת����ԭ��������ԭ����
	Point3f pointtemp = Point3f(PoseT);
	pointtemp = pointtemp-centroid;
	PoseT = Mat(pointtemp);
	PoseT.convertTo(PoseT,CV_64F);
	 //�����������ת������������ϵ�µ���ά�����㣬��Ϊ���
	 Mat _R = Mat::zeros(3,3,CV_64F);
	 Rodrigues(PoseR,_R);
	for(unsigned int i = 0;i < objectPoints.size();i++)
	{
		Mat pointtemp1;
		Mat pointtemp4;
		Point3f pointtemp2 = Point3f(0,0,0);
		cv::Point3d pointtemp3 = cv::Point3d(0,0,0);
		pointtemp1.setTo(Scalar(0));
		pointtemp4.setTo(Scalar(0));
		pointtemp3.x = objectPoints[i].x;
		pointtemp3.y = objectPoints[i].y;
		pointtemp3.z = objectPoints[i].z;
		pointtemp1 = Mat(pointtemp3);
		pointtemp4 = _R*pointtemp1 + PoseT;
		pointtemp2 = Point3f(pointtemp4);
		PointToCam.push_back(pointtemp2);
	}
	return;
 }
void CoreAlgorithm::PnPMethod(vector<Point3f> objectPoints,
                        vector<Point2f> imagePoints,
						 Mat cameraMatrix, Mat distCoeffs,
						 Mat &PoseR,Mat &PoseT,
						 vector<double> &dpdrot,vector<double> &dpdt,int Flag)
 {
	float xsum=0,ysum=0,zsum=0;
	for(size_t h=0;h<objectPoints.size();h++)
	{
		xsum +=objectPoints[h].x;
		ysum +=objectPoints[h].y;
		zsum +=objectPoints[h].z;
	}
	Point3f centroid = Point3f(xsum/objectPoints.size(),ysum/objectPoints.size(),zsum/objectPoints.size());
	vector<Point3f> objectPointsnew;
	objectPointsnew.resize(objectPoints.size());
	for(size_t l=0;l<objectPoints.size();l++)
	{
		objectPointsnew[l] = objectPoints[l]-centroid;
	}
	PoseR = Mat::zeros(3,1,CV_64F);
	PoseT = Mat::zeros(3,1,CV_64F);
	 //��һ����ʹ��sovlePnP�õ���ת������ƽ������
	 if(!(solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs,PoseR,PoseT,false,Flag)))
	 {
		 return;
	 }
	 //�ڶ�����ʹ��projectPoints�õ���ͶӰ�Ķ�ά�����㣬�õ��ſɱȾ��󣬽�����ȡǰ���У� ��ת������ƽ���������򹹳�J
     vector<Point2f> imageReprojected;
	 Mat jacobian;
	 Mat dpoutpara(objectPoints.size()*2,6,CV_64FC1);
	 projectPoints(objectPoints,PoseR,PoseT,cameraMatrix,distCoeffs,imageReprojected,jacobian);
	 //��drot dt�ŵ�doutpara��(��ȡǰ����)
	for( int i=0;i<6;i++ )
	{
		jacobian.col(i).copyTo(dpoutpara.col(i));
	}
	 //�������������������ͼ�����꣬�����ͼ�����������������Ϊ����Ͷ������Ϊdelta����2nX1�ľ���
	Mat delta(dpoutpara.rows,1,CV_64FC1);
	for (unsigned int i = 0;i < imagePoints.size();i++)
	{
		delta.at<double>(2*i,0) = fabs(double(imagePoints[i].x-imageReprojected[i].x));
		delta.at<double>(2*i+1,0) =fabs(double(imagePoints[i].y-imageReprojected[i].y));

	}
	//XMLWriter::writeMatData("F:/1.xml",delta);
	 //���Ĳ������ݹ�ʽ���һ������ֵ��6X6�ľ���ȡ�Խ���Ԫ�ؼ�Ϊ���Ϊ6X6�ľ�������ƽ������λ��׼�����3���������Ϊ3����׼������
	Mat covariance_pre;
	Mat dpoutpara_invert;
	Mat covariance;
	double error = invert(dpoutpara,dpoutpara_invert,CV_SVD);
	gemm(dpoutpara_invert,delta,1,0,0,covariance_pre);
	//covariance_pre = dpoutpara_invert*delta;//���־�����˷�ʽҲ�ǿ��е�
	mulTransposed(covariance_pre,covariance,0);
	 //���岽�����3����׼��
	Mat diag_covariance(covariance.rows,1,CV_64FC1);
	diag_covariance = covariance.diag(0);//ȡ���Խ���
	for(int i =0;i<diag_covariance.rows;i++)
	{
		if(i<3)
		{
			dpdrot.push_back(3*sqrt(abs(diag_covariance.at<double>(i,0))));
		}
		else
		{
			dpdt.push_back(3*sqrt(abs(diag_covariance.at<double>(i,0))));
		}
	}
	//������ϵԪԭ��ת����ԭ��������ԭ����
	Point3f pointtemp = Point3f(PoseT);
	pointtemp = pointtemp-centroid;
	PoseT = Mat(pointtemp);
	return;
 }
bool CoreAlgorithm::iterativePnPMethod(Mat &img,vector<Point3f> objectPoints,
                        vector<Point2f>imagePoints,
						Mat cameraMatrix, Mat distCoeffs,
						Mat &PoseR,Mat &PoseT,
						vector<double> &dpdrot,vector<double> &dpdt,vector<Point3f> &PointToCam,int Flag,int iterativeTimes)
{
	Mat srcimgMat1;
		if(img.channels()==1)
        {
            srcimgMat1 = img;
        }
        else
        {
			cvtColor(img, srcimgMat1, CV_BGR2GRAY);
        }
	Mat PoseR_Src = Mat::zeros(3,1,CV_64F);
	Mat PoseT_Src = Mat::zeros(3,1,CV_64F);
	Mat PoseR_Dst = Mat::zeros(3,1,CV_64F);
	Mat PoseT_Dst = Mat::zeros(3,1,CV_64F);
	Mat parallelCamR;
	vector<double> dpdrot_Src,dpdt_Src,dpdrot_Dst,dpdt_Dst;
	generateParallelCamR(parallelCamR);
	Mat argaMat = Mat::zeros(3,3,CV_64F);
	Mat argaMatt = Mat::zeros(3,1,CV_64F);
	argaMat.at<double>(1,2) =1;argaMat.at<double>(0,1) =1;argaMat.at<double>(2,0) =1;
	vector<Point3f> pnts3d;
	pointPosetranslation(objectPoints,pnts3d,argaMat,argaMatt);
	PnPMethod(pnts3d,imagePoints,cameraMatrix,distCoeffs,PoseR_Src,PoseT_Src,dpdrot_Src,dpdt_Src,Flag);
	for(unsigned int k = iterativeTimes;k>0;k--)
	{
		 //STEP-1 ����ÿ��������ͼ���ƽ����ͼ��R'����������ת���� 3x3
		Mat parallelImgR,t1;
		calculateParallelViewR(PoseR_Src,PoseT_Src,parallelImgR);
		//STEP-2 ������������ת����ƽ����ͼ��
		vector<Point2f> corner_Parallel;
		//Mat t1 = Mat::zeros(3,1,CV_64F);
		PoseT_Src.copyTo(t1);
		undistortPoints2DifferentView(imagePoints,corner_Parallel,cameraMatrix,PoseR_Src,PoseT_Src,distCoeffs,
			cameraMatrix,parallelCamR,t1,vector<double>());
        //STEP-3 ����ƽ����ͼ�ϵ������㣬��ƽ����ͼ�Ͻ��������ؽǵ���
        ////// image is too small
        Mat resultimgMat1(2*srcimgMat1.rows, 2*srcimgMat1.cols, srcimgMat1.type());
        undistortImg(srcimgMat1, resultimgMat1, cameraMatrix, distCoeffs, parallelImgR);
	/*	namedWindow("window",2);
		imshow("window",resultimgMat1);
		waitKey();*/
	    //��ƽ����ͼ�Ͻ���������Բ���
		double roisize = distancePoints2f(corner_Parallel[0],corner_Parallel[5]);
		double roiwidth = distancePoints2f(corner_Parallel[5],corner_Parallel[6])+roisize;
		double roiheight = distancePoints2f(corner_Parallel[0],corner_Parallel[4])+roisize;
		cv::Rect baseROI = cv::Rect(int(corner_Parallel[5].x-roisize/2),int(corner_Parallel[5].y-roisize/2),(int)roiwidth,(int)roiheight);
		vector<TagPoint2f> tagPnts2f;
		vector<Point2f> detectorResult;
		//if(!CoreAlgorithm::getTagPoint2f2(resultimgMat1,tagPnts2f,baseROI))return false;
		pntsTagConfirm(tagPnts2f,detectorResult);
        undistortPoints2DifferentView(detectorResult,detectorResult,
            cameraMatrix,parallelCamR,t1,vector<double>(),
            cameraMatrix,PoseR_Src,PoseT_Src,distCoeffs);
        //STEP-5 ʹ��PNP�㷨����λ��
		dpdrot_Dst.clear();
		dpdt_Dst.clear();
		PnPMethod(pnts3d,detectorResult,cameraMatrix,distCoeffs,PoseR_Dst,PoseT_Dst,dpdrot_Dst,dpdt_Dst,Flag);
        //// ������ֹ�������������û��ʹ��λ�����������׼���С����ֹͣ����
		//�ֱ��������ǰ��λ�����������׼���ģ
		double dpdrot_Dst_value =0;
		double dpdt_Dst_value =0;
		double dpdrot_Src_value =0;
		double dpdt_Src_value =0;
		for(unsigned int i=0;i<3;i++)
		{
			dpdrot_Dst_value +=pow(dpdrot_Dst[i],2);
			dpdt_Dst_value +=pow(dpdt_Dst[i],2);
			dpdrot_Src_value +=pow(dpdrot_Src[i],2);
			dpdt_Src_value +=pow(dpdt_Src[i],2);
		}
        if(!(dpdrot_Dst_value<dpdrot_Src_value||dpdt_Dst_value<dpdt_Src_value))
        {
			PoseR = argaMat.inv()*PoseR_Dst;
			PoseT = PoseT_Dst; 
			dpdrot = dpdrot_Dst;
			dpdt = dpdt_Dst;
			//�����������ת������������ϵ�µ���ά�����㣬��Ϊ���
			Mat _R = Mat::zeros(3,3,CV_64FC1);
			Rodrigues(PoseR,_R);
			pointPosetranslation(objectPoints,PointToCam,_R,PoseT);
			return true;
		}
		else
		{
			PoseR_Src = PoseR_Dst;
			PoseT_Src = PoseT_Dst; 
			dpdrot_Src = dpdrot_Dst;
			dpdt_Src = dpdt_Dst;    
		}
	}
	return true;
}
bool CoreAlgorithm::iterativePnPMethod(Mat &img,vector<Point3f> objectPoints,
                        vector<Point2f>imagePoints,
						Mat cameraMatrix, Mat distCoeffs,
						Mat &PoseR,Mat &PoseT,
						vector<double> &dpdrot,vector<double> &dpdt,int Flag,int iterativeTimes)
{
	Mat srcimgMat1;
		if(img.channels()==1)
        {
            srcimgMat1 = img;
        }
        else
        {
			cvtColor(img, srcimgMat1, CV_BGR2GRAY);
        }
	Mat PoseR_Src = Mat::zeros(3,1,CV_64F);
	Mat PoseT_Src = Mat::zeros(3,1,CV_64F);
	Mat PoseR_Dst = Mat::zeros(3,1,CV_64F);
	Mat PoseT_Dst = Mat::zeros(3,1,CV_64F);
	Mat parallelCamR;
	vector<double> dpdrot_Src,dpdt_Src,dpdrot_Dst,dpdt_Dst;
	generateParallelCamR(parallelCamR);
	Mat argaMat = Mat::zeros(3,3,CV_64F);
	Mat argaMatt = Mat::zeros(3,1,CV_64F);
	//argaMat.at<double>(1,2) =1;argaMat.at<double>(0,1) =1;argaMat.at<double>(2,0) =1;
	vector<Point3f> pnts3d;
	//pointPosetranslation(objectPoints,pnts3d,argaMat,argaMatt);
	PnPMethod(objectPoints,imagePoints,cameraMatrix,distCoeffs,PoseR_Src,PoseT_Src,dpdrot_Src,dpdt_Src,Flag);
	for(unsigned int k = iterativeTimes;k>0;k--)
	{
		 //STEP-1 ����ÿ��������ͼ���ƽ����ͼ��R'����������ת���� 3x3
		Mat parallelImgR,t1;
		calculateParallelViewR(PoseR_Src,PoseT_Src,parallelImgR);
		//STEP-2 ������������ת����ƽ����ͼ��
		vector<Point2f> corner_Parallel;
		//Mat t1 = Mat::zeros(3,1,CV_64F);
		PoseT_Src.copyTo(t1);
		undistortPoints2DifferentView(imagePoints,corner_Parallel,cameraMatrix,PoseR_Src,PoseT_Src,distCoeffs,
			cameraMatrix,parallelCamR,t1,vector<double>());
        //STEP-3 ����ƽ����ͼ�ϵ������㣬��ƽ����ͼ�Ͻ��������ؽǵ���
        ////// image is too small
        Mat resultimgMat1(2*srcimgMat1.rows, 2*srcimgMat1.cols, srcimgMat1.type());
        undistortImg(srcimgMat1, resultimgMat1, cameraMatrix, distCoeffs, parallelImgR);
	/*	namedWindow("window",2);
		imshow("window",resultimgMat1);
		waitKey();*/
	    //��ƽ����ͼ�Ͻ���������Բ���
		double roisize = distancePoints2f(corner_Parallel[0],corner_Parallel[5]);
		double roiwidth = distancePoints2f(corner_Parallel[5],corner_Parallel[6])+roisize;
		double roiheight = distancePoints2f(corner_Parallel[0],corner_Parallel[4])+roisize;
		cv::Rect baseROI = cv::Rect(int(corner_Parallel[5].x-roisize/2),int(corner_Parallel[5].y-roisize/2),(int)roiwidth,(int)roiheight);
		vector<TagPoint2f> tagPnts2f;
		vector<Point2f> detectorResult;
		//if(!CoreAlgorithm::getTagPoint2f2(resultimgMat1,tagPnts2f,baseROI))return false;
		pntsTagConfirm(tagPnts2f,detectorResult);
        undistortPoints2DifferentView(detectorResult,detectorResult,
            cameraMatrix,parallelCamR,t1,vector<double>(),
            cameraMatrix,PoseR_Src,PoseT_Src,distCoeffs);
        //STEP-5 ʹ��PNP�㷨����λ��
		dpdrot_Dst.clear();
		dpdt_Dst.clear();
		PnPMethod(objectPoints,detectorResult,cameraMatrix,distCoeffs,PoseR_Dst,PoseT_Dst,dpdrot_Dst,dpdt_Dst,Flag);
        //// ������ֹ�������������û��ʹ��λ�����������׼���С����ֹͣ����
		//�ֱ��������ǰ��λ�����������׼���ģ
		double dpdrot_Dst_value =0;
		double dpdt_Dst_value =0;
		double dpdrot_Src_value =0;
		double dpdt_Src_value =0;
		for(unsigned int i=0;i<3;i++)
		{
			dpdrot_Dst_value +=pow(dpdrot_Dst[i],2);
			dpdt_Dst_value +=pow(dpdt_Dst[i],2);
			dpdrot_Src_value +=pow(dpdrot_Src[i],2);
			dpdt_Src_value +=pow(dpdt_Src[i],2);
		}
        if(!(dpdrot_Dst_value<dpdrot_Src_value||dpdt_Dst_value<dpdt_Src_value))
        {
			//PoseR = argaMat.inv()*PoseR_Dst;
			PoseR = PoseR_Dst;
			PoseT = PoseT_Dst; 
			dpdrot = dpdrot_Dst;
			dpdt = dpdt_Dst;
			return true;
		}
		else
		{

			PoseR_Src = PoseR_Dst;
			PoseT_Src = PoseT_Dst; 
			dpdrot_Src = dpdrot_Dst;
			dpdt_Src = dpdt_Dst;    
		}
	}
	return true;
}
bool CoreAlgorithm::PlaneFitting(vector<Point3f> obsPoints,Plane& model)
{
	Mat coefficient_matrix = Mat::zeros(3,3,CV_32F);
	Mat variable_matrix = Mat::zeros(3,1,CV_32F);
	Mat equation_right = Mat::zeros(3,1,CV_32F);
	for(unsigned int i =0;i<obsPoints.size();i++)
	{
		coefficient_matrix.at<float>(0,0) += pow(obsPoints[i].x,2); 
		coefficient_matrix.at<float>(0,1) += obsPoints[i].x*obsPoints[i].y;
		coefficient_matrix.at<float>(0,2) += obsPoints[i].x;
		coefficient_matrix.at<float>(1,1) += pow(obsPoints[i].y,2);
		coefficient_matrix.at<float>(1,2) += obsPoints[i].y;
		equation_right.at<float>(0,0) += obsPoints[i].x*obsPoints[i].z;
		equation_right.at<float>(1,0) += obsPoints[i].y*obsPoints[i].z;
		equation_right.at<float>(2,0) += obsPoints[i].z;
	
	}
	coefficient_matrix.at<float>(1,0) = coefficient_matrix.at<float>(0,1);
	coefficient_matrix.at<float>(2,0) = coefficient_matrix.at<float>(0,2);
	coefficient_matrix.at<float>(2,1) = coefficient_matrix.at<float>(1,2);
	coefficient_matrix.at<float>(2,2) = float(obsPoints.size());
	if(!solve(coefficient_matrix,equation_right,variable_matrix,cv::DECOMP_CHOLESKY))//��˹��Ԫ��?�˴�ϵ������Ϊ�Գƾ���
	{
		return false;
	}
	//����������λ��
	float dist = sqrt(pow(variable_matrix.at<float>(0,0),2)+ pow(variable_matrix.at<float>(1,0),2)+float(1.0));
	if ( dist == 0 ) 
	{ 
		model.normal.x=1;   
		return false;

	}
	else
	{
		model.normal.x=variable_matrix.at<float>(0,0)/dist;        		
		model.normal.y=variable_matrix.at<float>(1,0)/dist;  		
		model.normal.z=float(-1.0)/dist;   		
	}
	model.orignal.x =0;
	model.orignal.y =0;
	model.orignal.z =variable_matrix.at<float>(2,0);
	return true;
}
bool CoreAlgorithm::CircleCylinderFitting(vector<Point3f> obsPoints,CircleCylinder& models,int errorRank)
{
	//step-1 ���һ�����淽�̵�ʮ������ϵ��
	//����һ���ж�ϵ��K.��ϵ��ָ�껹�д�ȷ������
	float K;
	switch (errorRank)
	{
	case 0:
		 K = float(1e-5);
		 break;
	case 1:
		 K = float(1e-4);
		 break;
	case 2:
		 K = float(1e-3);
		 break;
	default:
		break;
	}
	
	Mat coefficient_matrix(obsPoints.size(),10,CV_32F);
	Mat variable_matrix = Mat::zeros(10,1,CV_32F);
	//Mat equation_right = Mat::zeros(10,1,CV_32F);
	for(int i = 0;i<coefficient_matrix.rows;i++)
	{	
		coefficient_matrix.at<float>(i,0)=pow(obsPoints[i].x,2);
		coefficient_matrix.at<float>(i,1)=pow(obsPoints[i].y,2);
		coefficient_matrix.at<float>(i,2)=pow(obsPoints[i].z,2);
		coefficient_matrix.at<float>(i,3)=obsPoints[i].x*obsPoints[i].y;
		coefficient_matrix.at<float>(i,4)=obsPoints[i].y*obsPoints[i].z;
		coefficient_matrix.at<float>(i,5)=obsPoints[i].x*obsPoints[i].z;
		coefficient_matrix.at<float>(i,6)=obsPoints[i].x;
		coefficient_matrix.at<float>(i,7)=obsPoints[i].y;
		coefficient_matrix.at<float>(i,8)=obsPoints[i].z;
		coefficient_matrix.at<float>(i,9)=1;
	}
	cv::SVD::solveZ(coefficient_matrix,variable_matrix);
	float lamada;
	//��һ���������������������Ϊ�㣬��Բ����ƽ����ĳ����ά����ƽ��
	//���ݽ������жϷ���������ֵ�Ƿ�Ϊ��
	if(abs(variable_matrix.at<float>(3,0))<K&&abs(variable_matrix.at<float>(4,0))<K&&abs(variable_matrix.at<float>(5,0))<K)
	{
		//��Ϊ�㣬���������������������㣬һ��Ϊ����
		//step-1�ж���һ����Ϊ����
		if(abs(variable_matrix.at<float>(0,0))-abs(variable_matrix.at<float>(1,0))<K||abs(variable_matrix.at<float>(0,0))-abs(variable_matrix.at<float>(2,0))<K)
		{
			lamada = variable_matrix.at<float>(0,0);		
		}
		else
		{
		     lamada = variable_matrix.at<float>(1,0);
		}
		models.axisNormal.x = sqrt(abs(1-variable_matrix.at<float>(0,0)/lamada));
		models.axisNormal.y = sqrt(abs(1-variable_matrix.at<float>(1,0)/lamada));
		models.axisNormal.z = sqrt(abs(1-variable_matrix.at<float>(2,0)/lamada));
	}
	//�ڶ������������������һ��Ϊ�㣬���಻Ϊ��
	if((abs(variable_matrix.at<float>(3,0))>K||abs(variable_matrix.at<float>(4,0))>K||abs(variable_matrix.at<float>(5,0))>K)&&
		(abs(variable_matrix.at<float>(3,0))<K||abs(variable_matrix.at<float>(4,0))<K||abs(variable_matrix.at<float>(5,0))<K))
	{
		for(int i =3;i<6;i++)
		{
			if(abs(variable_matrix.at<float>(i,0))>K)
			{
				switch (i)
				{
				case 3:
					lamada = variable_matrix.at<float>(2,0);
					if(variable_matrix.at<float>(i,0)*lamada>0)
						 models.axisNormal.y = sqrt(abs(1-variable_matrix.at<float>(1,0)/lamada));
						 models.axisNormal.z = -1*sqrt(abs(1-variable_matrix.at<float>(2,0)/lamada));
						 models.axisNormal.x = sqrt(abs(1-variable_matrix.at<float>(0,0)/lamada));
					break;
				case 4:
					lamada = variable_matrix.at<float>(0,0);
					if(variable_matrix.at<float>(i,0)*lamada>0)
						 models.axisNormal.y = sqrt(abs(1-variable_matrix.at<float>(1,0)/lamada));
						 models.axisNormal.z = -1*sqrt(abs(1-variable_matrix.at<float>(2,0)/lamada));
						 models.axisNormal.x = sqrt(abs(1-variable_matrix.at<float>(0,0)/lamada));
					break;
				case 5:
					lamada = variable_matrix.at<float>(1,0);
					if(variable_matrix.at<float>(i,0)*lamada>0)
						 models.axisNormal.y = sqrt(abs(1-variable_matrix.at<float>(1,0)/lamada));
						 models.axisNormal.z = sqrt(abs(1-variable_matrix.at<float>(2,0)/lamada));
						 models.axisNormal.x = -1*sqrt(abs(1-variable_matrix.at<float>(0,0)/lamada));

					break;
				default:
					break;
				}
				if(variable_matrix.at<float>(i,0)*lamada<0)
				{
					 models.axisNormal.y = sqrt(abs(1-variable_matrix.at<float>(1,0)/lamada));
					 models.axisNormal.z = sqrt(abs(1-variable_matrix.at<float>(2,0)/lamada));
					 models.axisNormal.x = sqrt(abs(1-variable_matrix.at<float>(0,0)/lamada));
				}
			}
		}
	}
	//�����������������������Ϊ��
	if(abs(variable_matrix.at<float>(3,0))>K&&abs(variable_matrix.at<float>(4,0))>K&&abs(variable_matrix.at<float>(5,0))>K)
	{
		if(abs(pow(variable_matrix.at<float>(3,0),2)-pow(variable_matrix.at<float>(5,0),2))<K)
		{
			if(abs(variable_matrix.at<float>(1,0))<K&&abs(variable_matrix.at<float>(2,0)<K))
			{
				models.axisNormal.y =1;
				models.axisNormal.z =1;
				lamada =  variable_matrix.at<float>(4,0)/(-2);
				models.axisNormal.x = variable_matrix.at<float>(5,0)/(lamada*models.axisNormal.z*(-2));
			}
			else
			{
				lamada = (variable_matrix.at<float>(1,0)+variable_matrix.at<float>(2,0)-variable_matrix.at<float>(4,0))/2;
				models.axisNormal.y =sqrt(abs(1-variable_matrix.at<float>(1,0)/lamada));
				models.axisNormal.z =sqrt(abs(1-variable_matrix.at<float>(2,0)/lamada));
				models.axisNormal.x =sqrt(abs(1-variable_matrix.at<float>(0,0)/lamada));
			}
		}
		else
		{
			lamada =(pow(variable_matrix.at<float>(5,0),2)*variable_matrix.at<float>(1,0)-variable_matrix.at<float>(2,0)*pow(variable_matrix.at<float>(3,0),2))/(pow(variable_matrix.at<float>(5,0),2)-pow(variable_matrix.at<float>(3,0),2));
			models.axisNormal.y = sqrt(abs(1-variable_matrix.at<float>(1,0)/lamada));
			models.axisNormal.z = sqrt(abs(1-variable_matrix.at<float>(2,0)/lamada));
			models.axisNormal.x = sqrt(abs(1-variable_matrix.at<float>(0,0)/lamada));
		}
		//�жϷ���������������
		if(variable_matrix.at<float>(3,0)*lamada>0)
		{
			models.axisNormal.y = -1*models.axisNormal.y;
			if(variable_matrix.at<float>(5,0)*lamada>0)
			{
				models.axisNormal.z = -1*models.axisNormal.z;
			}
		}
		else
		{
			if(variable_matrix.at<float>(5,0)*lamada>0)
			{
				models.axisNormal.z = -1*models.axisNormal.z;
			}
		}

	}
	Mat coefficient_matrix2(3,3,CV_32F);
	Mat variable_matrix2(3,1,CV_32F);
	Mat equation_right2(3,1,CV_32F);
	coefficient_matrix2.at<float>(0,0)=variable_matrix.at<float>(0,0)*2;
	coefficient_matrix2.at<float>(0,1)=variable_matrix.at<float>(3,0);
	coefficient_matrix2.at<float>(0,2)=variable_matrix.at<float>(5,0);
	coefficient_matrix2.at<float>(1,0)=variable_matrix.at<float>(3,0);
	coefficient_matrix2.at<float>(1,1)=variable_matrix.at<float>(1,0)*2;
	coefficient_matrix2.at<float>(1,2)=variable_matrix.at<float>(4,0);
	coefficient_matrix2.at<float>(2,0)=variable_matrix.at<float>(5,0);
	coefficient_matrix2.at<float>(2,1)=variable_matrix.at<float>(4,0);
	coefficient_matrix2.at<float>(2,2)=variable_matrix.at<float>(2,0)*2;
	equation_right2.at<float>(0,0)=-1*variable_matrix.at<float>(6,0);
	equation_right2.at<float>(1,0)=-1*variable_matrix.at<float>(7,0);
	equation_right2.at<float>(2,0)=-1*variable_matrix.at<float>(8,0);
	if(!solve(coefficient_matrix2,equation_right2,variable_matrix2,cv::DECOMP_LU))//��˹��Ԫ��?�˴�ϵ������Ϊ�Գƾ���
		return false;
	models.orignal.x =variable_matrix2.at<float>(0,0);
	models.orignal.y =variable_matrix2.at<float>(1,0);
	models.orignal.z =variable_matrix2.at<float>(2,0);
	//���r  
	models.r = sqrt(abs((variable_matrix.at<float>(0,0)*pow(models.orignal.x,2)+variable_matrix.at<float>(1,0)*pow(models.orignal.y,2)+
				variable_matrix.at<float>(2,0)*pow(models.orignal.z,2)+variable_matrix.at<float>(3,0)*models.orignal.x*models.orignal.y+
				variable_matrix.at<float>(4,0)*models.orignal.z*models.orignal.y+variable_matrix.at<float>(5,0)*models.orignal.x*models.orignal.z-
				variable_matrix.at<float>(9,0))/lamada));
	//����������λ��
	float dist = sqrt(pow(models.axisNormal.x,2)+ pow(models.axisNormal.y,2)+pow(models.axisNormal.z,2)); 
	if ( dist == 0 ) 
	{        					
		return false;
	}
	else
	{
		models.axisNormal.x = models.axisNormal.x / dist;        		
		models.axisNormal.y = models.axisNormal.y / dist;        		
		models.axisNormal.z =models.axisNormal.z / dist;        		
	}
	return true;
}
bool CoreAlgorithm::CalPlaneLineIntersectPoint(const Plane _plane,const Line _line,Point3f &IntersectPoint)
{
	float vp1, vp2, vp3, n1, n2, n3, v1, v2, v3, m1, m2, m3, t,vpt;  
	vp1 = _plane.normal.x;  
	vp2 = _plane.normal.y;  
	vp3 = _plane.normal.z;  
	n1 = _plane.orignal.x;  
	n2 = _plane.orignal.y;  
	n3 = _plane.orignal.z;  
	v1 = _line.normal.x;  
	v2 = _line.normal.y;  
	v3 = _line.normal.z;  
	m1 = _line.orignal.x;  
	m2 = _line.orignal.y;  
	m3 = _line.orignal.z;  
	vpt = v1 * vp1 + v2 * vp2 + v3 * vp3;
	//�����ж�ֱ���Ƿ���ƽ��ƽ��  
	if (vpt == 0)  
	{  
		return false;  
	}  
	else  
	{  
		t = ((n1 - m1) * vp1 + (n2 - m2) * vp2 + (n3 - m3) * vp3) / vpt;  
		IntersectPoint.x = m1 + v1 * t;  
		IntersectPoint.y = m2 + v2 * t;  
		IntersectPoint.z = m3 + v3 * t;
		return true; 
	}  	   
}
double CoreAlgorithm::distancePoints2f(const Point2f pnt1, const Point2f pnt2)
{
	
	double distance = sqrt(pow((pnt1.x-pnt2.x),2)+pow((pnt1.y-pnt2.y),2));
	return distance;
}
double CoreAlgorithm::distancePoints(const Point3f pnt1,const Point3f pnt2)
{
	double distance = sqrt(pow((pnt1.x-pnt2.x),2)+pow((pnt1.y-pnt2.y),2)+pow((pnt1.z-pnt2.z),2));
	return distance;
}
bool CoreAlgorithm::findshortAxisPoints(const Point2f footpointNearPnts1,const Point2f footpointNearPnts2,const Point2f footpointNearPnts3,const Point2f footPoint,vector<TagPoint2f>& TagPoints)
{
	vector<TagPoint2f> tagPnts;
	TagPoint2f _tagPnt1, _tagPnt2,_tagPnt6,_tagPnt7;
	_tagPnt1[0] = TAG1;
	_tagPnt1[1] = footPoint.x;
	_tagPnt1[2] = footPoint.y;
	tagPnts.push_back(_tagPnt1);
	//�ҳ����ڶ����ϵĵ�
	float dis_threshold = 15;
	float a, b;//ֱ�߹�ʽax+by+1=0;a=(y1-y2)/(x1y2-x2y1),b=(x1-x2)/(x2y1-x1y2);
	Point2f pointIndex2,pnt1,pnt2,pnt3;
	for(int i=0;i<3;i++)
	{
		if(i==0)
		{
			pnt1 = footpointNearPnts1;
			pnt2 = footpointNearPnts2;
			pnt3 = footpointNearPnts3;
		}
		if(i==1)
		{
			pnt1 = footpointNearPnts2;
			pnt2 = footpointNearPnts3;
			pnt3 = footpointNearPnts1;
		}
		if(i==2)
		{
			pnt1 = footpointNearPnts1;
			pnt2 = footpointNearPnts3;
			pnt3 = footpointNearPnts2;
		}
		a = (pnt1.y - pnt2.y) / (pnt1.x*pnt2.y - pnt2.x*pnt1.y);
		b = (pnt1.x - pnt2.x) / (pnt2.x*pnt1.y - pnt1.x*pnt2.y);
		//�㵽ֱ�ߵľ���d=fabs(a*x+b*y+1)/sqrt(a*a+b*b);
		float dis = float(fabs(float(a*footPoint.x + b*footPoint.y + 1))/sqrt(a*a + b*b));
		if (dis < dis_threshold)//�������ҵĲ�׼��ʱ�������ֵҪ�ſ�һ��
		{
			_tagPnt2[0] = TAG2;
			_tagPnt2[1] = pnt3.x;
			_tagPnt2[2] = pnt3.y;
			tagPnts.push_back(_tagPnt2);
			//����AB����x1,y1,z1��, ����CD����x2,y2,z2��
			//����AB������CD����y1z2-z1y2��z1x2-x1z2��x1y2-y1x2��
			Vec3f norm_X,norm_Y,norm_Z;//norm_XΪ����ķ�������������Ϊ1��5��Ϊ������
			norm_X[0] = pnt3.x - footPoint.x;
			norm_X[1] = pnt3.y - footPoint.y;
			norm_X[2] = 0; 
			norm_Y[0] = pnt1.x -footPoint.x;  
			norm_Y[1] = pnt1.y -footPoint.y; 
			norm_Y[2] = 0; 
			norm_Z[0] = norm_X[1] * norm_Y[2] - norm_X[2] * norm_Y[1];
			norm_Z[1] = norm_X[2] * norm_Y[0] - norm_X[0] * norm_Y[2];
			norm_Z[2] = norm_X[0] * norm_Y[1] - norm_X[1] * norm_Y[0];
			if(norm_Z[2]>0)
			{
				_tagPnt6[0] = TAG6;
				_tagPnt6[1] = pnt1.x;
				_tagPnt6[2] = pnt1.y;
				tagPnts.push_back(_tagPnt6);
				_tagPnt7[0] = TAG7;
				_tagPnt7[1] = pnt2.x;
				_tagPnt7[2] = pnt2.y;
				tagPnts.push_back(_tagPnt7);
				TagPoints = tagPnts;				
			}
			else
			{
				_tagPnt6[0] = TAG6;
				_tagPnt6[1] = pnt2.x;
				_tagPnt6[2] = pnt2.y;
				tagPnts.push_back(_tagPnt6);
				_tagPnt7[0] = TAG7;
				_tagPnt7[1] = pnt1.x;
				_tagPnt7[2] = pnt1.y;
				tagPnts.push_back(_tagPnt7);
				TagPoints = tagPnts;				
			}
			return true;
		}
	}
	return false;
}
bool CoreAlgorithm::findEllipses(const Mat img,const cv::Rect mask,vector<RotatedRect>& findResults,const double precisionlevel,bool multi,int kenelsize)
{
	//step-1 ��ͼ��ת���ɻҶ�ͼ
		Mat ImageGray;
        if(img.channels()==1)
        {
            ImageGray = img;
        }
        else
        {
			cvtColor(img, ImageGray, CV_BGR2GRAY);
        }
		cv::Rect MaskRoi = mask;
		if (MaskRoi.x < 0 || MaskRoi.y < 0)
		{
			MaskRoi.x = 0;
			MaskRoi.y = 0;
		}
		if (MaskRoi.x>img.cols || MaskRoi.y>img.rows)
		{
			MaskRoi.x = img.cols - MaskRoi.width;
			MaskRoi.y = img.rows - MaskRoi.height;
		}
		if ((MaskRoi.x + MaskRoi.width) > img.cols || (MaskRoi.y + MaskRoi.height) > img.rows)
		{
			MaskRoi.width = img.cols - MaskRoi.x;
			MaskRoi.height = img.rows - MaskRoi.y;
		}
		ImageGray = Mat(ImageGray, MaskRoi);
	//step-2 ����ͼ���ݶ���Ϣ
	//step-2-1 ���ɸ�˹�˲���ģ��	
		//�ж��˲�Ƭģ���С�ǲ�������
		if(kenelsize%2==0)
			return false;
		Mat X,Y;
		cv::Range xgv =cv::Range(-abs((kenelsize-1)/2),abs((kenelsize-1)/2));
		cv::Range ygv = xgv;
		std::vector<int> t_x, t_y;  
		for(int i = xgv.start; i <= xgv.end; i++) t_x.push_back(i);  
		for(int j = ygv.start; j <= ygv.end; j++) t_y.push_back(j);  
		cv::repeat(cv::Mat(t_x),1,t_y.size(), X); 
	    cv::repeat(cv::Mat(t_y).t(), t_x.size(),1, Y);
		Mat GaussianKenelx(kenelsize,kenelsize,CV_64F);
		Mat GaussianKenely(kenelsize,kenelsize,CV_64F);	
		for(int i=0;i<kenelsize;i++)
		{
			for(int j=0;j<kenelsize;j++)
			{
				GaussianKenelx.at<double>(i,j) = double(-X.at<int>(i,j)*exp(pow((double)X.at<int>(i,j),2)/-2)*exp(pow((double)Y.at<int>(i,j),2)/-2));
				GaussianKenely.at<double>(i,j) = double(-Y.at<int>(i,j)*exp(pow((double)X.at<int>(i,j),2)/-2)*exp(pow((double)Y.at<int>(i,j),2)/-2));
			}
		}
	//step-2-2 ��ά�˲�������
		Mat dx,dy;
		filter2D(ImageGray,dx,CV_64F,GaussianKenelx);
		filter2D(ImageGray,dy,CV_64F,GaussianKenely);
	//step-2-3 �ݶȷ�������
		Mat gradientnorm,gradientnormBinary;;
		magnitude(dx,dy,gradientnorm);//�����ݶȷ���
		//double minvalue,maxvalue;
		//cv::minMaxLoc(gradientnorm,&minvalue,&maxvalue);
	//step-3 �������ݶ������ѡ��
	//step-3-1 ���������ݶȾ�����ж�ֵ��
		//int thresholdvalue = int(minvalue+maxvalue/5);
		int thresholdvalue = 70;
		//����ֱ�Ӷ�ֵ���ķ���
		gradientnorm.convertTo(gradientnorm,CV_32F);
		double value = threshold(gradientnorm,gradientnormBinary,thresholdvalue,255, CV_THRESH_BINARY);
		gradientnormBinary.convertTo(gradientnormBinary,CV_8UC1);
		//���ϲ����ǲ��ǿ���ͨ���ȸ�˹�˲�������ʹ��canny���ӽ��б�Ե��⣿����������������
	//step-3-2 ��ͨ�����ʶ
		Mat contoursMask;
		int contoursNum = connectedComponents(gradientnormBinary,contoursMask,8);
		contoursMask.convertTo(contoursMask,CV_8UC1);
		contoursNum--;
	//step-3-3 ��������
		vector<vector<cv::Point2d>> conectAreasPos,conectAreasGrad;
		conectAreasGrad.resize(contoursNum);
		conectAreasPos.resize(contoursNum);
		for(int i=0;i<contoursMask.rows;i++)
		{
			for(int j=0;j<contoursMask.cols;j++)
			{			
				if(contoursMask.at<uchar>(i,j)!=0)
				{
					int tempnum = contoursMask.at<uchar>(i,j)-1;
					conectAreasPos[tempnum].push_back(cv::Point2d(double(i),double(j)));
					conectAreasGrad[tempnum].push_back(cv::Point2d(dx.at<double>(i,j),dy.at<double>(i,j)));
				}
			}
		}
	//step-4 ���ö�ż��Բ����

		vector<Mat> dCVec,precisionVec;
		vector<double> angleIncertitudeVec;
		if(!multi)
		{
			for(int i =0;i<contoursNum;i++)
			{
				Mat dC,precision;
				double AngleIncertitude;
				if(conectAreasPos[i].size()<10)
					continue;
				if(!DualConicFitting(conectAreasGrad[i],conectAreasPos[i],dC,precision,AngleIncertitude))
					continue;
				//double num1 = precision.at<double>(0,0);
				if(precision.at<double>(0,0)==-1||precision.at<double>(0,0)>precisionlevel)
					continue;
				dCVec.push_back(dC);
				precisionVec.push_back(precision);
				angleIncertitudeVec.push_back(AngleIncertitude);
			}
		}
		else
		{
			if(!MultiEllipseFitting(conectAreasGrad,conectAreasPos,dCVec,precisionVec,angleIncertitudeVec))
				{
					return false;
				}
		}
		//step-5 ��Բ�������� Ax^2+Bxy+Cy^2+Dx+Ey+F=0
		vector<Mat> EllipsesparaVec;
		for(unsigned int i =0;i<dCVec.size();i++)
		{
			Mat Ellipsespara = Mat(6,1,CV_64F);
			Mat _C =  dCVec[i].inv();
			_C = _C/_C.at<double>(2,2);
			Ellipsespara.at<double>(0,0) = _C.at<double>(1,1);
			Ellipsespara.at<double>(1,0) = _C.at<double>(0,1)*2;
			Ellipsespara.at<double>(2,0) = _C.at<double>(0,0);
			Ellipsespara.at<double>(3,0) = _C.at<double>(1,2)*2;
			Ellipsespara.at<double>(4,0) = _C.at<double>(2,0)*2;
			Ellipsespara.at<double>(5,0) = _C.at<double>(2,2);
			EllipsesparaVec.push_back(Ellipsespara);
		}
		//step-6 ����Բһ�㷽�������Բ��׼���̲���
		vector<RotatedRect> findResultstemp;
		for(unsigned int i=0;i<EllipsesparaVec.size();i++)
		{
			RotatedRect temppara;
			if(!conicaEllipseTostd(EllipsesparaVec[i],temppara))
				continue;
			findResultstemp.push_back(temppara);
		}
		for(unsigned int i=0;i<findResultstemp.size();i++)
		{
			findResultstemp[i].center.x += MaskRoi.x;
			findResultstemp[i].center.y += MaskRoi.y;
		}
		findResults = findResultstemp;
	return true;
}
bool CoreAlgorithm::DualConicFitting(vector<cv::Point2d>areaGrad,vector<cv::Point2d>areaPos,Mat& dC,Mat& precision,double& angleIncertitude)
{
	precision = Mat::zeros(1,2,CV_64F);
	Mat a,b,c,_M;
	Mat areaGradmat = Mat(areaGrad).reshape(1);
	Mat areaPosmat = Mat(areaPos).reshape(1);
	a = areaGradmat.col(0);
	b = areaGradmat.col(1);
	Mat multitemp = areaGradmat.mul(areaPosmat);
	addWeighted(multitemp.col(0),-1,multitemp.col(1),-1,0,c);

	//Ϊ�˸߾��ȼ�⣬�����ݽ��������Թ�һ��
	Mat M= Mat(a.rows,2,CV_64F);
	Mat tempb = -1*b;
	tempb.copyTo(M.col(0));
	a.copyTo(M.col(1));
	Mat B = -1*c;
	Mat mpts,Lnorm,Lnormt,Minvert;
	if(!solve(M,B,mpts,cv::DECOMP_SVD))
		return false;
	Mat H = Mat::eye(3,3,CV_64F);
	H.at<double>(0,2) = mpts.at<double>(0,0);
	H.at<double>(1,2) = mpts.at<double>(1,0);
	Mat abc=Mat(a.rows,3,CV_64F);
	a.copyTo(abc.col(0));
	b.copyTo(abc.col(1));
	c.copyTo(abc.col(2));
	
	Lnorm = H.t()*abc.t();
	Lnormt = Lnorm.t();
	a = Lnormt.col(0).clone();
	b = Lnormt.col(1).clone();
	c = Lnormt.col(2).clone();
	Mat AA = Mat(5,5,CV_64F);
	Mat BB = Mat(5,1,CV_64F);
	Mat a2 = a.mul(a);
	Mat ab = a.mul(b);
	Mat b2 = b.mul(b);
	Mat ac = a.mul(c);
	Mat bc = b.mul(c);
	Mat c2 = c.mul(c);
	//solution par least-square
	//AA*THITA=BB;
	//��AA
	Mat aaaa = a2.mul(a2);Mat aaab = a2.mul(ab);Mat aabb = a2.mul(b2);Mat aaac = a2.mul(ac);Mat aabc = a2.mul(bc);
	Mat abab = ab.mul(ab);Mat abbb = ab.mul(b2);Mat abac = ab.mul(ac);Mat abbc = ab.mul(bc);
	Mat bbbb = b2.mul(b2);Mat bbac = b2.mul(ac);Mat bbbc = b2.mul(bc);
	Mat acac = ac.mul(ac);Mat acbc = ac.mul(bc);
	Mat bcbc = bc.mul(bc);
	AA.at<double>(0,0)= sum(aaaa).val[0];AA.at<double>(0,1)= sum(aaab).val[0];AA.at<double>(0,2)= sum(aabb).val[0];AA.at<double>(0,3)= sum(aaac).val[0];AA.at<double>(0,4)= sum(aabc).val[0];
	AA.at<double>(1,0)= sum(aaab).val[0];AA.at<double>(1,1)= sum(abab).val[0];AA.at<double>(1,2)= sum(abbb).val[0];AA.at<double>(1,3)= sum(abac).val[0];AA.at<double>(1,4)= sum(abbc).val[0];
	AA.at<double>(2,0)= sum(aabb).val[0];AA.at<double>(2,1)= sum(abbb).val[0];AA.at<double>(2,2)= sum(bbbb).val[0];AA.at<double>(2,3)= sum(bbac).val[0];AA.at<double>(2,4)= sum(bbbc).val[0];
	AA.at<double>(3,0)= sum(aaac).val[0];AA.at<double>(3,1)= sum(abac).val[0];AA.at<double>(3,2)= sum(bbac).val[0];AA.at<double>(3,3)= sum(acac).val[0];AA.at<double>(3,4)= sum(acbc).val[0];
	AA.at<double>(4,0)= sum(aabc).val[0];AA.at<double>(4,1)= sum(abbc).val[0];AA.at<double>(4,2)= sum(bbbc).val[0];AA.at<double>(4,3)= sum(acbc).val[0];AA.at<double>(4,4)= sum(bcbc).val[0];
	//��BB
	Mat _ccaa = -1*(c2.mul(a2));Mat _ccab = -1*(c2.mul(ab));Mat _ccbb = -1*(c2.mul(b2));Mat _ccac = -1*(c2.mul(ac));Mat _ccbc = -1*(c2.mul(bc));
	BB.at<double>(0,0)= sum(_ccaa).val[0];BB.at<double>(1,0)= sum(_ccab).val[0];BB.at<double>(2,0)= sum(_ccbb).val[0];BB.at<double>(3,0)= sum(_ccac).val[0];BB.at<double>(4,0)= sum(_ccbc).val[0];
	if(determinant(AA)<10e-10)
	{
		//�Ƿ�û�б�Ҫ�����湤����ֱ��return false'
		dC = Mat::ones(3,3,CV_64F);
		dC = -1*dC;
		precision.at<double>(0,0) = -1;
		angleIncertitude = -1;
		return false;
	}
	//��A*THITA=BB;
	Mat w,u,vt;
	Mat sol = Mat(5,1,CV_64F);
	if(!solve(AA,BB,sol))
		return false;
	//denormalisation
	Mat dCnorm = Mat(3,3,CV_64F);
	dCnorm.at<double>(0,0) = sol.at<double>(0,0);
	dCnorm.at<double>(0,1) = sol.at<double>(1,0)/2;
	dCnorm.at<double>(0,2) = sol.at<double>(3,0)/2;
	dCnorm.at<double>(1,0) = sol.at<double>(1,0)/2;
	dCnorm.at<double>(1,1) = sol.at<double>(2,0);
	dCnorm.at<double>(1,2) = sol.at<double>(4,0)/2;
	dCnorm.at<double>(2,0) = sol.at<double>(3,0)/2;
	dCnorm.at<double>(2,1) = sol.at<double>(4,0)/2;
	dCnorm.at<double>(2,2) = 1;
	dC = H*dCnorm*H.t();
	//������
	Mat ss = sol.t();
	Mat cccc = c2.mul(c2);
	double BIB = sum(cccc).val[0];
	Mat R = (ss*AA*sol-2*ss*BB+BIB)/(a.rows-5);
	double RmatValue = R.at<double>(0,0);
	Mat cvar2_constantVariance = RmatValue*AA.inv();
	double vD = cvar2_constantVariance.at<double>(3,3);
	double vDE = cvar2_constantVariance.at<double>(3,4);
	double vE = cvar2_constantVariance.at<double>(4,4);
	Mat errorMatrics = Mat(2,2,CV_64F);
	errorMatrics.at<double>(0,0)= cvar2_constantVariance.at<double>(3,3);
	errorMatrics.at<double>(0,1)= cvar2_constantVariance.at<double>(3,4);
	errorMatrics.at<double>(1,0)= cvar2_constantVariance.at<double>(4,3);
	errorMatrics.at<double>(1,1)= cvar2_constantVariance.at<double>(4,4);
	cv::SVD::compute(errorMatrics,w,u,vt);
	Mat diagresult;
	sqrt(w,diagresult);
	diagresult = diagresult/4;
	precision = diagresult.t();
	angleIncertitude = atan2(vt.at<double>(1,1),vt.at<double>(0,1));
	return true;
}
bool CoreAlgorithm::conicaEllipseTostd(const Mat ellipsePara,RotatedRect& result)
{
	double thetarad,cost,sint,sin_squared,cos_squared,cos_sin;
	thetarad = 0.5*atan2(ellipsePara.at<double>(1,0),(ellipsePara.at<double>(0,0) - ellipsePara.at<double>(2,0)));
	cost = cos(thetarad);
	sint = sin(thetarad);
	sin_squared = sint*sint;
	cos_squared = cost*cost;
	cos_sin = sint*cost;
	double Ao,Au,Av,Auu,Avv;
	Ao = ellipsePara.at<double>(5,0);
	Au = ellipsePara.at<double>(3,0)*cost + ellipsePara.at<double>(4,0)*sint;
	Av = -ellipsePara.at<double>(3,0)*sint + ellipsePara.at<double>(4,0)*cost;
	Auu =ellipsePara.at<double>(0,0)*cos_squared + ellipsePara.at<double>(2,0)*sin_squared + ellipsePara.at<double>(1,0)*cos_sin;
	Avv =ellipsePara.at<double>(0,0)*sin_squared + ellipsePara.at<double>(2,0)*cos_squared - ellipsePara.at<double>(1,0)*cos_sin;
	if(Auu==0 || Avv==0)
	{
		 //problem.  this is not a valid ellipse
		 //make sure param are invalid and be easy to spot
		result.center.x = -1;
		result.center.y = -1;
		result.size.height = 0;
		result.size.width = 0;
		result.angle = 0;
		return false;  
	}
	double tuCentre,tvCentre,wCentre,uCentre,vCentre,Ru,Rv;
	// ROTATED = [Ao Au Av Auu Avv]
	tuCentre = - Au/(2*Auu);
	tvCentre = - Av/(2*Avv);
	wCentre = Ao - Auu*tuCentre*tuCentre-Avv*tvCentre*tvCentre;
	uCentre = tuCentre * cost - tvCentre * sint;
	vCentre = tuCentre * sint + tvCentre * cost;

	Ru = -wCentre/Auu;
	Rv = -wCentre/Avv;
	if(Ru<0)
	{
		Ru =-1*sqrt(abs(Ru));
	}
	else
	{
		Ru =sqrt(abs(Ru));
	}
	if(Rv<0)
	{
		Rv =-1*sqrt(abs(Rv));
	}
	else
	{
		Rv =sqrt(abs(Rv));
	}
	result.center.x = (float)uCentre;
	result.center.y = (float)vCentre;
	if(Ru<Rv)
	{
		result.size.height =  (float)Ru;
		result.size.width =  (float)Rv;
		result.angle =  (float)thetarad;
	}
	else
	{
		result.size.height =  (float)Rv;
		result.size.width =  (float)Ru;
		result.angle =  (float)thetarad;
	}  
	return true;
}
bool CoreAlgorithm::MultiEllipseFitting(vector<vector<cv::Point2d>>areaGrads,
                                        vector<vector<cv::Point2d>>areaPoses,
                                        vector<Mat>& dCs,vector<Mat>& precisions,
                                        vector<double> angleIncertitudes)
{
    if (areaGrads.size() != areaPoses.size())//���������
        return false;

    //������ͳһ���������Թ�һ��,����ͳһ��H����
    Mat a,b,c;
    for(uint i = 0; i < areaGrads.size(); i++)
    {
        Mat ctemp;
        Mat areaGradmat = Mat(areaGrads[i]).reshape(1);
        Mat areaPosmat = Mat(areaPoses[i]).reshape(1);
        a.push_back(areaGradmat.col(0));
        b.push_back(areaGradmat.col(1));
        Mat multitemp = areaGradmat.mul(areaPosmat);
        addWeighted(multitemp.col(0),-1,multitemp.col(1),-1,0,ctemp);
        c.push_back(ctemp);
    }
    Mat M= Mat(a.rows,2,CV_64F);
    Mat tempb = -1*b;
    tempb.copyTo(M.col(0));
    a.copyTo(M.col(1));
    Mat B = -1*c;
    Mat mpts,Lnorm,Lnormt,Minvert;
    if(!solve(M,B,mpts,cv::DECOMP_SVD))
        return false;
    Mat H = Mat::eye(3,3,CV_64F);
    H.at<double>(0,2) = mpts.at<double>(0,0);
    H.at<double>(1,2) = mpts.at<double>(1,0);
    //// �������H

    vector<Mat> AAVec,BBVec,aVec,cVec;
    for(uint i = 0; i < areaGrads.size(); i++)
    {
        Mat a,b,c;
        Mat areaGradmat = Mat(areaGrads[i]).reshape(1);
        Mat areaPosmat = Mat(areaPoses[i]).reshape(1);
        a = areaGradmat.col(0);
        b = areaGradmat.col(1);
        Mat multitemp = areaGradmat.mul(areaPosmat);
        addWeighted(multitemp.col(0),-1,multitemp.col(1),-1,0,c);

        //����[a,b,c]
        Mat abc=Mat(a.rows,3,CV_64F);
        a.copyTo(abc.col(0));
        b.copyTo(abc.col(1));
        c.copyTo(abc.col(2));

        //�õ���һ�����a,b��c
        Lnorm = H.t()*abc.t();
        Lnormt = Lnorm.t();
        a = Lnormt.col(0).clone();
        b = Lnormt.col(1).clone();
        c = Lnormt.col(2).clone();
        Mat AA = Mat(5,5,CV_64F);
        Mat BB = Mat(5,1,CV_64F);
        Mat a2 = a.mul(a);
        Mat ab = a.mul(b);
        Mat b2 = b.mul(b);
        Mat ac = a.mul(c);
        Mat bc = b.mul(c);
        Mat c2 = c.mul(c);
        aVec.push_back(a);//�洢a��c,����������
        cVec.push_back(c);

        //solution par least-square
        //��AA
        Mat aaaa = a2.mul(a2);Mat aaab = a2.mul(ab);Mat aabb = a2.mul(b2);Mat aaac = a2.mul(ac);Mat aabc = a2.mul(bc);
        Mat abab = ab.mul(ab);Mat abbb = ab.mul(b2);Mat abac = ab.mul(ac);Mat abbc = ab.mul(bc);
        Mat bbbb = b2.mul(b2);Mat bbac = b2.mul(ac);Mat bbbc = b2.mul(bc);
        Mat acac = ac.mul(ac);Mat acbc = ac.mul(bc);
        Mat bcbc = bc.mul(bc);
        AA.at<double>(0,0)= sum(aaaa).val[0];AA.at<double>(0,1)= sum(aaab).val[0];AA.at<double>(0,2)= sum(aabb).val[0];AA.at<double>(0,3)= sum(aaac).val[0];AA.at<double>(0,4)= sum(aabc).val[0];
        AA.at<double>(1,0)= sum(aaab).val[0];AA.at<double>(1,1)= sum(abab).val[0];AA.at<double>(1,2)= sum(abbb).val[0];AA.at<double>(1,3)= sum(abac).val[0];AA.at<double>(1,4)= sum(abbc).val[0];
        AA.at<double>(2,0)= sum(aabb).val[0];AA.at<double>(2,1)= sum(abbb).val[0];AA.at<double>(2,2)= sum(bbbb).val[0];AA.at<double>(2,3)= sum(bbac).val[0];AA.at<double>(2,4)= sum(bbbc).val[0];
        AA.at<double>(3,0)= sum(aaac).val[0];AA.at<double>(3,1)= sum(abac).val[0];AA.at<double>(3,2)= sum(bbac).val[0];AA.at<double>(3,3)= sum(acac).val[0];AA.at<double>(3,4)= sum(acbc).val[0];
        AA.at<double>(4,0)= sum(aabc).val[0];AA.at<double>(4,1)= sum(abbc).val[0];AA.at<double>(4,2)= sum(bbbc).val[0];AA.at<double>(4,3)= sum(acbc).val[0];AA.at<double>(4,4)= sum(bcbc).val[0];

        //��BB
        Mat _ccaa = -1*(c2.mul(a2));Mat _ccab = -1*(c2.mul(ab));Mat _ccbb = -1*(c2.mul(b2));Mat _ccac = -1*(c2.mul(ac));Mat _ccbc = -1*(c2.mul(bc));
        BB.at<double>(0,0)= sum(_ccaa).val[0];BB.at<double>(1,0)= sum(_ccab).val[0];BB.at<double>(2,0)= sum(_ccbb).val[0];BB.at<double>(3,0)= sum(_ccac).val[0];BB.at<double>(4,0)= sum(_ccbc).val[0];
        if(determinant(AA)<10e-10)
        {
            //�Ƿ�û�б�Ҫ�����湤����ֱ��return false'
//            dC = Mat::ones(3,3,CV_64F);
//            dC = -1*dC;
//            precision.at<double>(0,0) = -1;
//            angleIncertitude = -1;
            return false;
        }
        AAVec.push_back(AA);
        BBVec.push_back(BB);
    }

    //��AAU��BBU
    Mat AAU;// = Mat(5*areaGrads.size(),3*areaGrads.size()+2,CV_64F);
    Mat BBU;// = Mat(5*areaGrads.size(),1,CV_64F);
    for(uint j = 0; j < areaGrads.size(); j++)//5��5�е�ѹ�뵽AAU��BBU��
    {
        Mat AAURow,AAURowT;
        for(uint k = 0; k < 3*areaGrads.size(); k++)//��ѹ��AA��ǰ���е�AAU��
        {
            if (j != k%areaGrads.size())
            {
                Mat O = Mat::zeros(1,5,CV_64F);
                AAURowT.push_back(O);
            }
            else
            {
                Mat tempM = Mat::zeros(5,5,CV_64F);
                AAVec[j].copyTo(tempM);
                tempM = tempM.t();
                AAURowT.push_back((tempM.row(k/areaGrads.size())));
            }
        }
        for(uint k = 0; k < 2; k++)//��ѹ��AA�ĺ����е�AAU��
        {
            Mat tempM = Mat::zeros(5,5,CV_64F);
            AAVec[j].copyTo(tempM);
            tempM = tempM.t();
            AAURowT.push_back((tempM.row(k+3)));
        }
        AAURow = AAURowT.t();//ת�ò�ѹ�뵽AAU��
        AAU.push_back(AAURow);
        BBU.push_back(BBVec[j]);//ֱ�ӽ���Ӧ��BBѹ�뵽BBU��
    }

    //��AAU*THITAU=BBU;
//    cout<<AAU<<endl;
//    cout<<BBU<<endl;
    Mat solu;// = Mat(3*areaGrads.size()+2,1,CV_64F);
    if(!solve(AAU,BBU,solu,cv::DECOMP_SVD))
        return false;

    //�Ȳ��
    vector<Mat> dCnormVec;
    vector<Mat> solVec;
    for(uint i = 0; i < areaGrads.size(); i++)
    {
        Mat dCnorm = Mat::ones(3,3,CV_64F);
        Mat sol = Mat::ones(5,1,CV_64F);
        dCnorm.at<double>(0,2) = solu.at<double>(3*areaGrads.size(),0)/2;
        dCnorm.at<double>(2,0) = solu.at<double>(3*areaGrads.size(),0)/2;
        dCnorm.at<double>(1,2) = solu.at<double>(3*areaGrads.size()+1,0)/2;
        dCnorm.at<double>(2,1) = solu.at<double>(3*areaGrads.size()+1,0)/2;
        sol.at<double>(3,0) = solu.at<double>(3*areaGrads.size(),0);
        sol.at<double>(4,0) = solu.at<double>(3*areaGrads.size()+1,0);
        dCnormVec.push_back(dCnorm);
        solVec.push_back(sol);
    }
    for(uint i = 0; i < 3*areaGrads.size(); i++)
    {

        uint j = 0;//��Ӧ��A,B,C
        if (0 == i%areaGrads.size())
            j++;
        if (j == 1)//A
        {
            dCnormVec[i%areaGrads.size()].at<double>(0,0) = solu.at<double>(i,0);
            solVec[i%areaGrads.size()].at<double>(0,0) = solu.at<double>(i,0);
        }
        else if (j == 2)//B
        {
            dCnormVec[i%areaGrads.size()].at<double>(0,1) = solu.at<double>(i,0)/2;
            dCnormVec[i%areaGrads.size()].at<double>(1,0) = solu.at<double>(i,0)/2;
            solVec[i%areaGrads.size()].at<double>(1,0) = solu.at<double>(i,0);
        }
        else if (j == 3)//C
        {
            dCnormVec[i%areaGrads.size()].at<double>(1,1) = solu.at<double>(i,0);
            solVec[i%areaGrads.size()].at<double>(2,0) = solu.at<double>(i,0);
        }
    }

    //denormalisation
    for(uint i = 0; i < areaGrads.size(); i++)
    {
        Mat dC;
        dC = H*dCnormVec[i]*H.t();
        dCs.push_back(dC);
    }

    //������
    for(uint i = 0; i < areaGrads.size(); i++)
    {
        Mat w,u,vt,precision;
        Mat ss = solVec[i].t();
        Mat c2 = cVec[i].mul(cVec[i]);
        Mat cccc = c2.mul(c2);
        double angleIncertitude;
        double BIB = sum(cccc).val[0];
        Mat R = (ss*AAVec[i]*solVec[i]-2*ss*BBVec[i]+BIB)/(aVec[i].rows-5);
        double RmatValue = R.at<double>(0,0);
        Mat cvar2_constantVariance = RmatValue*AAVec[i].inv();
        double vD = cvar2_constantVariance.at<double>(3,3);
        double vDE = cvar2_constantVariance.at<double>(3,4);
        double vE = cvar2_constantVariance.at<double>(4,4);
        Mat errorMatrics = Mat(2,2,CV_64F);
        errorMatrics.at<double>(0,0)= cvar2_constantVariance.at<double>(3,3);
        errorMatrics.at<double>(0,1)= cvar2_constantVariance.at<double>(3,4);
        errorMatrics.at<double>(1,0)= cvar2_constantVariance.at<double>(4,3);
        errorMatrics.at<double>(1,1)= cvar2_constantVariance.at<double>(4,4);
        cv::SVD::compute(errorMatrics,w,u,vt);
        Mat diagresult;
        sqrt(w,diagresult);
        diagresult = diagresult/4;
        precision = diagresult.t();
        precisions.push_back(precision);
        angleIncertitude = atan2(vt.at<double>(1,1),vt.at<double>(0,1));
        angleIncertitudes.push_back(angleIncertitude);
    }

    return true;

}


bool CoreAlgorithm::ringSubPixAdv(const Mat& img, vector<Point2f>& centers, uint mode)
{
    //�ж�����������,�������ľ�ͻȻ���ڶ�����ľ�֮��ʱ,�����ľ�Ϊ��ĩ���������ʼ��֮��
    int cols = 1;//������ʼ��Ϊ1��
    float sum_dis = 0;
    for (unsigned int i = 0; i < centers.size()-1; i++)
    {
        float _dis = distance(centers[i], centers[i+1]);
        if (i >= 2)
        {
            if (_dis > sum_dis)
                break;//�����ľ�Ϊ��ĩ���������ʼ��֮��
        }
        sum_dis += _dis;
        cols++;
    }
    int rows = centers.size() / cols;

    //��ȡÿ�����ĵ��ROI
    vector<cv::Rect> centers_rect;
    float roiWidth,roiHeight;
    roiWidth = distance(centers[0], centers[1]);//�������ĵ���
    roiHeight = distance(centers[0], centers[cols]);
    for(unsigned int n = 0; n < centers.size(); n++)
    {
        cv::Point2i leftTop, rightBottom;
        leftTop.x = cvRound(centers[n].x - roiWidth/2);
        leftTop.y = cvRound(centers[n].y - roiHeight/2);
        rightBottom.x = cvRound(centers[n].x + roiWidth/2);
        rightBottom.y = cvRound(centers[n].y + roiHeight/2);
        cv::Rect rectROI(leftTop,rightBottom);
        centers_rect.push_back(rectROI);
    }

    for(unsigned int n = 0; n < centers.size(); n++)
    {
        vector<RotatedRect> findResults;
        if (mode == 0)
        {
            findEllipses(img,centers_rect[n],findResults,5,0);//��Բ
            if (findResults.size() != 2)
                return false;
            float areamin = findResults[0].size.area();
            if (findResults[1].size.area()<=areamin)
            {
                centers[n].x = findResults[1].center.x;
                centers[n].y = findResults[1].center.y;
            }
            else
            {
                centers[n].x = findResults[0].center.x;
                centers[n].y = findResults[0].center.y;
            }
        }
        else if(mode == 1)
        {
            findEllipses(img,centers_rect[n],findResults,5,0);//��Բ
            if (findResults.size() != 2)
                return false;
            float areamax = findResults[0].size.area();
            if (findResults[1].size.area()>=areamax)
            {
                centers[n].x = findResults[1].center.x;
                centers[n].y = findResults[1].center.y;
            }
            else
            {
                centers[n].x = findResults[0].center.x;
                centers[n].y = findResults[0].center.y;
            }
        }
        else if(mode == 2)
        {
            findEllipses(img,centers_rect[n],findResults,5,0);//����Բƽ��ֵ
            if (findResults.size() != 2)
                return false;
            centers[n].x = (findResults[0].center.x+findResults[1].center.x)/2;
            centers[n].y = (findResults[0].center.y+findResults[1].center.y)/2;
        }
        else
        {
            findEllipses(img,centers_rect[n],findResults,5,1);//����Բ�Ż�ƽ��ֵ
            if (findResults.size() != 2)
                return false;
            centers[n].x = (findResults[0].center.x+findResults[1].center.x)/2;
            centers[n].y = (findResults[0].center.y+findResults[1].center.y)/2;
        }
    }
    return true;
}
void CoreAlgorithm::pointPosetranslation(const vector<Point3f> PointSrc,vector<Point3f>& PointDst,Mat R,Mat t)
{
	for(unsigned int i = 0;i < PointSrc.size();i++)
		{
			Mat pointtemp1;
			Mat pointtemp4;
			Point3f pointtemp2 = Point3f(0,0,0);
			cv::Point3d pointtemp3 = cv::Point3d(0,0,0);
			pointtemp1.setTo(Scalar(0));
			pointtemp4.setTo(Scalar(0));
			pointtemp3.x = PointSrc[i].x;
			pointtemp3.y = PointSrc[i].y;
			pointtemp3.z = PointSrc[i].z;
			pointtemp1 = Mat(pointtemp3);
			pointtemp4 = R*pointtemp1+t;
			pointtemp2 = Point3f(pointtemp4);
			PointDst.push_back(pointtemp2);
		}
}
void CoreAlgorithm::findTtypeROI(vector<Point2f>& imagePoints,cv::Rect& ROI)
{
	Mat PointMat = Mat(imagePoints).reshape(1);
	Mat PointX,PointY;
	PointMat.col(0).copyTo(PointX);
	PointMat.col(1).copyTo(PointY);
	cv::Point2d minPoint,maxPoint;
	cv::Point2d PointOffset(abs(imagePoints[0].x-imagePoints[1].x),abs(imagePoints[0].y-imagePoints[1].y));
	cv::minMaxLoc(PointX,&minPoint.x,&maxPoint.x);
	cv::minMaxLoc(PointY,&minPoint.y,&maxPoint.y);
	minPoint -=PointOffset;
	maxPoint +=PointOffset;
	Mat minPointMat = Mat(7,1,CV_32FC2,Scalar(minPoint.x,minPoint.y)).reshape(1);
	PointMat -=minPointMat;
	ROI = cv::Rect(cv::Point2i(minPoint),cv::Point2i(maxPoint));
	imagePoints = cv::Mat_<Point2f>(PointMat);
}
float CoreAlgorithm::distancePlanes(const Plane plane1,const Plane plane2)
{
	//�ֽ�ƽ�����ת����Ax+By+Cz+D=0�����ʽ
	Mat inplanepnts = Mat(plane2.orignal-plane1.orignal);
	Mat direction1 = Mat(plane1.normal);
	Mat direction2 = Mat(plane2.normal);
	cv::normalize(direction1,direction1);
	cv::normalize(direction2,direction2);
	double tempnum = abs(direction1.dot(direction2));
	if(tempnum<0.8||tempnum>1.2)
		return -1;
	tempnum = abs(direction1.dot(inplanepnts));
	return (float)tempnum;
}
void CoreAlgorithm::SellipseBiasCorrection(const std::vector<Point3f> xw,const double radius,const  std::vector<Point2f> ptl,
	const CamPara cam_l,RT &poseLeft)
{
	cv::Mat cam_intr_para(3, 3, CV_64FC1);
	cv::Mat distCoeffs(1, 4, CV_64FC1);
	for (int i=0; i < 3; i++)
	{
		for (int j=0; j < 3; j++)
		{
			cam_intr_para.at<double>(i,j) = cam_l.CameraIntrinsic[i][j];
		}
	}
	for (int i = 0; i < 4; i++)
	{
		distCoeffs.at<double>(0,i) = cam_l.DistortionCoeffs[i];
		//distCoeffs.at<double>(0,i) = 0;
	}
	//compute the 3D points
	//estimate the pose of two cameras
	Mat rlo,tlo,rlc,tlc;
	cv::solvePnP(xw,ptl,cam_intr_para,distCoeffs,rlo,tlo);
	//compute the bias of ellipse
	vector<Point2f> xpl,xpl_c;
	//vector<Point2f>  bias_l = calculateEllipseBias(radius,xw,rlo,tlo, cam_intr_para);
	vector<Point2f>  bias_l = calculateEllipseBias(radius,xw,rlo,tlo,cam_l);
	//get the corrected coordinate in the image
	Mat ptlc = vector2Mat(ptl)+vector2Mat(bias_l);
	cv::solvePnP(xw,Mat2vector(ptlc),cam_intr_para,distCoeffs,rlc,tlc);
	double k = 180/3.1415926;
	//compute the error of pose compose to origin
	Mat Hel = Mat::eye(4,4,CV_64F);
	Mat Hel1,Hel2,Rrlo,Rrro,Rrlc,Rrrc;
	Hel.copyTo(Hel1);
	Hel.copyTo(Hel2);
	cv::Rodrigues(rlo,rlo);
	cv::Rodrigues(rlc,rlc);
	rlo.copyTo(Hel1.rowRange(0,3).colRange(0,3));
	tlo.copyTo(Hel1.rowRange(0,3).colRange(3,4));
	rlc.copyTo(Hel2.rowRange(0,3).colRange(0,3));
	tlc.copyTo(Hel2.rowRange(0,3).colRange(3,4));
	Hel = Hel1*Hel2.inv();
	double El = norm(Hel.rowRange(0,3).colRange(3,4))+k*acos((Hel.at<double>(0,0)+Hel.at<double>(1,1)+Hel.at<double>(2,2)-1)/2);
	//loop
	double Eallo = 1e10;
	int itr = 0;
	while(El<Eallo)
	{
		Eallo = El;
		//compute the bias of ellipse
		vector<Point2f>  bias_l =  calculateEllipseBias(radius,xw,rlc,tlc,cam_l);
		//vector<Point2f>  bias_l =  calculateEllipseBias(radius,xw,rlc,tlc,cam_intr_para);
		//compute_ellipse_center(xw,radius,rlc,tlc,cam_l,bias_l,xpl,xpl_c);
		ptlc = vector2Mat(ptl)+vector2Mat(bias_l);
		rlo = rlc;
		tlo = tlc;
		cv::solvePnP(xw,Mat2vector(ptlc),cam_intr_para,distCoeffs,rlc,tlc);
		cv::Rodrigues(rlc,rlc);
		//compute the error of pose compose to origin
		rlo.copyTo(Hel1.rowRange(0,3).colRange(0,3));
		tlo.copyTo(Hel1.rowRange(0,3).colRange(3,4));
		rlc.copyTo(Hel2.rowRange(0,3).colRange(0,3));
		tlc.copyTo(Hel2.rowRange(0,3).colRange(3,4));
		Hel = Hel1*Hel2.inv();
		El = norm(Hel.rowRange(0,3).colRange(3,4))+k*acos((Hel.at<double>(0,0)+Hel.at<double>(1,1)+Hel.at<double>(2,2)-1)/2);
		itr++;
		if(itr>100)
			break;
	}
	cv::Rodrigues(rlo,poseLeft.R);
	poseLeft.T = tlo;
	return;
}
Mat CoreAlgorithm::vector2Mat(const vector<Point2f> pnts)
{
	Mat matrix = Mat::zeros(pnts.size(),2,CV_32F);
	for(size_t i=0;i<pnts.size();i++)
	{
		matrix.at<float>(i,0) = pnts[i].x;
		matrix.at<float>(i,1) = pnts[i].y;
	}
	return matrix;
}
vector<Point2f> CoreAlgorithm::Mat2vector(const Mat pnts)
{
	vector<Point2f> pnts1;
	for(int i=0;i<pnts.rows;i++)
	{
		pnts1.push_back(Point2f(pnts.at<float>(i,0),pnts.at<float>(i,1)));
	}
	return pnts1;
}



bool CoreAlgorithm::findPntsWithTagV(vector<Point2f>& centerPnt,vector<float>& longaxisRadius,vector<TagPoint2f>& tagPnts,float &firstFeaturelength,const double gamma)
{
	///////////////////////////////////////////////
	//�������ݾ��󣬵ó�ÿ�����ڽ������ź͸���
	if(centerPnt.size()<8)
		return false;
	vector<TagPoint2f> TagPnts;
	if(centerPnt.size()!=longaxisRadius.size())
		return false;
	int pntsSize = centerPnt.size();
	Mat distanceMatrix1 = Mat::zeros(pntsSize,pntsSize,CV_64F);
	Mat distanceMatrix = Mat::zeros(pntsSize,pntsSize,CV_64F);
	//step-2 ������Բ����֮��ľ��벢��������Բ����뾶������洢�ھ�����
	for(int i = 0;i<pntsSize;i++)
	{
		for(int j=i+1;j<pntsSize;j++)
		{
			distanceMatrix1.at<double>(i,j) = distancePoints2f(centerPnt[i],centerPnt[j])/(longaxisRadius[i]*gamma);
		}
	}
	cv::add(distanceMatrix1,distanceMatrix1.t(),distanceMatrix);
	vector<pair<int,vector<int>>> twoVec,threeVec,fourVec;
	for(int i=0;i<distanceMatrix.rows;i++)
	{
		pair<int,vector<int>> tempdata;
		tempdata.first=i;
		for(int j=0;j<distanceMatrix.cols;j++)
		{
			if (distanceMatrix.at<double>(i, j)>0.7&&distanceMatrix.at<double>(i, j)<1
				&& longaxisRadius[i] / longaxisRadius[j]>0.7&&longaxisRadius[i] / longaxisRadius[j] < 1.2)
				tempdata.second.push_back(j);
		}
		switch (tempdata.second.size())
		{
		case 2:
			twoVec.push_back(tempdata);
			continue;
		case 3:
			threeVec.push_back(tempdata);
			continue;
		case 4:
			fourVec.push_back(tempdata);
			continue;
		default:
			continue;
		}
	}
	//�����ڽ�������ֲ���������з��ࡣ
	//1�ŵ�
	if(fourVec.size()!=1)
		return false;
	TagPoint2f firstPoint;
	firstPoint[0] = TAG1;
	firstPoint[1] = centerPnt[fourVec[0].first].x;
	firstPoint[2] = centerPnt[fourVec[0].first].y;
	TagPnts.push_back(firstPoint);
	firstFeaturelength = longaxisRadius[fourVec[0].first];
	//4,5,8�ŵ�
	int eightIndex = 0;
	vector<pair<int,vector<int>>> fourFivePoints;
	for(size_t i=0;i<twoVec.size();i++)
	{
		bool isfind = false;
		for(size_t j=0;j<twoVec[i].second.size();j++)
		{
			if(twoVec[i].second[j]==fourVec[0].first)
				isfind=true;
		}
		if(!isfind)
		{
			TagPoint2f eightPoint;
			eightPoint[0] = TAG8;
			eightPoint[1] = centerPnt[twoVec[i].first].x;
			eightPoint[2] = centerPnt[twoVec[i].first].y;
			TagPnts.push_back(eightPoint);
			eightIndex=twoVec[i].first;
		}
		else
		{
			fourFivePoints.push_back(twoVec[i]);
		}

	}
	//ʶ��4��5��
	if(fourFivePoints.size()!=2)
		return false;
	//����AB����x1,y1,z1��, ����CD����x2,y2,z2��
	//����AB������CD����y1z2-z1y2��z1x2-x1z2��x1y2-y1x2��
	Vec3f norm_X,norm_Y1,norm_Z1;//norm_XΪ����ķ�������������Ϊ1��5��Ϊ������
	TagPoint2f fourPoint,fivePoint;
	norm_X[0] = centerPnt[eightIndex].x-centerPnt[fourVec[0].first].x;
	norm_X[1] = centerPnt[eightIndex].y-centerPnt[fourVec[0].first].y;
	norm_X[2] = 0; 
	norm_Y1[0] = centerPnt[fourFivePoints[0].first].x -centerPnt[fourVec[0].first].x;  
	norm_Y1[1] = centerPnt[fourFivePoints[0].first].y -centerPnt[fourVec[0].first].y; 
	norm_Y1[2] = 0; 
	norm_Z1[0] = norm_X[1] * norm_Y1[2] - norm_X[2] * norm_Y1[1];
	norm_Z1[1] = norm_X[2] * norm_Y1[0] - norm_X[0] * norm_Y1[2];
	norm_Z1[2] = norm_X[0] * norm_Y1[1] - norm_X[1] * norm_Y1[0];
	if(norm_Z1[2]<0)
	{
		fivePoint[0] = TAG5;
		fivePoint[1] = centerPnt[fourFivePoints[0].first].x;
		fivePoint[2] = centerPnt[fourFivePoints[0].first].y;
		TagPnts.push_back(fivePoint);
		fourPoint[0] = TAG4;
		fourPoint[1] = centerPnt[fourFivePoints[1].first].x;
		fourPoint[2] = centerPnt[fourFivePoints[1].first].y;
		TagPnts.push_back(fourPoint);				
	}
	else
	{
		fourPoint[0] = TAG4;
		fourPoint[1] = centerPnt[fourFivePoints[0].first].x;
		fourPoint[2] = centerPnt[fourFivePoints[0].first].y;
		TagPnts.push_back(fourPoint);
		fivePoint[0] = TAG5;
		fivePoint[1] = centerPnt[fourFivePoints[1].first].x;
		fivePoint[2] = centerPnt[fourFivePoints[1].first].y;
		TagPnts.push_back(fivePoint);			
	}
	//2,3,6,7��
	vector<pair<int,vector<int>>> twothreePoints,sixsevenPoints;
	for(size_t i=0;i<threeVec.size();i++)
	{
		bool isfind = false;
		for(size_t j=0;j<threeVec[i].second.size();j++)
		{
			if(threeVec[i].second[j]==fourVec[0].first)
				isfind=true;
		}
		if(!isfind)
		{
			sixsevenPoints.push_back(threeVec[i]);
		}
		else
		{
			twothreePoints.push_back(threeVec[i]);
		}

	}
	if(twothreePoints.size()!=2||sixsevenPoints.size()!=2)
		return false;
	//����AB����x1,y1,z1��, ����CD����x2,y2,z2��
	//����AB������CD����y1z2-z1y2��z1x2-x1z2��x1y2-y1x2��
	Vec3f norm_Y2,norm_Z2,norm_Y3,norm_Z3;//norm_XΪ1,8�ķ�������������Ϊ1��8��Ϊ������
	//2,3��
	TagPoint2f twoPoint,threePoint;
	norm_Y2[0] = centerPnt[twothreePoints[0].first].x -centerPnt[fourVec[0].first].x;  
	norm_Y2[1] = centerPnt[twothreePoints[0].first].y -centerPnt[fourVec[0].first].y; 
	norm_Y2[2] = 0; 
	norm_Z2[0] = norm_X[1] * norm_Y2[2] - norm_X[2] * norm_Y2[1];
	norm_Z2[1] = norm_X[2] * norm_Y2[0] - norm_X[0] * norm_Y2[2];
	norm_Z2[2] = norm_X[0] * norm_Y2[1] - norm_X[1] * norm_Y2[0];
	if(norm_Z2[2]<0)
	{
		threePoint[0] = TAG3;
		threePoint[1] = centerPnt[twothreePoints[0].first].x;
		threePoint[2] = centerPnt[twothreePoints[0].first].y;
		TagPnts.push_back(threePoint);
		twoPoint[0] = TAG2;
		twoPoint[1] = centerPnt[twothreePoints[1].first].x;
		twoPoint[2] = centerPnt[twothreePoints[1].first].y;
		TagPnts.push_back(twoPoint);				
	}
	else
	{
		twoPoint[0] = TAG2;
		twoPoint[1] = centerPnt[twothreePoints[0].first].x;
		twoPoint[2] = centerPnt[twothreePoints[0].first].y;
		TagPnts.push_back(twoPoint);
		threePoint[0] = TAG3;
		threePoint[1] = centerPnt[twothreePoints[1].first].x;
		threePoint[2] = centerPnt[twothreePoints[1].first].y;
		TagPnts.push_back(threePoint);			
	}
	//6.7��
	TagPoint2f sixPoint,sevenPoint;
	norm_Y3[0] = centerPnt[sixsevenPoints[0].first].x -centerPnt[fourVec[0].first].x;  
	norm_Y3[1] = centerPnt[sixsevenPoints[0].first].y -centerPnt[fourVec[0].first].y; 
	norm_Y3[2] = 0; 
	norm_Z3[0] = norm_X[1] * norm_Y3[2] - norm_X[2] * norm_Y3[1];
	norm_Z3[1] = norm_X[2] * norm_Y3[0] - norm_X[0] * norm_Y3[2];
	norm_Z3[2] = norm_X[0] * norm_Y3[1] - norm_X[1] * norm_Y3[0];
	if(norm_Z3[2]<0)
	{
		sevenPoint[0] = TAG7;
		sevenPoint[1] = centerPnt[sixsevenPoints[0].first].x;
		sevenPoint[2] = centerPnt[sixsevenPoints[0].first].y;
		TagPnts.push_back(sevenPoint);
		sixPoint[0] = TAG6;
		sixPoint[1] = centerPnt[sixsevenPoints[1].first].x;
		sixPoint[2] = centerPnt[sixsevenPoints[1].first].y;
		TagPnts.push_back(sixPoint);				
	}
	else
	{
		sixPoint[0] = TAG6;
		sixPoint[1] = centerPnt[sixsevenPoints[0].first].x;
		sixPoint[2] = centerPnt[sixsevenPoints[0].first].y;
		TagPnts.push_back(sixPoint);
		sevenPoint[0] = TAG7;
		sevenPoint[1] = centerPnt[sixsevenPoints[1].first].x;
		sevenPoint[2] = centerPnt[sixsevenPoints[1].first].y;
		TagPnts.push_back(sevenPoint);			
	}
	// step-6�Ա�Ǻõĵ㣬���ձ�ǵ��������
	//��С��������//chengwei added
	if(TagPnts.size()!=8)
	{
		return false;
	}
	for(unsigned int i=0;i<TagPnts.size()-1;i++)
	{
		for (unsigned int j = i+1; j<TagPnts.size(); j++)
		{
			if (TagPnts[i][0] > TagPnts[j][0])
			{
				swap(TagPnts[i], TagPnts[j]);
			}
		}
	}
	tagPnts = TagPnts;
	return true;
}

Mat CoreAlgorithm::DrawEllipse(Mat img, double EllipseCenter_x, double EllipseCenter_y, double EllipseLong_axis, double EllipseShort_axis, double angle)
{

	int thickness = -2;
	int lineType = 8;
	ellipse(img,
		Point(EllipseCenter_x, EllipseCenter_y),
		Size(EllipseLong_axis, EllipseShort_axis),   ////ellipse()�����в����᳤Ӧ���ǳ������һ�룬�˴�����Ӧ�Ĳ������Զ������������뼴����Ϊ�ǳ������᳤��
		angle,
		0,
		360,
		Scalar(255, 255, 255),
		thickness,
		lineType);
	//Mat out;
	//blur(img, out, Size(ksizeWidth, ksizeWidth));
	return img;
}




bool CoreAlgorithm::findPntsWithTagVStrong(vector<Point2f> &centerPnt, vector<float>& longaxisRadius, vector<TagPoint2f>& tagPnts, float &firstFeaturelength, const double gamma, Mat cameraMatrix,double angle)
{
	/////////////////////////////added by QiYong///////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////
	//////����undistortPoints()�����������ͼ�����Ϊ�������ӽǣ����ڼ�⡣
	if (centerPnt.size() != 8)
	{
		return false;
	}
	vector<Point2f>  DistortCenterPnt;
	double k1 = 0;
	double k2 = 0;
	double k3 = 0;
	double k4 = 0;
	vector<double> Distort_Coefficients;
	Distort_Coefficients.push_back(k1);
	Distort_Coefficients.push_back(k2);
	Distort_Coefficients.push_back(k3);
	Distort_Coefficients.push_back(k4);
	Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(angle), -sin(angle), 0, sin(angle), cos(angle));
	Mat Ry = (cv::Mat_<double>(3, 3) << cos(angle), 0, sin(angle), 0, 1, 0, -sin(angle), 0, cos(angle));
	Mat Rz = (cv::Mat_<double>(3, 3) << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1);
	Mat R = Rx*Ry;
	undistortPoints(centerPnt, DistortCenterPnt, cameraMatrix, Distort_Coefficients, R);
	Mat centerPntsThreeCols = Mat::zeros(8, 3, CV_64FC1);
	for (size_t i = 0; i < centerPnt.size(); i++)
	{
		centerPntsThreeCols.at<double>(i, 0) = DistortCenterPnt[i].x;
		centerPntsThreeCols.at<double>(i, 1) = DistortCenterPnt[i].y;
		centerPntsThreeCols.at<double>(i, 2) = 1;
	}
	Mat undistortCenterThreeCols = cameraMatrix*centerPntsThreeCols.t();
	vector<Point2f> undistortCenterPnts;
	for (size_t i = 0; i < centerPnt.size(); i++)
	{
		Point2f Point = Point2f(undistortCenterThreeCols.at<double>(0, i), undistortCenterThreeCols.at<double>(1, i));
		undistortCenterPnts.push_back(Point);
	}
	DistortCenterPnt = undistortCenterPnts;
	///////////////////////////////**********************************************

	///////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////
	//�������ݾ��󣬵ó�ÿ�����ڽ������ź͸���
	if (DistortCenterPnt.size() < 8)
		return false;
	vector<TagPoint2f> TagPnts;
	if (DistortCenterPnt.size() != longaxisRadius.size())
		return false;
	int pntsSize = DistortCenterPnt.size();
	Mat distanceMatrix1 = Mat::zeros(pntsSize, pntsSize, CV_64F);
	Mat distanceMatrix = Mat::zeros(pntsSize, pntsSize, CV_64F);
	//step-2 ������Բ����֮��ľ��벢��������Բ����뾶������洢�ھ�����
	for (int i = 0; i < pntsSize; i++)
	{
		for (int j = i + 1; j < pntsSize; j++)
		{
			distanceMatrix1.at<double>(i, j) = distancePoints2f(DistortCenterPnt[i], DistortCenterPnt[j]) / (longaxisRadius[i] * gamma);
		}
	}
	cv::add(distanceMatrix1, distanceMatrix1.t(), distanceMatrix);

	vector<pair<int, vector<int>>> twoVec, threeVec, fourVec;
	for (int i = 0; i < distanceMatrix.rows; i++)
	{
		pair<int, vector<int>> tempdata;
		tempdata.first = i;
		for (int j = 0; j<distanceMatrix.cols; j++)
		{
			/*if (distanceMatrix.at<double>(i, j)>0.7 && distanceMatrix.at<double>(i, j)<1.15)*/
			if (distanceMatrix.at<double>(i, j)>0.7 && distanceMatrix.at<double>(i, j)<1.3)
				tempdata.second.push_back(j);
		}
		switch (tempdata.second.size())
		{
		case 2:
			twoVec.push_back(tempdata);
			continue;
		case 3:
			threeVec.push_back(tempdata);
			continue;
		case 4:
			fourVec.push_back(tempdata);
			continue;
		default:
			continue;
		}
	}
	//�����ڽ�������ֲ���������з��ࡣ
	//1�ŵ�
	if (fourVec.size() != 1)
		return false;
	TagPoint2f firstPoint;
	firstPoint[0] = TAG1;
	firstPoint[1] = centerPnt[fourVec[0].first].x;
	firstPoint[2] = centerPnt[fourVec[0].first].y;
	TagPnts.push_back(firstPoint);
	firstFeaturelength = longaxisRadius[fourVec[0].first];
	//4,5,8�ŵ�
	int eightIndex = 0;
	vector<pair<int, vector<int>>> fourFivePoints;
	for (size_t i = 0; i < twoVec.size(); i++)
	{
		bool isfind = false;
		for (size_t j = 0; j < twoVec[i].second.size(); j++)
		{
			if (twoVec[i].second[j] == fourVec[0].first)
				isfind = true;
		}
		if (!isfind)
		{
			TagPoint2f eightPoint;
			eightPoint[0] = TAG8;
			eightPoint[1] = centerPnt[twoVec[i].first].x;
			eightPoint[2] = centerPnt[twoVec[i].first].y;
			TagPnts.push_back(eightPoint);
			eightIndex = twoVec[i].first;
		}
		else
		{
			fourFivePoints.push_back(twoVec[i]);
		}

	}
	//ʶ��4��5��
	if (fourFivePoints.size() != 2)
		return false;
	//����AB����x1,y1,z1��, ����CD����x2,y2,z2��
	//����AB������CD����y1z2-z1y2��z1x2-x1z2��x1y2-y1x2��
	Vec3f norm_X, norm_Y1, norm_Z1;//norm_XΪ����ķ�������������Ϊ1��5��Ϊ������
	TagPoint2f fourPoint, fivePoint;
	norm_X[0] = DistortCenterPnt[eightIndex].x - DistortCenterPnt[fourVec[0].first].x;
	norm_X[1] = DistortCenterPnt[eightIndex].y - DistortCenterPnt[fourVec[0].first].y;
	norm_X[2] = 0;
	norm_Y1[0] = DistortCenterPnt[fourFivePoints[0].first].x - DistortCenterPnt[fourVec[0].first].x;
	norm_Y1[1] = DistortCenterPnt[fourFivePoints[0].first].y - DistortCenterPnt[fourVec[0].first].y;
	norm_Y1[2] = 0;
	norm_Z1[0] = norm_X[1] * norm_Y1[2] - norm_X[2] * norm_Y1[1];
	norm_Z1[1] = norm_X[2] * norm_Y1[0] - norm_X[0] * norm_Y1[2];
	norm_Z1[2] = norm_X[0] * norm_Y1[1] - norm_X[1] * norm_Y1[0];
	if (norm_Z1[2] < 0)
	{
		fivePoint[0] = TAG5;
		fivePoint[1] = centerPnt[fourFivePoints[0].first].x;
		fivePoint[2] = centerPnt[fourFivePoints[0].first].y;
		TagPnts.push_back(fivePoint);
		fourPoint[0] = TAG4;
		fourPoint[1] = centerPnt[fourFivePoints[1].first].x;
		fourPoint[2] = centerPnt[fourFivePoints[1].first].y;
		TagPnts.push_back(fourPoint);
	}
	else
	{
		fourPoint[0] = TAG4;
		fourPoint[1] = centerPnt[fourFivePoints[0].first].x;
		fourPoint[2] = centerPnt[fourFivePoints[0].first].y;
		TagPnts.push_back(fourPoint);
		fivePoint[0] = TAG5;
		fivePoint[1] = centerPnt[fourFivePoints[1].first].x;
		fivePoint[2] = centerPnt[fourFivePoints[1].first].y;
		TagPnts.push_back(fivePoint);
	}
	//2,3,6,7��
	vector<pair<int, vector<int>>> twothreePoints, sixsevenPoints;
	for (size_t i = 0; i < threeVec.size(); i++)
	{
		bool isfind = false;
		for (size_t j = 0; j < threeVec[i].second.size(); j++)
		{
			if (threeVec[i].second[j] == fourVec[0].first)
				isfind = true;
		}
		if (!isfind)
		{
			sixsevenPoints.push_back(threeVec[i]);
		}
		else
		{
			twothreePoints.push_back(threeVec[i]);
		}

	}
	if (twothreePoints.size() != 2 || sixsevenPoints.size() != 2)
		return false;
	//����AB����x1,y1,z1��, ����CD����x2,y2,z2��
	//����AB������CD����y1z2-z1y2��z1x2-x1z2��x1y2-y1x2��
	Vec3f norm_Y2, norm_Z2, norm_Y3, norm_Z3;//norm_XΪ1,8�ķ�������������Ϊ1��8��Ϊ������
	//2,3��
	TagPoint2f twoPoint, threePoint;
	norm_Y2[0] = DistortCenterPnt[twothreePoints[0].first].x - DistortCenterPnt[fourVec[0].first].x;
	norm_Y2[1] = DistortCenterPnt[twothreePoints[0].first].y - DistortCenterPnt[fourVec[0].first].y;
	norm_Y2[2] = 0;
	norm_Z2[0] = norm_X[1] * norm_Y2[2] - norm_X[2] * norm_Y2[1];
	norm_Z2[1] = norm_X[2] * norm_Y2[0] - norm_X[0] * norm_Y2[2];
	norm_Z2[2] = norm_X[0] * norm_Y2[1] - norm_X[1] * norm_Y2[0];
	if (norm_Z2[2] < 0)
	{
		threePoint[0] = TAG3;
		threePoint[1] = centerPnt[twothreePoints[0].first].x;
		threePoint[2] = centerPnt[twothreePoints[0].first].y;
		TagPnts.push_back(threePoint);
		twoPoint[0] = TAG2;
		twoPoint[1] = centerPnt[twothreePoints[1].first].x;
		twoPoint[2] = centerPnt[twothreePoints[1].first].y;
		TagPnts.push_back(twoPoint);
	}
	else
	{
		twoPoint[0] = TAG2;
		twoPoint[1] = centerPnt[twothreePoints[0].first].x;
		twoPoint[2] = centerPnt[twothreePoints[0].first].y;
		TagPnts.push_back(twoPoint);
		threePoint[0] = TAG3;
		threePoint[1] = centerPnt[twothreePoints[1].first].x;
		threePoint[2] = centerPnt[twothreePoints[1].first].y;
		TagPnts.push_back(threePoint);
	}
	//6.7��
	TagPoint2f sixPoint, sevenPoint;
	norm_Y3[0] = DistortCenterPnt[sixsevenPoints[0].first].x - DistortCenterPnt[fourVec[0].first].x;
	norm_Y3[1] = DistortCenterPnt[sixsevenPoints[0].first].y - DistortCenterPnt[fourVec[0].first].y;
	norm_Y3[2] = 0;
	norm_Z3[0] = norm_X[1] * norm_Y3[2] - norm_X[2] * norm_Y3[1];
	norm_Z3[1] = norm_X[2] * norm_Y3[0] - norm_X[0] * norm_Y3[2];
	norm_Z3[2] = norm_X[0] * norm_Y3[1] - norm_X[1] * norm_Y3[0];
	if (norm_Z3[2] < 0)
	{
		sevenPoint[0] = TAG7;
		sevenPoint[1] = centerPnt[sixsevenPoints[0].first].x;
		sevenPoint[2] = centerPnt[sixsevenPoints[0].first].y;
		TagPnts.push_back(sevenPoint);
		sixPoint[0] = TAG6;
		sixPoint[1] = centerPnt[sixsevenPoints[1].first].x;
		sixPoint[2] = centerPnt[sixsevenPoints[1].first].y;
		TagPnts.push_back(sixPoint);
	}
	else
	{
		sixPoint[0] = TAG6;
		sixPoint[1] = centerPnt[sixsevenPoints[0].first].x;
		sixPoint[2] = centerPnt[sixsevenPoints[0].first].y;
		TagPnts.push_back(sixPoint);
		sevenPoint[0] = TAG7;
		sevenPoint[1] = centerPnt[sixsevenPoints[1].first].x;
		sevenPoint[2] = centerPnt[sixsevenPoints[1].first].y;
		TagPnts.push_back(sevenPoint);
	}
	// step-6�Ա�Ǻõĵ㣬���ձ�ǵ��������
	//��С��������//chengwei added
	if (TagPnts.size() != 8)
	{
		return false;
	}
	for (unsigned int i = 0; i < TagPnts.size() - 1; i++)
	{
		for (unsigned int j = i + 1; j<TagPnts.size(); j++)
		{
			if (TagPnts[i][0] > TagPnts[j][0])
			{
				swap(TagPnts[i], TagPnts[j]);
			}
		}
	}
	tagPnts = TagPnts;
	return true;
}

bool CoreAlgorithm::Cal3dPoint( const vector<Point2f> pointLeft
                                ,const CamPara& camParaLeft
                                ,const vector<Point2f> pointRight
                                ,const CamPara& camParaRight
                                ,const Mat rotVector
,const Mat traVector
,vector<Point3f>& point3d )
{
   if (pointLeft.size() != pointRight.size())
        return false;

    point3d.clear();
    vector<Point2f> NormPixLeft;
    vector<Point2f> NormPixRight;

    double Rfc[2], Rcc[2], Rkc[5], Ralfpha_c=0;
    double Lfc[2], Lcc[2], Lkc[5], Lalfpha_c=0;
    double L2RRotVector[3];
    double L2RTraVector[3];

    for(int i=0;i<3;i++)
    {
        L2RRotVector[i] = rotVector.at<double>(i,0);
        L2RTraVector[i] = traVector.at<double>(i,0);
    }

    Rfc[0] = camParaRight.CameraIntrinsic[0][0];
    Rfc[1] = camParaRight.CameraIntrinsic[1][1];
    Rcc[0] = camParaRight.CameraIntrinsic[0][2];
    Rcc[1] = camParaRight.CameraIntrinsic[1][2];
    for(int i=0;i<4;i++)
    {
        Rkc[i] = camParaRight.DistortionCoeffs[i];
    }
    Rkc[4] = 0;

    Lfc[0] = camParaLeft.CameraIntrinsic[0][0];
    Lfc[1] = camParaLeft.CameraIntrinsic[1][1];
    Lcc[0] = camParaLeft.CameraIntrinsic[0][2];
    Lcc[1] = camParaLeft.CameraIntrinsic[1][2];
    for(int i=0;i<4;i++)
    {
        Lkc[i] = camParaLeft.DistortionCoeffs[i];
    }
    Lkc[4] = 0;

    //// ���򻯡������ͼ������
    normalizPixel(pointLeft,NormPixLeft,Lfc, Lcc, Lkc, Lalfpha_c);
    //// ���򻯡�ͶӰ��ͼ������
    normalizPixel(pointRight,NormPixRight,Rfc, Rcc, Rkc, Ralfpha_c);

    Mat Kp(3,3,CV_64F);

    //���Mat���͵�R��T
    Mat R(3,3,CV_64F);
    Mat Rvec(3,1,CV_64F);
    for(int i=0;i<3;i++)
    {
        Rvec.at<double>(i,0) = rotVector.at<double>(i,0);
    }
    Rodrigues(Rvec,R);

    Mat T(3,1,CV_64F);
    for(int i=0;i<3;i++)
    {
        T.at<double>(i,0) = traVector.at<double>(i,0);
    }

    //������ά����,ʹ��right�����x�����y����
    for (size_t i=0;i<NormPixLeft.size();i++)
    {
        //����ϵ�����󣬱�����Kp�����أ�
        double temp = NormPixRight[i].x * T.at<double>(2,0) - T.at<double>(0,0);//���ֵ��ʲô���壿

        Kp.at<double>(0,0) = 1; Kp.at<double>(0,1) = 0; Kp.at<double>(0,2) = -NormPixLeft[i].x;
        Kp.at<double>(1,0) = 0; Kp.at<double>(1,1) = 1; Kp.at<double>(1,2) = -NormPixLeft[i].y;

        Kp.at<double>(2,0) = (R.at<double>(0,0) - NormPixRight[i].x * R.at<double>(2,0))/temp;
        Kp.at<double>(2,1) = (R.at<double>(0,1) - NormPixRight[i].x * R.at<double>(2,1))/temp;
        Kp.at<double>(2,2) = (R.at<double>(0,2) - NormPixRight[i].x * R.at<double>(2,2))/temp;

        Rvec.at<double>(2,0) = 1;

        Mat inverA(3,3,CV_64F);
        Mat point(3,1,CV_64F);

        invert(Kp,inverA);//�������

        inverA.col(2).copyTo(point.col(0));
        Point3f pnt;
        if (point.at<double>(2,0)<0)
        {
            pnt.x = -point.at<double>(0,0);
            pnt.y = -point.at<double>(1,0);
            pnt.z = -point.at<double>(2,0);
        }
        else
        {
            pnt.x = point.at<double>(0,0);
            pnt.y = point.at<double>(1,0);
            pnt.z = point.at<double>(2,0);
        }
        point3d.push_back(pnt);
    }

    return true;
}
bool CoreAlgorithm::ellipseBiasCorrection(const std::vector<Point3f> xw,const double radius,const  std::vector<Point2f> ptl,const std::vector<Point2f> ptr,
						   const CamPara cam_l,const CamPara cam_r,const RT rt,vector<Point3f> &pntsl,int &iters)
{
	//compute the 3D points
	vector<Point3f> pnts3dl,pnts3dr;
	bool isok;
	isok = Cal3dPoint(ptl,cam_l,ptr,cam_r,rt.R,rt.T,pnts3dl);
	Mat R_ ;
	Rodrigues(rt.R,R_);
	Mat pnt3dr1 = (vector32Mat(pnts3dl)-cv::repeat(rt.T.t(),pnts3dl.size(),1))*R_.inv();
	pnts3dr = cv::Mat_<cv::Point3d>(pnt3dr1);
	//estimate the pose of two cameras
	Mat rlo,tlo,rro,tro,rlc,rrc,trc,tlc;
	rigidTransform(xw,pnts3dl,rlo,tlo);
	rigidTransform(xw,pnts3dr,rro,tro);
	//compute the bias of ellips
	Mat bias_l,bias_r;
	cv::Mat cam_intr_para_l(3, 3, CV_64F);
	for (int i=0; i < 3; i++)
	{
		for (int j=0; j < 3; j++)
		{
			 cam_intr_para_l.at<double>(i,j) = cam_l.CameraIntrinsic[i][j];
		}
	}
	cv::Mat cam_intr_para_r(3, 3, CV_64F);
	for (int i=0; i < 3; i++)
	{
		for (int j=0; j < 3; j++)
		{
			 cam_intr_para_r.at<double>(i,j) = cam_r.CameraIntrinsic[i][j];
		}
	}
	bias_l = vector2MatD(calculateEllipseBias(radius,xw,rlo,tlo,cam_intr_para_l));
	bias_r = vector2MatD(calculateEllipseBias(radius,xw,rro,tro,cam_intr_para_r));
	//get the corrected coordinate in the image
	Mat ptl2 = vector2MatD(ptl)+bias_l;
	Mat ptr2 = vector2MatD(ptr)+bias_r;
	isok = Cal3dPoint(cv::Mat_<cv::Point2d>(ptl2),cam_l,cv::Mat_<cv::Point2d>(ptr2),cam_r,rt.R,rt.T,pnts3dl);
	Mat pnt3dr11 = (vector32Mat(pnts3dl)-cv::repeat(rt.T.t(),pnts3dl.size(),1))*R_.inv();
	pnts3dr = cv::Mat_<cv::Point3d>(pnt3dr1);
	//estimate the pose of two cameras  real value
	rigidTransform(xw,pnts3dl,rlc,tlc);
	rigidTransform(xw,pnts3dr,rrc,trc);
	double k = 180/3.1415926;
	//compute the error of pose compose to origin
	Mat Hel = Mat::eye(4,4,CV_64F);
	Mat Hel1,Hel2,Rrlo,Rrro,Rrlc,Rrrc;
	Hel.copyTo(Hel1);
	Hel.copyTo(Hel2);
	rlo.copyTo(Hel1.rowRange(0,3).colRange(0,3));
	tlo.copyTo(Hel1.rowRange(0,3).colRange(3,4));
	rlc.copyTo(Hel2.rowRange(0,3).colRange(0,3));
	tlc.copyTo(Hel2.rowRange(0,3).colRange(3,4));
	Hel = Hel1*Hel2.inv();
	Mat Her = Mat::eye(4,4,CV_64F);
	Mat Her1,Her2;
	Her.copyTo(Her1);
	Her.copyTo(Her2);
	rro.copyTo(Her1.rowRange(0,3).colRange(0,3));
	tro.copyTo(Her1.rowRange(0,3).colRange(3,4));
	rrc.copyTo(Her2.rowRange(0,3).colRange(0,3));
	trc.copyTo(Her2.rowRange(0,3).colRange(3,4));
	Her = Her1*Her2.inv();
	double El = norm(Hel.rowRange(0,3).colRange(3,4))+k*acos((Hel.at<double>(0,0)+Hel.at<double>(1,1)+Hel.at<double>(2,2)-1)/2);
	double Er = norm(Her.rowRange(0,3).colRange(3,4))+k*acos((Her.at<double>(0,0)+Her.at<double>(1,1)+Her.at<double>(2,2)-1)/2);
	double Eallt = El+Er;
	//loop
	double Eallo = 20.0;
	int itr = 0;
	while(Eallt<Eallo)
	{
		pntsl = pnts3dl;
		Eallo = Eallt;
		//compute the bias of ellipse
		bias_l = vector2MatD(calculateEllipseBias(radius,xw,rlo,tlo,cam_intr_para_l));
		bias_r = vector2MatD(calculateEllipseBias(radius,xw,rro,tro,cam_intr_para_r));
		ptl2 = vector2MatD(ptl)+bias_l;
		ptr2 = vector2MatD(ptr)+bias_r;
		rlo = rlc;
        tlo = tlc;
        rro = rrc;
        tro = trc;
		isok = Cal3dPoint(cv::Mat_<cv::Point2d>(ptl2),cam_l,cv::Mat_<cv::Point2d>(ptr2),cam_r,rt.R,rt.T,pnts3dl);
		pnt3dr1 = (vector32Mat(pnts3dl)-cv::repeat(rt.T.t(),pnts3dl.size(),1))*R_.inv();
		pnts3dr = cv::Mat_<cv::Point3d>(pnt3dr1);
		//estimate the pose of two cameras  real value
		rigidTransform(xw,pnts3dl,rlc,tlc);
		rigidTransform(xw,pnts3dr,rrc,trc);
		//compute the error of pose compose to origin
		rlo.copyTo(Hel1.rowRange(0,3).colRange(0,3));
		tlo.copyTo(Hel1.rowRange(0,3).colRange(3,4));
		rlc.copyTo(Hel2.rowRange(0,3).colRange(0,3));
		tlc.copyTo(Hel2.rowRange(0,3).colRange(3,4));
		Hel = Hel1*Hel2.inv();
		rro.copyTo(Her1.rowRange(0,3).colRange(0,3));
		tro.copyTo(Her1.rowRange(0,3).colRange(3,4));
		rrc.copyTo(Her2.rowRange(0,3).colRange(0,3));
		trc.copyTo(Her2.rowRange(0,3).colRange(3,4));
		Her = Her1*Her2.inv();
		El = norm(Hel.rowRange(0,3).colRange(3,4))+k*acos((Hel.at<double>(0,0)+Hel.at<double>(1,1)+Hel.at<double>(2,2)-1)/2);
		Er = norm(Her.rowRange(0,3).colRange(3,4))+k*acos((Her.at<double>(0,0)+Her.at<double>(1,1)+Her.at<double>(2,2)-1)/2);
		Eallt = El+Er;
		itr++;
		if(itr>100)
			break;
	}
	iters = itr;
	return isok;
}



bool CoreAlgorithm::getTagPoint3f(const Mat& img1,const CamPara _campara1, const Mat& img2, const CamPara _campara2,
                                  const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw,vector<TagPoint3f>& tagPnts3f)
{
	Mat cameraMatrixLeft = (cv::Mat_<double>(3, 3) << _campara1.CameraIntrinsic[0][0], _campara1.CameraIntrinsic[0][1], _campara1.CameraIntrinsic[0][2]
		, _campara1.CameraIntrinsic[1][0], _campara1.CameraIntrinsic[1][1], _campara1.CameraIntrinsic[1][2]
		, _campara1.CameraIntrinsic[2][0], _campara1.CameraIntrinsic[2][1], _campara1.CameraIntrinsic[2][2]);
	Mat cameraMatrixRight = (cv::Mat_<double>(3, 3) << _campara2.CameraIntrinsic[0][0], _campara2.CameraIntrinsic[0][1], _campara2.CameraIntrinsic[0][2]
		, _campara2.CameraIntrinsic[1][0], _campara2.CameraIntrinsic[1][1], _campara2.CameraIntrinsic[1][2]
		, _campara2.CameraIntrinsic[2][0], _campara2.CameraIntrinsic[2][1], _campara2.CameraIntrinsic[2][2]);
	
    //step-1 ������������Ķ�ά������㼯
	vector<Point3f> pntsw;
	pntsTagConfirm(tagPnts3fw,pntsw);
    vector<TagPoint2f> leftTagsPnts2f;
    vector<TagPoint2f> rightTagsPnts2f;
	/*if (!getTagPoint2f(img1, leftTagsPnts2f, 1))
		return false;
	if (!getTagPoint2f(img2, rightTagsPnts2f, 1))
		return false;*/
	if (!getTagPoint2fStrong(img1, leftTagsPnts2f,cameraMatrixLeft, cameraMatrixRight))
		return false;
	if (!getTagPoint2fStrong(img2, rightTagsPnts2f,cameraMatrixLeft, cameraMatrixRight))
		return false;

    /*if(!(getTagPoint2f(img1,leftTagsPnts2f)&&getTagPoint2f(img2,rightTagsPnts2f)))
        return false;*/
    //step-2 ��������ı�ǩ��Ϣȥ��
    vector<Point2f> leftPnts2f,rightPnts2f;
    vector<int> TagVec;
    pntsTagConfirm(leftTagsPnts2f,rightTagsPnts2f,leftPnts2f,rightPnts2f,TagVec);
    //step-3 ������������������ϵ�µ���ά����Ϣ
    vector<Point3f> pnts3fVec;
    //��һ�ּ�����ά����Ϣ�ķ���

	int iters;
	RT CAMG;
	Mat rotation  = Mat::zeros(3,1,CV_64F);
	Mat translation  = Mat::zeros(3,1,CV_64F);
	for(int i=0;i<3;i++)
	{
		rotation.at<double>(i,0) = _camgrouppara.right2LeftRotVector[i];
		translation.at<double>(i,0) = _camgrouppara.right2LeftTraVector[i];
	}
	CAMG.R = rotation;CAMG.T = translation;
	//if (!CoreAlgorithm::ellipseBiasCorrection(pntsw, 7.5, leftPnts2f, rightPnts2f,
	//	_campara1, _campara2, CAMG, pntsl, iters))
	//{
	//	return false;
	//}
	vector<Point3f> pntsl;
	bool isok;
	isok = Cal3dPoint(leftPnts2f, _campara1, rightPnts2f, _campara2, CAMG.R, CAMG.T, pntsl);
	vector<TagPoint3f> result;
	addPntsTag(pntsl, TagVec, result);
    tagPnts3f = result;
    return true;
}

bool CoreAlgorithm::getTagPoint2f(const Mat& img1, const Mat& img2,
	Point2f &CentroidPnts2f_Left, Point2f &CentroidPnts2f_Right, float &CirDia_LP)
{

	vector<Point2f> CenterPnts2f_Left, CenterPnts2f_Right;
	//STEP-1:���ĵ���ȡ
	Mat ImageGray_Left, ImageGray_Right;
	if (img1.channels() == 1 && img2.channels() == 1)
	{
		ImageGray_Left = img1;
		ImageGray_Right = img2;
	}
	else
	{
		cvtColor(img1, ImageGray_Left, CV_BGR2GRAY);
		cvtColor(img2, ImageGray_Right, CV_BGR2GRAY);
	}
	vector<float> longaxisRadius;
	CoreAlgorithm::detectLightSpot_LED(ImageGray_Left, CenterPnts2f_Left, longaxisRadius);
	vector<float>  longaxisRadius_Right;
	CoreAlgorithm::detectLightSpot_LED(ImageGray_Right, CenterPnts2f_Right, longaxisRadius_Right);
	if (CenterPnts2f_Left.size() != 8||CenterPnts2f_Right.size()!=8)
	{
		return false;
	}
	cv::Scalar meanValue = cv::mean(longaxisRadius);
	CirDia_LP = meanValue.val[0];


	///�Լ�������Բ����������������
	if (!ClpMeasurement::calculateCentroid(CenterPnts2f_Left, CentroidPnts2f_Left))
	{
		return false;
	}
	///�Լ�������Բ����������������
	if (!ClpMeasurement::calculateCentroid(CenterPnts2f_Right, CentroidPnts2f_Right))
	{
		return false;
	}

	return true;
}




 bool CoreAlgorithm::findLightPenCircleCenter(const Mat& img, const CamPara _campara1, const CamPara _campara2,
	 const cv::Rect maskRoi, Point2f &CentroidPnts2f, float &CirDia_LP, vector<TagPoint2f>& tagPnts2f)
 {
	 Mat cameraMatrixLeft = (cv::Mat_<double>(3, 3) << _campara1.CameraIntrinsic[0][0], _campara1.CameraIntrinsic[0][1], _campara1.CameraIntrinsic[0][2]
		 , _campara1.CameraIntrinsic[1][0], _campara1.CameraIntrinsic[1][1], _campara1.CameraIntrinsic[1][2]
		 , _campara1.CameraIntrinsic[2][0], _campara1.CameraIntrinsic[2][1], _campara1.CameraIntrinsic[2][2]);
	 Mat cameraMatrixRight = (cv::Mat_<double>(3, 3) << _campara2.CameraIntrinsic[0][0], _campara2.CameraIntrinsic[0][1], _campara2.CameraIntrinsic[0][2]
		 , _campara2.CameraIntrinsic[1][0], _campara2.CameraIntrinsic[1][1], _campara2.CameraIntrinsic[1][2]
		 , _campara2.CameraIntrinsic[2][0], _campara2.CameraIntrinsic[2][1], _campara2.CameraIntrinsic[2][2]);
	
	 if (!CoreAlgorithm::getTagPoint2fStrong(img, maskRoi, tagPnts2f, cameraMatrixLeft, cameraMatrixRight))
	 {
		 return false;
	 }
	 if (!CoreAlgorithm::calculateLPTag2fCentroid(tagPnts2f, CentroidPnts2f))
	 {
		 return false;
	 }



	 return true;
 }










////////���غ���getTagPoint3f()������ROI����
bool CoreAlgorithm::getTagPoint3f(const Mat& img1, const cv::Rect maskLeftLightPen, const CamPara _campara1, const Mat& img2, const cv::Rect maskRightLightPen, const CamPara _campara2,
	const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& tagPnts3f)
{
	Mat cameraMatrixLeft = (cv::Mat_<double>(3, 3) << _campara1.CameraIntrinsic[0][0], _campara1.CameraIntrinsic[0][1], _campara1.CameraIntrinsic[0][2]
		, _campara1.CameraIntrinsic[1][0], _campara1.CameraIntrinsic[1][1], _campara1.CameraIntrinsic[1][2]
		, _campara1.CameraIntrinsic[2][0], _campara1.CameraIntrinsic[2][1], _campara1.CameraIntrinsic[2][2]);
	Mat cameraMatrixRight = (cv::Mat_<double>(3, 3) << _campara2.CameraIntrinsic[0][0], _campara2.CameraIntrinsic[0][1], _campara2.CameraIntrinsic[0][2]
		, _campara2.CameraIntrinsic[1][0], _campara2.CameraIntrinsic[1][1], _campara2.CameraIntrinsic[1][2]
		, _campara2.CameraIntrinsic[2][0], _campara2.CameraIntrinsic[2][1], _campara2.CameraIntrinsic[2][2]);

	//step-1 ������������Ķ�ά������㼯
	vector<Point3f> pntsw;
	pntsTagConfirm(tagPnts3fw, pntsw);
	vector<TagPoint2f> leftTagsPnts2f;
	vector<TagPoint2f> rightTagsPnts2f;
	/*if (!getTagPoint2f(img1, leftTagsPnts2f, 1))
	return false;
	if (!getTagPoint2f(img2, rightTagsPnts2f, 1))
	return false;*/
	if (!getTagPoint2fStrong(img1,maskLeftLightPen,leftTagsPnts2f,cameraMatrixLeft,cameraMatrixRight))
		return false;
	if (!getTagPoint2fStrong(img2,maskRightLightPen,rightTagsPnts2f,cameraMatrixLeft,cameraMatrixRight))
		return false;

	/*if(!(getTagPoint2f(img1,leftTagsPnts2f)&&getTagPoint2f(img2,rightTagsPnts2f)))
	return false;*/
	//step-2 ��������ı�ǩ��Ϣȥ��
	vector<Point2f> leftPnts2f, rightPnts2f;
	vector<int> TagVec;
	pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);
	//step-3 ������������������ϵ�µ���ά����Ϣ
	vector<Point3f> pnts3fVec;
	//��һ�ּ�����ά����Ϣ�ķ���

	int iters;
	RT CAMG;
	Mat rotation = Mat::zeros(3, 1, CV_64F);
	Mat translation = Mat::zeros(3, 1, CV_64F);
	for (int i = 0; i<3; i++)
	{
		rotation.at<double>(i, 0) = _camgrouppara.right2LeftRotVector[i];
		translation.at<double>(i, 0) = _camgrouppara.right2LeftTraVector[i];
	}
	CAMG.R = rotation; CAMG.T = translation;
	//if (!CoreAlgorithm::ellipseBiasCorrection(pntsw, 7.5, leftPnts2f, rightPnts2f,
	//	_campara1, _campara2, CAMG, pntsl, iters))
	//{
	//	return false;
	//}
	vector<Point3f> pntsl;
	bool isok;
	isok = Cal3dPoint(leftPnts2f, _campara1, rightPnts2f, _campara2, CAMG.R, CAMG.T, pntsl);
	vector<TagPoint3f> result;
	addPntsTag(pntsl, TagVec, result);
	tagPnts3f = result;
	return true;
}




////////���غ���getTagPoint3f()������ROI����,�������ͼ���еĹ�����ĵ�
bool CoreAlgorithm::getTagPoint3f(const Mat& img1, const cv::Rect maskLeftLightPen, const CamPara _campara1, const Mat& img2, const cv::Rect maskRightLightPen, const CamPara _campara2,
	const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& tagPnts3f, Point2f &LightPenCentroidPnt_Left, Point2f &LightPenCentroidPnt_Right)
{
	Mat cameraMatrixLeft = (cv::Mat_<double>(3, 3) << _campara1.CameraIntrinsic[0][0], _campara1.CameraIntrinsic[0][1], _campara1.CameraIntrinsic[0][2]
		, _campara1.CameraIntrinsic[1][0], _campara1.CameraIntrinsic[1][1], _campara1.CameraIntrinsic[1][2]
		, _campara1.CameraIntrinsic[2][0], _campara1.CameraIntrinsic[2][1], _campara1.CameraIntrinsic[2][2]);
	Mat cameraMatrixRight = (cv::Mat_<double>(3, 3) << _campara2.CameraIntrinsic[0][0], _campara2.CameraIntrinsic[0][1], _campara2.CameraIntrinsic[0][2]
		, _campara2.CameraIntrinsic[1][0], _campara2.CameraIntrinsic[1][1], _campara2.CameraIntrinsic[1][2]
		, _campara2.CameraIntrinsic[2][0], _campara2.CameraIntrinsic[2][1], _campara2.CameraIntrinsic[2][2]);

	//step-1 ������������Ķ�ά������㼯
	vector<Point3f> pntsw;
	pntsTagConfirm(tagPnts3fw, pntsw);
	vector<TagPoint2f> leftTagsPnts2f;
	vector<TagPoint2f> rightTagsPnts2f;

	if (!getTagPoint2fStrong(img1, maskLeftLightPen, leftTagsPnts2f, cameraMatrixLeft, cameraMatrixRight))
		return false;
	if (!CoreAlgorithm::calculateLPTag2fCentroid(leftTagsPnts2f, LightPenCentroidPnt_Left))
		return false;
	//cout << "LightPenCentroidPnt_Left:" << LightPenCentroidPnt_Left << endl;
	if (!getTagPoint2fStrong(img2, maskRightLightPen, rightTagsPnts2f, cameraMatrixLeft, cameraMatrixRight))
	{
		return false;
	}
		
	if (!CoreAlgorithm::calculateLPTag2fCentroid(rightTagsPnts2f, LightPenCentroidPnt_Right))
		return false;
	//cout << "LightPenCentroidPnt_Right:" << LightPenCentroidPnt_Right << endl;
	
	//step-2 ��������ı�ǩ��Ϣȥ��
	vector<Point2f> leftPnts2f, rightPnts2f;
	vector<int> TagVec;
	pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);
	//step-3 ������������������ϵ�µ���ά����Ϣ
	vector<Point3f> pnts3fVec;
	//��һ�ּ�����ά����Ϣ�ķ���

	int iters;
	RT CAMG;
	Mat rotation = Mat::zeros(3, 1, CV_64F);
	Mat translation = Mat::zeros(3, 1, CV_64F);
	for (int i = 0; i<3; i++)
	{
		rotation.at<double>(i, 0) = _camgrouppara.right2LeftRotVector[i];
		translation.at<double>(i, 0) = _camgrouppara.right2LeftTraVector[i];
	}
	CAMG.R = rotation; CAMG.T = translation;
	//if (!CoreAlgorithm::ellipseBiasCorrection(pntsw, 7.5, leftPnts2f, rightPnts2f,
	//	_campara1, _campara2, CAMG, pntsl, iters))
	//{
	//	return false;
	//}
	vector<Point3f> pntsl;
	bool isok;
	isok = Cal3dPoint(leftPnts2f, _campara1, rightPnts2f, _campara2, CAMG.R, CAMG.T, pntsl);
	vector<TagPoint3f> result;
	addPntsTag(pntsl, TagVec, result);
	tagPnts3f = result;
	return true;
}





Mat CoreAlgorithm::vector2MatD(const vector<Point2f> pnts)
{
	Mat matrix = Mat::zeros(pnts.size(),2,CV_64F);
	for(size_t i=0;i<pnts.size();i++)
	{
		matrix.at<double>(i,0) = pnts[i].x;
		matrix.at<double>(i,1) = pnts[i].y;
	}
	return matrix; 
}
Mat CoreAlgorithm::vector32Mat(const vector<Point3f> pnts)
{
	Mat matrix = Mat::zeros(pnts.size(),3,CV_64F);
	for(size_t i=0;i<pnts.size();i++)
	{
		matrix.at<double>(i,0) = pnts[i].x;
		matrix.at<double>(i,1) = pnts[i].y;
		matrix.at<double>(i,2) = pnts[i].z;
	}
	return matrix;
}
vector<Point2f> CoreAlgorithm::calculateEllipseBias(double radius, vector<Point3f> Ccenter, Mat Rotation, Mat Translation, Mat Camera_Matrix)
{
	Mat _R = Mat::zeros(3,3,CV_64FC1);
	Mat t = Mat::zeros(3,1,CV_64FC1);
	if(!(Rotation.cols==3&&Rotation.rows==3))
	{
		Rodrigues(Rotation,_R);
	}
	else
	{
		_R =Rotation;
	}
	if(Translation.rows!=3)
	{
		t = Translation.t();
	}
	else
	{
		t = Translation;
	}
	vector<cv::Point3d> PointToCam;
    for(unsigned int i = 0;i < Ccenter.size();i++)
    {
        Mat pointtemp2;
        Mat pointtemp3;
        cv::Point3d pointtemp1 = cv::Point3d(0,0,0);
        pointtemp2.setTo(Scalar(0));
        pointtemp3.setTo(Scalar(0));
        pointtemp1.x = Ccenter[i].x;
        pointtemp1.y = Ccenter[i].y;
        pointtemp1.z = Ccenter[i].z;
        pointtemp2 = Mat(pointtemp1);
        pointtemp3 = _R*pointtemp2 + t;
		pointtemp1 = cv::Point3d(pointtemp3);
        PointToCam.push_back(pointtemp1);
    }
	cv::Mat cam_intr_para(3, 3, CV_64F);
	cv::Mat distCoeffs(1, 4, CV_64F);
	cam_intr_para = Camera_Matrix;
	for (int i = 0; i < 4; i++)
	{
		 distCoeffs.at<double>(0,i) = 0;
	}
	//���Բ����ͶӰ��
	vector<Point2f> imgPnts,xp1;
	cv::projectPoints(Ccenter,_R,t,cam_intr_para,distCoeffs,imgPnts);
	for(size_t i=0;i<imgPnts.size();i++)
	{
		Mat rt = Mat::zeros(3,3,CV_64F);
		_R.colRange(0,2).copyTo(rt.colRange(0,2));
		Mat(PointToCam[i]).copyTo(rt.colRange(2,3));
		Mat H = cam_intr_para*rt;
		Mat circle = Mat::eye(3,3,CV_64F);
		circle.at<double>(2,2) = pow(radius,2);
		Mat ellipseM = H.t().inv()*circle*H.inv();
		//���һ�㷽��
		double A = ellipseM.at<double>(0,0);double B = 2*ellipseM.at<double>(0,1);
		double C = ellipseM.at<double>(1,1);double D = 2*ellipseM.at<double>(0,2);
		double E = 2*ellipseM.at<double>(1,2);
		Point2f center;
		center.x = (B*E - 2 * C*D) / (4 * A*C - B*B);
        center.y = (B*D - 2 * A*E) / (4 * A*C - B*B); 
		xp1.push_back(center);
	}
	vector<Point2f> bias;
	for(size_t i=0;i<xp1.size();i++)
	{
		bias.push_back(imgPnts[i]-xp1[i]);
	}
	return bias;
}
vector<Point2f> CoreAlgorithm::calculateEllipseBias(double radius, vector<Point3f> Ccenter, Mat Rotation, Mat Translation,const CamPara campara)
{
	Mat _R = Mat::zeros(3,3,CV_64FC1);
	Mat t = Mat::zeros(3,1,CV_64FC1);
	if(!(Rotation.cols==3&&Rotation.rows==3))
	{
		Rodrigues(Rotation,_R);
	}
	else
	{
		_R =Rotation;
	}
	if(Translation.rows!=3)
	{
		t = Translation.t();
	}
	else
	{
		t = Translation;
	}
	vector<cv::Point3d> PointToCam;
    for(unsigned int i = 0;i < Ccenter.size();i++)
    {
        Mat pointtemp2;
        Mat pointtemp3;
        cv::Point3d pointtemp1 = cv::Point3d(0,0,0);
        pointtemp2.setTo(Scalar(0));
        pointtemp3.setTo(Scalar(0));
        pointtemp1.x = Ccenter[i].x;
        pointtemp1.y = Ccenter[i].y;
        pointtemp1.z = Ccenter[i].z;
        pointtemp2 = Mat(pointtemp1);
        pointtemp3 = _R*pointtemp2 + t;
		pointtemp1 = cv::Point3d(pointtemp3);
        PointToCam.push_back(pointtemp1);
    }
	cv::Mat cam_intr_para(3, 3, CV_64F);
	cv::Mat distCoeffs(1, 4, CV_64F);
	for (int i=0; i < 3; i++)
	{
		for (int j=0; j < 3; j++)
		{
			 cam_intr_para.at<double>(i,j) = campara.CameraIntrinsic[i][j];
		}
	}
	for (int i = 0; i < 4; i++)
	{
		 distCoeffs.at<double>(0,i) = campara.DistortionCoeffs[i];
	}
	//���Բ����ͶӰ��
	vector<Point2f> imgPnts,xp1;
	cv::projectPoints(Ccenter,_R,t,cam_intr_para,distCoeffs,imgPnts);
	for(size_t i=0;i<imgPnts.size();i++)
	{
		Mat rt = Mat::zeros(3,3,CV_64F);
		_R.colRange(0,2).copyTo(rt.colRange(0,2));
		Mat(PointToCam[i]).copyTo(rt.colRange(2,3));
		Mat H = cam_intr_para*rt;
		Mat circle = Mat::eye(3,3,CV_64F);
		circle.at<double>(2,2) = pow(radius,2);
		Mat ellipseM = H.t().inv()*circle*H.inv();
		//���һ�㷽��
		double A = ellipseM.at<double>(0,0);double B = 2*ellipseM.at<double>(0,1);
		double C = ellipseM.at<double>(1,1);double D = 2*ellipseM.at<double>(0,2);
		double E = 2*ellipseM.at<double>(1,2);
		Point2f center;
		center.x = (B*E - 2 * C*D) / (4 * A*C - B*B);
        center.y = (B*D - 2 * A*E) / (4 * A*C - B*B); 
		xp1.push_back(center);
	}
	vector<Point2f> bias;
	for(size_t i=0;i<xp1.size();i++)
	{
		bias.push_back(imgPnts[i]-xp1[i]);
	}
	return bias;
}

/////qiyong added
bool CoreAlgorithm::detectEllipse(const Mat  &img, vector<RotatedRect> &data)
{
	vector<vector<Point>>  contours0;
	vector<cv::Vec4i> hierarchy;
	///////STEP1:ͼ���ֵ������
	Mat thresholdImg;
	threshold(img, thresholdImg, 100, 255, 0);///�˴���ֵ���趨�����С�Ļ������������ӽϴ󣬽�Ӱ�����ս����
	//////STEP2:ͼ�����̬ѧ������ʴ�����ͣ�(�����㣺�ȸ�ʴ������)
	/////STEP2.1:�ȸ�ʴ��
	//int dilateKernelSize = 1;///�������͵��ں˴�С
	//Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2 * dilateKernelSize + 1, 2 * dilateKernelSize + 1), Point(-1, -1));///�˴���Point��-1��-1����ʾêλ�����ģ�ΪĬ��ֵ�����Բ�д��
	//Mat erodeImg;
	//erode(thresholdImg, erodeImg, element);
	////////STEP2.2:�����ͣ�
	//Mat dilateImg;
	//dilate(erodeImg, dilateImg, element);
	//////STEP3:Ѱ����Բ����
	///����ֵ�����ͼ����Ƴ���
	//cv::namedWindow("maskRoi", 2);
	//cv::imshow("maskRoi", thresholdImg);
	//cv::waitKey(100);

	findContours(thresholdImg, contours0, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, Point(0, 0));



	if (!contours0.size())
	{
		return false;
	}
	else
	{
		///////STEP4:����Բ��϶�ά�㼯
		for (int i = 0; i < contours0.size(); i++)
		{
			if (contours0[i].size()>5)

				data.push_back(fitEllipse(contours0[i]));
		}
	}
	return true;
}
double CoreAlgorithm::get_distance(Point2f pointO, Point2f pointA)
{
	double distance;
	distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
	distance = sqrtf(distance);

	return distance;
}

//////ʶ������
//�����־��ʶ��
bool CoreAlgorithm::findCircularMarker(const Mat img, vector<pair<unsigned int, Point2f>> &results_center)
{

	//��ͼ��ת���ɻҶ�ͼ
	Mat imgOriginal;
	if (img.channels() == 1)
	{
		imgOriginal = img;
	}
	else
	{
		cvtColor(img, imgOriginal, CV_BGR2GRAY);
	}
	//namedWindow("Original", 0);
	//imshow("Original", imgOriginal);
	//while (char(waitKey(1)) != ' ') {}

	//������������Բ���
	vector<RotatedRect> ellipses0;
	CoreAlgorithm::detectEllipse(imgOriginal, ellipses0);

	//ellipses0�д���ĳ��ᡢ����ת��Ϊ�볤�ᡢ�����
	for (int i = 0; i < ellipses0.size(); i++)
	{
		ellipses0[i].size.width = ellipses0[i].size.width / 2;
		ellipses0[i].size.height = ellipses0[i].size.height / 2;
	}

	//��һ��ɸѡ��Բ
	vector<RotatedRect> ellipses1;
	for (int i = 0; i < ellipses0.size(); ++i) {
		if (ellipses0[i].size.height / ellipses0[i].size.width > 0 && ellipses0[i].size.height / ellipses0[i].size.width < 1.6 && ellipses0[i].size.height > 4)
		{
			ellipses1.push_back(ellipses0[i]);
		}
	}

	//�ڶ���ɸѡ������Բ�ľ���С�ڴ�Բ�뾶���޳���Բ
	vector<RotatedRect> ellipses2;
	bool WrongCircle = false;
	for (int i = 0; i < ellipses1.size(); ++i) {
		//�ж��Ƿ�������Բ���ڴ�Բ��,���Ҵ�Բ�뾶�ϴ�
		for (int j = 0; j < i; ++j)
		{
			if (get_distance(ellipses1[i].center, ellipses1[j].center) < ellipses1[i].size.height && ellipses1[i].size.height > ellipses1[j].size.height)
			{
				WrongCircle = true;
				break;
			}
		}
		if (!WrongCircle)
		{
			for (int j = i + 1; j < ellipses1.size(); ++j)
			{
				if (get_distance(ellipses1[i].center, ellipses1[j].center) < ellipses1[i].size.height && ellipses1[i].size.height > ellipses1[j].size.height)
				{
					WrongCircle = true;
					break;
				}
			}
		}
		//���ޣ���Ǵ�ԲΪ�����ڶ���ɸѡ��Բ
		if (!WrongCircle)
		{
			ellipses2.push_back(ellipses1[i]);
		}
		WrongCircle = false;
	}

	//������ɸѡ���޳�̫���Բ
	//����ƫ��ƽ��ֵ�ĳ̶��ų��������Բ�����ڵ�һ��ɸѡ��Բʱ���޳��볤��С��4�����ص���Բ���˴�ֻ��̫���Բ�����޳�
	vector<RotatedRect> ellipses3;

	float sum = 0, average;
	for (int i = 0; i < ellipses2.size(); ++i)
	{
		sum = sum + ellipses2[i].size.height;
	}
	average = sum / ellipses2.size();

	for (int i = 0; i < ellipses2.size(); ++i)
	{
		if (ellipses2[i].size.height < (1.6*average))
		{
			ellipses3.push_back(ellipses2[i]);
		}
	}

	//���Ĵ�ɸѡ���޳������ϱ����־����̬������Բ
	Mat img1, img2;
	vector<RotatedRect> ellipses4;
	cv::Rect_<float> region(0, 0, 0, 0);
	Point2f img1_center;
	vector<vector<Point>> contours1;

	for (int j = 0; j < ellipses3.size(); j++)
	{
		//ѡ����Χ����
		region.x = ellipses3[j].center.x - 4 * ellipses3[j].size.height;
		region.y = ellipses3[j].center.y - 4 * ellipses3[j].size.height;
		region.width = 8 * ellipses3[j].size.height;
		region.height = 8 * ellipses3[j].size.height;

		//�ų�̫������Ե��Բ
		if (region.x < 0 || region.y < 0 || (ellipses3[j].center.y + 4 * ellipses3[j].size.height) > imgOriginal.rows || (ellipses3[j].center.x + 4 * ellipses3[j].size.height) > imgOriginal.cols)
		{
			continue;
		}

		img1 = imgOriginal.clone();
		img1 = Mat(img1, region);
		img1_center.x = img1.cols / 2;
		img1_center.y = img1.rows / 2;
		circle(img1, img1_center, 3.8 * ellipses3[j].size.height + 2 * ellipses3[j].size.height, Scalar(0, 0, 0), 4 * ellipses3[j].size.height, 8, 0);
		circle(img1, img1_center, ellipses3[j].size.height / 2, Scalar(0, 0, 0), 2 * ellipses3[j].size.height, 8, 0);

		//��ֵ��
		threshold(img1, img2, 100, 255, CV_THRESH_BINARY);

		//������ͨ��
		findContours(img2, contours1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//�ų���Χ��ͨ�����̫С��Բ
		for (int i = 0; i < contours1.size(); i++) {
			if (contourArea(contours1[i], false) >(0.35 * 3.14 * ellipses3[j].size.height * ellipses3[j].size.height))
			{
				ellipses4.push_back(ellipses3[j]);
				break;
			}
		}
	}

	//�����ݶȵ���Բ���
	cv::Rect_<float> mask(0, 0, 0, 0);
	int kenelsize = 3;
	double precisionlevel = 0.04;
	bool multi = 0;
	vector<RotatedRect>  findResults;
	Point2f center_temp;
	vector<Point2f> vec_center_temp;//��⵽��Բ�ģ����������ݶȵ���Բ��⺯��findEllipses����ȷ���ı����־������Բ��
	vector<float> vec_radius_temp;//�뾶��ȡ�볤��Ϊ�뾶�������������ݶȵ���Բ��⺯��findEllipses����ȷ���ı����־������Բ��
	Mat imgROI = imgOriginal.clone();

	for (int i = 0; i < ellipses4.size(); i++)
	{
		if (ellipses4[i].center.x - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.y - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.x + 2 * ellipses4[i].size.height < imgOriginal.size().width && ellipses4[i].center.y + 2 * ellipses4[i].size.height < imgOriginal.size().height)
		{
			//ȷ��ÿ��������Բ����С����
			mask.x = ellipses4[i].center.x - 2 * ellipses4[i].size.height;
			mask.y = ellipses4[i].center.y - 2 * ellipses4[i].size.height;
			mask.width = 4 * ellipses4[i].size.height;
			mask.height = 4 * ellipses4[i].size.height;

			//����ÿ��������Բ����С����
			rectangle(imgROI, mask, 255, 3);

			//�����ݶȵ���Բ���
			CoreAlgorithm::findEllipses(imgOriginal, mask, findResults, precisionlevel, multi, kenelsize);

			for (int i = 0; i < findResults.size(); ++i)
			{
				if (findResults[i].size.width / findResults[i].size.height > 0 && findResults[i].size.width / findResults[i].size.height < 1.6 && findResults[i].size.width > 4) {
					center_temp.x = findResults[i].center.x;
					center_temp.y = findResults[i].center.y;
					vec_center_temp.push_back(center_temp);
					vec_radius_temp.push_back(findResults[i].size.width);
				}
			}
		}
	}

	////��ʾ���н�����Բ����С����
	//namedWindow("ROI", 0);
	//imshow("ROI", imgROI);
	//while (char(waitKey(1)) != ' ') {}

	//����������ȫ��Բ
	Mat img4;
	cvtColor(imgOriginal, img4, CV_GRAY2BGR);
	for (int i = 0; i < vec_radius_temp.size(); i++) {
		circle(img4, vec_center_temp[i], vec_radius_temp[i], Scalar(0, 0, 255), 2, 8, 0);
	}
	/*namedWindow("Circles", 0);
	imshow("Circles", img4);
	while (char(waitKey(1)) != ' ') {}*/


	//����ÿ����־�ı���
	Mat img5, img6, img7, img8;
	vector<vector<Point>> contours, contours_temp, contours_center;
	vector<Point> contours_center_max;
	double ss;
	vector<double> area_temp, area;
	int Num_Contours;
	vector<cv::Moments> mu;
	vector<Point2f> mc;
	vector<int> SingleNumber;
	vector<float> Angle;
	float Angle_Between;
	float min, max;
	int Num_Between1, Num_Between2;
	string TempCode;
	vector<string> BinaryCode;//�������ȷ���Ķ����Ʊ���
	vector<int> Code;//�������ȷ����ʮ���Ʊ���
	cv::Rect_<float> ROI_marker(0, 0, 0, 0);
	cv::Rect_<float> ROI_circle(0, 0, 0, 0);
	Point2f img5_center, img7_center;
	int maxcontour;
	vector<Point2f> Coordinate;	//�������ȷ���ı����־����������

	for (int i = 0; i < vec_center_temp.size(); i++)//��i�������־
	{
		//�Ի��α�������д���

		//ѡ������־����������
		ROI_marker.x = vec_center_temp[i].x - 4 * vec_radius_temp[i];
		ROI_marker.y = vec_center_temp[i].y - 4 * vec_radius_temp[i];
		ROI_marker.width = 8 * vec_radius_temp[i];
		ROI_marker.height = 8 * vec_radius_temp[i];

		//�ų�̫������Ե��Բ
		if (ROI_marker.x < 0 || ROI_marker.y < 0 || (vec_center_temp[i].x + 4 * vec_radius_temp[i]) > imgOriginal.cols || (vec_center_temp[i].y + 4 * vec_radius_temp[i]) > imgOriginal.rows)
		{
			continue;
		}

		//�ú�ɫ������Բ�ͻ��α������Χ����
		img5 = imgOriginal.clone();
		img5 = Mat(img5, ROI_marker);
		img5_center.x = img5.cols / 2;
		img5_center.y = img5.rows / 2;
		//circle(img5, img5_center, 3.8 * vec_radius_temp[i] + 2 * vec_radius_temp[i], Scalar(0, 0, 0), 4 * vec_radius_temp[i], 8, 0);//���α������Χ���ƺ�����ҪͿ�ڣ�
		circle(img5, img5_center, vec_radius_temp[i] / 2, Scalar(0, 0, 0), 2 * vec_radius_temp[i], 8, 0);//����Բ
		//namedWindow("�鿴���α����", 0);
		//imshow("�鿴���α����", img5);
		//while (char(waitKey(1)) != ' ') {}

		//��ֵ��
		threshold(img5, img6, 100, 255, CV_THRESH_BINARY);


		//������ͨ��
		findContours(img6, contours_temp, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//���α�������
		for (int j = 0; j < contours_temp.size(); j++) {
			area_temp.push_back(contourArea(contours_temp[j], false));
		}

		//�ų�̫С����ͨ��
		for (int j = 0; j < contours_temp.size(); j++)
		{
			if (area_temp[j] >(0.35 * 3.14 * vec_radius_temp[i] * vec_radius_temp[i]))
			{
				area.push_back(area_temp[j]);
				contours.push_back(contours_temp[j]);
			}
		}
		Num_Contours = contours.size();
		contours_temp.clear();
		area_temp.clear();

		//��������Ϊ0�����Ǳ����־��
		if (Num_Contours == 0)
		{
			continue;
		}

		//�����ĸ��������������ȷ��
		if (Num_Contours == 4)
		{
			BinaryCode.push_back("01010101");
			Coordinate.push_back(vec_center_temp[i]);

			//�������ѭ�����������
			contours.clear();
			area.clear();
			//continue;
		}

		//���������������������ȷ��
		else if (Num_Contours == 3)
		{
			BinaryCode.push_back("01010111");
			Coordinate.push_back(vec_center_temp[i]);

			//�������ѭ�����������
			contours.clear();
			area.clear();
			//continue;
		}

		//�������ĸ������������������������������

		//������Բ������ͨ��ķ��������
		//��ȡ��������Բ��С����
		else
		{
			ROI_circle.x = vec_center_temp[i].x - 1.5 * vec_radius_temp[i];
			ROI_circle.y = vec_center_temp[i].y - 1.5 * vec_radius_temp[i];
			ROI_circle.width = 3 * vec_radius_temp[i];
			ROI_circle.height = 3 * vec_radius_temp[i];

			img7 = imgOriginal.clone();
			img7 = Mat(img7, ROI_circle);

			//����Բ����ΧͿ��
			img7_center.x = img7.cols / 2;
			img7_center.y = img7.rows / 2;
			circle(img7, img7_center, 1.5 * vec_radius_temp[i] + 0.75 * vec_radius_temp[i], Scalar(0, 0, 0), 1.5 * vec_radius_temp[i], 8, 0);

			//��ֵ��
			threshold(img7, img8, 100, 255, CV_THRESH_BINARY);

			//������ͨ��
			findContours(img8, contours_center, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

			//��������һ����ͨ��
			maxcontour = 0;
			for (int j = 1; j < contours_center.size(); j++)
			{
				if (contours_center[j].size() > contours_center[maxcontour].size())
				{
					maxcontour = j;
				}
			}
			contours_center_max = contours_center[maxcontour];

			//������Բ���
			ss = contourArea(contours_center_max, false);

			//ȷ��ÿһ����ͨ���Ӧ�ı���

			//��Ʊ����־��ʱ���������α����ֻ��1��3��5��7�����
			for (int j = 0; j < Num_Contours; ++j)
			{
				if (area[j] >(0.388*ss) && area[j] < (1.550*ss))   SingleNumber.push_back(1);
				else if (area[j] > (1.550*ss) && area[j] < (3.100*ss)) SingleNumber.push_back(3);
				else if (area[j] > (3.100*ss) && area[j] < (4.650*ss)) SingleNumber.push_back(5);
				else if (area[j] > (4.650*ss) && area[j] < (6.200*ss)) SingleNumber.push_back(7);
			}

			//��ֻ��һ���������������ȷ��

			if (Num_Contours == 1)
			{
				for (int j = 0; j < 8 - SingleNumber[0]; j++)
				{
					TempCode.push_back('0');
				}
				for (int j = 0; j < SingleNumber[0]; j++)
				{
					TempCode.push_back('1');
				}

				BinaryCode.push_back(TempCode);
				Coordinate.push_back(vec_center_temp[i]);

				//�������ѭ�����������
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				SingleNumber.clear();
				area.clear();
				//continue;
			}

			//������һ�������������������������

			//������������
			else if (Num_Contours == 2)
			{
				//����������
				for (int j = 0; j < Num_Contours; j++)
				{
					mu.push_back(moments(contours[j], false));
				}

				//��������������
				for (int j = 0; j < Num_Contours; j++)
				{
					mc.push_back(Point2f(mu[j].m10 / mu[j].m00, mu[j].m01 / mu[j].m00));
				}

				//������ͨ������������ԲԲ�����ߵĽǶ�
				for (int j = 0; j < Num_Contours; ++j) {
					Angle.push_back(atan2(mc[j].y - img5_center.y, mc[j].x - img5_center.x) / 3.1416 * 180);
				}

				//���Ƕȷ�Χת��Ϊ0-360��
				for (int j = 0; j < Num_Contours; ++j) {
					if (Angle[j] < 0) Angle[j] = Angle[j] + 360;
				}

				//�������λ��α����֮��ļнǣ�ȷ������֮������˼�����λ
				int Num_Between;
				if (Angle[0] < Angle[1])
				{
					Angle_Between = Angle[1] - Angle[0] - (SingleNumber[0] + SingleNumber[1])*22.5;
					if (Angle_Between < 67.5) Num_Between = 1;
					else if (Angle_Between > 67.5 && Angle_Between < 112.5) Num_Between = 2;
					else if (Angle_Between > 112.5 && Angle_Between < 157.5) Num_Between = 3;
					else if (Angle_Between > 157.5 && Angle_Between < 202.5) Num_Between = 4;
					else if (Angle_Between > 202.5 && Angle_Between < 247.5) Num_Between = 5;

					for (int j = 0; j < SingleNumber[0]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < Num_Between; j++)
					{
						TempCode.push_back('0');
					}
					for (int j = 0; j < SingleNumber[1]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < 8 - SingleNumber[0] - Num_Between - SingleNumber[1]; j++)
					{
						TempCode.push_back('0');
					}
				}
				else
				{
					Angle_Between = Angle[0] - Angle[1] - (SingleNumber[0] + SingleNumber[1])*22.5;
					if (Angle_Between < 67.5) Num_Between = 1;
					else if (Angle_Between > 67.5 && Angle_Between < 112.5) Num_Between = 2;
					else if (Angle_Between > 112.5 && Angle_Between < 157.5) Num_Between = 3;
					else if (Angle_Between > 157.5 && Angle_Between < 202.5) Num_Between = 4;
					else if (Angle_Between > 202.5 && Angle_Between < 247.5) Num_Between = 5;

					for (int j = 0; j < SingleNumber[1]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < Num_Between; j++)
					{
						TempCode.push_back('0');
					}
					for (int j = 0; j < SingleNumber[0]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < 8 - SingleNumber[0] - Num_Between - SingleNumber[1]; j++)
					{
						TempCode.push_back('0');
					}
				}

				//ת��ΪΨһ�Ķ����Ʊ���
				if (TempCode == "10011100" || TempCode == "11100100")
				{
					BinaryCode.push_back("00100111");
				}
				else if (TempCode == "11101110")
				{
					BinaryCode.push_back("01110111");
				}
				else if (TempCode == "10001000")
				{
					BinaryCode.push_back("00010001");
				}
				else if (TempCode == "10111110" || TempCode == "11111010")
				{
					BinaryCode.push_back("01011111");
				}

				Coordinate.push_back(vec_center_temp[i]);

				//�������ѭ�����������
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				area.clear();
				mu.clear();
				mc.clear();
				SingleNumber.clear();
				Angle.clear();
				//continue;
			}
		}
	}//��ÿ�������־�����forѭ������


	//�������Ʊ���ת��Ϊʮ���Ʊ���
	for (int j = 0; j < BinaryCode.size(); j++)
	{
		if (BinaryCode[j] == "01111111")
		{
			Code.push_back(127);
		}
		else if (BinaryCode[j] == "00000001")
		{
			Code.push_back(1);
		}
		else if (BinaryCode[j] == "00100111")
		{
			Code.push_back(39);
		}
		else if (BinaryCode[j] == "01011111")
		{
			Code.push_back(95);
		}
		else if (BinaryCode[j] == "01010111")
		{
			Code.push_back(87);
		}
		else if (BinaryCode[j] == "00011111")
		{
			Code.push_back(31);
		}
		else if (BinaryCode[j] == "00000111")
		{
			Code.push_back(7);
		}
		else if (BinaryCode[j] == "01110111")
		{
			Code.push_back(119);
		}
		else if (BinaryCode[j] == "00010001")
		{
			Code.push_back(17);
		}
		else if (BinaryCode[j] == "01010101")
		{
			Code.push_back(85);
		}
	}

	//�������������롢����Բ��Բ������
	/*cout << "\n����\t����Բ��Բ������" << endl;
	for (int j = 0; j < Code.size(); j++)
	{
	cout << Code[j] << "\t" << Coordinate[j] << endl;
	}*/
	//char *fname = "E:/Data/���������.txt";
	//ofstream fout(fname, ios::app);
	//fout << "\n����\t����Բ��Բ������" << endl;
	//for (int j = 0; j < Code.size(); j++)
	//{
	//	fout << Code[j] << "\t" << Coordinate[j] << endl;
	//}
	//fout.close();

	pair<unsigned int, Point2f> marker;

	//������
	for (int j = 0; j < Code.size(); j++)
	{
		marker.first = Code[j];
		marker.second = Coordinate[j];
		results_center.push_back(marker);
	}
	/////��������
	if (results_center.size()>=1)
	{
		for (unsigned int i = 0; i < results_center.size() - 1; i++)
		{
			for (unsigned int j = i + 1; j<results_center.size(); j++)
			{
				if (results_center[i].first > results_center[j].first)
				{
					swap(results_center[i], results_center[j]);
				}
			}
		}

	}
	else
	{
		printf("The area cannot detect codes.\n");
		return false;
	}

	return true;
}

bool CoreAlgorithm::findCircularMarker(const Mat img, vector<pair<unsigned int, Point2f>> &results_center,float &Cir_Diameter)
{

	//��ͼ��ת���ɻҶ�ͼ
	Mat imgOriginal;
	if (img.channels() == 1)
	{
		imgOriginal = img;
	}
	else
	{
		cvtColor(img, imgOriginal, CV_BGR2GRAY);
	}
	//namedWindow("Original", 0);
	//imshow("Original", imgOriginal);
	//while (char(waitKey(1)) != ' ') {}

	//������������Բ���
	vector<RotatedRect> ellipses0;
	CoreAlgorithm::detectEllipse(imgOriginal, ellipses0);

	//ellipses0�д���ĳ��ᡢ����ת��Ϊ�볤�ᡢ�����
	for (int i = 0; i < ellipses0.size(); i++)
	{
		ellipses0[i].size.width = ellipses0[i].size.width / 2;
		ellipses0[i].size.height = ellipses0[i].size.height / 2;
	}

	//��һ��ɸѡ��Բ
	vector<RotatedRect> ellipses1;
	for (int i = 0; i < ellipses0.size(); ++i) {
		if (ellipses0[i].size.height / ellipses0[i].size.width > 0 && ellipses0[i].size.height / ellipses0[i].size.width < 1.6 && ellipses0[i].size.height > 4)
		{
			ellipses1.push_back(ellipses0[i]);
		}
	}

	//�ڶ���ɸѡ������Բ�ľ���С�ڴ�Բ�뾶���޳���Բ
	vector<RotatedRect> ellipses2;
	bool WrongCircle = false;
	for (int i = 0; i < ellipses1.size(); ++i) {
		//�ж��Ƿ�������Բ���ڴ�Բ��,���Ҵ�Բ�뾶�ϴ�
		for (int j = 0; j < i; ++j)
		{
			if (get_distance(ellipses1[i].center, ellipses1[j].center) < ellipses1[i].size.height && ellipses1[i].size.height > ellipses1[j].size.height)
			{
				WrongCircle = true;
				break;
			}
		}
		if (!WrongCircle)
		{
			for (int j = i + 1; j < ellipses1.size(); ++j)
			{
				if (get_distance(ellipses1[i].center, ellipses1[j].center) < ellipses1[i].size.height && ellipses1[i].size.height > ellipses1[j].size.height)
				{
					WrongCircle = true;
					break;
				}
			}
		}
		//���ޣ���Ǵ�ԲΪ�����ڶ���ɸѡ��Բ
		if (!WrongCircle)
		{
			ellipses2.push_back(ellipses1[i]);
		}
		WrongCircle = false;
	}

	//������ɸѡ���޳�̫���Բ
	//����ƫ��ƽ��ֵ�ĳ̶��ų��������Բ�����ڵ�һ��ɸѡ��Բʱ���޳��볤��С��4�����ص���Բ���˴�ֻ��̫���Բ�����޳�
	vector<RotatedRect> ellipses3;

	float sum = 0, average;
	for (int i = 0; i < ellipses2.size(); ++i)
	{
		sum = sum + ellipses2[i].size.height;
	}
	average = sum / ellipses2.size();

	for (int i = 0; i < ellipses2.size(); ++i)
	{
		if (ellipses2[i].size.height < (1.6*average))
		{
			ellipses3.push_back(ellipses2[i]);
		}
	}

	//���Ĵ�ɸѡ���޳������ϱ����־����̬������Բ
	Mat img1, img2;
	vector<RotatedRect> ellipses4;
	cv::Rect_<float> region(0, 0, 0, 0);
	Point2f img1_center;
	vector<vector<Point>> contours1;

	for (int j = 0; j < ellipses3.size(); j++)
	{
		//ѡ����Χ����
		region.x = ellipses3[j].center.x - 4 * ellipses3[j].size.height;
		region.y = ellipses3[j].center.y - 4 * ellipses3[j].size.height;
		region.width = 8 * ellipses3[j].size.height;
		region.height = 8 * ellipses3[j].size.height;

		//�ų�̫������Ե��Բ
		if (region.x < 0 || region.y < 0 || (ellipses3[j].center.y + 4 * ellipses3[j].size.height) > imgOriginal.rows || (ellipses3[j].center.x + 4 * ellipses3[j].size.height) > imgOriginal.cols)
		{
			continue;
		}

		img1 = imgOriginal.clone();
		img1 = Mat(img1, region);
		img1_center.x = img1.cols / 2;
		img1_center.y = img1.rows / 2;
		circle(img1, img1_center, 3.8 * ellipses3[j].size.height + 2 * ellipses3[j].size.height, Scalar(0, 0, 0), 4 * ellipses3[j].size.height, 8, 0);
		circle(img1, img1_center, ellipses3[j].size.height / 2, Scalar(0, 0, 0), 2 * ellipses3[j].size.height, 8, 0);

		//��ֵ��
		threshold(img1, img2, 100, 255, CV_THRESH_BINARY);

		//������ͨ��
		findContours(img2, contours1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//�ų���Χ��ͨ�����̫С��Բ
		for (int i = 0; i < contours1.size(); i++) {
			if (contourArea(contours1[i], false) >(0.35 * 3.14 * ellipses3[j].size.height * ellipses3[j].size.height))
			{
				ellipses4.push_back(ellipses3[j]);
				break;
			}
		}
	}

	//�����ݶȵ���Բ���
	cv::Rect_<float> mask(0, 0, 0, 0);
	int kenelsize = 3;
	double precisionlevel = 0.04;
	bool multi = 0;
	vector<RotatedRect>  findResults;
	Point2f center_temp;
	vector<Point2f> vec_center_temp;//��⵽��Բ�ģ����������ݶȵ���Բ��⺯��findEllipses����ȷ���ı����־������Բ��
	vector<float> vec_radius_temp;//�뾶��ȡ�볤��Ϊ�뾶�������������ݶȵ���Բ��⺯��findEllipses����ȷ���ı����־������Բ��
	Mat imgROI = imgOriginal.clone();

	for (int i = 0; i < ellipses4.size(); i++)
	{
		if (ellipses4[i].center.x - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.y - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.x + 2 * ellipses4[i].size.height < imgOriginal.size().width && ellipses4[i].center.y + 2 * ellipses4[i].size.height < imgOriginal.size().height)
		{
			//ȷ��ÿ��������Բ����С����
			mask.x = ellipses4[i].center.x - 2 * ellipses4[i].size.height;
			mask.y = ellipses4[i].center.y - 2 * ellipses4[i].size.height;
			mask.width = 4 * ellipses4[i].size.height;
			mask.height = 4 * ellipses4[i].size.height;

			//����ÿ��������Բ����С����
			rectangle(imgROI, mask, 255, 3);

			//�����ݶȵ���Բ���
			CoreAlgorithm::findEllipses(imgOriginal, mask, findResults, precisionlevel, multi, kenelsize);
			////qiyong added 
			////��ȡ��������ĵ�Բֱ��
			if (findResults.size()>=1)
			{
				Cir_Diameter = findResults[0].size.height * 2;
			}
		    ////ended

			for (int i = 0; i < findResults.size(); ++i)
			{
				if (findResults[i].size.width / findResults[i].size.height > 0 && findResults[i].size.width / findResults[i].size.height < 1.6 && findResults[i].size.width > 4) {
					center_temp.x = findResults[i].center.x;
					center_temp.y = findResults[i].center.y;
					vec_center_temp.push_back(center_temp);
					vec_radius_temp.push_back(findResults[i].size.width);
				}
			}
		}
	}

	////��ʾ���н�����Բ����С����
	//namedWindow("ROI", 0);
	//imshow("ROI", imgROI);
	//while (char(waitKey(1)) != ' ') {}

	//����������ȫ��Բ
	Mat img4;
	cvtColor(imgOriginal, img4, CV_GRAY2BGR);
	for (int i = 0; i < vec_radius_temp.size(); i++) {
		circle(img4, vec_center_temp[i], vec_radius_temp[i], Scalar(0, 0, 255), 2, 8, 0);
	}
	/*namedWindow("Circles", 0);
	imshow("Circles", img4);
	while (char(waitKey(1)) != ' ') {}*/


	//����ÿ����־�ı���
	Mat img5, img6, img7, img8;
	vector<vector<Point>> contours, contours_temp, contours_center;
	vector<Point> contours_center_max;
	double ss;
	vector<double> area_temp, area;
	int Num_Contours;
	vector<cv::Moments> mu;
	vector<Point2f> mc;
	vector<int> SingleNumber;
	vector<float> Angle;
	float Angle_Between;
	float min, max;
	int Num_Between1, Num_Between2;
	string TempCode;
	vector<string> BinaryCode;//�������ȷ���Ķ����Ʊ���
	vector<int> Code;//�������ȷ����ʮ���Ʊ���
	cv::Rect_<float> ROI_marker(0, 0, 0, 0);
	cv::Rect_<float> ROI_circle(0, 0, 0, 0);
	Point2f img5_center, img7_center;
	int maxcontour;
	vector<Point2f> Coordinate;	//�������ȷ���ı����־����������

	for (int i = 0; i < vec_center_temp.size(); i++)//��i�������־
	{
		//�Ի��α�������д���

		//ѡ������־����������
		ROI_marker.x = vec_center_temp[i].x - 4 * vec_radius_temp[i];
		ROI_marker.y = vec_center_temp[i].y - 4 * vec_radius_temp[i];
		ROI_marker.width = 8 * vec_radius_temp[i];
		ROI_marker.height = 8 * vec_radius_temp[i];

		//�ų�̫������Ե��Բ
		if (ROI_marker.x < 0 || ROI_marker.y < 0 || (vec_center_temp[i].x + 4 * vec_radius_temp[i]) > imgOriginal.cols || (vec_center_temp[i].y + 4 * vec_radius_temp[i]) > imgOriginal.rows)
		{
			continue;
		}

		//�ú�ɫ������Բ�ͻ��α������Χ����
		img5 = imgOriginal.clone();
		img5 = Mat(img5, ROI_marker);
		img5_center.x = img5.cols / 2;
		img5_center.y = img5.rows / 2;
		//circle(img5, img5_center, 3.8 * vec_radius_temp[i] + 2 * vec_radius_temp[i], Scalar(0, 0, 0), 4 * vec_radius_temp[i], 8, 0);//���α������Χ���ƺ�����ҪͿ�ڣ�
		circle(img5, img5_center, vec_radius_temp[i] / 2, Scalar(0, 0, 0), 2 * vec_radius_temp[i], 8, 0);//����Բ
		//namedWindow("�鿴���α����", 0);
		//imshow("�鿴���α����", img5);
		//while (char(waitKey(1)) != ' ') {}

		//��ֵ��
		threshold(img5, img6, 100, 255, CV_THRESH_BINARY);


		//������ͨ��
		findContours(img6, contours_temp, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//���α�������
		for (int j = 0; j < contours_temp.size(); j++) {
			area_temp.push_back(contourArea(contours_temp[j], false));
		}

		//�ų�̫С����ͨ��
		for (int j = 0; j < contours_temp.size(); j++)
		{
			if (area_temp[j] >(0.35 * 3.14 * vec_radius_temp[i] * vec_radius_temp[i]))
			{
				area.push_back(area_temp[j]);
				contours.push_back(contours_temp[j]);
			}
		}
		Num_Contours = contours.size();
		contours_temp.clear();
		area_temp.clear();

		//��������Ϊ0�����Ǳ����־��
		if (Num_Contours == 0)
		{
			continue;
		}

		//�����ĸ��������������ȷ��
		if (Num_Contours == 4)
		{
			BinaryCode.push_back("01010101");
			Coordinate.push_back(vec_center_temp[i]);

			//�������ѭ�����������
			contours.clear();
			area.clear();
			//continue;
		}

		//���������������������ȷ��
		else if (Num_Contours == 3)
		{
			BinaryCode.push_back("01010111");
			Coordinate.push_back(vec_center_temp[i]);

			//�������ѭ�����������
			contours.clear();
			area.clear();
			//continue;
		}

		//�������ĸ������������������������������

		//������Բ������ͨ��ķ��������
		//��ȡ��������Բ��С����
		else
		{
			ROI_circle.x = vec_center_temp[i].x - 1.5 * vec_radius_temp[i];
			ROI_circle.y = vec_center_temp[i].y - 1.5 * vec_radius_temp[i];
			ROI_circle.width = 3 * vec_radius_temp[i];
			ROI_circle.height = 3 * vec_radius_temp[i];

			img7 = imgOriginal.clone();
			img7 = Mat(img7, ROI_circle);

			//����Բ����ΧͿ��
			img7_center.x = img7.cols / 2;
			img7_center.y = img7.rows / 2;
			circle(img7, img7_center, 1.5 * vec_radius_temp[i] + 0.75 * vec_radius_temp[i], Scalar(0, 0, 0), 1.5 * vec_radius_temp[i], 8, 0);

			//��ֵ��
			threshold(img7, img8, 100, 255, CV_THRESH_BINARY);

			//������ͨ��
			findContours(img8, contours_center, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

			//��������һ����ͨ��
			maxcontour = 0;
			for (int j = 1; j < contours_center.size(); j++)
			{
				if (contours_center[j].size() > contours_center[maxcontour].size())
				{
					maxcontour = j;
				}
			}
			contours_center_max = contours_center[maxcontour];

			//������Բ���
			ss = contourArea(contours_center_max, false);

			//ȷ��ÿһ����ͨ���Ӧ�ı���

			//��Ʊ����־��ʱ���������α����ֻ��1��3��5��7�����
			for (int j = 0; j < Num_Contours; ++j)
			{
				if (area[j] >(0.388*ss) && area[j] < (1.550*ss))   SingleNumber.push_back(1);
				else if (area[j] > (1.550*ss) && area[j] < (3.100*ss)) SingleNumber.push_back(3);
				else if (area[j] > (3.100*ss) && area[j] < (4.650*ss)) SingleNumber.push_back(5);
				else if (area[j] > (4.650*ss) && area[j] < (6.200*ss)) SingleNumber.push_back(7);
			}

			//��ֻ��һ���������������ȷ��

			if (Num_Contours == 1)
			{
				for (int j = 0; j < 8 - SingleNumber[0]; j++)
				{
					TempCode.push_back('0');
				}
				for (int j = 0; j < SingleNumber[0]; j++)
				{
					TempCode.push_back('1');
				}

				BinaryCode.push_back(TempCode);
				Coordinate.push_back(vec_center_temp[i]);

				//�������ѭ�����������
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				SingleNumber.clear();
				area.clear();
				//continue;
			}

			//������һ�������������������������

			//������������
			else if (Num_Contours == 2)
			{
				//����������
				for (int j = 0; j < Num_Contours; j++)
				{
					mu.push_back(moments(contours[j], false));
				}

				//��������������
				for (int j = 0; j < Num_Contours; j++)
				{
					mc.push_back(Point2f(mu[j].m10 / mu[j].m00, mu[j].m01 / mu[j].m00));
				}

				//������ͨ������������ԲԲ�����ߵĽǶ�
				for (int j = 0; j < Num_Contours; ++j) {
					Angle.push_back(atan2(mc[j].y - img5_center.y, mc[j].x - img5_center.x) / 3.1416 * 180);
				}

				//���Ƕȷ�Χת��Ϊ0-360��
				for (int j = 0; j < Num_Contours; ++j) {
					if (Angle[j] < 0) Angle[j] = Angle[j] + 360;
				}

				//�������λ��α����֮��ļнǣ�ȷ������֮������˼�����λ
				int Num_Between;
				if (Angle[0] < Angle[1])
				{
					Angle_Between = Angle[1] - Angle[0] - (SingleNumber[0] + SingleNumber[1])*22.5;
					if (Angle_Between < 67.5) Num_Between = 1;
					else if (Angle_Between > 67.5 && Angle_Between < 112.5) Num_Between = 2;
					else if (Angle_Between > 112.5 && Angle_Between < 157.5) Num_Between = 3;
					else if (Angle_Between > 157.5 && Angle_Between < 202.5) Num_Between = 4;
					else if (Angle_Between > 202.5 && Angle_Between < 247.5) Num_Between = 5;

					for (int j = 0; j < SingleNumber[0]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < Num_Between; j++)
					{
						TempCode.push_back('0');
					}
					for (int j = 0; j < SingleNumber[1]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < 8 - SingleNumber[0] - Num_Between - SingleNumber[1]; j++)
					{
						TempCode.push_back('0');
					}
				}
				else
				{
					Angle_Between = Angle[0] - Angle[1] - (SingleNumber[0] + SingleNumber[1])*22.5;
					if (Angle_Between < 67.5) Num_Between = 1;
					else if (Angle_Between > 67.5 && Angle_Between < 112.5) Num_Between = 2;
					else if (Angle_Between > 112.5 && Angle_Between < 157.5) Num_Between = 3;
					else if (Angle_Between > 157.5 && Angle_Between < 202.5) Num_Between = 4;
					else if (Angle_Between > 202.5 && Angle_Between < 247.5) Num_Between = 5;

					for (int j = 0; j < SingleNumber[1]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < Num_Between; j++)
					{
						TempCode.push_back('0');
					}
					for (int j = 0; j < SingleNumber[0]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < 8 - SingleNumber[0] - Num_Between - SingleNumber[1]; j++)
					{
						TempCode.push_back('0');
					}
				}

				//ת��ΪΨһ�Ķ����Ʊ���
				if (TempCode == "10011100" || TempCode == "11100100")
				{
					BinaryCode.push_back("00100111");
				}
				else if (TempCode == "11101110")
				{
					BinaryCode.push_back("01110111");
				}
				else if (TempCode == "10001000")
				{
					BinaryCode.push_back("00010001");
				}
				else if (TempCode == "10111110" || TempCode == "11111010")
				{
					BinaryCode.push_back("01011111");
				}

				Coordinate.push_back(vec_center_temp[i]);

				//�������ѭ�����������
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				area.clear();
				mu.clear();
				mc.clear();
				SingleNumber.clear();
				Angle.clear();
				//continue;
			}
		}
	}//��ÿ�������־�����forѭ������


	//�������Ʊ���ת��Ϊʮ���Ʊ���
	for (int j = 0; j < BinaryCode.size(); j++)
	{
		if (BinaryCode[j] == "01111111")
		{
			Code.push_back(127);
		}
		else if (BinaryCode[j] == "00000001")
		{
			Code.push_back(1);
		}
		else if (BinaryCode[j] == "00100111")
		{
			Code.push_back(39);
		}
		else if (BinaryCode[j] == "01011111")
		{
			Code.push_back(95);
		}
		else if (BinaryCode[j] == "01010111")
		{
			Code.push_back(87);
		}
		else if (BinaryCode[j] == "00011111")
		{
			Code.push_back(31);
		}
		else if (BinaryCode[j] == "00000111")
		{
			Code.push_back(7);
		}
		else if (BinaryCode[j] == "01110111")
		{
			Code.push_back(119);
		}
		else if (BinaryCode[j] == "00010001")
		{
			Code.push_back(17);
		}
		else if (BinaryCode[j] == "01010101")
		{
			Code.push_back(85);
		}
	}

	//�������������롢����Բ��Բ������
	/*cout << "\n����\t����Բ��Բ������" << endl;
	for (int j = 0; j < Code.size(); j++)
	{
	cout << Code[j] << "\t" << Coordinate[j] << endl;
	}*/
	//char *fname = "E:/Data/���������.txt";
	//ofstream fout(fname, ios::app);
	//fout << "\n����\t����Բ��Բ������" << endl;
	//for (int j = 0; j < Code.size(); j++)
	//{
	//	fout << Code[j] << "\t" << Coordinate[j] << endl;
	//}
	//fout.close();

	pair<unsigned int, Point2f> marker;

	//������
	for (int j = 0; j < Code.size(); j++)
	{
		marker.first = Code[j];
		marker.second = Coordinate[j];
		results_center.push_back(marker);
	}
	/////��������
	if (results_center.size() >= 1)
	{
		for (unsigned int i = 0; i < results_center.size() - 1; i++)
		{
			for (unsigned int j = i + 1; j<results_center.size(); j++)
			{
				if (results_center[i].first > results_center[j].first)
				{
					swap(results_center[i], results_center[j]);
				}
			}
		}

	}
	else
	{
		printf("The area cannot detect codes.\n");
		return false;
	}

	return true;
}







////����
bool CoreAlgorithm::findCircularMarker(const Mat img, const cv::Rect Mask, vector<pair<unsigned int, Point2f>> &results_center)
{

	//��ͼ��ת���ɻҶ�ͼ
	Mat imgOriginal;

	if (img.channels() == 1)
	{
		imgOriginal = img;
	}
	else
	{
		cv::cvtColor(img, imgOriginal, CV_BGR2GRAY);
	}
	//namedWindow("Original", 0);
	//imshow("Original", imgOriginal);
	//while (char(waitKey(1)) != ' ') {}

	//������������Բ���
	vector<RotatedRect> ellipses0;
	cv::Rect MaskRoi = Mask;
	if (MaskRoi.x < 0 || MaskRoi.y < 0)
	{
		MaskRoi.x = 0;
		MaskRoi.y = 0;
	}
	if (MaskRoi.x>img.cols || MaskRoi.y>img.rows)
	{
		MaskRoi.x = img.cols - MaskRoi.width;
		MaskRoi.y = img.rows - MaskRoi.height;
	}
	if ((MaskRoi.x + MaskRoi.width) > img.cols || (MaskRoi.y + MaskRoi.height) > img.rows)
	{
		MaskRoi.width = img.cols - MaskRoi.x;
		MaskRoi.height = img.rows - MaskRoi.y;
	}

	imgOriginal = Mat(imgOriginal, MaskRoi);
	///����ROI������ʾ����
	//cv::namedWindow("maskRoi", 1);
	//cv::imshow("maskRoi", imgOriginal);
	//cv::waitKey(10);
	
 	CoreAlgorithm::detectEllipse(imgOriginal, ellipses0); 

	//ellipses0�д���ĳ��ᡢ����ת��Ϊ�볤�ᡢ�����
	for (int i = 0; i < ellipses0.size(); i++)
	{
		ellipses0[i].size.width = ellipses0[i].size.width / 2;
		ellipses0[i].size.height = ellipses0[i].size.height / 2;
	}

	//��һ��ɸѡ��Բ
	vector<RotatedRect> ellipses1;
	for (int i = 0; i < ellipses0.size(); ++i) {
		if (ellipses0[i].size.height / ellipses0[i].size.width > 0 && ellipses0[i].size.height / ellipses0[i].size.width < 1.6 && ellipses0[i].size.height > 4)
		{
			ellipses1.push_back(ellipses0[i]);
		}
	}

	//�ڶ���ɸѡ������Բ�ľ���С�ڴ�Բ�뾶���޳���Բ
	vector<RotatedRect> ellipses2;
	bool WrongCircle = false;
	for (int i = 0; i < ellipses1.size(); ++i) {
		//�ж��Ƿ�������Բ���ڴ�Բ��,���Ҵ�Բ�뾶�ϴ�
		for (int j = 0; j < i; ++j)
		{
			if (get_distance(ellipses1[i].center, ellipses1[j].center) < ellipses1[i].size.height && ellipses1[i].size.height > ellipses1[j].size.height)
			{
				WrongCircle = true;
				break;
			}
		}
		if (!WrongCircle)
		{
			for (int j = i + 1; j < ellipses1.size(); ++j)
			{
				if (get_distance(ellipses1[i].center, ellipses1[j].center) < ellipses1[i].size.height && ellipses1[i].size.height > ellipses1[j].size.height)
				{
					WrongCircle = true;
					break;
				}
			}
		}
		//���ޣ���Ǵ�ԲΪ�����ڶ���ɸѡ��Բ
		if (!WrongCircle)
		{
			ellipses2.push_back(ellipses1[i]);
		}
		WrongCircle = false;
	}

	//������ɸѡ���޳�̫���Բ
	//����ƫ��ƽ��ֵ�ĳ̶��ų��������Բ�����ڵ�һ��ɸѡ��Բʱ���޳��볤��С��4�����ص���Բ���˴�ֻ��̫���Բ�����޳�
	vector<RotatedRect> ellipses3;

	float sum = 0, average;
	for (int i = 0; i < ellipses2.size(); ++i)
	{
		sum = sum + ellipses2[i].size.height;
	}
	average = sum / ellipses2.size();

	for (int i = 0; i < ellipses2.size(); ++i)
	{
		if (ellipses2[i].size.height < (1.6*average))
		{
			ellipses3.push_back(ellipses2[i]);
		}
	}

	//���Ĵ�ɸѡ���޳������ϱ����־����̬������Բ
	Mat img1, img2;
	vector<RotatedRect> ellipses4;
	cv::Rect_<float> region(0, 0, 0, 0);
	Point2f img1_center;
	vector<vector<Point>> contours1;

	for (int j = 0; j < ellipses3.size(); j++)
	{
		//ѡ����Χ����
		region.x = ellipses3[j].center.x - 4 * ellipses3[j].size.height;
		region.y = ellipses3[j].center.y - 4 * ellipses3[j].size.height;
		region.width = 8 * ellipses3[j].size.height;
		region.height = 8 * ellipses3[j].size.height;

		//�ų�̫������Ե��Բ
		if (region.x < 0 || region.y < 0 || (ellipses3[j].center.y + 4 * ellipses3[j].size.height) > imgOriginal.rows || (ellipses3[j].center.x + 4 * ellipses3[j].size.height) > imgOriginal.cols)
		{
			continue;
		}

		img1 = imgOriginal.clone();
		img1 = Mat(img1, region);
		img1_center.x = img1.cols / 2;
		img1_center.y = img1.rows / 2;
		circle(img1, img1_center, 3.8 * ellipses3[j].size.height + 2 * ellipses3[j].size.height, Scalar(0, 0, 0), 4 * ellipses3[j].size.height, 8, 0);
		circle(img1, img1_center, ellipses3[j].size.height / 2, Scalar(0, 0, 0), 2 * ellipses3[j].size.height, 8, 0);

		//��ֵ��
		threshold(img1, img2, 100, 255, CV_THRESH_BINARY);

		//������ͨ��
		findContours(img2, contours1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//�ų���Χ��ͨ�����̫С��Բ
		for (int i = 0; i < contours1.size(); i++) {
			if (contourArea(contours1[i], false) >(0.35 * 3.14 * ellipses3[j].size.height * ellipses3[j].size.height))
			{
				ellipses4.push_back(ellipses3[j]);
				break;
			}
		}
	}

	//�����ݶȵ���Բ���
	cv::Rect_<float> mask(0, 0, 0, 0);
	int kenelsize = 3;
	double precisionlevel = 0.04;
	bool multi = 0;
	vector<RotatedRect>  findResults;
	Point2f center_temp;
	vector<Point2f> vec_center_temp;//��⵽��Բ�ģ����������ݶȵ���Բ��⺯��findEllipses����ȷ���ı����־������Բ��
	vector<float> vec_radius_temp;//�뾶��ȡ�볤��Ϊ�뾶�������������ݶȵ���Բ��⺯��findEllipses����ȷ���ı����־������Բ��
	Mat imgROI = imgOriginal.clone();

	for (int i = 0; i < ellipses4.size(); i++)
	{
		if (ellipses4[i].center.x - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.y - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.x + 2 * ellipses4[i].size.height < imgOriginal.size().width && ellipses4[i].center.y + 2 * ellipses4[i].size.height < imgOriginal.size().height)
		{
			//ȷ��ÿ��������Բ����С����
			mask.x = ellipses4[i].center.x - 2 * ellipses4[i].size.height;
			mask.y = ellipses4[i].center.y - 2 * ellipses4[i].size.height;
			mask.width = 4 * ellipses4[i].size.height;
			mask.height = 4 * ellipses4[i].size.height;

			//����ÿ��������Բ����С����
			rectangle(imgROI, mask, 255, 3);

			//�����ݶȵ���Բ���
			CoreAlgorithm::findEllipses(imgOriginal, mask, findResults, precisionlevel, multi, kenelsize);

			for (int i = 0; i < findResults.size(); ++i)
			{
				if (findResults[i].size.width / findResults[i].size.height > 0 && findResults[i].size.width / findResults[i].size.height < 1.6 && findResults[i].size.width > 4) {
					center_temp.x = findResults[i].center.x;
					center_temp.y = findResults[i].center.y;
					vec_center_temp.push_back(center_temp);
					vec_radius_temp.push_back(findResults[i].size.width);
				}
			}
		}
	}

	////��ʾ���н�����Բ����С����
	//namedWindow("ROI", 0);
	//imshow("ROI", imgROI);
	//while (char(waitKey(1)) != ' ') {}

	//����������ȫ��Բ
	Mat img4;
	cvtColor(imgOriginal, img4, CV_GRAY2BGR);
	for (int i = 0; i < vec_radius_temp.size(); i++) {
		circle(img4, vec_center_temp[i], vec_radius_temp[i], Scalar(0, 0, 255), 2, 8, 0);
	}
	/*namedWindow("Circles", 0);
	imshow("Circles", img4);
	while (char(waitKey(1)) != ' ') {}*/


	//����ÿ����־�ı���
	Mat img5, img6, img7, img8;
	vector<vector<Point>> contours, contours_temp, contours_center;
	vector<Point> contours_center_max;
	double ss;
	vector<double> area_temp, area;
	int Num_Contours;
	vector<cv::Moments> mu;
	vector<Point2f> mc;
	vector<int> SingleNumber;
	vector<float> Angle;
	float Angle_Between;
	float min, max;
	int Num_Between1, Num_Between2;
	string TempCode;
	vector<string> BinaryCode;//�������ȷ���Ķ����Ʊ���
	vector<int> Code;//�������ȷ����ʮ���Ʊ���
	cv::Rect_<float> ROI_marker(0, 0, 0, 0);
	cv::Rect_<float> ROI_circle(0, 0, 0, 0);
	Point2f img5_center, img7_center;
	int maxcontour;
	vector<Point2f> Coordinate;	//�������ȷ���ı����־����������

	for (int i = 0; i < vec_center_temp.size(); i++)//��i�������־
	{
		//�Ի��α�������д���

		//ѡ������־����������
		ROI_marker.x = vec_center_temp[i].x - 4 * vec_radius_temp[i];
		ROI_marker.y = vec_center_temp[i].y - 4 * vec_radius_temp[i];
		ROI_marker.width = 8 * vec_radius_temp[i];
		ROI_marker.height = 8 * vec_radius_temp[i];

		//�ų�̫������Ե��Բ
		if (ROI_marker.x < 0 || ROI_marker.y < 0 || (vec_center_temp[i].x + 4 * vec_radius_temp[i]) > imgOriginal.cols || (vec_center_temp[i].y + 4 * vec_radius_temp[i]) > imgOriginal.rows)
		{
			continue;
		}

		//�ú�ɫ������Բ�ͻ��α������Χ����
		img5 = imgOriginal.clone();
		img5 = Mat(img5, ROI_marker);
		img5_center.x = img5.cols / 2;
		img5_center.y = img5.rows / 2;
		//circle(img5, img5_center, 3.8 * vec_radius_temp[i] + 2 * vec_radius_temp[i], Scalar(0, 0, 0), 4 * vec_radius_temp[i], 8, 0);//���α������Χ���ƺ�����ҪͿ�ڣ�
		circle(img5, img5_center, vec_radius_temp[i] / 2, Scalar(0, 0, 0), 2 * vec_radius_temp[i], 8, 0);//����Բ
		//namedWindow("�鿴���α����", 0);
		//imshow("�鿴���α����", img5);
		//while (char(waitKey(1)) != ' ') {}

		//��ֵ��
		threshold(img5, img6, 100, 255, CV_THRESH_BINARY);


		//������ͨ��
		findContours(img6, contours_temp, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//���α�������
		for (int j = 0; j < contours_temp.size(); j++) {
			area_temp.push_back(contourArea(contours_temp[j], false));
		}

		//�ų�̫С����ͨ��
		for (int j = 0; j < contours_temp.size(); j++)
		{
			if (area_temp[j] >(0.35 * 3.14 * vec_radius_temp[i] * vec_radius_temp[i]))
			{
				area.push_back(area_temp[j]);
				contours.push_back(contours_temp[j]);
			}
		}
		Num_Contours = contours.size();
		contours_temp.clear();
		area_temp.clear();

		//��������Ϊ0�����Ǳ����־��
		if (Num_Contours == 0)
		{
			continue;
		}

		//�����ĸ��������������ȷ��
		if (Num_Contours == 4)
		{
			BinaryCode.push_back("01010101");
			Coordinate.push_back(vec_center_temp[i]);

			//�������ѭ�����������
			contours.clear();
			area.clear();
			//continue;
		}

		//���������������������ȷ��
		else if (Num_Contours == 3)
		{
			BinaryCode.push_back("01010111");
			Coordinate.push_back(vec_center_temp[i]);

			//�������ѭ�����������
			contours.clear();
			area.clear();
			//continue;
		}

		//�������ĸ������������������������������

		//������Բ������ͨ��ķ��������
		//��ȡ��������Բ��С����
		else
		{
			ROI_circle.x = vec_center_temp[i].x - 1.5 * vec_radius_temp[i];
			ROI_circle.y = vec_center_temp[i].y - 1.5 * vec_radius_temp[i];
			ROI_circle.width = 3 * vec_radius_temp[i];
			ROI_circle.height = 3 * vec_radius_temp[i];

			img7 = imgOriginal.clone();
			img7 = Mat(img7, ROI_circle);

			//����Բ����ΧͿ��
			img7_center.x = img7.cols / 2;
			img7_center.y = img7.rows / 2;
			circle(img7, img7_center, 1.5 * vec_radius_temp[i] + 0.75 * vec_radius_temp[i], Scalar(0, 0, 0), 1.5 * vec_radius_temp[i], 8, 0);

			//��ֵ��
			threshold(img7, img8, 100, 255, CV_THRESH_BINARY);

			//������ͨ��
			findContours(img8, contours_center, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

			//��������һ����ͨ��
			maxcontour = 0;
			for (int j = 1; j < contours_center.size(); j++)
			{
				if (contours_center[j].size() > contours_center[maxcontour].size())
				{
					maxcontour = j;
				}
			}
			contours_center_max = contours_center[maxcontour];

			//������Բ���
			ss = contourArea(contours_center_max, false);

			//ȷ��ÿһ����ͨ���Ӧ�ı���

			//��Ʊ����־��ʱ���������α����ֻ��1��3��5��7�����
			for (int j = 0; j < Num_Contours; ++j)
			{
				if (area[j] >(0.388*ss) && area[j] < (1.550*ss))   SingleNumber.push_back(1);
				else if (area[j] > (1.550*ss) && area[j] < (3.100*ss)) SingleNumber.push_back(3);
				else if (area[j] > (3.100*ss) && area[j] < (4.650*ss)) SingleNumber.push_back(5);
				else if (area[j] > (4.650*ss) && area[j] < (6.200*ss)) SingleNumber.push_back(7);
			}

			//��ֻ��һ���������������ȷ��

			if (Num_Contours == 1)
			{
				for (int j = 0; j < 8 - SingleNumber[0]; j++)
				{
					TempCode.push_back('0');
				}
				for (int j = 0; j < SingleNumber[0]; j++)
				{
					TempCode.push_back('1');
				}

				BinaryCode.push_back(TempCode);
				Coordinate.push_back(vec_center_temp[i]);

				//�������ѭ�����������
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				SingleNumber.clear();
				area.clear();
				//continue;
			}

			//������һ�������������������������

			//������������
			else if (Num_Contours == 2)
			{
				//����������
				for (int j = 0; j < Num_Contours; j++)
				{
					mu.push_back(moments(contours[j], false));
				}

				//��������������
				for (int j = 0; j < Num_Contours; j++)
				{
					mc.push_back(Point2f(mu[j].m10 / mu[j].m00, mu[j].m01 / mu[j].m00));
				}

				//������ͨ������������ԲԲ�����ߵĽǶ�
				for (int j = 0; j < Num_Contours; ++j) {
					Angle.push_back(atan2(mc[j].y - img5_center.y, mc[j].x - img5_center.x) / 3.1416 * 180);
				}

				//���Ƕȷ�Χת��Ϊ0-360��
				for (int j = 0; j < Num_Contours; ++j) {
					if (Angle[j] < 0) Angle[j] = Angle[j] + 360;
				}

				//�������λ��α����֮��ļнǣ�ȷ������֮������˼�����λ
				int Num_Between;
				if (Angle[0] < Angle[1])
				{
					Angle_Between = Angle[1] - Angle[0] - (SingleNumber[0] + SingleNumber[1])*22.5;
					if (Angle_Between < 67.5) Num_Between = 1;
					else if (Angle_Between > 67.5 && Angle_Between < 112.5) Num_Between = 2;
					else if (Angle_Between > 112.5 && Angle_Between < 157.5) Num_Between = 3;
					else if (Angle_Between > 157.5 && Angle_Between < 202.5) Num_Between = 4;
					else if (Angle_Between > 202.5 && Angle_Between < 247.5) Num_Between = 5;

					for (int j = 0; j < SingleNumber[0]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < Num_Between; j++)
					{
						TempCode.push_back('0');
					}
					for (int j = 0; j < SingleNumber[1]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < 8 - SingleNumber[0] - Num_Between - SingleNumber[1]; j++)
					{
						TempCode.push_back('0');
					}
				}
				else
				{
					Angle_Between = Angle[0] - Angle[1] - (SingleNumber[0] + SingleNumber[1])*22.5;
					if (Angle_Between < 67.5) Num_Between = 1;
					else if (Angle_Between > 67.5 && Angle_Between < 112.5) Num_Between = 2;
					else if (Angle_Between > 112.5 && Angle_Between < 157.5) Num_Between = 3;
					else if (Angle_Between > 157.5 && Angle_Between < 202.5) Num_Between = 4;
					else if (Angle_Between > 202.5 && Angle_Between < 247.5) Num_Between = 5;

					for (int j = 0; j < SingleNumber[1]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < Num_Between; j++)
					{
						TempCode.push_back('0');
					}
					for (int j = 0; j < SingleNumber[0]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < 8 - SingleNumber[0] - Num_Between - SingleNumber[1]; j++)
					{
						TempCode.push_back('0');
					}
				}

				//ת��ΪΨһ�Ķ����Ʊ���
				if (TempCode == "10011100" || TempCode == "11100100")
				{
					BinaryCode.push_back("00100111");
				}
				else if (TempCode == "11101110")
				{
					BinaryCode.push_back("01110111");
				}
				else if (TempCode == "10001000")
				{
					BinaryCode.push_back("00010001");
				}
				else if (TempCode == "10111110" || TempCode == "11111010")
				{
					BinaryCode.push_back("01011111");
				}

				Coordinate.push_back(vec_center_temp[i]);

				//�������ѭ�����������
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				area.clear();
				mu.clear();
				mc.clear();
				SingleNumber.clear();
				Angle.clear();
				//continue;
			}
		}
	}//��ÿ�������־�����forѭ������


	//�������Ʊ���ת��Ϊʮ���Ʊ���
	for (int j = 0; j < BinaryCode.size(); j++)
	{
		if (BinaryCode[j] == "01111111")
		{
			Code.push_back(127);
		}
		else if (BinaryCode[j] == "00000001")
		{
			Code.push_back(1);
		}
		else if (BinaryCode[j] == "00100111")
		{
			Code.push_back(39);
		}
		else if (BinaryCode[j] == "01011111")
		{
			Code.push_back(95);
		}
		else if (BinaryCode[j] == "01010111")
		{
			Code.push_back(87);
		}
		else if (BinaryCode[j] == "00011111")
		{
			Code.push_back(31);
		}
		else if (BinaryCode[j] == "00000111")
		{
			Code.push_back(7);
		}
		else if (BinaryCode[j] == "01110111")
		{
			Code.push_back(119);
		}
		else if (BinaryCode[j] == "00010001")
		{
			Code.push_back(17);
		}
		else if (BinaryCode[j] == "01010101")
		{
			Code.push_back(85);
		}
	}

	//�������������롢����Բ��Բ������
	/*cout << "\n����\t����Բ��Բ������" << endl;
	for (int j = 0; j < Code.size(); j++)
	{
	cout << Code[j] << "\t" << Coordinate[j] << endl;
	}*/
	//char *fname = "E:/Data/���������.txt";
	//ofstream fout(fname, ios::app);
	//fout << "\n����\t����Բ��Բ������" << endl;
	//for (int j = 0; j < Code.size(); j++)
	//{
	//	fout << Code[j] << "\t" << Coordinate[j] << endl;
	//}
	//fout.close();

	pair<unsigned int, Point2f> marker;

	//������
	for (int j = 0; j < Code.size(); j++)
	{
		marker.first = Code[j];
		marker.second = Coordinate[j];
		marker.second.x += MaskRoi.x;
		marker.second.y += MaskRoi.y;
		results_center.push_back(marker);
	}
	///// �Ա�Ǻõĵ㣬���ձ�ǵ��������
	if (results_center.size()>1)
	{
		for (unsigned int i = 0; i < results_center.size() - 1; i++)
		{
			for (unsigned int j = i + 1; j<results_center.size(); j++)
			{
				if (results_center[i].first > results_center[j].first)
				{
					swap(results_center[i], results_center[j]);
				}
			}
		}
	}


	return true;
}


bool CoreAlgorithm::calculateLPTag2fCentroid(const vector<TagPoint2f> leftTagsPnts2f, Point2f &LP_Centroid)
{
	CV_Assert(leftTagsPnts2f.size() != 0);
	double sum_x = 0;
	double sum_y = 0;
	for (size_t i = 0; i < leftTagsPnts2f.size(); i++)
	{
		sum_x = sum_x + leftTagsPnts2f[i][1];
		sum_y = sum_y + leftTagsPnts2f[i][2];
	}
	LP_Centroid.x = sum_x / leftTagsPnts2f.size();
	LP_Centroid.y = sum_y / leftTagsPnts2f.size();
	return true;
}











/////����ռ�һ��ֱ����һ�㵽����ֱ�ߵĴ�������ꡣ
Point3f CoreAlgorithm::GetFootOfPerpendicular(
	const Point3f &pt,     // ֱ����һ��  
	const Point3f &begin,  // ֱ�߿�ʼ��  
	const Point3f &end)   // ֱ�߽�����  
{
	Point3f retVal;

	double dx = begin.x - end.x;
	double dy = begin.y - end.y;
	double dz = begin.z - end.z;
	if (abs(dx) < 0.00000001 && abs(dy) < 0.00000001 && abs(dz) < 0.00000001)
	{
		retVal = begin;
		return retVal;
	}

	double u = (pt.x - begin.x)*(begin.x - end.x) +
		(pt.y - begin.y)*(begin.y - end.y) + (pt.z - begin.z)*(begin.z - end.z);
	u = u / ((dx*dx) + (dy*dy) + (dz*dz));

	retVal.x = begin.x + u*dx;
	retVal.y = begin.y + u*dy;
	retVal.z = begin.z + u*dz;
	return retVal;
}

//////�����������������������ϵת��
//void CoreAlgorithm::createCodepntsCoordinateSVD(vector<TagPoint3f> tagPnts, vector<TagPoint3f>& CodepntsCenterToCam)
//{
//	//////��tagPnts�еı�ǩȡ��������������
//	vector<int> tags;
//	for (size_t i = 0; i < tagPnts.size(); i++)
//	{
//		tags.push_back(int(tagPnts[i][0]));
//	}
//	///������Щ�ź������ά������ģ���Ϊ���������ϵ��ԭ��
//   Point3f CentroidPoint;
//   CoreAlgorithm::CalculateCentroid(tagPnts,CentroidPoint);
//   Mat srcMat;
//   CoreAlgorithm::TagPoint3fToMat(tagPnts, srcMat);
//   for (size_t i = 0; i < srcMat.rows; i++)
//   {
//	   srcMat.at<float>(i, 0) = srcMat.at<float>(i, 0) - CentroidPoint.x;
//	   srcMat.at<float>(i, 1) = srcMat.at<float>(i, 1) - CentroidPoint.y;
//	   srcMat.at<float>(i, 2) = srcMat.at<float>(i, 2) - CentroidPoint.z;
//   }
//   ///////��srcMat���������txt�ı���
//   //char *fname = "F:/Data/srcMat and R.txt.txt";
//   //ofstream fout(fname, ios::app);
//   //fout <<"srcMat== "<<srcMat << endl;
//   //fout.close();
//
//   /////������һϵ����ά���������ֵ�ֽ�
//   Mat S, U, VT;
//   SVD::compute(srcMat,S,U,VT,SVD::FULL_UV);////VT�˴�ΪV��ת�þ���Ҳ��������������ת��
//   //////ѡ����������ֵ������������Ϊx�ᣬѡ��ڶ�������ֵ������������x�ᴹֱ��ʸ����Ϊy�ᣬz����x,y��Ĳ��
//
//   Vec3f norm_X, norm_Y;
//   norm_X[0] = VT.at<float>(0, 0);
//   norm_X[1] = VT.at<float>(1, 0);
//   norm_X[2] = VT.at<float>(2, 0);
//   /////�����������ֱ�������ϵX,Y,Z,����x=(1,0,0);y=(0,1,0);z=(0,0,1),��ˣ����Ϊ������ȡ����
//   float temp_x = norm_X[0];
//   if (temp_x < 0)
//   {
//	   norm_X = -norm_X;
//   }
//   norm_Y[0] = VT.at<float>(0, 1);
//   norm_Y[1] = VT.at<float>(1, 1);
//   norm_Y[2] = VT.at<float>(2, 1);
//   float temp_y = norm_Y[1];
//   if (temp_y < 0)
//   {
//	   norm_Y = -norm_Y;
//   }
//
//   Vec3f norm_Z;
//   norm_Z[0] = VT.at<float>(0, 2);
//   norm_Z[1] = VT.at<float>(1, 2);
//   norm_Z[2] = VT.at<float>(2, 2);
//   float temp_z = norm_Z[2];
//   if (temp_z < 0)
//   {
//	   norm_Z = -norm_Z;
//   }
//   ////������λ��������ΪR�����������ԭ��ΪT
//    Mat R(3, 3, CV_32F);
//  /*  T.at<float>(0, 0) = CentroidPoint.x;
//   T.at<float>(1, 0) = CentroidPoint.y;
//   T.at<float>(2, 0) = CentroidPoint.z;*/
//   R.at<float>(0, 0) = norm_X[0]; R.at<float>(1, 0) = norm_X[1]; R.at<float>(2, 0) = norm_X[2];
//   R.at<float>(0, 1) = norm_Y[0]; R.at<float>(1, 1) = norm_Y[1]; R.at<float>(2, 1) = norm_Y[2];
//   R.at<float>(0, 2) = norm_Z[0]; R.at<float>(1, 2) = norm_Z[1]; R.at<float>(2, 2) = norm_Z[2];
//
//   ///////��srcMat���������txt�ı���
//   //char *fname = "F:/Data/srcMat and R.txt";
//   //ofstream fout(fname, ios::app);
//   //fout <<"R== "<< R << endl;
//   //fout.close();
//
//   // /*  Mat R_invert;
//   //invert(R, R_invert);*/
//   ////STEP-5:����������ϵ�µĵ�ת�������������ϵ�£�P2 = R*P1 + T
//   //for (unsigned int i = 0; i < tagPnts.size(); i++)
//   //{
//	  // Mat pnt(3, 1, CV_32F);
//	  // pnt.at<float>(0, 0) = srcMat.at<float>(i, 0);
//	  // pnt.at<float>(1, 0) = srcMat.at<float>(i, 1);
//	  // pnt.at<float>(2, 0) = srcMat.at<float>(i, 2);
//	  // pnt = R*(pnt);
//	  //  // pnt = R*(pnt);
//	  // tagPnts[i][0] = tags[i];
//	  // tagPnts[i][1] = pnt.at<float>(0, 0);
//	  // tagPnts[i][2] = pnt.at<float>(1, 0);
//	  // tagPnts[i][3] = pnt.at<float>(2, 0); 
//   //}
//   ////���������ϵ�µĵ�����ת�������������ϵ�£����ڴ˴���R�Ѿ���������ˣ�����opencv�е�SVD���ʿɵá�
//   Mat CodePntsMat = R*srcMat.t();
//   for (size_t i = 0; i < CodePntsMat.cols; i++)
//   {
//	   tagPnts[i][0] = tags[i];
//	   tagPnts[i][1] = CodePntsMat.at<float>(0, i);
//	   tagPnts[i][2] = CodePntsMat.at<float>(1, i);
//	   tagPnts[i][3] = CodePntsMat.at<float>(2, i);
//   }
//   CodepntsCenterToCam = tagPnts;
//
//}
void CoreAlgorithm::createCodepntsCoordinateSVD(vector<TagPoint3f> tagPnts, vector<TagPoint3f>& CodepntsCenterToCam)
{
	//////��tagPnts�еı�ǩȡ��������������
	vector<int> tags;
	for (size_t i = 0; i < tagPnts.size(); i++)
	{
		tags.push_back(int(tagPnts[i][0]));
	}
	///������Щ�ź������ά������ģ���Ϊ���������ϵ��ԭ��
	Point3f CentroidPoint;
	CoreAlgorithm::CalculateCentroid(tagPnts, CentroidPoint);
	Mat srcMat;
	CoreAlgorithm::TagPoint3fToMat(tagPnts, srcMat);
	for (size_t i = 0; i < srcMat.rows; i++)
	{
		srcMat.at<float>(i, 0) = srcMat.at<float>(i, 0) - CentroidPoint.x;
		srcMat.at<float>(i, 1) = srcMat.at<float>(i, 1) - CentroidPoint.y;
		srcMat.at<float>(i, 2) = srcMat.at<float>(i, 2) - CentroidPoint.z;
	}
	/////������һϵ����ά���������ֵ�ֽ�
	Mat S, U, VT;
	cv::SVD::compute(srcMat, S, U, VT, cv::SVD::FULL_UV);////VT�˴�ΪV��ת�þ���Ҳ��������������ת��
	//////ѡ����������ֵ������������Ϊx�ᣬѡ��ڶ�������ֵ������������x�ᴹֱ��ʸ����Ϊy�ᣬz����x,y��Ĳ��
	////���������ϵ�µĵ�����ת�������������ϵ�£����ڴ˴���R�Ѿ���������ˣ�����opencv�е�SVD���ʿɵá�
	Mat CodePntsMat = VT*srcMat.t();
	for (size_t i = 0; i < CodePntsMat.cols; i++)
	{
		tagPnts[i][0] = tags[i];
		tagPnts[i][1] = CodePntsMat.at<float>(0, i);
		tagPnts[i][2] = CodePntsMat.at<float>(1, i);
		tagPnts[i][3] = CodePntsMat.at<float>(2, i);
	}
	CodepntsCenterToCam = tagPnts;
}


//////�����������������������ϵת��
//void CoreAlgorithm::createCodepntsCoordinate(vector<TagPoint3f> tagPnts, vector<TagPoint3f>& CodepntsCenterToCodeOrigin)
//{
//	//////��tagPnts�еı�ǩȡ��������������
//	vector<int> tags;
//	for (size_t i = 0; i < tagPnts.size(); i++)
//	{
//		tags.push_back(int(tagPnts[i][0]));
//	}
//	Point3f CentroidPoint;
//	CoreAlgorithm::CalculateCentroid(tagPnts, CentroidPoint);
//	
//	//////����X������
//	vector<Point3f>  Z_AxisPnts;
//	for (unsigned int i = 0; i < 2; i++)
//	{
//		Z_AxisPnts.push_back(Point3f(tagPnts[0][1], tagPnts[0][2], tagPnts[0][3]));
//		Z_AxisPnts.push_back(Point3f(tagPnts[1][1], tagPnts[1][2], tagPnts[1][3]));
//	}
//	Vec6f Z_Line;
//	fitLine(Z_AxisPnts, Z_Line, CV_DIST_L2, 0, 0.01, 0.01);
//	Vec3f norm_Z;
//	norm_Z[0] = Z_Line[0]; norm_Z[1] = Z_Line[1]; norm_Z[2] = Z_Line[2];
//	///////�����жϷ��߷����Ƿ�Ϊ�ӵڶ����㵽��һ����ķ��򣬵ڶ������ȥ��һ���������
//	Vec3f norm_12;
//	norm_12[0] = Z_AxisPnts[1].x - Z_AxisPnts[0].x;
//	norm_12[1] = Z_AxisPnts[1].y - Z_AxisPnts[0].y;
//	norm_12[2] = Z_AxisPnts[1].z - Z_AxisPnts[0].z;
//	//////���
//	float temp_Z = norm_Z[0] * norm_12[0] + norm_Z[1] * norm_12[1] + norm_Z[2] * norm_12[2];
//	if (temp_Z < 0)///////����ͬ������Ӹ���
//	{
//		norm_Z = -1 * norm_Z;
//	}
//
//	//////�󾭹�����������㣬�Ҵ�ֱ��Z��ĵ�λ���������Y�ᵥλ������
//
//	Point3f pt;/////pt0Ϊֱ����һ�㣬pt1Ϊֱ����һ�㡣
//	pt = Point3f(tagPnts[2][1], tagPnts[2][2], tagPnts[2][3]);
//	Point3f PerpendicularFoot = CoreAlgorithm::GetFootOfPerpendicular(pt, Z_AxisPnts[0], Z_AxisPnts[1]);
//    /////ֱ�����Y��
//	vector<Point3f> Y_AxisPnts;
//	Y_AxisPnts.push_back(PerpendicularFoot);
//	Y_AxisPnts.push_back(pt);
//
//	Vec6f Y_Line;
//	fitLine(Y_AxisPnts, Y_Line, CV_DIST_L2, 0, 0.01, 0.01);
//	Vec3f norm_Y;
//	norm_Y[0] = Y_Line[0]; norm_Y[1] = Y_Line[1]; norm_Y[2] = Y_Line[2];
//	///////�����жϷ��߷����Ƿ�Ϊ�ӵ������㵽�����ķ��򣬵��������ȥ����������
//	Vec3f norm_3foot;
//	norm_3foot[0] = Y_AxisPnts[1].x - Y_AxisPnts[0].x;
//	norm_3foot[1] = Y_AxisPnts[1].y - Y_AxisPnts[0].y;
//	norm_3foot[2] = Y_AxisPnts[1].z - Y_AxisPnts[0].z;
//	float temp_Y = norm_Y[0] * norm_3foot[0] + norm_Y[1] * norm_3foot[1] + norm_Y[2] * norm_3foot[2];
//	if (temp_Y < 0)
//	{
//		norm_Y = -1 * norm_Y;
//	}
//     ////Y���Z�����X�ᵥλ���������ڴ˴���Z���Y���õ���XӦ��ȡ�������������ͬ
//	Vec3f norm_X;
//	norm_X[0] = norm_Z[1] * norm_Y[2] - norm_Z[2] * norm_Y[1];
//	norm_X[1] = norm_Z[2] * norm_Y[0] - norm_Z[0] * norm_Y[2];
//	norm_X[2] = norm_Z[0] * norm_Y[1] - norm_Z[1] * norm_Y[0];
//	//////��Ϊ���˳���ˣ���������һ������  
//	norm_X = -norm_X;
//
//	/////������λ��������ΪR����һ�����������ΪT;
//	Mat T(3, 1, CV_32F); Mat R(3, 3, CV_32F);
//	T.at<float>(0, 0) = CentroidPoint.x;
//	T.at<float>(1, 0) = CentroidPoint.y;
//	T.at<float>(2, 0) = CentroidPoint.z;
//
//	R.at<float>(0, 0) = norm_X[0]; R.at<float>(1, 0) = norm_X[1]; R.at<float>(2, 0) = norm_X[2];
//	R.at<float>(0, 1) = norm_Y[0]; R.at<float>(1, 1) = norm_Y[1]; R.at<float>(2, 1) = norm_Y[2];
//	R.at<float>(0, 2) = norm_Z[0]; R.at<float>(1, 2) = norm_Z[1]; R.at<float>(2, 2) = norm_Z[2];
//	Mat R_invert;
//	invert(R, R_invert);
//	//STEP-5:����������ϵ�µĵ�ת�������������ϵ�£�P2 = R*P1 + T
//	for (unsigned int i = 0; i < tagPnts.size(); i++)
//	{
//		Mat pnt(3, 1, CV_32F);
//		pnt.at<float>(0, 0) = tagPnts[i][1];
//		pnt.at<float>(1, 0) = tagPnts[i][2];
//		pnt.at<float>(2, 0) = tagPnts[i][3];
//		pnt = R_invert*(pnt - T);
//		tagPnts[i][0] = tags[i];
//		tagPnts[i][1] = pnt.at<float>(0, 0);
//		tagPnts[i][2] = pnt.at<float>(1, 0);
//		tagPnts[i][3] = pnt.at<float>(2, 0);
//	}
//	CodepntsCenterToCodeOrigin = tagPnts;
//}

void CoreAlgorithm::createCodepntsCoordinate(vector<TagPoint3f> tagPnts, vector<TagPoint3f>& CodepntsCenterToCodeOrigin)
{
	//////��tagPnts�еı�ǩȡ��������������
	vector<int> tags;
	for (size_t i = 0; i < tagPnts.size(); i++)
	{
		tags.push_back(int(tagPnts[i][0]));
	}
	Point3f CentroidPoint;
	CoreAlgorithm::CalculateCentroid(tagPnts, CentroidPoint);

	//////����X������
	vector<Point3f>  X_AxisPnts;
	for (unsigned int i = 0; i < 2; i++)
	{
		X_AxisPnts.push_back(Point3f(tagPnts[0][1], tagPnts[0][2], tagPnts[0][3]));
		X_AxisPnts.push_back(Point3f(tagPnts[1][1], tagPnts[1][2], tagPnts[1][3]));
	}
	cv::Vec6f X_Line;
	fitLine(X_AxisPnts, X_Line, CV_DIST_L2, 0, 0.01, 0.01);
	Vec3f norm_X;
	norm_X[0] = X_Line[0]; norm_X[1] = X_Line[1]; norm_X[2] = X_Line[2];
	///////�����жϷ��߷����Ƿ�Ϊ�ӵڶ����㵽��һ����ķ��򣬵ڶ������ȥ��һ���������
	Vec3f norm_12;
	norm_12[0] = X_AxisPnts[1].x - X_AxisPnts[0].x;
	norm_12[1] = X_AxisPnts[1].y - X_AxisPnts[0].y;
	norm_12[2] = X_AxisPnts[1].z - X_AxisPnts[0].z;
	//////���
	float temp_X = norm_X[0] * norm_12[0] + norm_X[1] * norm_12[1] + norm_X[2] * norm_12[2];
	if (temp_X < 0)///////����ͬ������Ӹ���
	{
		norm_X = -1 * norm_X;
	}

	//////�󾭹�����������㣬�Ҵ�ֱ��Z��ĵ�λ���������Y�ᵥλ������

	Point3f pt;/////pt0Ϊֱ����һ�㣬pt1Ϊֱ����һ�㡣
	pt = Point3f(tagPnts[2][1], tagPnts[2][2], tagPnts[2][3]);
	Point3f PerpendicularFoot = CoreAlgorithm::GetFootOfPerpendicular(pt, X_AxisPnts[0], X_AxisPnts[1]);
	/////ֱ�����Y��
	vector<Point3f> Y_AxisPnts;
	Y_AxisPnts.push_back(PerpendicularFoot);
	Y_AxisPnts.push_back(pt);

	cv::Vec6f Y_Line;
	fitLine(Y_AxisPnts, Y_Line, CV_DIST_L2, 0, 0.01, 0.01);
	Vec3f norm_Y;
	norm_Y[0] = Y_Line[0]; norm_Y[1] = Y_Line[1]; norm_Y[2] = Y_Line[2];
	///////�����жϷ��߷����Ƿ�Ϊ�ӵ������㵽�����ķ��򣬵��������ȥ����������
	Vec3f norm_3foot;
	norm_3foot[0] = Y_AxisPnts[1].x - Y_AxisPnts[0].x;
	norm_3foot[1] = Y_AxisPnts[1].y - Y_AxisPnts[0].y;
	norm_3foot[2] = Y_AxisPnts[1].z - Y_AxisPnts[0].z;
	float temp_Y = norm_Y[0] * norm_3foot[0] + norm_Y[1] * norm_3foot[1] + norm_Y[2] * norm_3foot[2];
	if (temp_Y < 0)
	{
		norm_Y = -1 * norm_Y;
	}

	////X���Y�����Z�ᵥλ������
	Vec3f norm_Z;
	norm_Z[0] = norm_X[1] * norm_Y[2] - norm_X[2] * norm_Y[1];
	norm_Z[1] = norm_X[2] * norm_Y[0] - norm_X[0] * norm_Y[2];
	norm_Z[2] = norm_X[0] * norm_Y[1] - norm_X[1] * norm_Y[0];

	/////������λ��������ΪR����һ�����������ΪT;
	Mat T(3, 1, CV_32F); Mat R(3, 3, CV_32F);
	T.at<float>(0, 0) = CentroidPoint.x;
	T.at<float>(1, 0) = CentroidPoint.y;
	T.at<float>(2, 0) = CentroidPoint.z;

	R.at<float>(0, 0) = norm_X[0]; R.at<float>(1, 0) = norm_X[1]; R.at<float>(2, 0) = norm_X[2];
	R.at<float>(0, 1) = norm_Y[0]; R.at<float>(1, 1) = norm_Y[1]; R.at<float>(2, 1) = norm_Y[2];
	R.at<float>(0, 2) = norm_Z[0]; R.at<float>(1, 2) = norm_Z[1]; R.at<float>(2, 2) = norm_Z[2];
	Mat R_invert;
	invert(R, R_invert);
	//STEP-5:����������ϵ�µĵ�ת�������������ϵ�£�P2 = R*P1 + T
	for (unsigned int i = 0; i < tagPnts.size(); i++)
	{
		Mat pnt(3, 1, CV_32F);
		pnt.at<float>(0, 0) = tagPnts[i][1];
		pnt.at<float>(1, 0) = tagPnts[i][2];
		pnt.at<float>(2, 0) = tagPnts[i][3];
		pnt = R_invert*(pnt - T);
		tagPnts[i][0] = tags[i];
		tagPnts[i][1] = pnt.at<float>(0, 0);
		tagPnts[i][2] = pnt.at<float>(1, 0);
		tagPnts[i][3] = pnt.at<float>(2, 0);
	}
	CodepntsCenterToCodeOrigin = tagPnts;
}




/////��5��TagPoint3f���͵�����ת����һ��Mat
void CoreAlgorithm::TagPoint3fToMat(vector<TagPoint3f> tagPnts, Mat &srcMat)
{
	int size = tagPnts.size();
	srcMat = Mat::zeros(size,3,CV_32F);
	for (size_t h = 0; h < size; h++)
	{
		srcMat.at<float>(h, 0) = tagPnts[h][1];
		srcMat.at<float>(h, 1) = tagPnts[h][2];
		srcMat.at<float>(h, 2) = tagPnts[h][3];
	}
}

///////����һϵ����ά������
 void CoreAlgorithm::CalculateCentroid(vector<TagPoint3f> tagPnts, Point3f &CentroidPoint)
{
	 double sum_x = 0;
	 double sum_y = 0;
	 double sum_z = 0;
	 for (size_t i = 0; i < tagPnts.size(); i++)
	 {
		 sum_x = sum_x + tagPnts[i][1];
		 sum_y = sum_y + tagPnts[i][2];
		 sum_z = sum_z + tagPnts[i][3];
	 }
	 CentroidPoint.x = sum_x / tagPnts.size();
	 CentroidPoint.y = sum_y / tagPnts.size();
	 CentroidPoint.z = sum_z / tagPnts.size();
}

 /////������������ͬ����Mat�ϲ���һ����Mat
 Mat CoreAlgorithm::comMatR(Mat Matrix1, Mat Matrix2, Mat &MatrixCom)
 {
	 CV_Assert(Matrix1.rows == Matrix2.rows);//��������ȣ����ִ����ж�    
	 MatrixCom.create(Matrix1.rows, Matrix1.cols + Matrix2.cols, CV_64FC1);
	 Mat temp = MatrixCom.colRange(0, Matrix1.cols);
	 Matrix1.copyTo(temp);
	 Mat temp1 = MatrixCom.colRange(Matrix1.cols, Matrix1.cols + Matrix2.cols);
	 Matrix2.copyTo(temp1);
	 return MatrixCom;
 }
 ////������������ͬ����Mat�ϲ���һ����Mat
 Mat CoreAlgorithm::comMatC(Mat Matrix1, Mat Matrix2, Mat &MatrixCom)
 {
	 CV_Assert(Matrix1.cols == Matrix2.cols);//��������ȣ����ִ����ж�    
	 MatrixCom.create(Matrix1.rows + Matrix2.rows, Matrix1.cols,CV_64FC1);
	 Mat temp = MatrixCom.rowRange(0, Matrix1.rows);
	 Matrix1.copyTo(temp);
	 Mat temp1 = MatrixCom.rowRange(Matrix1.rows, Matrix1.rows + Matrix2.rows);
	 Matrix2.copyTo(temp1);
	 return MatrixCom;
 }
  


 /////����getCodePoint3f()����
 bool CoreAlgorithm::getCodePoint3f(const Mat imgLeft, const cv::Rect maskLeft_First, const cv::Rect maskLeft_Second, const cv::Rect maskLeft_Third, const CamPara _campara1, const Mat imgRight, const cv::Rect maskRight_First, const cv::Rect maskRight_Second, const cv::Rect maskRight_Third, const CamPara _campara2,
	 const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& tagPnts3f,Point2f &CodeCenterPntLeft_First, Point2f &CodeCenterPntLeft_Second, Point2f &CodeCenterPntLeft_Third,
	 Point2f &CodeCenterPntRight_First, Point2f &CodeCenterPntRight_Second, Point2f &CodeCenterPntRight_Third)
 {
	 cv::Mat cameraMatrixLeft = (cv::Mat_<double>(3, 3) << _campara1.CameraIntrinsic[0][0], _campara1.CameraIntrinsic[0][1], _campara1.CameraIntrinsic[0][2]
		 , _campara1.CameraIntrinsic[1][0], _campara1.CameraIntrinsic[1][1], _campara1.CameraIntrinsic[1][2]
		 , _campara1.CameraIntrinsic[2][0], _campara1.CameraIntrinsic[2][1], _campara1.CameraIntrinsic[2][2]);
	 cv::Mat cameraMatrixRight = (cv::Mat_<double>(3, 3) << _campara2.CameraIntrinsic[0][0], _campara2.CameraIntrinsic[0][1], _campara2.CameraIntrinsic[0][2]
		 , _campara2.CameraIntrinsic[1][0], _campara2.CameraIntrinsic[1][1], _campara2.CameraIntrinsic[1][2]
		 , _campara2.CameraIntrinsic[2][0], _campara2.CameraIntrinsic[2][1], _campara2.CameraIntrinsic[2][2]);

	 //step-1 ������������Ķ�ά������㼯
	 vector<Point3f> pntsw;
	 pntsTagConfirm(tagPnts3fw, pntsw);
	 ///////���ñ����ʶ���㷨���ʶ�����ͼ�б���������
	 vector<pair<unsigned int, Point2f>>  LeftmarkerResults_First, RightmarkerResults_First,
		 LeftmarkerResults_Second, RightmarkerResults_Second,
		 LeftmarkerResults_Third, RightmarkerResults_Third;
	 pair<unsigned int, Point2f>  CodeCenterPntLeft_First_pair, CodeCenterPntLeft_Second_pair, CodeCenterPntLeft_Third_pair,
		 CodeCenterPntRight_First_pair, CodeCenterPntRight_Second_pair, CodeCenterPntRight_Third_pair;
		 
	 if (!CoreAlgorithm::findCircularMarker(imgLeft, maskLeft_First, LeftmarkerResults_First))
	 {
		 return false;
	 }
	 if (LeftmarkerResults_First.size() == 0)
	 {
		 return false;
	 }
	 if (LeftmarkerResults_First.size() >=1)
	 {
		 CodeCenterPntLeft_First = LeftmarkerResults_First[0].second;
		 CodeCenterPntLeft_First_pair = LeftmarkerResults_First[0];
	 }
	
	 if (!CoreAlgorithm::findCircularMarker(imgLeft, maskLeft_Second, LeftmarkerResults_Second))
	 {
		 return false;
	 }
	 if (LeftmarkerResults_Second.size() == 0)
	 {
		 return false;
	 }
	 if (LeftmarkerResults_Second.size() ==2)
	 {
		 if (LeftmarkerResults_Second[0].first == LeftmarkerResults_First[0].first)
		 {
			 CodeCenterPntLeft_Second = LeftmarkerResults_Second[1].second;
			 CodeCenterPntLeft_Second_pair = LeftmarkerResults_Second[1];
		 }
		 else
		 {
			 CodeCenterPntLeft_Second = LeftmarkerResults_Second[0].second;
			 CodeCenterPntLeft_Second_pair = LeftmarkerResults_Second[0];
		 }
	 }
	 if (LeftmarkerResults_Second.size() == 3)
	 {
		 CodeCenterPntLeft_Second = LeftmarkerResults_Second[1].second;
		 CodeCenterPntLeft_Second_pair = LeftmarkerResults_Second[1];
	 }

	 if (!CoreAlgorithm::findCircularMarker(imgLeft, maskLeft_Third, LeftmarkerResults_Third))
	 {
		 return false;
	 }
	 if (LeftmarkerResults_Third.size() == 0)
	 {
		 return false;
	 }
	 if (LeftmarkerResults_Third.size() == 2)
	 {
		 if (LeftmarkerResults_Third[0].first == LeftmarkerResults_First[0].first||LeftmarkerResults_Third[0].first==LeftmarkerResults_Second[0].first)
		 {
			 CodeCenterPntLeft_Second = LeftmarkerResults_Second[1].second;
			 CodeCenterPntLeft_Second_pair = LeftmarkerResults_Second[1];
		 }
		 else
		 {
			 CodeCenterPntLeft_Second = LeftmarkerResults_Second[0].second;
			 CodeCenterPntLeft_Second_pair = LeftmarkerResults_Second[0];
		 }	
	 }
	 if (LeftmarkerResults_Third.size() == 1)
	 {
		 if (LeftmarkerResults_Third[0].first == LeftmarkerResults_First[0].first || LeftmarkerResults_Third[0].first == LeftmarkerResults_Second[0].first)
		 {
			 return false;
		 }
		 else
		 {
			 CodeCenterPntLeft_Third = LeftmarkerResults_Third[0].second;
			 CodeCenterPntLeft_Third_pair = LeftmarkerResults_Third[0];
		 }
	 }
	 if (LeftmarkerResults_Third.size() == 3)
	 {
		 for (size_t i = 0; i < LeftmarkerResults_Third.size(); i++)
		 {
			 if ((LeftmarkerResults_Third[i].first != LeftmarkerResults_First[0].first) && (LeftmarkerResults_Third[i].first != LeftmarkerResults_Second[0].first))
			 {
				 CodeCenterPntLeft_Third = LeftmarkerResults_Third[i].second;
				 CodeCenterPntLeft_Third_pair = LeftmarkerResults_Third[i];
			 }
			 else
			 {
				 continue;
			 }
		 }
	 }
	 ///////���ñ����ʶ���㷨���ʶ�����ͼ�б���������
	 /////*********
	 if (!CoreAlgorithm::findCircularMarker(imgRight, maskRight_First, RightmarkerResults_First))
	 {
		 return false;
	 }
	 if (RightmarkerResults_First.size() == 0)
	 {
		 return false;
	 }
	 if (RightmarkerResults_First.size() >= 1)
	 {
		 CodeCenterPntRight_First = RightmarkerResults_First[0].second;
		 CodeCenterPntRight_First_pair = RightmarkerResults_First[0];
	 }


	 if (!CoreAlgorithm::findCircularMarker(imgRight, maskRight_Second, RightmarkerResults_Second))
	 {
		 return false;
	 }
	 if (RightmarkerResults_Second.size() == 0)
	 {
		 return false;
	 }
	 if (RightmarkerResults_Second.size() == 2)
	 {
		 if (RightmarkerResults_Second[0].first == RightmarkerResults_First[0].first)
		 {
			 CodeCenterPntRight_Second = RightmarkerResults_Second[1].second;
			 CodeCenterPntRight_Second_pair = RightmarkerResults_Second[1];
		 }
		 else
		 {
			 CodeCenterPntRight_Second = RightmarkerResults_Second[0].second;
			 CodeCenterPntRight_Second_pair = RightmarkerResults_Second[0];
		 }
	 }
	 if (RightmarkerResults_Second.size() == 3)
	 {
		 CodeCenterPntRight_Second = RightmarkerResults_Second[1].second;
		 CodeCenterPntRight_Second_pair = RightmarkerResults_Second[1];
	 }

	 if (!CoreAlgorithm::findCircularMarker(imgRight, maskRight_Third, RightmarkerResults_Third))
	 {
		 return false;
	 }
	 if (RightmarkerResults_Third.size() == 0)
	 {
		 return false;
	 }
	 if (RightmarkerResults_Third.size() == 2)
	 {
		 if (RightmarkerResults_Third[0].first == RightmarkerResults_First[0].first || RightmarkerResults_Third[0].first == RightmarkerResults_Second[0].first)
		 {
			 CodeCenterPntRight_Second = RightmarkerResults_Second[1].second;
			 CodeCenterPntRight_Second_pair = RightmarkerResults_Second[1];
		 }
		 else
		 {
			 CodeCenterPntRight_Second = RightmarkerResults_Second[0].second;
			 CodeCenterPntRight_Second_pair = RightmarkerResults_Second[0];
		 }
	 }
	 if (RightmarkerResults_Third.size() == 1)
	 {
		 if (RightmarkerResults_Third[0].first == RightmarkerResults_First[0].first || RightmarkerResults_Third[0].first == RightmarkerResults_Second[0].first)
		 {
			 return false;
		 }
		 else
		 {
			 CodeCenterPntRight_Third = RightmarkerResults_Third[0].second;
			 CodeCenterPntRight_Third_pair = RightmarkerResults_Third[0];
		 }
	 }
	 if (RightmarkerResults_Third.size() == 3)
	 {
		 for (size_t i = 0; i < RightmarkerResults_Third.size(); i++)
		 {
			 if ((RightmarkerResults_Third[i].first != RightmarkerResults_First[0].first) && (RightmarkerResults_Third[i].first != RightmarkerResults_Second[0].first))
			 {
				 CodeCenterPntRight_Third = RightmarkerResults_Third[i].second;
				 CodeCenterPntRight_Third_pair = RightmarkerResults_Third[i];
			 }
			 else
			 {
				 continue;
			 }
		 }
	 }
	 /////********
	 vector<pair<unsigned int, Point2f>>  LeftmarkerResults, RightmarkerResults;
	 LeftmarkerResults.push_back(CodeCenterPntLeft_First_pair);
	 LeftmarkerResults.push_back(CodeCenterPntLeft_Second_pair);
	 LeftmarkerResults.push_back(CodeCenterPntLeft_Third_pair);
	 RightmarkerResults.push_back(CodeCenterPntRight_First_pair);
	 RightmarkerResults.push_back(CodeCenterPntRight_Second_pair);
	 RightmarkerResults.push_back(CodeCenterPntRight_Third_pair);
	 /////��ʶ����ı������������У����ں�������
	 vector<TagPoint2f> leftTagsPnts2f;
	 vector<TagPoint2f> rightTagsPnts2f;
	 for (size_t k = 0; k < LeftmarkerResults.size(); k++)
	 {
		 TagPoint2f  LeftTagPnts, RightTagPnts;
		 LeftTagPnts[0] = LeftmarkerResults[k].first;
		 LeftTagPnts[1] = LeftmarkerResults[k].second.x;
		 LeftTagPnts[2] = LeftmarkerResults[k].second.y;
		 RightTagPnts[0] = RightmarkerResults[k].first;
		 RightTagPnts[1] = RightmarkerResults[k].second.x;
		 RightTagPnts[2] = RightmarkerResults[k].second.y;
		 leftTagsPnts2f.push_back(LeftTagPnts);
		 rightTagsPnts2f.push_back(RightTagPnts);
	 }
	 ////��������ı�ǩ��Ϣȥ��
	 vector<Point2f> leftPnts2f, rightPnts2f;
	 vector<int> TagVec;
	 pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);

	 ////������������������ϵ�µ���ά����Ϣ
	 vector<Point3f> pnts3fVec;
	 //��һ�ּ�����ά����Ϣ�ķ���
	 RT CAMG;
	 Mat rotation = Mat::zeros(3, 1, CV_64FC1);
	 Mat translation = Mat::zeros(3, 1, CV_64FC1);
	 for (int i = 0; i < 3; i++)
	 {
		 rotation.at<double>(i, 0) = _camgrouppara.right2LeftRotVector[i];
		 translation.at<double>(i, 0) = _camgrouppara.right2LeftTraVector[i];
	 }
	 CAMG.R = rotation; CAMG.T = translation;
	 vector<Point3f> pntsl;
	 bool isok;
	 isok = Cal3dPoint(leftPnts2f, _campara1, rightPnts2f, _campara2, CAMG.R, CAMG.T, pntsl);
	 vector<TagPoint3f> result;
	 addPntsTag(pntsl, TagVec, result);
	 tagPnts3f = result;
	 return true;
 }

 bool CoreAlgorithm::getCodePoint3f(const Mat imgLeft, cv::Rect maskLeftRoi_First, const Mat imgRight, cv::Rect maskRightRoi_First, const CamPara _campara1, const CamPara _campara2, const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& TagPnt_First)
 {
	 cv::Mat cameraMatrixLeft = (cv::Mat_<double>(3, 3) << _campara1.CameraIntrinsic[0][0], _campara1.CameraIntrinsic[0][1], _campara1.CameraIntrinsic[0][2]
		 , _campara1.CameraIntrinsic[1][0], _campara1.CameraIntrinsic[1][1], _campara1.CameraIntrinsic[1][2]
		 , _campara1.CameraIntrinsic[2][0], _campara1.CameraIntrinsic[2][1], _campara1.CameraIntrinsic[2][2]);
	 cv::Mat cameraMatrixRight = (cv::Mat_<double>(3, 3) << _campara2.CameraIntrinsic[0][0], _campara2.CameraIntrinsic[0][1], _campara2.CameraIntrinsic[0][2]
		 , _campara2.CameraIntrinsic[1][0], _campara2.CameraIntrinsic[1][1], _campara2.CameraIntrinsic[1][2]
		 , _campara2.CameraIntrinsic[2][0], _campara2.CameraIntrinsic[2][1], _campara2.CameraIntrinsic[2][2]);

	 //step-1 ������������Ķ�ά������㼯
	 vector<Point3f> pntsw;
	 pntsTagConfirm(tagPnts3fw, pntsw);
	 ///////���ñ����ʶ���㷨���ʶ�����ͼ�б���������
	 vector<pair<unsigned int, Point2f>>  LeftmarkerResults_First, RightmarkerResults_First,
		 LeftmarkerResults_Second, RightmarkerResults_Second,
		 LeftmarkerResults_Third, RightmarkerResults_Third;
	 pair<unsigned int, Point2f>  CodeCenterPntLeft_First_pair, CodeCenterPntLeft_Second_pair, CodeCenterPntLeft_Third_pair,
		 CodeCenterPntRight_First_pair, CodeCenterPntRight_Second_pair, CodeCenterPntRight_Third_pair;

	 if (!CoreAlgorithm::findCircularMarker(imgLeft, maskLeftRoi_First, LeftmarkerResults_First))
	 {
		 return false;
	 }
	 if (LeftmarkerResults_First.size() == 0)
	 {
		 return false;
	 }
	 if (LeftmarkerResults_First.size() >= 1)
	 {
		
		 CodeCenterPntLeft_First_pair = LeftmarkerResults_First[0];
	 }

	
	 
	 ///////���ñ����ʶ���㷨���ʶ�����ͼ�б���������
	 /////*********
	 if (!CoreAlgorithm::findCircularMarker(imgRight, maskRightRoi_First, RightmarkerResults_First))
	 {
		 return false;
	 }
	 if (RightmarkerResults_First.size() == 0)
	 {
		 return false;
	 }
	 if (RightmarkerResults_First.size() >= 1)
	 {
		
		 CodeCenterPntRight_First_pair = RightmarkerResults_First[0];
	 }

	 /////********
	 vector<pair<unsigned int, Point2f>>  LeftmarkerResults, RightmarkerResults;
	 LeftmarkerResults.push_back(CodeCenterPntLeft_First_pair);
	 RightmarkerResults.push_back(CodeCenterPntRight_First_pair);
	
	 /////��ʶ����ı������������У����ں�������
	 vector<TagPoint2f> leftTagsPnts2f;
	 vector<TagPoint2f> rightTagsPnts2f;
	 for (size_t k = 0; k < LeftmarkerResults.size(); k++)
	 {
		 TagPoint2f  LeftTagPnts, RightTagPnts;
		 LeftTagPnts[0] = LeftmarkerResults[k].first;
		 LeftTagPnts[1] = LeftmarkerResults[k].second.x;
		 LeftTagPnts[2] = LeftmarkerResults[k].second.y;
		 RightTagPnts[0] = RightmarkerResults[k].first;
		 RightTagPnts[1] = RightmarkerResults[k].second.x;
		 RightTagPnts[2] = RightmarkerResults[k].second.y;
		 leftTagsPnts2f.push_back(LeftTagPnts);
		 rightTagsPnts2f.push_back(RightTagPnts);
	 }
	 ////��������ı�ǩ��Ϣȥ��
	 vector<Point2f> leftPnts2f, rightPnts2f;
	 vector<int> TagVec;
	 pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);

	 ////������������������ϵ�µ���ά����Ϣ
	 vector<Point3f> pnts3fVec;
	 //��һ�ּ�����ά����Ϣ�ķ���
	 RT CAMG;
	 Mat rotation = Mat::zeros(3, 1, CV_64FC1);
	 Mat translation = Mat::zeros(3, 1, CV_64FC1);
	 for (int i = 0; i < 3; i++)
	 {
		 rotation.at<double>(i, 0) = _camgrouppara.right2LeftRotVector[i];
		 translation.at<double>(i, 0) = _camgrouppara.right2LeftTraVector[i];
	 }
	 CAMG.R = rotation; CAMG.T = translation;
	 vector<Point3f> pntsl;
	 bool isok;
	 isok = Cal3dPoint(leftPnts2f, _campara1, rightPnts2f, _campara2, CAMG.R, CAMG.T, pntsl);
	 vector<TagPoint3f> result;
	 addPntsTag(pntsl, TagVec, result);
	 TagPnt_First= result;
 }


 ///////���ñ����־��ʶ����ʶ���ͼƬ�б�־��λ�ã����ؽ����������ϵ�µ���ά�㡣
 bool  CoreAlgorithm::getCodePoint3f(const Mat& imgLeft, const CamPara _campara1, const Mat& imgRight, const CamPara _campara2,
	 const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<pair<unsigned int, Point2f>> &LeftmarkerResults, vector<pair<unsigned int, Point2f>> &RightmarkerResults, vector<TagPoint3f>& tagPnts3f)
 {
	 cv::Mat cameraMatrixLeft = (cv::Mat_<double>(3, 3) << _campara1.CameraIntrinsic[0][0], _campara1.CameraIntrinsic[0][1], _campara1.CameraIntrinsic[0][2]
		 , _campara1.CameraIntrinsic[1][0], _campara1.CameraIntrinsic[1][1], _campara1.CameraIntrinsic[1][2]
		 , _campara1.CameraIntrinsic[2][0], _campara1.CameraIntrinsic[2][1], _campara1.CameraIntrinsic[2][2]);
	 cv::Mat cameraMatrixRight = (cv::Mat_<double>(3, 3) << _campara2.CameraIntrinsic[0][0], _campara2.CameraIntrinsic[0][1], _campara2.CameraIntrinsic[0][2]
		 , _campara2.CameraIntrinsic[1][0], _campara2.CameraIntrinsic[1][1], _campara2.CameraIntrinsic[1][2]
		 , _campara2.CameraIntrinsic[2][0], _campara2.CameraIntrinsic[2][1], _campara2.CameraIntrinsic[2][2]);

	 //step-1 ������������Ķ�ά������㼯
	 vector<Point3f> pntsw;
	 pntsTagConfirm(tagPnts3fw, pntsw);

	 ///////���ñ����ʶ���㷨���ʶ�����ͼ�б���������
	 LeftmarkerResults.clear();
	 bool leftResult = CoreAlgorithm::findCircularMarker(imgLeft, LeftmarkerResults);
	 ///////���ñ����ʶ���㷨���ʶ�����ͼ�б���������
	 RightmarkerResults.clear();
	 bool rightResult = CoreAlgorithm::findCircularMarker(imgRight, RightmarkerResults);
	 if (LeftmarkerResults.size() == RightmarkerResults.size() && LeftmarkerResults.size() ==3)
	 {
		 /////��ʶ����ı������������У����ں�������
		 vector<TagPoint2f> leftTagsPnts2f;
		 vector<TagPoint2f> rightTagsPnts2f;
		 for (size_t k = 0; k < LeftmarkerResults.size(); k++)
		 {
			 TagPoint2f  LeftTagPnts, RightTagPnts;
			 LeftTagPnts[0] = LeftmarkerResults[k].first;
			 LeftTagPnts[1] = LeftmarkerResults[k].second.x;
			 LeftTagPnts[2] = LeftmarkerResults[k].second.y;
			 RightTagPnts[0] = RightmarkerResults[k].first;
			 RightTagPnts[1] = RightmarkerResults[k].second.x;
			 RightTagPnts[2] = RightmarkerResults[k].second.y;
			 leftTagsPnts2f.push_back(LeftTagPnts);
			 rightTagsPnts2f.push_back(RightTagPnts);
		 }
		 ////��������ı�ǩ��Ϣȥ��
		 vector<Point2f> leftPnts2f, rightPnts2f;
		 vector<int> TagVec;
		 pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);

		 ////������������������ϵ�µ���ά����Ϣ
		 vector<Point3f> pnts3fVec;
		 //��һ�ּ�����ά����Ϣ�ķ���
		 RT CAMG;
		 Mat rotation = Mat::zeros(3, 1, CV_64FC1);
		 Mat translation = Mat::zeros(3, 1, CV_64FC1);
		 for (int i = 0; i < 3; i++)
		 {
			 rotation.at<double>(i, 0) = _camgrouppara.right2LeftRotVector[i];
			 translation.at<double>(i, 0) = _camgrouppara.right2LeftTraVector[i];
		 }
		 CAMG.R = rotation; CAMG.T = translation;
		 vector<Point3f> pntsl;
		 bool isok;
		 isok = Cal3dPoint(leftPnts2f, _campara1, rightPnts2f, _campara2, CAMG.R, CAMG.T, pntsl);
		 vector<TagPoint3f> result;
		 addPntsTag(pntsl, TagVec, result);
		 tagPnts3f = result;
	 }
	 else
	 {
		 return false;
	 }

	 return true;
 }
 ////end;



















 ////����getCodePoint3f()����
 bool CoreAlgorithm::getCodePoint3f_First(const Mat imgLeft, cv::Rect maskLeftRoi_First, const Mat imgRight, cv::Rect maskRightRoi_First, const CamPara _campara1, const CamPara _campara2, const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& TagPnt_First, pair<unsigned int, Point2f> &CodeLeft_First, pair<unsigned int, Point2f> &CodeRight_First, int flag_First)
 {
	 cv::Mat cameraMatrixLeft = (cv::Mat_<double>(3, 3) << _campara1.CameraIntrinsic[0][0], _campara1.CameraIntrinsic[0][1], _campara1.CameraIntrinsic[0][2]
		 , _campara1.CameraIntrinsic[1][0], _campara1.CameraIntrinsic[1][1], _campara1.CameraIntrinsic[1][2]
		 , _campara1.CameraIntrinsic[2][0], _campara1.CameraIntrinsic[2][1], _campara1.CameraIntrinsic[2][2]);
	 cv::Mat cameraMatrixRight = (cv::Mat_<double>(3, 3) << _campara2.CameraIntrinsic[0][0], _campara2.CameraIntrinsic[0][1], _campara2.CameraIntrinsic[0][2]
		 , _campara2.CameraIntrinsic[1][0], _campara2.CameraIntrinsic[1][1], _campara2.CameraIntrinsic[1][2]
		 , _campara2.CameraIntrinsic[2][0], _campara2.CameraIntrinsic[2][1], _campara2.CameraIntrinsic[2][2]);

	 //step-1 ������������Ķ�ά������㼯
	 vector<Point3f> pntsw;
	 pntsTagConfirm(tagPnts3fw, pntsw);
	 ///////���ñ����ʶ���㷨���ʶ�����ͼ�б���������
	 vector<pair<unsigned int, Point2f>>  LeftmarkerResults_First, RightmarkerResults_First;
	
	 pair<unsigned int, Point2f>  CodeCenterPntLeft_First_pair,
		 CodeCenterPntRight_First_pair;

	 if (!CoreAlgorithm::findCircularMarker(imgLeft, maskLeftRoi_First, LeftmarkerResults_First))
	 {
		 return false;
	 }
	 if (LeftmarkerResults_First.size() == 0)
	 {
		 return false;
	 }
	 if (LeftmarkerResults_First.size() >= 1)
	 {
		 for (size_t i = 0; i < LeftmarkerResults_First.size(); i++)
		 {
			 if (LeftmarkerResults_First[i].first == flag_First)
			 {
				 CodeLeft_First = LeftmarkerResults_First[i];
			 }
			 else
			 {
				 continue;
			 }
		 }
		 if (CodeLeft_First.first != flag_First)
		 {
			 return false;
		 }
	 }
	

	 ///////���ñ����ʶ���㷨���ʶ�����ͼ�б���������
	 /////*********
	 if (!CoreAlgorithm::findCircularMarker(imgRight, maskRightRoi_First, RightmarkerResults_First))
	 {
		 return false;
	 }
	 if (RightmarkerResults_First.size() == 0)
	 {
		 return false;
	 }
	 if (RightmarkerResults_First.size() >= 1)
	 {
		 for (size_t i = 0; i < RightmarkerResults_First.size(); i++)
		 {
			 if (RightmarkerResults_First[i].first == flag_First)
			 {
				 CodeRight_First = RightmarkerResults_First[i];
			 }
			 else
			 {
				 continue;
			 }
		
		 }
		 if (CodeRight_First.first!=flag_First)
		 {
			 return false;
		 }
	
	 }

	 /////********
	 vector<pair<unsigned int, Point2f>>  LeftmarkerResults, RightmarkerResults;
	 LeftmarkerResults.push_back(CodeLeft_First);
	 RightmarkerResults.push_back(CodeRight_First);

	 /////��ʶ����ı������������У����ں�������
	 vector<TagPoint2f> leftTagsPnts2f;
	 vector<TagPoint2f> rightTagsPnts2f;
	 for (size_t k = 0; k < LeftmarkerResults.size(); k++)
	 {
		 TagPoint2f  LeftTagPnts, RightTagPnts;
		 LeftTagPnts[0] = LeftmarkerResults[k].first;
		 LeftTagPnts[1] = LeftmarkerResults[k].second.x;
		 LeftTagPnts[2] = LeftmarkerResults[k].second.y;
		 RightTagPnts[0] = RightmarkerResults[k].first;
		 RightTagPnts[1] = RightmarkerResults[k].second.x;
		 RightTagPnts[2] = RightmarkerResults[k].second.y;
		 leftTagsPnts2f.push_back(LeftTagPnts);
		 rightTagsPnts2f.push_back(RightTagPnts);
	 }
	 ////��������ı�ǩ��Ϣȥ��
	 vector<Point2f> leftPnts2f, rightPnts2f;
	 vector<int> TagVec;
	 pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);

	 ////������������������ϵ�µ���ά����Ϣ
	 vector<Point3f> pnts3fVec;
	 //��һ�ּ�����ά����Ϣ�ķ���
	 RT CAMG;
	 Mat rotation = Mat::zeros(3, 1, CV_64FC1);
	 Mat translation = Mat::zeros(3, 1, CV_64FC1);
	 for (int i = 0; i < 3; i++)
	 {
		 rotation.at<double>(i, 0) = _camgrouppara.right2LeftRotVector[i];
		 translation.at<double>(i, 0) = _camgrouppara.right2LeftTraVector[i];
	 }
	 CAMG.R = rotation; CAMG.T = translation;
	 vector<Point3f> pntsl;
	 bool isok;
	 isok = Cal3dPoint(leftPnts2f, _campara1, rightPnts2f, _campara2, CAMG.R, CAMG.T, pntsl);
	 vector<TagPoint3f> result;
	 addPntsTag(pntsl, TagVec, result);
	 TagPnt_First = result;
	// cout << TagPnt_First[0] << endl;
	 return true;
 }



 bool CoreAlgorithm::getCodePoint3f_Second(const Mat imgLeft, cv::Rect maskLeftRoi_Second, const Mat imgRight, cv::Rect maskRightRoi_Second, const CamPara _campara1, const CamPara _campara2, const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& TagPnt_Second, pair<unsigned int, Point2f> &CodeLeft_Second, pair<unsigned int, Point2f> &CodeRight_Second,int flag_Second)
 {
	 cv::Mat cameraMatrixLeft = (cv::Mat_<double>(3, 3) << _campara1.CameraIntrinsic[0][0], _campara1.CameraIntrinsic[0][1], _campara1.CameraIntrinsic[0][2]
		 , _campara1.CameraIntrinsic[1][0], _campara1.CameraIntrinsic[1][1], _campara1.CameraIntrinsic[1][2]
		 , _campara1.CameraIntrinsic[2][0], _campara1.CameraIntrinsic[2][1], _campara1.CameraIntrinsic[2][2]);
	 cv::Mat cameraMatrixRight = (cv::Mat_<double>(3, 3) << _campara2.CameraIntrinsic[0][0], _campara2.CameraIntrinsic[0][1], _campara2.CameraIntrinsic[0][2]
		 , _campara2.CameraIntrinsic[1][0], _campara2.CameraIntrinsic[1][1], _campara2.CameraIntrinsic[1][2]
		 , _campara2.CameraIntrinsic[2][0], _campara2.CameraIntrinsic[2][1], _campara2.CameraIntrinsic[2][2]);
	 //step-1 ������������Ķ�ά������㼯
	 vector<Point3f> pntsw;
	 pntsTagConfirm(tagPnts3fw, pntsw);
	 ///////���ñ����ʶ���㷨���ʶ�����ͼ�б���������
	 vector<pair<unsigned int, Point2f>>  LeftmarkerResults_Second, RightmarkerResults_Second;

	 pair<unsigned int, Point2f>  CodeCenterPntLeft_Second_pair, 
		  CodeCenterPntRight_Second_pair;

	 if (!CoreAlgorithm::findCircularMarker(imgLeft, maskLeftRoi_Second, LeftmarkerResults_Second))
	 {
		 return false;
	 }
	 if (LeftmarkerResults_Second.size() == 0)
	 {
		 return false;
	 }
	 if (LeftmarkerResults_Second.size() >= 1)
	 {
		 for (size_t i = 0; i < LeftmarkerResults_Second.size();i++)
		 {
			 if (LeftmarkerResults_Second[i].first == flag_Second)
			 {
				 CodeLeft_Second = LeftmarkerResults_Second[i];
			 }
			 else
			 {
				 continue;
			 }
		 }
		 if (CodeLeft_Second.first!=flag_Second)
		 {
			 return false;
		 }
		
	 }

	 ///////���ñ����ʶ���㷨���ʶ�����ͼ�б���������
	 /////*********
	 if (!CoreAlgorithm::findCircularMarker(imgRight, maskRightRoi_Second, RightmarkerResults_Second))
	 {
		 return false;
	 }
	 if (RightmarkerResults_Second.size() == 0)
	 {
		 return false;
	 }
	 if (RightmarkerResults_Second.size() >= 1)
	 {
		 for (size_t i = 0; i < RightmarkerResults_Second.size(); i++)
		 {
			 if (RightmarkerResults_Second[i].first == flag_Second)
			 {
				 CodeRight_Second = RightmarkerResults_Second[i];
			 }
			 else
			 {
				 continue;
			 }

		 }
		 if (CodeRight_Second.first!=flag_Second)
		 {
			 return false;
		 }
		 
	 }

	 /////********
	 vector<pair<unsigned int, Point2f>>  LeftmarkerResults, RightmarkerResults;
	 LeftmarkerResults.push_back(CodeLeft_Second);
	 RightmarkerResults.push_back(CodeRight_Second);

	 /////��ʶ����ı������������У����ں�������
	 vector<TagPoint2f> leftTagsPnts2f;
	 vector<TagPoint2f> rightTagsPnts2f;
	 for (size_t k = 0; k < LeftmarkerResults.size(); k++)
	 {
		 TagPoint2f  LeftTagPnts, RightTagPnts;
		 LeftTagPnts[0] = LeftmarkerResults[k].first;
		 LeftTagPnts[1] = LeftmarkerResults[k].second.x;
		 LeftTagPnts[2] = LeftmarkerResults[k].second.y;
		 RightTagPnts[0] = RightmarkerResults[k].first;
		 RightTagPnts[1] = RightmarkerResults[k].second.x;
		 RightTagPnts[2] = RightmarkerResults[k].second.y;
		 leftTagsPnts2f.push_back(LeftTagPnts);
		 rightTagsPnts2f.push_back(RightTagPnts);
	 }
	 ////��������ı�ǩ��Ϣȥ��
	 vector<Point2f> leftPnts2f, rightPnts2f;
	 vector<int> TagVec;
	 pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);

	 ////������������������ϵ�µ���ά����Ϣ
	 vector<Point3f> pnts3fVec;
	 //��һ�ּ�����ά����Ϣ�ķ���
	 RT CAMG;
	 Mat rotation = Mat::zeros(3, 1, CV_64FC1);
	 Mat translation = Mat::zeros(3, 1, CV_64FC1);
	 for (int i = 0; i < 3; i++)
	 {
		 rotation.at<double>(i, 0) = _camgrouppara.right2LeftRotVector[i];
		 translation.at<double>(i, 0) = _camgrouppara.right2LeftTraVector[i];
	 }
	 CAMG.R = rotation; CAMG.T = translation;
	 vector<Point3f> pntsl;
	 bool isok;
	 isok = Cal3dPoint(leftPnts2f, _campara1, rightPnts2f, _campara2, CAMG.R, CAMG.T, pntsl);
	 vector<TagPoint3f> result;
	 addPntsTag(pntsl, TagVec, result);
	 TagPnt_Second = result;
	// cout << TagPnt_Second[0] << endl;
	 return true;

 }
 bool CoreAlgorithm::getCodePoint3f_Third(const Mat imgLeft, cv::Rect maskLeftRoi_Third, const Mat imgRight, cv::Rect maskRightRoi_Third, const CamPara _campara1, const CamPara _campara2, const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& TagPnt_Third, pair<unsigned int, Point2f> &CodeLeft_Third, pair<unsigned int, Point2f> &CodeRight_Third,int flag_Third)
 {
	 cv::Mat cameraMatrixLeft = (cv::Mat_<double>(3, 3) << _campara1.CameraIntrinsic[0][0], _campara1.CameraIntrinsic[0][1], _campara1.CameraIntrinsic[0][2]
		 , _campara1.CameraIntrinsic[1][0], _campara1.CameraIntrinsic[1][1], _campara1.CameraIntrinsic[1][2]
		 , _campara1.CameraIntrinsic[2][0], _campara1.CameraIntrinsic[2][1], _campara1.CameraIntrinsic[2][2]);
	 cv::Mat cameraMatrixRight = (cv::Mat_<double>(3, 3) << _campara2.CameraIntrinsic[0][0], _campara2.CameraIntrinsic[0][1], _campara2.CameraIntrinsic[0][2]
		 , _campara2.CameraIntrinsic[1][0], _campara2.CameraIntrinsic[1][1], _campara2.CameraIntrinsic[1][2]
		 , _campara2.CameraIntrinsic[2][0], _campara2.CameraIntrinsic[2][1], _campara2.CameraIntrinsic[2][2]);
	 //step-1 ������������Ķ�ά������㼯
	 vector<Point3f> pntsw;
	 pntsTagConfirm(tagPnts3fw, pntsw);
	 ///////���ñ����ʶ���㷨���ʶ�����ͼ�б���������
	 vector<pair<unsigned int, Point2f>>  LeftmarkerResults_Third, RightmarkerResults_Third;

	 pair<unsigned int, Point2f>  CodeCenterPntLeft_Third_pair, 
		 CodeCenterPntRight_Third_pair;

	 if (!CoreAlgorithm::findCircularMarker(imgLeft, maskLeftRoi_Third, LeftmarkerResults_Third))
	 {
		 return false;
	 }
	 if (LeftmarkerResults_Third.size() == 0)
	 {
		 return false;
	 }
	 if (LeftmarkerResults_Third.size() >= 1)
	 {

		 for (size_t i = 0; i < LeftmarkerResults_Third.size(); i++)
		 {
			 if (LeftmarkerResults_Third[i].first == flag_Third)
			 {
				 CodeLeft_Third = LeftmarkerResults_Third[i];
			 }
			 else
			 {
				 continue;
			 }
		 }
		 if (CodeLeft_Third.first != flag_Third)
		 {
			 return false;
		 }
	
	 }

	 ///////���ñ����ʶ���㷨���ʶ�����ͼ�б���������
	 /////*********
	 if (!CoreAlgorithm::findCircularMarker(imgRight, maskRightRoi_Third, RightmarkerResults_Third))
	 {
		 return false;
	 }
	 if (RightmarkerResults_Third.size() == 0)
	 {
		 return false;
	 }
	 if (RightmarkerResults_Third.size() >= 1)
	 {
		 for (size_t i = 0; i < RightmarkerResults_Third.size(); i++)
		 {
			 if (RightmarkerResults_Third[i].first == flag_Third)
			 {
				 CodeRight_Third = RightmarkerResults_Third[i];
			 }
			 else
			 {
				 continue;
			 }
		 }
		 if (CodeRight_Third.first!=flag_Third)
		 {
			 return false;
		 }
		
	 }

	 /////********
	 vector<pair<unsigned int, Point2f>>  LeftmarkerResults, RightmarkerResults;
	 LeftmarkerResults.push_back(CodeLeft_Third);
	 RightmarkerResults.push_back(CodeRight_Third);

	 /////��ʶ����ı������������У����ں�������
	 vector<TagPoint2f> leftTagsPnts2f;
	 vector<TagPoint2f> rightTagsPnts2f;
	 for (size_t k = 0; k < LeftmarkerResults.size(); k++)
	 {
		 TagPoint2f  LeftTagPnts, RightTagPnts;
		 LeftTagPnts[0] = LeftmarkerResults[k].first;
		 LeftTagPnts[1] = LeftmarkerResults[k].second.x;
		 LeftTagPnts[2] = LeftmarkerResults[k].second.y;
		 RightTagPnts[0] = RightmarkerResults[k].first;
		 RightTagPnts[1] = RightmarkerResults[k].second.x;
		 RightTagPnts[2] = RightmarkerResults[k].second.y;
		 leftTagsPnts2f.push_back(LeftTagPnts);
		 rightTagsPnts2f.push_back(RightTagPnts);
	 }
	 ////��������ı�ǩ��Ϣȥ��
	 vector<Point2f> leftPnts2f, rightPnts2f;
	 vector<int> TagVec;
	 pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);

	 ////������������������ϵ�µ���ά����Ϣ
	 vector<Point3f> pnts3fVec;
	 //��һ�ּ�����ά����Ϣ�ķ���
	 RT CAMG;
	 Mat rotation = Mat::zeros(3, 1, CV_64FC1);
	 Mat translation = Mat::zeros(3, 1, CV_64FC1);
	 for (int i = 0; i < 3; i++)
	 {
		 rotation.at<double>(i, 0) = _camgrouppara.right2LeftRotVector[i];
		 translation.at<double>(i, 0) = _camgrouppara.right2LeftTraVector[i];
	 }
	 CAMG.R = rotation; CAMG.T = translation;
	 vector<Point3f> pntsl;
	 bool isok;
	 isok = Cal3dPoint(leftPnts2f, _campara1, rightPnts2f, _campara2, CAMG.R, CAMG.T, pntsl);
	 vector<TagPoint3f> result;
	 addPntsTag(pntsl, TagVec, result);
	 TagPnt_Third = result;
	// cout << TagPnt_Third[0] << endl;
	 return true;

 }




 /////����getCodePoint3f()����
 bool CoreAlgorithm::getCodePoint3f(const Mat imgLeft, const CamPara _campara1, const Mat imgRight, const CamPara _campara2,
	 const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& tagPnts3f,
	 Point2f &LightPenCentroidPntLeft, Point2f &LightPenCentroidPntRight, float &CircleDiameter, vector<pair<unsigned int, Point2f>> &markers_Left, vector<pair<unsigned int, Point2f>> &markers_Right)
 {
	 ////////��ʶ�������и���������������ꡣ

	 cv::Mat cameraMatrixLeft = (cv::Mat_<double>(3, 3) << _campara1.CameraIntrinsic[0][0], _campara1.CameraIntrinsic[0][1], _campara1.CameraIntrinsic[0][2]
		 , _campara1.CameraIntrinsic[1][0], _campara1.CameraIntrinsic[1][1], _campara1.CameraIntrinsic[1][2]
		 , _campara1.CameraIntrinsic[2][0], _campara1.CameraIntrinsic[2][1], _campara1.CameraIntrinsic[2][2]);
	 cv::Mat cameraMatrixRight = (cv::Mat_<double>(3, 3) << _campara2.CameraIntrinsic[0][0], _campara2.CameraIntrinsic[0][1], _campara2.CameraIntrinsic[0][2]
		 , _campara2.CameraIntrinsic[1][0], _campara2.CameraIntrinsic[1][1], _campara2.CameraIntrinsic[1][2]
		 , _campara2.CameraIntrinsic[2][0], _campara2.CameraIntrinsic[2][1], _campara2.CameraIntrinsic[2][2]);

	 //step-1 ������������Ķ�ά������㼯
	 vector<Point3f> pntsw;
	 pntsTagConfirm(tagPnts3fw, pntsw);
	 ///////���ñ����ʶ���㷨���ʶ�����ͼ�б���������
	 vector<pair<unsigned int, Point2f>> LeftmarkerResults, RightmarkerResults;
	 if (!CoreAlgorithm::findCircularMarker(imgLeft, LeftmarkerResults))
	 {
		 return false;
	 }
	 ////��ʶ����ı�����������
	 if (LeftmarkerResults.size() == 3)
	 {
		 markers_Left = LeftmarkerResults;
	 }
	 else
	 {
		 return false;
	 }


	 ///////���ñ����ʶ���㷨���ʶ�����ͼ�б���������
	 if (!CoreAlgorithm::findCircularMarker(imgRight, RightmarkerResults))
	 {
		 return false;
	 }

	 if (!ClpMeasurement::calculateLightPenCentroid(imgRight, LightPenCentroidPntRight))
	 {
		 return false;
	 }
	 if (RightmarkerResults.size() == 3)
	 {
		 markers_Right = RightmarkerResults;
	 }
	 else
	 {
		 return false;
	 }

	 if (!ClpMeasurement::calculateLightPenCentroid(imgLeft, LightPenCentroidPntLeft, CircleDiameter))
	 {
		 return false;
	 }
	 /////��ʶ����ı������������У����ں�������
	 vector<TagPoint2f> leftTagsPnts2f;
	 vector<TagPoint2f> rightTagsPnts2f;
	 for (size_t k = 0; k < LeftmarkerResults.size(); k++)
	 {
		 TagPoint2f  LeftTagPnts, RightTagPnts;
		 LeftTagPnts[0] = LeftmarkerResults[k].first;
		 LeftTagPnts[1] = LeftmarkerResults[k].second.x;
		 LeftTagPnts[2] = LeftmarkerResults[k].second.y;
		 RightTagPnts[0] = RightmarkerResults[k].first;
		 RightTagPnts[1] = RightmarkerResults[k].second.x;
		 RightTagPnts[2] = RightmarkerResults[k].second.y;
		 leftTagsPnts2f.push_back(LeftTagPnts);
		 rightTagsPnts2f.push_back(RightTagPnts);
	 }
	 ////��������ı�ǩ��Ϣȥ��
	 vector<Point2f> leftPnts2f, rightPnts2f;
	 vector<int> TagVec;
	 pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);

	 ////������������������ϵ�µ���ά����Ϣ
	 vector<Point3f> pnts3fVec;
	 //��һ�ּ�����ά����Ϣ�ķ���
	 RT CAMG;
	 Mat rotation = Mat::zeros(3, 1, CV_64FC1);
	 Mat translation = Mat::zeros(3, 1, CV_64FC1);
	 for (int i = 0; i < 3; i++)
	 {
		 rotation.at<double>(i, 0) = _camgrouppara.right2LeftRotVector[i];
		 translation.at<double>(i, 0) = _camgrouppara.right2LeftTraVector[i];
	 }
	 CAMG.R = rotation; CAMG.T = translation;
	 vector<Point3f> pntsl;
	 bool isok;
	 isok = Cal3dPoint(leftPnts2f, _campara1, rightPnts2f, _campara2, CAMG.R, CAMG.T, pntsl);
	 vector<TagPoint3f> result;
	 addPntsTag(pntsl, TagVec, result);
	 tagPnts3f = result;
	 
	 return true;
 }

 //////




///ended