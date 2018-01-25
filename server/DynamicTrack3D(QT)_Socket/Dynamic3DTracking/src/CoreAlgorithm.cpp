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
	//相机参数及图片外参数赋值
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
	//covariance的前8个位置用来存放内参数协方差
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

		//计算重投影误差和导数？
		//矩阵大小 2Nx(10+<numDistCoeffs>)，这里numDistCoeffs=4
		//所以大小为 2N x 14，按照drot dt df dc ddist的顺序进行排列
		//在c接口中，雅克比矩阵被放在几个分开的部分中
		Mat jacobian;
		projectPoints(perframe_objectpnt
			,rot_vector_perframe,tra_vector_perframe
			,cam_intr_para,distCoeffs,
			perframe_imagepnt,jacobian);
		//将重投影得到的计算角点保存起来
		error_image_points.push_back(perframe_imagepnt);

		//将drot dt放到doutpara中
		for( int j=0;j<6;j++ )
		{
			jacobian.col(j).copyTo(dpoutpara_perframe.col(j));
		}
		//将df dc ddist合并到dpinpara中
		for( int j=0;j<8;j++ )
		{
			jacobian.col(j+6).copyTo(dpinpara_perframe.col(j));
		}

		//求协方差矩阵
		// outpara_covariance[6X6] = dpoutpara_perframe(2N*6  转置)＊dpoutpara_perframe(2N*6)
		mulTransposed(dpoutpara_perframe, outpara_covariance, 1); 
		// inpara_covariance[8X8] = dpinpara_perframe(2N*8  转置)＊dpinpara_perframe(2N*8)
		mulTransposed(dpinpara_perframe, inpara_covariance, 1);  
		// inpara_outpara_covariance[8X6] = dpinpara_perframe[2NX8 转置]＊dpoutpara_perframe(2N*6)
		gemm(dpinpara_perframe,dpoutpara_perframe,1,0,0,inpara_outpara_covariance,CV_GEMM_A_T);

		//更新本帧的协方差矩阵
		for( int row=0;row<8;row++ )
		{
			for( int col=0;col<8;col++ )
			{
				//内参数的协方差不断叠加
				covariance.at<double>(row,col) += inpara_covariance.at<double>(row,col);
			}
		}
		for (int row = 0; row < 8; row++)
		{
			for (int col = 0; col < 6; col++)
			{
				//填充inpara_outpara_covariance转置，前8个位置除外
				covariance.at<double>( row , col+8+i*6) = inpara_outpara_covariance.at<double>(row,col);
				covariance.at<double>( col+8+i*6, row) = inpara_outpara_covariance.at<double>(row,col);
			}
		}
		for (int row = 0; row < 6; row++)
		{
			for (int col = 0; col < 6; col++)
			{
				//在剩下的位置填充outpara_covariance
				covariance.at<double>( row+8+i*6 , col+8+i*6 ) = outpara_covariance.at<double>(row,col);
			}
		}
	}
	//完成for循环以后，将初始角点和计算角点相减，获得差值
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
//	//计算所有点的误差平均值和标准差
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
	//计算标定参数误差，3倍标准差，这里是怎么算的？
	vector<double> para_error;
	Mat inver_converiance(covariance.rows,covariance.cols,CV_64FC1);
	invert(covariance,inver_converiance);
	Mat diag_covariance(inver_converiance.rows,1,CV_64FC1);
	diag_covariance = inver_converiance.diag(0);//取主对角线

	for( int row=0;row<diag_covariance.rows;row++ )
	{
		//为什么只取val[0]???
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
	//opencv3.0以下版本
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
	
	///////利用编码点识别算法检测识别出图中编码点的坐标
	////先识别出编码标志点中心
	vector<pair<unsigned int, Point2f>>  markerResults;
	bool leftResult = CoreAlgorithm::findCircularMarker(imgMat, markerResults);
    ////将编码点用最小包围圆形将其包围，得到圆心以及半径。
	vector<Point2f>   markerPnts;
	for (size_t i = 0; i < markerResults.size(); i++)
	{
		markerPnts.push_back(markerResults[i].second);
	}
	
	///定义最小包围圆形的圆心
	Point2f center_minAreaRectImg;
	float radius_minAreaRectImg;
	minEnclosingCircle(markerPnts, center_minAreaRectImg, radius_minAreaRectImg);
	/////将最小包围圆形用黑色像素涂黑；
	Mat imgMat1;
	imgMat.copyTo(imgMat1);
	circle(imgMat1, center_minAreaRectImg, radius_minAreaRectImg*1.3, Scalar::all(0),-1);

	float width_height_ratio = 4;
	Mat img_threshold;
	//STEP-1:二值化
	//blur(imgMat1,img_threshold,Size(8,8),Point(-1,-1));
	//imgMat1.copyTo(img_threshold);
	//double minvalue,maxvalue;
	//cv::minMaxLoc(img_threshold,&minvalue,&maxvalue);
	//step-3 高置信梯度区域的选择
	//step-3-1 针对求出的梯度矩阵进行二值化
	//int thresholdvalue = (int)(minvalue+maxvalue)/2;
	threshold(imgMat1, img_threshold, 100, 255, cv::THRESH_BINARY);
	//在处理佳能相机里当感光度不同时，设置的阈值也不同。ISO1600为50合适。
	///// 自适应二值化
	/* int block_size = 145;
	cv::Scalar meanvalue = cv::mean(img_threshold);
	int C_threshold = int(-1*(abs(2*meanvalue.val[0])+13));
	adaptiveThreshold(img_threshold,img_threshold,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,block_size,C_threshold);*/
	//CV_ADAPTIVE_THRESH_MEAN_C:使用3x3方格中像素的平均值减去5来作为自适应的阈值
	//对白色区域膨胀再腐蚀
	//int close_type = MORPH_ELLIPSE;
	//int dilate_size = 1;
	// Mat element = getStructuringElement(close_type,Size(2*dilate_size+1,2*dilate_size+1),Point(-1,-1));
	//erode(img_threshold, img_threshold, element,Point(-1, -1),1);
	//dilate(img_threshold, img_threshold, element,Point(-1, -1),1);//开运算
	//STEP-2：寻找轮廓
	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;
	findContours(img_threshold, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	////////////////////
	////////qiyong added 将所有椭圆求出其中心
	vector<RotatedRect> data;
	if (!contours.size())
	{
		return;
	}
	else
	{
		///////STEP4:用椭圆拟合二维点集
		for (int i = 0; i < contours.size(); i++)
		{
			data.push_back(fitEllipse(contours[i]));
		}

	}
	/////ended
	/////

	//////对初次检测出的多个中心坐标进行筛选，找到光笔的八个靶标圆中心
	
	///////////////////////
	//STEP-3:椭圆拟合,找中心,并进行筛选
	RotatedRect rotatedBox;
	//vector<RotatedRect> rotatedBoxVec;
	//vector<RotatedRect> temp_rotatedBoxVec;
	for (vector<vector<Point>>::iterator itr = contours.begin(); itr != contours.end(); ++itr)
	{
		static int n = 0;
		//根据圆特征的封闭性进行滤波操作
		int distanceToHead = abs(itr->at(itr->size() - 1).x + itr->at(itr->size() - 1).y - itr->at(0).x - itr->at(0).y);
		if (itr->size()<5 || itr->size()>500 || distanceToHead>4)//此处需要根据工作最远离和工作最近距离以及圆斑真实大小确定其阈值
			//if(itr->size()<10||itr->size()>500)//此处需要根据工作最远离和工作最近距离以及圆斑真实大小确定其阈值
			continue;
		try
		{
			rotatedBox = fitEllipse((*itr));
			//////qiyong added 绘制椭圆
			//cv::ellipse(img_threshold, rotatedBox, Scalar(0, 0, 255), 1);
			//cv::namedWindow("rightResult", WINDOW_NORMAL);
			//cv::imshow("rightResult", img_threshold);
			//cv::waitKey(600);
		}
		catch (...)//三个点表示任何异常
		{
			continue;
		}
		//temp_rotatedBoxVec.push_back(rotatedBox);
		float height = rotatedBox.size.height;
		float width = rotatedBox.size.width;
		//根据轮廓长度判断其是否为椭圆
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
		//如果光斑点的形状不规则的话，这个比例可能要大一点
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
	//STEP-4 对检测出的中心点进行过滤和剔除

}



bool CoreAlgorithm::detectLightSpot_LED2(const Mat img_threshold, vector<Point2f>& centerPnt, vector<float>& longaxisRadius)
{

	float width_height_ratio = 4;

	//STEP-1:二值化

	//在处理佳能相机里当感光度不同时，设置的阈值也不同。ISO1600为50合适。
	//STEP-2：寻找轮廓
	vector<vector<cv::Point>> contoursOrigin;
	vector<cv::Vec4i> hierarchy;
	findContours(img_threshold, contoursOrigin, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	////////////////////
	////////qiyong added 将所有椭圆求出其中心
	vector<RotatedRect> data;
	if (!contoursOrigin.size())
	{
		return false;
	}
	else
	{
		///////STEP4:用椭圆拟合二维点集
		for (int i = 0; i < contoursOrigin.size(); i++)
		{
			if (contoursOrigin[i].size()>=5)
			{
				data.push_back(fitEllipse(contoursOrigin[i]));
			}
			
		}
	}
	/////将所有拟合好的椭圆用有颜色的线条绘制出来  
	/////先将单通道的imgMat图片转换成为三通道的图片
	//Mat colorImg;
	//cv::cvtColor(img_threshold, colorImg, CV_GRAY2BGR);
	//for (size_t j = 0; j < data.size(); j++)
	//{
	//	cv::ellipse(colorImg, data[j], Scalar(0, 255, 0), 1);
	//	////对其中的椭圆圆心添加数字编号
	//	string Num = to_string(j);
	//	cv::putText(colorImg, Num, Point(data[j].center.x, data[j].center.y), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0));

	//}
	//////既然已经知道编码点的中心圆的坐标，也已经将图中的所有圆心都已经知道，
	//////那么只要将容器中的22个中心坐标点通过约束条件删除掉便可以了。
	/////先将各个圆心的坐标放到一个vector容器中
	vector<Point2f>  CenterPnts;
	for (size_t j = 0; j < data.size(); j++)
	{
		CenterPnts.push_back(data[j].center);
	}

	////定义一个用来存放任意两点之间距离的容器
	Mat distanceMatrix1 = Mat::zeros(CenterPnts.size(), CenterPnts.size(), CV_64F);
	Mat distanceMatrix = Mat::zeros(CenterPnts.size(), CenterPnts.size(), CV_64F);
	////step-2 计算椭圆圆心之间的距离，将其存储在矩阵中。
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
			if (distanceMatrix.at<double>(i, j)<40&&distanceMatrix.at<double>(i, j)>0) ////此处的可取范围为：30~80
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

	//////对初次检测出的多个中心坐标进行筛选，找到光笔的八个靶标圆中心
    ///////////////////////
	//STEP-3:椭圆拟合,找中心,并进行筛选
	RotatedRect rotatedBox;
	//vector<RotatedRect> rotatedBoxVec;
	//vector<RotatedRect> temp_rotatedBoxVec;
	for (vector<vector<Point>>::iterator itr = contours.begin(); itr != contours.end(); ++itr)
	{
		static int n = 0;
		//根据圆特征的封闭性进行滤波操作
		int distanceToHead = abs(itr->at(itr->size() - 1).x + itr->at(itr->size() - 1).y - itr->at(0).x - itr->at(0).y);
		if (itr->size()<5 || itr->size()>500 || distanceToHead>4)//此处需要根据工作最远离和工作最近距离以及圆斑真实大小确定其阈值
			//if(itr->size()<10||itr->size()>500)//此处需要根据工作最远离和工作最近距离以及圆斑真实大小确定其阈值
			continue;
		try
		{
			rotatedBox = fitEllipse((*itr));
			
		}
		catch (...)//三个点表示任何异常
		{
			continue;
		}
		//temp_rotatedBoxVec.push_back(rotatedBox);
		float height = rotatedBox.size.height;
		float width = rotatedBox.size.width;
		//根据轮廓长度判断其是否为椭圆
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
		//如果光斑点的形状不规则的话，这个比例可能要大一点
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
	//STEP-4 对检测出的中心点进行过滤和剔除
}


bool CoreAlgorithm::detectLightSpot_LED(const Mat& imgMat,vector<Point2f>& centerPnt, vector<float>& longaxisRadius)
{

	float width_height_ratio = 4;
	Mat img_threshold;
	//STEP-1:二值化
	//blur(imgMat,img_threshold,Size(8,8),Point(-1,-1));
	//imgMat.copyTo(img_threshold);
	//double minvalue,maxvalue;
	//cv::minMaxLoc(img_threshold,&minvalue,&maxvalue);
	//step-3 高置信梯度区域的选择
	//step-3-1 针对求出的梯度矩阵进行二值化
	//int thresholdvalue = (int)(minvalue+maxvalue)/2;
	threshold(imgMat, img_threshold, 80, 255, cv::THRESH_BINARY);
	//在处理佳能相机里当感光度不同时，设置的阈值也不同。ISO1600为50合适。
	///// 自适应二值化
	/* int block_size = 145;
	cv::Scalar meanvalue = cv::mean(img_threshold);
	int C_threshold = int(-1*(abs(2*meanvalue.val[0])+13));
	adaptiveThreshold(img_threshold,img_threshold,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,block_size,C_threshold);*/
	//CV_ADAPTIVE_THRESH_MEAN_C:使用3x3方格中像素的平均值减去5来作为自适应的阈值
	//对白色区域膨胀再腐蚀
	//int close_type = MORPH_ELLIPSE;
	//int dilate_size = 1;
	// Mat element = getStructuringElement(close_type,Size(2*dilate_size+1,2*dilate_size+1),Point(-1,-1));
	//erode(img_threshold, img_threshold, element,Point(-1, -1),1);
	//dilate(img_threshold, img_threshold, element,Point(-1, -1),1);//开运算
	//STEP-2：寻找轮廓
	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;
	findContours(img_threshold, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	if (contours.size() == 8)
	{
		//STEP-3:椭圆拟合,找中心,并进行筛选
		RotatedRect rotatedBox;
		//vector<RotatedRect> rotatedBoxVec;
		//vector<RotatedRect> temp_rotatedBoxVec;
		for (vector<vector<Point>>::iterator itr = contours.begin(); itr != contours.end(); ++itr)
		{
			static int n = 0;
			//根据圆特征的封闭性进行滤波操作
			int distanceToHead = abs(itr->at(itr->size() - 1).x + itr->at(itr->size() - 1).y - itr->at(0).x - itr->at(0).y);
			if (itr->size()<5 || itr->size()>500 || distanceToHead > 4)//此处需要根据工作最远离和工作最近距离以及圆斑真实大小确定其阈值
				//if(itr->size()<10||itr->size()>500)//此处需要根据工作最远离和工作最近距离以及圆斑真实大小确定其阈值

				continue;

			try
			{ 
				rotatedBox = fitEllipse((*itr));
			}
			catch (...)//三个点表示任何异常
			{
				continue;
			}
			//temp_rotatedBoxVec.push_back(rotatedBox);
			float height = rotatedBox.size.height;
			float width = rotatedBox.size.width;
			//根据轮廓长度判断其是否为椭圆
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
			//如果光斑点的形状不规则的话，这个比例可能要大一点
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
	//STEP-4 对检测出的中心点进行过滤和剔除

}




bool CoreAlgorithm::Cal3dPointStrLight(const vector<Point2f>& pointCam,const vector<float>& proImg_x,const CamPara& camParaLeft, const CamPara& camParapro,const double rotVector[3] ,const double traVector[3] ,vector<Point3f>& point3d)
{

	point3d.clear();
	//// 正则化　摄像机图像坐标
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
	Kp.at<float>(0,0) = float(camParapro.CameraIntrinsic[0][0]);	//// 注意赋值的时候 at 坐标先y，后x
	Kp.at<float>(1,0) = float(camParapro.CameraIntrinsic[1][0]);	
	Kp.at<float>(2,0) = float(camParapro.CameraIntrinsic[2][0]);	
	Kp.at<float>(0,1) = float(camParapro.CameraIntrinsic[0][1]);	
	Kp.at<float>(1,1) = float(camParapro.CameraIntrinsic[1][1]);	
	Kp.at<float>(2,1) = float(camParapro.CameraIntrinsic[2][1]);	
	Kp.at<float>(0,2) = float(camParapro.CameraIntrinsic[0][2]);	
	Kp.at<float>(1,2) = float(camParapro.CameraIntrinsic[1][2]);	
	Kp.at<float>(2,2) = float(camParapro.CameraIntrinsic[2][2]);	

	//获得Mat类型的R、T
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
	//// 赋值平移向量
	for(int i=0;i<3;i++)
	{
		Kt.at<float>(i,0) =  float(traVector[i]);
	}
	Kt = Kp*Kt; /// 构造出P矩阵的第四列


	///// 计算三维坐标
	for (unsigned int i=0;i<NormPixCam.size();i++)
	{
		/// 给定系数矩阵，保存在Kp　　３Ｘ３;给定等式右边向量　保存在Rvec　３Ｘ１	
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

		invert(Kp,inverA,cv::DECOMP_SVD);//求逆矩阵
	
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

	//// 正则化　摄像机图像坐标
	normalizPixel(pointLeft,NormPixLeft,Lfc, Lcc, Lkc, Lalfpha_c);
	//// 正则化　投影机图像坐标
	normalizPixel(pointRight,NormPixRight,Rfc, Rcc, Rkc, Ralfpha_c);

	Mat Kp(3,3,CV_32FC1);

	//获得Mat类型的R、T
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

	//计算三维坐标,使用right相机的x坐标和y坐标
	for (unsigned int i=0;i<NormPixLeft.size();i++)
	{
		//给定系数矩阵，保存在Kp　３Ｘ３
		float temp = NormPixRight[i].x * T.at<float>(2,0) - T.at<float>(0,0);//这个值是什么含义？

		Kp.at<float>(0,0) = 1; Kp.at<float>(0,1) = 0; Kp.at<float>(0,2) = -NormPixLeft[i].x;
		Kp.at<float>(1,0) = 0; Kp.at<float>(1,1) = 1; Kp.at<float>(1,2) = -NormPixLeft[i].y;

		Kp.at<float>(2,0) = (R.at<float>(0,0) - NormPixRight[i].x * R.at<float>(2,0))/temp; 
		Kp.at<float>(2,1) = (R.at<float>(0,1) - NormPixRight[i].x * R.at<float>(2,1))/temp; 
		Kp.at<float>(2,2) = (R.at<float>(0,2) - NormPixRight[i].x * R.at<float>(2,2))/temp; 

		Rvec.at<float>(2,0) = 1;		

		Mat inverA(3,3,CV_32FC1);
		Mat point(3,1,CV_32FC1);

		invert(Kp,inverA);//求逆矩阵

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
        //cwq图像像素坐标到图像物理坐标的转化
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
            for (int j=0;j<400;j++)///迭代400次求解非畸变量
            {
                r2 = temp[i].x * temp[i].x + temp[i].y * temp[i].y;
                //径向
                k_radial = 1 + kc[0]*r2 + kc[1]*r2*r2 + kc[4]*r2*r2*r2;
                //切向
                delta_x = 2*kc[2]*temp[i].x*temp[i].y + kc[3]*(r2+2*temp[i].x*temp[i].x);
                delta_y = kc[2]*( r2 + 2*temp[i].y*temp[i].y ) + 2*kc[3]*temp[i].x*temp[i].y;
                //畸变校正公式
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

	tempPnt = imgpntlefttop;//cwq标注，tempPnt用来保存要push的点
	leftPnt = imgpntlefttop;
	rightPnt = imgpntrighttop;

	vector<Point2f> col_Pnt;// 行角点
	for (int row=1;row<r+1;row=row+1)
	{	
		//col_Pnt.clear();
		for (int col=1;col<c+1;col=col+1)
		{
			//保存该点到行
			//col_Pnt.push_back(tempPnt);
			ImageCorner.push_back(tempPnt);
			//cwq标注，计算边长的x分量和y分量
			rowLength.x = rightPnt.x-leftPnt.x;
			rowLength.y = rightPnt.y-leftPnt.y;
			//cwq标注，行的初始点根据列数加上点间隔
			tempPnt.x = int ((float)col/(float)(c-1)*(float)rowLength.x)+leftPnt.x;
			tempPnt.y = int ((float)col/(float)(c-1)*(float)rowLength.y)+leftPnt.y;
		}
		//ImageCorner.push_back(col_Pnt);///把整行保存
		if (row<r-1)//如果不是最后一行
		{
			//cwq标注，移动行的左右端点到下一行
			leftPnt.x = int ((float)row/(float)(r-1)*(float)colLength_l.x)+imgpntlefttop.x;
			leftPnt.y = int ((float)row/(float)(r-1)*(float)colLength_l.y)+imgpntlefttop.y;
			rightPnt.x = int ((float)row/(float)(r-1)*(float)colLength_r.x)+imgpntrighttop.x;
			rightPnt.y = int ((float)row/(float)(r-1)*(float)colLength_r.y)+imgpntrighttop.y;			
		}
		else//cwq标注，如果是最后一行
		{
			leftPnt = imgpntleftdown;
			rightPnt = imgpntrightdown;
		}
		
		tempPnt = leftPnt;		
	}
	//*********************************************************************************//

	//亚像素角点提取
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
		row = (int)cornercol_l.size();    // 获得角点行方向个数
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
			col = ImageCorner.size();				// 获得角点列方向个数
	}

	if (ImageCorner.size() != HEIGHT_CORNER_COUNT*WIDTH_CORNER_COUNT)
		return false;

	cornerSubPix(ImageGray,ImageCorner, cvSize(10, 10), cvSize(-1, -1)
		,cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 60, 0.01));

	////为了与自动提取的焦点顺序相符，翻转corner的顺序
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
	double threhold;    // 自适应确定阈值，根据前两个格子确定
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

	//cwq防止两点重合
	if( 0 == length)
	{
		return false;
	}
	dir[0] = dir[0] / length;//cwq  sin\cos
	dir[1] = dir[1] / length;
	// 设定初始标志
	pixel = 0;
	//cwq 循环获得25个像素值，用来求平均阈值
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j > -5; j--)
		{
			// 在dir垂直的方向上加3个，所以反过来了。另外，图像上step相当于y
			//3/22 cwq检查是超出图像边缘
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
			// 在dir垂直的方向上加3个，所以反过来了。另外，图像上step相当于y
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

	// 寻找角点
	for (int l = 0;l < length + 5; l++)
	{
		Point2f presentpnt;
		//cwq根据sin、cos获得扫描直线上的下一点
		presentpnt.x = float(imgpnt_left.x + l * dir[0]);
		presentpnt.y = float(imgpnt_left.y + l * dir[1]);
		pixel = 0;
		for (int i = 0;i < 5; i++)
		{
			for (int j = 0;j < 5; j++)
			{
				// 在dir垂直的方向上加3个，所以反过来了。另外，图像上step相当于y
				pixel = pixel+ImageGray_data[(int)(presentpnt.y + 10 * dir[0] + j) * step + (int)(presentpnt.x - 10 * dir[1] + i) * chennels];
			}
		}
		if (pixel / 25 < threhold && signwb == TRUE)
		{
			signwb = FALSE;
			cornerrow.push_back(presentpnt);
			// 进行最大值抑制，跳跃一个步长
			l = l + 4;
			continue;
		}
		if (pixel / 25 > threhold && signwb == FALSE)
		{
			signwb = TRUE;
			cornerrow.push_back(presentpnt);
			// 进行最大值抑制，跳跃一个步长
			l = l + 5;
		}
	}

	return TRUE;
}



bool CoreAlgorithm::rigidTransform(const std::vector<Point3f>& oriPoints,
					const std::vector<Point3f>& terminatePoints,
					cv::Mat& Rot,cv::Mat& Tra)
{
	//检查输入点的数目是否相同,且不为0
	if( oriPoints.size()<3 || terminatePoints.size()<3 )
		return FALSE;
	if( oriPoints.size() != terminatePoints.size() )
		return FALSE;
	
	//获得点的总数
	int pointNum = oriPoints.size();
	vector<Point3f> X1 = oriPoints;
	vector<Point3f> X2 = terminatePoints;

	//求起始点和终止点的重心
	Point3f C1(0,0,0),C2(0,0,0);
	for( int i=0; i<pointNum; i++ )
	{
		C1 += X1[i];
		C2 += X2[i];
	}
	C1.x = C1.x/pointNum;	C2.x = C2.x/pointNum;
	C1.y = C1.y/pointNum;	C2.y = C2.y/pointNum;
	C1.z = C1.z/pointNum;	C2.z = C2.z/pointNum;

	//原始点减去重心点坐标，获得新的点
	for( int i=0; i<pointNum; i++ )
	{
		X1[i] -= C1;
		X2[i] -= C2;
	}

	//创建N矩阵
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

	//对N矩阵赋值
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

	//计算N矩阵的特征向量和特征值
	vector<double> eigenvalues;
	Mat eigenvectors;
	if( !eigen(N,eigenvalues,eigenvectors) )
		return FALSE;

	//求最大正特征值对应的特征向量
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

	//创建旋转矩阵
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

	//计算平移矩阵
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

	//STEP-1:长轴点直线拟合
	Vec4f L_Axis;//(xx,xy)前两项为与直线平行的单位向量，(x0,y0)后两项为直线上的一个点
	float xx, xy, x0, y0;
	fitLine(L_pnts, L_Axis, CV_DIST_L2, 0, 0.01, 0.01);//最小二乘拟合
	xx = L_Axis[0]; xy = L_Axis[1]; x0 = L_Axis[2]; y0 = L_Axis[3];

	////// zhang xu added  判断直线的方向是不是1-5点的方向
	int l_num = L_pnts.size() - 1;
	float pnt15x,pnt15y;
	pnt15x = L_pnts[l_num].x - L_pnts[0].x;
	pnt15y = L_pnts[l_num].y - L_pnts[0].y;
	float dis = pnt15x*xx + pnt15y*xy;
	if(dis<0)  //// 如果L_pnts顺序 不是从1到5的顺序，则反向，
	{
		xx = -xx;
		xy = -xy;
	}
	////// end


	//STEP-2:求出十字交叉点（虚），求垂足算法网上拷的，还没看懂和测试
	Point2f crossPnt;
	Point2f interceptPnt;//长轴直线与Y轴角点
	interceptPnt.x = 0; interceptPnt.y = y0 - (xy/xx) * x0;//y1=y0-(xy/xx)*x0;

	Point2f pt1, pt2, pt3;//pt1,pt2为直线上两点，pt3为直线外一点
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

	//STEP-3:短轴上的点，在十字交叉点左边就是tag6，右边就是tag7
	////// zhang xu added  判断长轴直线的方向 与 tag6  tag7  的方向 叉乘  是不是1-5点的方向
	Point2f p0;  //// 短轴上的点
	int signShort = 1; ////1表示p0选择的是 tag5， -1 表示p0选择的是tag6
	p0 = S_pnts[0];

	
	//向量AB＝（x1,y1,z1）, 向量CD＝（x2,y2,z2）
	//向量AB×向量CD＝（y1z2-z1y2，z1x2-x1z2，x1y2-y1x2）
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

		if(norm_Z[2]>0 && i==0) //// 叉乘后的Z等效于Z轴方向，则说明p0 是tag6， 否则其为tag7
		{
			_tagPnt[0] = TAG6;
			tagPnts.push_back(_tagPnt);
		}
		if(norm_Z[2]>0 && i==1) ////叉乘后的Z等效于Z轴方向， 第二个点是Tag7
		{
			_tagPnt[0] = TAG7;
			tagPnts.push_back(_tagPnt);
		}
		if(norm_Z[2]<0 && S_pnts.size()==1)   ///// 叉乘后与z方向，且S_pnts.size()  只有1个则其必然是TAG7
		{
			_tagPnt[0] = TAG7;
			tagPnts.push_back(_tagPnt);
		}

		//if (S_pnts[i].x < crossPnt.x)//在十字交叉左边
		//	_tagPnt[0] = TAG6;
		//else//在十字交叉右边
		//	_tagPnt[0] = TAG7;

	}
	//step-4 

	////// zhangxu added
	float unit_dis;//短轴上某点到十字交叉点的距离
	unit_dis = sqrt((S_pnts[0].x - crossPnt.x)*(S_pnts[0].x - crossPnt.x)
		+ (S_pnts[0].y - crossPnt.y)*(S_pnts[0].y - crossPnt.y));
	
	//判断长轴中有无十字交叉点（实），有，该点为tag1点//chengwei changed
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
	//// 点与 cross point 形成的向量与  xx  xy 方向一致，则其为 tag1 否则 为 tag3 tag4  tag5
	//点与 cross point 形成的向量与  xx  xy 方向相反，则其为 为 tag2 tag3 tag4  tag5 //chengwei changed
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

		Point2f pnt2cross; //// 由点到cross 构成的点
		pnt2cross.x = crossPnt.x - L_pnts[i].x;
		pnt2cross.y = crossPnt.y - L_pnts[i].y;

		double dis = pnt2cross.x*xx + pnt2cross.y*xy;  /// 方向向量点乘

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
//是否存在这种情况？   //chengwei changed
		//if (dis>0 && _k>0.5) ////tag1--》cross  是与向量xx  xy 方向一致，所以其为正,则为tag1
		//{
		//	_tagPnt[0] = TAG1;
		//	tagPnts.push_back(_tagPnt);
		//}
		//else ////否则 为 tag3 tag4  tag5
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
	///// 对标记好的点，按照标记点进行排序
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
	//step-2 计算椭圆中心之间的距离并除以主椭圆长轴半径，将其存储在矩阵中
	for(int i = 0;i<pntsSize;i++)
	{
		for(int j=i+1;j<pntsSize;j++)
		{
			distanceMatrix1.at<double>(i,j) = distancePoints2f(centerPnt[i],centerPnt[j])/(longaxisRadius[i]*gamma);
		}
	}
	Mat distanceMatrix1_T = distanceMatrix1.t();
	cv::add(distanceMatrix1,distanceMatrix1_T,distanceMatrix);
	//step-3 从矩阵中的找出存在三个元素值在0.7-1.3之间的行数及相应元素所对应的行标及列标
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
			//判断1267点
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
				//根据求得12两点确定的长轴直线，求出长轴上的点
			float dis_threshold = 10;
			vector<Point2f> longAxisPntsVec;
			float a, b;//直线公式ax+by+1=0;a=(y1-y2)/(x1y2-x2y1),b=(x1-x2)/(x2y1-x1y2);
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
				//点到直线的距离d=fabs(a*x+b*y+1)/sqrt(a*a+b*b);

				float dis = float(fabs(a*centerPnt[n].x + b*centerPnt[n].y + 1) / sqrt(a*a + b*b));
				axisVectorTemp.at<double>(0,0)= centerPnt[n].x-pnt1.x;
				axisVectorTemp.at<double>(1,0)= centerPnt[n].y-pnt1.y;
				double tempnum = ZaxisVector.dot(axisVectorTemp);
				if (dis < dis_threshold&&tempnum>0)//当中心找的不准的时候这个阈值要放宽一点
				{
					longAxisPntsVec.push_back(centerPnt[n]);
				}
			}
			//step-5 确定出3，4，5点
			float unit_dis;//2点到1点的距离作为比较单位距离
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
	// step-6对标记好的点，按照标记点进行排序
			//由小到大排序//chengwei added
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
	//STEP-1 找出不在长轴上的点,判断剩下的点是否在一条直线上
	for(int i=0;i<pntsSize;i++)
	{
		int count;//落在长轴上的点计数
		for (int j = i + 1; j < pntsSize; j++)
		{
			count = 0;	
			noisePntsIndexVec.clear();
			longAxisPntsIndexVec.clear();
			float a, b;//直线公式ax+by+1=0;a=(y1-y2)/(x1y2-x2y1),b=(x1-x2)/(x2y1-x1y2);
			Point2f pnt1 = pnts[i];
			Point2f pnt2 = pnts[j];
			a = (pnt1.y - pnt2.y) / (pnt1.x*pnt2.y - pnt2.x*pnt1.y);
			b = (pnt1.x - pnt2.x) / (pnt2.x*pnt1.y - pnt1.x*pnt2.y);

			for (unsigned int n = 0; n < pnts.size(); n++)
			{
				//点到直线的距离d=fabs(a*x+b*y+1)/sqrt(a*a+b*b);
				float dis = fabs(a*pnts[n].x + b*pnts[n].y + 1) / sqrt(a*a + b*b);
				if (dis < dis_threshold)//当中心找的不准的时候这个阈值要放宽一点
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

	//STEP-2 长轴上的点放到一个vector中;短轴上的点放到一个vector中
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

	//STEP-3 对长轴上的点按照y坐标从小到大排序,冒泡法
	sortByYMin2Max(longAxisPntsVec);
	
	//STEP-4 对短轴上点按照x坐标从小到大排序，冒泡法
	sortByXMin2Max(shortAxisPntsVec);

	////// zhangxu  added 为了避免出现长轴平行x轴的情况，需要对已经排好顺序的点进行，判断x坐标还是y坐标哪个更大。
	float tempx_len,tempy_len;
	int longAxisSize = longAxisPntsVec.size() -1;
	tempx_len = fabs(longAxisPntsVec[longAxisSize].x - longAxisPntsVec[0].x);
	tempy_len = fabs(longAxisPntsVec[longAxisSize].y - longAxisPntsVec[0].y);
	if(tempx_len>tempy_len) ///// 如果x方向的长度大于y方向的长度，说明长轴上的点分布是横着的，此时长轴需要按照x轴从小到大排序，短轴需要从大到小排序
	{
		sortByXMin2Max(longAxisPntsVec);
		sortByYMin2Max(shortAxisPntsVec);
		//// 然后反向排序
		reverse(shortAxisPntsVec);
	}
	///// end


	//STEP-5 判断排好序的长轴上的首末点，距离短轴中的第一点哪一个近
		//光笔颠倒标志位,
	bool isUpSideDown = false;
	
		//y最小点到短轴中点距离
	int dis_minY = int(sqrt((shortAxisPntsVec[0].x - longAxisPntsVec[0].x)*(shortAxisPntsVec[0].x - longAxisPntsVec[0].x)+
						(shortAxisPntsVec[0].y - longAxisPntsVec[0].y)*(shortAxisPntsVec[0].y - longAxisPntsVec[0].y)));
		//y最大点到短轴中点距离
	int dis_maxY = int(sqrt((shortAxisPntsVec[0].x - longAxisPntsVec[longAxisSize].x)*(shortAxisPntsVec[0].x - longAxisPntsVec[longAxisSize].x)+
						(shortAxisPntsVec[0].y - longAxisPntsVec[longAxisSize].y)*(shortAxisPntsVec[0].y - longAxisPntsVec[longAxisSize].y)));
		//y坐标最小的那个点距离短轴中点较远的话则认为光笔颠倒了
	if( dis_minY > dis_maxY )
		isUpSideDown = true;

	//STEP-6 如果光笔颠倒了则对短轴和长轴均重新排序
	if( isUpSideDown )
	{
		////// 将长轴 短轴上的点 次序颠倒
		reverse(longAxisPntsVec);
		reverse(shortAxisPntsVec);	
	}

	//STEP-7 将排好序的点输出
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
	//////  zhangxu added  对tagPnts 进行从小到大的排序
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

	///// zhang xu 修改，根据得到点是 tag6  还是tag7 计算Y方向时 是否取反
	double Ydir = 1;
	Point3f  pt1;//Y轴直线，pt0为直线上一点，pt1为直线外一点
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

	//STEP-1:长轴点的点，按照5点到1点方向进行直线拟合，获得Z轴单位向量
	cv::Vec6f Z_Line;
	fitLine(longAxisPnts, Z_Line, CV_DIST_L2, 0, 0.01, 0.01);//Z轴的方向
	Vec3f norm_Z;	
	norm_Z[0] = Z_Line[0];norm_Z[1] = Z_Line[1];norm_Z[2] = Z_Line[2];
	////// zhangxu  added 进行判断法线方向是否为从1点到5点的方向,5点减去1点坐标
	Vec3f norm_15;
	norm_15[0] = longAxisPnts[4].x - longAxisPnts[0].x;
	norm_15[1] = longAxisPnts[4].y - longAxisPnts[0].y;
	norm_15[2] = longAxisPnts[4].z - longAxisPnts[0].z;

	///// 点乘
	float temp = norm_Z[0]*norm_15[0]  + norm_Z[1]*norm_15[1]  + norm_Z[2]*norm_15[2];
	if(temp<0) //// 方向不相同，则添加负号
	{
		norm_Z = -1*norm_Z;
	}
	////// end
	float a, b, c;
	////a = Z_Line[0];b = Z_Line[1];c = Z_Line[2];
	a = norm_Z[0];b = norm_Z[1];c = norm_Z[2];

	//STEP-2:求经过短轴上的点6，且垂直于Z轴的单位向量，获得Y轴单位向量
	Vec3f norm_Y;
	Point3f perpendicularFoot;

	Point3f pt0;//pt0为直线上一点，pt1为直线外一点
	pt0 = Point3f(Z_Line[3],Z_Line[4],Z_Line[5]); 

	//pt1 = shortAxisPnts[0];  ////必须保证此点是第6点，这样定义，有可能不是第6点
	///// 因为其已经进行了排序，所以shortAxisPnts的第一个点是tagPnts   

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
	///// 判断 若为tag7，就应该反向  zhangxu added
	if(Ydir==-1)
	{
		norm_Y = -norm_Y;
	}
	/////// end
	
	//STEP-3:Z 叉乘 Y ，获得X轴单位向量   实际上 这个地方应该是 Y叉乘Z 得X
	//向量AB＝（x1,y1,z1）, 向量CD＝（x2,y2,z2）
	//向量AB×向量CD＝（y1z2-z1y2，z1x2-x1z2，x1y2-y1x2）
	Vec3f norm_X;
	norm_X[0] = norm_Z[1] * norm_Y[2] - norm_Z[2] * norm_Y[1];
	norm_X[1] = norm_Z[2] * norm_Y[0] - norm_Z[0] * norm_Y[2];
	norm_X[2] = norm_Z[0] * norm_Y[1] - norm_Z[1] * norm_Y[0];
	//////因为叉乘顺序反了，所以增加一个负号  zhangxu
	norm_X = -norm_X;

	//STEP-4:三个单位向量保存为R，测头中心点为T
	Mat T(3, 1, CV_32F); Mat R(3, 3, CV_32F);
	T.at<float>(0, 0) = probeCenter.x;
	T.at<float>(1, 0) = probeCenter.y;
	T.at<float>(2, 0) = probeCenter.z;

	R.at<float>(0, 0) = norm_X[0];R.at<float>(1, 0) = norm_X[1];R.at<float>(2, 0) = norm_X[2];
	R.at<float>(0, 1) = norm_Y[0];R.at<float>(1, 1) = norm_Y[1];R.at<float>(2, 1) = norm_Y[2];
	R.at<float>(0, 2) = norm_Z[0];R.at<float>(1, 2) = norm_Z[1];R.at<float>(2, 2) = norm_Z[2];
	
	Mat R_invert;
	invert(R, R_invert);
	//STEP-5:将测量坐标系下的点转化到光笔坐标系下：P2 = R*P1 + T
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

        //STEP-1 计算每幅标定板平行视图的R'（并不是旋转矩阵） 3x3
        vector<Mat> parallelImgR;
        calculateParallelViewR(camPara.imgRTVec, parallelImgR);

        //STEP-2 把所有特征点转换到平行视图上
        vector< vector<Point2f> > corner_Parallel;
        for (unsigned int i = 0; i < cData.plane2dPntsVec.size(); i++)
        {
            vector<Point2f> pnts;
			///// Mat T
			////// calculateParallelViewR  函数修改了平行视图的t1 为 t0 的1.5 z，x，y保持不变
			Mat t1 = Mat::zeros(3,1,CV_64F);
			camPara.imgRTVec[i].T.copyTo(t1);
			/*t1.at<double>(2,0) = 1.5*t1.at<double>(2,0);*/
			//////
            undistortPoints2DifferentView(cData.plane2dPntsVec[i], pnts,
                cameraMatrix,camPara.imgRTVec[i].R,camPara.imgRTVec[i].T,distCoeffs,
                cameraMatrix,camPara.parallelCamR,t1,vector<double>());
            corner_Parallel.push_back(pnts);
        }

        //STEP-3 根据平行视图上的特征点，在平行视图上进行亚像素角点检测
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

        //STEP-4 将平行视图上检测到的亚像素角点转换到原始视图
        for (unsigned int i = 0; i < corner_Parallel.size(); i++)
        {
            vector<Point2f> pnts;
            undistortPoints2DifferentView(corner_Parallel[i], pnts,
                cameraMatrix,camPara.parallelCamR,camPara.imgRTVec[i].T,vector<double>(),
                cameraMatrix,camPara.imgRTVec[i].R,camPara.imgRTVec[i].T,distCoeffs);
            corner_Parallel[i] = (pnts);
        }

        //STEP-5 使用张正友标定法标定
        _cData.plane2dPntsVec.clear();
        _cData.plane2dPntsVec = corner_Parallel;

        ZhangCalibrationMethod_CPP(_cData,camParaDst);
        //// added by zhang xu 迭代终止条件，如果迭代没有使得totalReproNormErr 减少，则停止迭代
        if(camParaDst.totalReproNormErr> camPara.totalReproNormErr)
        {
            camParaDst = camPara;
            return;  /// 返回
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
	//STEP-1 使用cam1的参数获得特征点的三维平面点
	vector<Point3f> pnts3d;
	project2CalibrationBoard(src,pnts3d,cameraMatrix1,distCoeffs1,R1,T1);
	//STEP-2 将特征点的三维平面点重投影到图像上，采用cam2的R和T
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
		//// zhangxu 修改
		//H1.at<double>(0, 2) = -SQUARE_SIZE*WIDTH_CORNER_COUNT/2;
		//H1.at<double>(1, 2) = -SQUARE_SIZE*HEIGHT_CORNER_COUNT/2;
		/*H1.at<double>(2, 2) = 1.5*H1.at<double>(2, 2);*/
		//////

		///////  以下的标定板坐标系是  x向下，y向右，z标定板向前，和matlabtool box camera calibration一样
		//H1.at<double>(0, 1) = 1;
		//H1.at<double>(1, 0) = 1;
		///////  以下的标定板坐标系是  x向右，y向下，z标定板向后，
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

	//根据文档《》生成平顶视图的转换矩阵
	//const Mat &PoseR,<in>  3X1
	//const Mat &PoseT,<in>  3X1 
	//Mat &parallelR<in & out>  输入平顶视图相对于摄像机的外参数R，3X3矩阵，输出是文档要求的H1 * inv(H0), 3X3
void CoreAlgorithm::calculateParallelViewR(const Mat &PoseR,const Mat &PoseT,Mat &parallelR)
{
	/// 判断矩阵的尺寸，不正确及跳出

	if(parallelR.rows ==0 ||(parallelR.cols == 0))
	{
		parallelR = Mat::eye(3,3,CV_64F);
	}
	///  step  生成H1
	Mat H1(3,3,CV_64F);
	parallelR.copyTo(H1);
	PoseT.col(0).copyTo(H1.col(2));
	/// step 生成H0
	Mat temR = Mat::zeros(3,3,CV_64F);
	Rodrigues(PoseR, temR);
	Mat H0(3,3,CV_64F);
	temR.copyTo(H0); 
	PoseT.col(0).copyTo(H0.col(2));
	
	//// zhangxu 修改
	//H1.at<double>(0, 2) = -SQUARE_SIZE*WIDTH_CORNER_COUNT/2;
	//H1.at<double>(1, 2) = -SQUARE_SIZE*HEIGHT_CORNER_COUNT/2;
	/*H1.at<double>(2, 2) = 1.5*H1.at<double>(2, 2);*/
	//////

	///////  以下的标定板坐标系是  x向下，y向右，z标定板向前，和matlabtool box camera calibration一样
	//H1.at<double>(0, 1) = 1;
	//H1.at<double>(1, 0) = 1;
	///////  以下的标定板坐标系是  x向右，y向下，z标定板向后，
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
		///// 图像不能为空
		return false;
	}

	///// 设置一些参数
	double PI = acos(double(-1));
	float A = bias/(bias+range);
	float B = range/(bias+range);
	//////增大信噪比的方法，即加大B的方法
	float Xmin = pow((A-B),gamma);
	float A2 = (A+B+Xmin)/2;
	float B2 = (A+B-Xmin)/2;

	///  对grid图像  x轴 生成图像Img，channel 有图像来控制  x指的就是col
	if (isGrid && isX)
	{
		for (int col=0;col<width; col++)
		{
			///// 计算亮度
			float norm = float(A2+B2*cos(2*PI*col/T-fai));
			float power = 1/gamma;
			float temp = pow(norm,power);
			float lus = (bias+range)*temp;//// 这里直接使用col除以pitch 乘以2PI

			///// 赋值到图像
			for (int row=0; row<height; row++)
			{
				for (int i=0;i<channels;i++)
				{
					Img_data[row*step + col*channels + i] = int(lus);
				}
			}
		}
	}
	///  对grid图像  y轴 生成图像Img，channel 有图像来控制  y指的就是row
	if (isGrid && !isX)		
	{
		for (int row = 0;row<height; row++)
		{
			///// 计算亮度
			float norm = float(A2+B2*cos(2*PI*row/T-fai));
			float power = 1/gamma;
			float temp = pow(norm,power);
			float lus = (bias+range)*temp;//// 这里直接使用col除以pitch 乘以2PI
			///// 赋值到图像
			for (int col=0; col<width; col++)
			{
				for (int i=0;i<channels;i++)
				{
					Img_data[row*step + col*channels + i] = int(lus);
				}
			}
		}

	}

	///  对diamond 像素网格图像  x轴 生成图像Img，channel 有图像来控制  x指的就是col
	/// 根据 LIghtcragter 4500 page11 说明 第0行是缩进半个像素，第1行是左突出半个像素，因而可的规律，偶数行像素的列坐标都是大于
	/// 奇数行两坐标半个像素（5.4微米），行与行的间距是5.4微米，列与列的间距是5.4微米,
	/// 总结起来就是 每列坐标上，奇数行列相对偶数行更靠近左边半个像素，即 偶数行列坐标加0.5，奇数行等于列坐标
	if (!isGrid && isX)
	{
		for (int col=0;col<width; col++)
		{	
			///// 赋值到图像奇数行
			for (int row=1; row<height; row=row+2)
			{
				///// 计算亮度
				float tempcol = float(col);
				float norm = float(A2+B2*cos(2*PI*tempcol/T-fai));
				float power = 1/gamma;
				float temp = pow(norm,power);
				float lus = (bias+range)*temp;//// 这里直接使用col除以pitch 乘以2PI

				for (int i=0;i<channels;i++)
				{
					Img_data[row*step + col*channels + i] = int(lus);
				}
			}	

			///// 赋值到图像偶数行
			for (int row=0; row<height; row=row+2)
			{
				///// 计算亮度
				float tempcol = float(col+0.5);
				float norm = float(A2+B2*cos(2*PI*tempcol/T-fai));
				float power = 1/gamma;
				float temp = pow(norm,power);
				float lus = (bias+range)*temp;//// 这里直接使用col除以pitch 乘以2PI

				for (int i=0;i<channels;i++)
				{
					Img_data[row*step + col*channels + i] = int(lus);
				}
			}	
		}
	}

	///  对diamond 像素网格图像  y轴 生成图像Img，channel 有图像来控制  x指的就是col
	/// 根据 LIghtcragter 4500 page11 说明 第0行是缩进半个像素，第1行是左突出半个像素，因而可的规律，偶数行像素的列坐标都是大于
	/// 奇数行两坐标半个像素（5.4微米），行与行的间距是5.4微米，列与列的间距是5.4微米,
	/// 总结起来就是 每行	坐标上，奇数列相对偶数列更靠近左边半个像素，即 偶数列 对应行坐标加0.5，奇数列等于行坐标

	if (!isGrid && !isX)		
	{
		for (int row = 0;row<height; row++)
		{
			///// 赋值到图像奇数列
			for (int col=1; col<width; col = col+2)
			{
				///// 计算亮度
				float temprow = float(row);
				float norm = float(A2+B2*cos(2*PI*temprow/T-fai));
				float power = 1/gamma;
				float temp = pow(norm,power);
				float lus = (bias+range)*temp;//// 这里直接使用col除以pitch 乘以2PI

				for (int i=0;i<channels;i++)
				{
					Img_data[row*step + col*channels + i] = int(lus);
				}
			}
			///// 赋值到图像偶数列  +0.5
			for (int col=0; col<width; col = col +2)
			{
				///// 计算亮度
				float temprow = float(row+0.5);
				float norm = float(A2+B2*cos(2*PI*temprow/T-fai));
				float power = 1/gamma;
				float temp = pow(norm,power);
				float lus = (bias+range)*temp;//// 这里直接使用col除以pitch 乘以2PI

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
	int frameNum = images.size();  //// 图片数量

	if (frameNum<3)
	{
		return false;  /// 图片数量不能少于3
	}

	int height = images[0].rows;
	int width = images[0].cols;
	int channels = images[0].channels();
	if (channels>1)
	{
		return false;  /// 本函数只接受灰度图像
	}

	/*if (height!= mod_phase.rows || height!= mean_lus.rows || height!= lus_range.rows || height!=mask.rows)
	{
		return false;  /// 输入参数的 行数不一致
	}

	if(width != mod_phase.cols || width!= mean_lus.cols || width != lus_range.cols || width != mask.cols  )
	{
		return false;  /// 输入的参数 列数不一致
	}*/
	

	///////
	Mat lusin = Mat::zeros(height,width,CV_32FC1);
	Mat lucos = Mat::zeros(height,width,CV_32FC1);
	mean_lus = Mat::zeros(height,width,CV_32FC1);  ////将它赋值为0
	mod_phase = Mat::zeros(height,width,CV_32FC1);  ////将它赋值为0
	lus_range = Mat::zeros(height,width,CV_32FC1);  ////将它赋值为0
	mask =  Mat::zeros(height,width,CV_8UC1); ////将它赋值为0
	Mat temMat ;  ////将它赋值为0   = Mat::zeros(height,width,CV_32FC1)

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
	
	//// 判断mask
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
		return false; //// 频率个数不一致
	}
	int height = mod_phases[0].rows;
	int width = mod_phases[0].cols;

	double PI = acos(double(-1));
	
	vector<Mat> q; /// 保存相减除以最大公约数的结果
	vector<int> taobari1;
	int gamma1 = 1; /// 所有质数的成绩除以第一个质数，即除了第一个质数之外的所有质数的乘积
	///// 将卷绕相位转换成余数
	for(int i=0;i<freNum;i++)
	{
		mod_phases[i] = mod_phases[i]*lamda[i]/2/PI;  /// 卷绕相位转换成余数
		lamda[i] = lamda[i]/m;  //// 转换成互质的数		
		////// 计算余数的差
		Mat temQ,temQ2;  ///临时矩阵		
		if (i>0)
		{
			////
			gamma1 = int(gamma1*lamda[i]); /// 计算所有质数的乘积除以 tao1
			///// 计算模乘系数
			int temtaoi1,temtao11,temq;
			gcd(int(lamda[0]),int(lamda[i]),temtaoi1,temtao11,temq);
			taobari1.push_back(temtaoi1);
			///// 计算余数差，除以公约数m，乘以模乘系数
			subtract(mod_phases[i], mod_phases[0], temQ); //// 相减	
			temQ = temQ/m; 
			temQ.convertTo(temQ2,CV_32SC1);  //// 这里转换成整数			
			q.push_back(temQ2);  /// 保存
		}
	}
	///// 			//// 计算bi1
	vector<int> b;
	for (int i=0;i<freNum-1;i++)
	{
		int bi1, b11,q11;
		gcd(int(gamma1/lamda[i+1]), int(lamda[i+1]),bi1,b11,q11);
		b.push_back(bi1);
	}
	////计算	 cor
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
				zn = zn + epiu*b[i]*int(gamma1)/int(lamda[i+1]); ///公式（25） 求和				
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
bool CoreAlgorithm::creatdistorImg(Mat& img,double fc[2] ,double cc[2] ,double kc[5] ,double alfpha_c, float meanLus, float rangelus,float T,float fai, int method )
{
	//// 判断要输出的图像大小是否大于0
	int height,width;
	height = img.rows;
	width  = img.cols;
	if (height<1 || width <1)
	{
		return false;  //// 图像输入为0，不是有效的图像
	}
	//// 判断图像的通道数，如果是3个通道的 报错
	int channel = img.channels();
	if (channel>1)
	{
		return false;
	}
	double PI = acos(double(-1));
	//// 将图像的坐标转换层 vector<point2f>
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
	///// 计算无畸变的像素坐标
	undistorPixcor(src ,dst ,fc ,cc ,kc ,alfpha_c);
	//// 根据无畸变的像素坐标计算亮度		
	int intensty;
	///////////////////////////  method =0  表示模型是 meanLus + rangelus*cos(2*pi*x/T -fai)
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
				///// 根据此亮度对图像进行设置
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
	///// 如果dx 小于等于0，则返回错误
	if(dx<=0)
	{
		return false;
	}

	//// 生成的圆环图像尺寸为dx，外环半径为 dx/3， 内环半径为dx/6
	//// 两个圆之间的部分为黑0.其他部分亮255
	Mat _img = Mat::zeros(dx, dx, CV_8U);  ////Mat _img = Mat::zeros(dx, dx, CV_32F); 
	//// 图像大小
	int height,width;
	height = _img.rows;
	width  = _img.cols;
	double rmax,rmin,dist,cen_x,cen_y;
	rmax = dx/3;
	rmin = dx/6;
	////// ，由于C++中矩阵的起始坐标为0，而不是1，所以坐标size减去1，则等效于（size-1）/2
	cen_x = (dx-1)/2;
	cen_y = (dx-1)/2;


	for(int i=0;i<height; i++)
	{
		for(int j=0;j<width;j++)
		{
			 dist  = double(i-cen_y)*double(i-cen_y) + double(j-cen_x)*double(j-cen_x);
			 dist = sqrt(dist);
			 if(dist<rmin || dist>rmax) //// 设置成为白色 255
			 {
				 _img.at<uchar>(i,j) = 255;
			 }
			 //// 否则，就是为黑色，不用做任何操作
		}
	}
	img = _img.clone();
	return true;
}

void CoreAlgorithm::generateParallelCamR(Mat& R)
{
	
	Mat r = Mat::zeros(3, 3, CV_64F);
		//光笔坐标系下
		/*r.at<double>(1, 0) = 1;
		r.at<double>(0, 2) = 1;
		r.at<double>(2, 1) = 1;*/
		//以下的标定板坐标系是  x向右，y向下，z标定板向后，
		r.at<double>(0, 0) = 1;
		r.at<double>(1, 1) = 1;
		r.at<double>(2, 2) = 1;
		///////  以下的标定板坐标系是  x向下，y向右，z标定板向前，和matlabtool box camera calibration一样
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

	//这里A不是方阵
	solve(A, B, X,cv::DECOMP_QR);

	Mat H(2,2,CV_32F);
	H.at<float>(0, 0) = X.at<float>(0, 0);
	H.at<float>(0, 1) = float(X.at<float>(1, 0) / 2.0);
	H.at<float>(1, 0) = float(X.at<float>(1, 0) / 2.0);
	H.at<float>(1, 1) = X.at<float>(2, 0);

	vector<double> eigenvalues;
	Mat eigenvectors(2,2,CV_32F);
	eigen(H, eigenvalues, eigenvectors);

	//后续添加
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
	  //转化成灰度图
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
        ///// 自适应二值化
        int block_size = 100*multiples+1;
		int C_threshold = 10;
        adaptiveThreshold(ImageGray,ImageGray,255,cv::ADAPTIVE_THRESH_MEAN_C ,cv::THRESH_BINARY,block_size,C_threshold);
        //对白色区域膨胀再腐蚀
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

        //创建标定板区域的ROI，向外围阔出去一圈
        RotatedRect ROI = minAreaRect(mousePnts_half);
        cv::Rect roi = ROI.boundingRect();

        ///// 检查ROI区域 如果超过范围，则赋值图像最大尺寸
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

        //填充所选区域外的部分为白色，从两边扫描，减少遍历时间
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
        //// 边缘检测
       Mat img_threshold;
       Canny(imgGray_ROI,img_threshold,50,255,3,true);/// 255,  3
        ///// end test
        //寻找轮廓
        vector< vector<Point> > contours;
		findContours(img_threshold,contours,CV_RETR_EXTERNAL ,CV_CHAIN_APPROX_SIMPLE);
  //     Mat contourImg = Mat::zeros(img_threshold.size(),CV_8U);
  //      drawContours(contourImg, contours, -1, Scalar(255, 255, 255));
		//namedWindow("window",2);
		//imshow("window",contourImg);
		//waitKey();

        //椭圆拟合,找中心,并进行筛选
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
            catch (...)//三个点表示任何异常
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

	//点到直线的距离d=fabs(a*x+b*y+1)/sqrt(a*a+b*b);
	float dis = fabs(a*pnt3.x + b*pnt3.y + 1) / sqrt(a*a + b*b);

	if (dis > dis_threshold)
		return false;

	return true;
}

bool CoreAlgorithm::sortRingCenter(const vector<Point2f>& mousePnts,vector<Point2f>& center,int& row,int& col)
{
	//找出四个角上的点
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

	//找出在左右两列上的点
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

	//左右两列的点按照top2down的顺序排序,既按照向量方向排序
	//左右两列点先按照Y坐标排序，再判断到top点的距离
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

	//从第最后一行开始找
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
    //找出四个角上的点
    Point2f rectAnglePnts[4];
    vector<Point2f> cornerRingPnts;
    minAreaRect(center).points(rectAnglePnts);//输出的点是按照顺时针排序的
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

    //找出左上角点
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

    //找出在左右两列上的点
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

    //左右两列点先按照Y坐标排序
    sortByYMin2Max(rightColPnts);

    sortByYMin2Max(leftColPnts);

    //从第最后一行开始找
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
	 //转化成灰度图
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
			//step-2 高精度圆检测
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
	//获得中心间距
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

	//如果输入的模板为空由中心距创建模板
	vector<float> dx_vec, dy_vec;
	Mat _tampl;
	if (tampl.empty())
	{
		_tampl = img(cv::Rect(Point2f(centers[0].x - (space-1) / 2, centers[0].y - (space-1) / 2),
		Point2f(centers[0].x + (space+1) / 2, centers[0].y + (space+1) / 2))).clone();
	}
	else
	{
		///// 对图像模板tampl  按照中心距离space进行resize
		cv::resize(tampl,_tampl,Size(int(space),int(space)),0,0,cv::INTER_CUBIC );		
	}

	
	//创建模板匹配ROI，这个ROI的边界扩展一个space是不够的，应该是1.5个space
	Mat img_ROI;
	cv::Rect ROI = boundingRect(centers);
	
	ROI.x =int( ROI.x - 1.5*space);   
	ROI.y = int(ROI.y - 1.5*space);
	ROI.width = int(ROI.width + 3 * space); 
	ROI.height = int(ROI.height + 3 * space);
	/// zhangxu added 判断ROI是否越界
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

	//CV_TM_CCORR_NORMED、CV_TM_CCOEFF、CV_TM_CCOEFF_NORMED，越亮匹配度越高
	Mat result;
	matchTemplate(img_ROI, _tampl, result, CV_TM_CCOEFF);  ///// 这个matchTemplate要求输入的两个矩阵的变量类型是一致的，要么都是无符号整型8位，要么都是浮点型32位
	normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, Mat());
	int centerArea_Size = 3;
	
	for (unsigned int n = 0; n < centers.size(); n++)
	{
		//获得中心点在匹配结果result图像中的坐标，因为result 的尺寸是  (W-w+1) X (H-h+1)，但是转换成坐标就不能使用用size来计算
				
		float x = centers[n].x - float(_tampl.cols-1) / 2 - ROI.x;
		float y = centers[n].y - float(_tampl.rows-1) / 2 - ROI.y;
		
		//在几何中心点附近找出匹配中心点
		double minVal; double maxVal; Point minLoc; Point maxLoc;
		///进行越界检查
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
        ////// 此处计算出像素级最大点，在result中的坐标
        matchLoc.x += cvRound(x-centerArea_Size);
        matchLoc.y += cvRound(y-centerArea_Size);

		//创建亚像素检测区域
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
		//// zhang xu added 得到此时的像素精度级的坐标
		centers[n].x = matchLoc.x + float(_tampl.cols-1) / 2 + ROI.x;
		centers[n].y = matchLoc.y + float(_tampl.rows-1) / 2 + ROI.y;
		//// 增加亚像素的坐标偏移量
		centers[n].x += dx;
		centers[n].y += dy;
		////// 张旭 test dx dy
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
	R = 0.25*space; r = 0.5*R;//根据固定的比例

	//R = 27; r = 0.5*R;//test ,这个半径不对的话问题会很大
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
   
    //转化成灰度图
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

        ////直方图均衡化
        //equalizeHist(ImageGray,ImageGray);

        //// 均值滤波
        blur(ImageGray,ImageGray,Size(5,5),Point(-1,-1));


    //自适应阈值处理
    Mat thresh_img;
    int block_size = 501;
    adaptiveThreshold(ImageGray,thresh_img,255,cv::ADAPTIVE_THRESH_MEAN_C ,cv::THRESH_BINARY,block_size,0 );

    int close_type = cv::MORPH_ELLIPSE;
    int dilate_size = 3;
    Mat element = getStructuringElement(close_type,Size(2*dilate_size+1,2*dilate_size+1),Point(-1,-1));
    dilate(thresh_img, thresh_img, element,Point(-1, -1),1);

    //找轮廓
    vector<vector<Point>> all_contours;
    vector<int> contoursIndex;
    vector<cv::Vec4i> all_hierarchy;
    Mat temp_LED;
    thresh_img.copyTo(temp_LED);
    findContours(temp_LED,all_contours,all_hierarchy,CV_RETR_EXTERNAL ,CV_CHAIN_APPROX_SIMPLE);

    //轮廓过滤
    vector<vector<Point>> contours;
    vector<cv::Vec4i> hierarchy;
    for (unsigned int n = 0; n<all_contours.size(); n++)
    {
        double area = contourArea(all_contours[n]);
        //按照组成轮廓的点的数量
       // if(  all_contours[n].size() < 80 || all_contours[n].size() > 700)
		if(  all_contours[n].size() < 80 )
        {continue;}
        //按照轮廓的面积，既包含的像素个数
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
                //判断第一个子轮廓的中心是否与其父轮廓接近
                int k = all_hierarchy[n][2];
                if (k>0)//若k<0则该轮廓没有子轮廓
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
	//STEP-1:中心点提取
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
	//double t = (double)getTickCount();////开始计时
	double pi = acos(-1);
	double angle;
	double a = 45.0;
	double d = 15.0;
	double gamma = 2 * (a + d) / d;

 	if (!findPntsWithTagV(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma))/////已修订
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
						if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))/////已修订
						{
							angle = -15.0 / 180.0 * pi;
							if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
							{
								angle = 20.0 / 180.0 * pi;
								if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))//////已修订
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
																						cout << "检测未成功，请保存图片" << endl;
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

    //STEP-3 针对特征点进行高精度的圆检测
	//t = ((double)getTickCount() - t) / getTickFrequency();///////放在函数结尾，计算时间，单位为：秒
	//cout << "时间== " << t << endl;////输出时间
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

////重载getTagPoint2fStrong()函数，锁定ROI。
bool CoreAlgorithm::getTagPoint2fStrong(const Mat& img, const cv::Rect maskLightPen, vector<TagPoint2f>& tagPnts2f, Mat cameraMatrixLeft, Mat cameraMatrixRight)
{
	//STEP-1:中心点提取
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
	//double t = (double)getTickCount();////开始计时
	double pi = acos(-1);
	double angle;
	double a = 45.0;
	double d = 15.0;
	double gamma = 2 * (a + d) / d;
	if (!findPntsWithTagV(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma))/////已修订
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
						if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))/////已修订
						{
							angle = -15.0 / 180.0 * pi;
							if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))
							{
								angle = 20.0 / 180.0 * pi;
								if (!findPntsWithTagVStrong(centerPnts, longaxisRadius, tagPnts, Featurelength, gamma, cameraMatrixLeft, angle))//////已修订
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
																						cout << "检测未成功，请保存图片" << endl;
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

	//STEP-3 针对特征点进行高精度的圆检测
	//t = ((double)getTickCount() - t) / getTickFrequency();///////放在函数结尾，计算时间，单位为：秒
	//cout << "时间== " << t << endl;////输出时间
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

	////对它进行排序
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
	//得到的表面法线符合右手原则
	
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
	 //第一步，使用sovlePnP得到旋转向量和平移向量
	 if(!(solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs,PoseR,PoseT,false,Flag)))
	 {
		 return;
	 }
	 //第二步，使用projectPoints得到重投影的二维特征点，得到雅可比矩阵，仅仅抽取前六列（ 旋转向量，平移向量）则构成J
     vector<Point2f> imageReprojected;
	 Mat jacobian;
	 Mat dpoutpara(objectPoints.size()*2,6,CV_64FC1);
	 projectPoints(objectPoints,PoseR,PoseT,cameraMatrix,distCoeffs,imageReprojected,jacobian);
	 //将drot dt放到doutpara中(抽取前六列)
	for( int i=0;i<6;i++ )
	{
		jacobian.col(i).copyTo(dpoutpara.col(i));
	}
	 //第三步，输出的特征点图像坐标，与检测的图像特征点坐标相减则为反向投射误差，即为delta他是2nX1的矩阵。
	Mat delta(dpoutpara.rows,1,CV_64FC1);
	for (unsigned int i = 0;i < imagePoints.size();i++)
	{
		delta.at<double>(2*i,0) = fabs(double(imagePoints[i].x-imageReprojected[i].x));
		delta.at<double>(2*i+1,0) =fabs(double(imagePoints[i].y-imageReprojected[i].y));

	}
	//XMLWriter::writeMatData("F:/1.xml",delta);
	 //第四步，根据公式求得一个方差值，6X6的矩阵，取对角线元素即为方差。为6X6的矩阵，其求平方根这位标准差，乘以3进行输出，为3被标准差结果。
	Mat covariance_pre;
	Mat dpoutpara_invert;
	Mat covariance;
	double error = invert(dpoutpara,dpoutpara_invert,CV_SVD);
	gemm(dpoutpara_invert,delta,1,0,0,covariance_pre);
	//covariance_pre = dpoutpara_invert*delta;//这种矩阵相乘方式也是可行的
	mulTransposed(covariance_pre,covariance,0);
	 //第五步，求出3倍标准差
	Mat diag_covariance(covariance.rows,1,CV_64FC1);
	diag_covariance = covariance.diag(0);//取主对角线
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
	//将坐标系元原点转化到原来的坐标原点上
	Point3f pointtemp = Point3f(PoseT);
	pointtemp = pointtemp-centroid;
	PoseT = Mat(pointtemp);
	PoseT.convertTo(PoseT,CV_64F);
	 //第六步，求出转换到测量坐标系下的三维特征点，作为输出
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
	 //第一步，使用sovlePnP得到旋转向量和平移向量
	 if(!(solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs,PoseR,PoseT,false,Flag)))
	 {
		 return;
	 }
	 //第二步，使用projectPoints得到重投影的二维特征点，得到雅可比矩阵，仅仅抽取前六列（ 旋转向量，平移向量）则构成J
     vector<Point2f> imageReprojected;
	 Mat jacobian;
	 Mat dpoutpara(objectPoints.size()*2,6,CV_64FC1);
	 projectPoints(objectPoints,PoseR,PoseT,cameraMatrix,distCoeffs,imageReprojected,jacobian);
	 //将drot dt放到doutpara中(抽取前六列)
	for( int i=0;i<6;i++ )
	{
		jacobian.col(i).copyTo(dpoutpara.col(i));
	}
	 //第三步，输出的特征点图像坐标，与检测的图像特征点坐标相减则为反向投射误差，即为delta他是2nX1的矩阵。
	Mat delta(dpoutpara.rows,1,CV_64FC1);
	for (unsigned int i = 0;i < imagePoints.size();i++)
	{
		delta.at<double>(2*i,0) = fabs(double(imagePoints[i].x-imageReprojected[i].x));
		delta.at<double>(2*i+1,0) =fabs(double(imagePoints[i].y-imageReprojected[i].y));

	}
	//XMLWriter::writeMatData("F:/1.xml",delta);
	 //第四步，根据公式求得一个方差值，6X6的矩阵，取对角线元素即为方差。为6X6的矩阵，其求平方根这位标准差，乘以3进行输出，为3被标准差结果。
	Mat covariance_pre;
	Mat dpoutpara_invert;
	Mat covariance;
	double error = invert(dpoutpara,dpoutpara_invert,CV_SVD);
	gemm(dpoutpara_invert,delta,1,0,0,covariance_pre);
	//covariance_pre = dpoutpara_invert*delta;//这种矩阵相乘方式也是可行的
	mulTransposed(covariance_pre,covariance,0);
	 //第五步，求出3倍标准差
	Mat diag_covariance(covariance.rows,1,CV_64FC1);
	diag_covariance = covariance.diag(0);//取主对角线
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
	//将坐标系元原点转化到原来的坐标原点上
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
		 //STEP-1 计算每幅特征点图像的平行视图的R'（并不是旋转矩阵） 3x3
		Mat parallelImgR,t1;
		calculateParallelViewR(PoseR_Src,PoseT_Src,parallelImgR);
		//STEP-2 把所有特征点转换到平行视图上
		vector<Point2f> corner_Parallel;
		//Mat t1 = Mat::zeros(3,1,CV_64F);
		PoseT_Src.copyTo(t1);
		undistortPoints2DifferentView(imagePoints,corner_Parallel,cameraMatrix,PoseR_Src,PoseT_Src,distCoeffs,
			cameraMatrix,parallelCamR,t1,vector<double>());
        //STEP-3 根据平行视图上的特征点，在平行视图上进行亚像素角点检测
        ////// image is too small
        Mat resultimgMat1(2*srcimgMat1.rows, 2*srcimgMat1.cols, srcimgMat1.type());
        undistortImg(srcimgMat1, resultimgMat1, cameraMatrix, distCoeffs, parallelImgR);
	/*	namedWindow("window",2);
		imshow("window",resultimgMat1);
		waitKey();*/
	    //在平行视图上进行亚像素圆检测
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
        //STEP-5 使用PNP算法计算位姿
		dpdrot_Dst.clear();
		dpdt_Dst.clear();
		PnPMethod(pnts3d,detectorResult,cameraMatrix,distCoeffs,PoseR_Dst,PoseT_Dst,dpdrot_Dst,dpdt_Dst,Flag);
        //// 迭代终止条件，如果迭代没有使得位姿误差三倍标准差减小，则停止迭代
		//分别求出迭代前后位姿误差三倍标准差的模
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
			//第六步，求出转换到测量坐标系下的三维特征点，作为输出
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
		 //STEP-1 计算每幅特征点图像的平行视图的R'（并不是旋转矩阵） 3x3
		Mat parallelImgR,t1;
		calculateParallelViewR(PoseR_Src,PoseT_Src,parallelImgR);
		//STEP-2 把所有特征点转换到平行视图上
		vector<Point2f> corner_Parallel;
		//Mat t1 = Mat::zeros(3,1,CV_64F);
		PoseT_Src.copyTo(t1);
		undistortPoints2DifferentView(imagePoints,corner_Parallel,cameraMatrix,PoseR_Src,PoseT_Src,distCoeffs,
			cameraMatrix,parallelCamR,t1,vector<double>());
        //STEP-3 根据平行视图上的特征点，在平行视图上进行亚像素角点检测
        ////// image is too small
        Mat resultimgMat1(2*srcimgMat1.rows, 2*srcimgMat1.cols, srcimgMat1.type());
        undistortImg(srcimgMat1, resultimgMat1, cameraMatrix, distCoeffs, parallelImgR);
	/*	namedWindow("window",2);
		imshow("window",resultimgMat1);
		waitKey();*/
	    //在平行视图上进行亚像素圆检测
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
        //STEP-5 使用PNP算法计算位姿
		dpdrot_Dst.clear();
		dpdt_Dst.clear();
		PnPMethod(objectPoints,detectorResult,cameraMatrix,distCoeffs,PoseR_Dst,PoseT_Dst,dpdrot_Dst,dpdt_Dst,Flag);
        //// 迭代终止条件，如果迭代没有使得位姿误差三倍标准差减小，则停止迭代
		//分别求出迭代前后位姿误差三倍标准差的模
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
	if(!solve(coefficient_matrix,equation_right,variable_matrix,cv::DECOMP_CHOLESKY))//高斯消元法?此处系数矩阵为对称矩阵
	{
		return false;
	}
	//方向向量单位化
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
	//step-1 解出一般曲面方程的十个待定系数
	//设置一个判定系数K.此系数指标还有待确定，误差？
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
	//第一种情况，方向向量有两个为零，即圆截面平行于某个二维坐标平面
	//根据解析解判断方向向量的值是否为零
	if(abs(variable_matrix.at<float>(3,0))<K&&abs(variable_matrix.at<float>(4,0))<K&&abs(variable_matrix.at<float>(5,0))<K)
	{
		//都为零，则方向向量必有两个量是零，一个为非零
		//step-1判断哪一个量为非零
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
	//第二种情况，方向向量有一个为零，其余不为零
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
	//第三种情况，方向向量都不为零
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
		//判断方向向量的正负号
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
	if(!solve(coefficient_matrix2,equation_right2,variable_matrix2,cv::DECOMP_LU))//高斯消元法?此处系数矩阵为对称矩阵
		return false;
	models.orignal.x =variable_matrix2.at<float>(0,0);
	models.orignal.y =variable_matrix2.at<float>(1,0);
	models.orignal.z =variable_matrix2.at<float>(2,0);
	//求出r  
	models.r = sqrt(abs((variable_matrix.at<float>(0,0)*pow(models.orignal.x,2)+variable_matrix.at<float>(1,0)*pow(models.orignal.y,2)+
				variable_matrix.at<float>(2,0)*pow(models.orignal.z,2)+variable_matrix.at<float>(3,0)*models.orignal.x*models.orignal.y+
				variable_matrix.at<float>(4,0)*models.orignal.z*models.orignal.y+variable_matrix.at<float>(5,0)*models.orignal.x*models.orignal.z-
				variable_matrix.at<float>(9,0))/lamada));
	//方向向量单位化
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
	//首先判断直线是否与平面平行  
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
	//找出不在短轴上的点
	float dis_threshold = 15;
	float a, b;//直线公式ax+by+1=0;a=(y1-y2)/(x1y2-x2y1),b=(x1-x2)/(x2y1-x1y2);
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
		//点到直线的距离d=fabs(a*x+b*y+1)/sqrt(a*a+b*b);
		float dis = float(fabs(float(a*footPoint.x + b*footPoint.y + 1))/sqrt(a*a + b*b));
		if (dis < dis_threshold)//当中心找的不准的时候这个阈值要放宽一点
		{
			_tagPnt2[0] = TAG2;
			_tagPnt2[1] = pnt3.x;
			_tagPnt2[2] = pnt3.y;
			tagPnts.push_back(_tagPnt2);
			//向量AB＝（x1,y1,z1）, 向量CD＝（x2,y2,z2）
			//向量AB×向量CD＝（y1z2-z1y2，z1x2-x1z2，x1y2-y1x2）
			Vec3f norm_X,norm_Y,norm_Z;//norm_X为长轴的方向向量，定义为1到5点为正方向
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
	//step-1 将图像转化成灰度图
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
	//step-2 计算图像梯度信息
	//step-2-1 生成高斯滤波器模版	
		//判断滤波片模版大小是不是奇数
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
	//step-2-2 二维滤波器操作
		Mat dx,dy;
		filter2D(ImageGray,dx,CV_64F,GaussianKenelx);
		filter2D(ImageGray,dy,CV_64F,GaussianKenely);
	//step-2-3 梯度范数计算
		Mat gradientnorm,gradientnormBinary;;
		magnitude(dx,dy,gradientnorm);//计算梯度范数
		//double minvalue,maxvalue;
		//cv::minMaxLoc(gradientnorm,&minvalue,&maxvalue);
	//step-3 高置信梯度区域的选择
	//step-3-1 针对求出的梯度矩阵进行二值化
		//int thresholdvalue = int(minvalue+maxvalue/5);
		int thresholdvalue = 70;
		//尝试直接二值化的方法
		gradientnorm.convertTo(gradientnorm,CV_32F);
		double value = threshold(gradientnorm,gradientnormBinary,thresholdvalue,255, CV_THRESH_BINARY);
		gradientnormBinary.convertTo(gradientnormBinary,CV_8UC1);
		//以上部分是不是可以通过先高斯滤波处理再使用canny算子进行边缘检测？？？？？？？？？
	//step-3-2 联通区域标识
		Mat contoursMask;
		int contoursNum = connectedComponents(gradientnormBinary,contoursMask,8);
		contoursMask.convertTo(contoursMask,CV_8UC1);
		contoursNum--;
	//step-3-3 数据整理
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
	//step-4 利用对偶椭圆算子

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
		//step-5 椭圆参数计算 Ax^2+Bxy+Cy^2+Dx+Ey+F=0
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
		//step-6 由椭圆一般方程求解椭圆标准方程参数
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

	//为了高精度检测，对数据进行了线性归一化
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
	//求AA
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
	//求BB
	Mat _ccaa = -1*(c2.mul(a2));Mat _ccab = -1*(c2.mul(ab));Mat _ccbb = -1*(c2.mul(b2));Mat _ccac = -1*(c2.mul(ac));Mat _ccbc = -1*(c2.mul(bc));
	BB.at<double>(0,0)= sum(_ccaa).val[0];BB.at<double>(1,0)= sum(_ccab).val[0];BB.at<double>(2,0)= sum(_ccbb).val[0];BB.at<double>(3,0)= sum(_ccac).val[0];BB.at<double>(4,0)= sum(_ccbc).val[0];
	if(determinant(AA)<10e-10)
	{
		//是否没有必要做下面工作，直接return false'
		dC = Mat::ones(3,3,CV_64F);
		dC = -1*dC;
		precision.at<double>(0,0) = -1;
		angleIncertitude = -1;
		return false;
	}
	//解A*THITA=BB;
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
	//误差估计
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
    if (areaGrads.size() != areaPoses.size())//意外错误检测
        return false;

    //对数据统一进行了线性归一化,计算统一的H矩阵
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
    //// 构造完毕H

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

        //构造[a,b,c]
        Mat abc=Mat(a.rows,3,CV_64F);
        a.copyTo(abc.col(0));
        b.copyTo(abc.col(1));
        c.copyTo(abc.col(2));

        //得到归一化后的a,b和c
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
        aVec.push_back(a);//存储a和c,用于误差计算
        cVec.push_back(c);

        //solution par least-square
        //求AA
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

        //求BB
        Mat _ccaa = -1*(c2.mul(a2));Mat _ccab = -1*(c2.mul(ab));Mat _ccbb = -1*(c2.mul(b2));Mat _ccac = -1*(c2.mul(ac));Mat _ccbc = -1*(c2.mul(bc));
        BB.at<double>(0,0)= sum(_ccaa).val[0];BB.at<double>(1,0)= sum(_ccab).val[0];BB.at<double>(2,0)= sum(_ccbb).val[0];BB.at<double>(3,0)= sum(_ccac).val[0];BB.at<double>(4,0)= sum(_ccbc).val[0];
        if(determinant(AA)<10e-10)
        {
            //是否没有必要做下面工作，直接return false'
//            dC = Mat::ones(3,3,CV_64F);
//            dC = -1*dC;
//            precision.at<double>(0,0) = -1;
//            angleIncertitude = -1;
            return false;
        }
        AAVec.push_back(AA);
        BBVec.push_back(BB);
    }

    //求AAU和BBU
    Mat AAU;// = Mat(5*areaGrads.size(),3*areaGrads.size()+2,CV_64F);
    Mat BBU;// = Mat(5*areaGrads.size(),1,CV_64F);
    for(uint j = 0; j < areaGrads.size(); j++)//5行5行的压入到AAU和BBU中
    {
        Mat AAURow,AAURowT;
        for(uint k = 0; k < 3*areaGrads.size(); k++)//先压入AA的前三列到AAU中
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
        for(uint k = 0; k < 2; k++)//再压入AA的后两列到AAU中
        {
            Mat tempM = Mat::zeros(5,5,CV_64F);
            AAVec[j].copyTo(tempM);
            tempM = tempM.t();
            AAURowT.push_back((tempM.row(k+3)));
        }
        AAURow = AAURowT.t();//转置并压入到AAU中
        AAU.push_back(AAURow);
        BBU.push_back(BBVec[j]);//直接将相应的BB压入到BBU中
    }

    //解AAU*THITAU=BBU;
//    cout<<AAU<<endl;
//    cout<<BBU<<endl;
    Mat solu;// = Mat(3*areaGrads.size()+2,1,CV_64F);
    if(!solve(AAU,BBU,solu,cv::DECOMP_SVD))
        return false;

    //先拆解
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

        uint j = 0;//对应于A,B,C
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

    //误差估计
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
    //判断行数和列数,即当中心距突然大于多个中心距之和时,该中心距为行末点和下行起始点之距
    int cols = 1;//列数初始化为1列
    float sum_dis = 0;
    for (unsigned int i = 0; i < centers.size()-1; i++)
    {
        float _dis = distance(centers[i], centers[i+1]);
        if (i >= 2)
        {
            if (_dis > sum_dis)
                break;//该中心距为行末点和下行起始点之距
        }
        sum_dis += _dis;
        cols++;
    }
    int rows = centers.size() / cols;

    //获取每个中心点的ROI
    vector<cv::Rect> centers_rect;
    float roiWidth,roiHeight;
    roiWidth = distance(centers[0], centers[1]);//计算中心点间距
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
            findEllipses(img,centers_rect[n],findResults,5,0);//内圆
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
            findEllipses(img,centers_rect[n],findResults,5,0);//外圆
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
            findEllipses(img,centers_rect[n],findResults,5,0);//内外圆平均值
            if (findResults.size() != 2)
                return false;
            centers[n].x = (findResults[0].center.x+findResults[1].center.x)/2;
            centers[n].y = (findResults[0].center.y+findResults[1].center.y)/2;
        }
        else
        {
            findEllipses(img,centers_rect[n],findResults,5,1);//内外圆优化平均值
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
	//现将平面参数转换成Ax+By+Cz+D=0表达形式
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
	//分析数据矩阵，得出每个点邻近点的序号和个数
	if(centerPnt.size()<8)
		return false;
	vector<TagPoint2f> TagPnts;
	if(centerPnt.size()!=longaxisRadius.size())
		return false;
	int pntsSize = centerPnt.size();
	Mat distanceMatrix1 = Mat::zeros(pntsSize,pntsSize,CV_64F);
	Mat distanceMatrix = Mat::zeros(pntsSize,pntsSize,CV_64F);
	//step-2 计算椭圆中心之间的距离并除以主椭圆长轴半径，将其存储在矩阵中
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
	//根据邻近点个数分布情况，进行分类。
	//1号点
	if(fourVec.size()!=1)
		return false;
	TagPoint2f firstPoint;
	firstPoint[0] = TAG1;
	firstPoint[1] = centerPnt[fourVec[0].first].x;
	firstPoint[2] = centerPnt[fourVec[0].first].y;
	TagPnts.push_back(firstPoint);
	firstFeaturelength = longaxisRadius[fourVec[0].first];
	//4,5,8号点
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
	//识别4，5点
	if(fourFivePoints.size()!=2)
		return false;
	//向量AB＝（x1,y1,z1）, 向量CD＝（x2,y2,z2）
	//向量AB×向量CD＝（y1z2-z1y2，z1x2-x1z2，x1y2-y1x2）
	Vec3f norm_X,norm_Y1,norm_Z1;//norm_X为长轴的方向向量，定义为1到5点为正方向
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
	//2,3,6,7点
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
	//向量AB＝（x1,y1,z1）, 向量CD＝（x2,y2,z2）
	//向量AB×向量CD＝（y1z2-z1y2，z1x2-x1z2，x1y2-y1x2）
	Vec3f norm_Y2,norm_Z2,norm_Y3,norm_Z3;//norm_X为1,8的方向向量，定义为1到8点为正方向
	//2,3点
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
	//6.7点
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
	// step-6对标记好的点，按照标记点进行排序
	//由小到大排序//chengwei added
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
		Size(EllipseLong_axis, EllipseShort_axis),   ////ellipse()函数中参数轴长应该是长短轴的一半，此处将对应的参数除以二，则我们输入即可认为是长短轴轴长。
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
	//////利用undistortPoints()函数将拍摄的图像矫正为正常的视角，便于检测。
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
	//分析数据矩阵，得出每个点邻近点的序号和个数
	if (DistortCenterPnt.size() < 8)
		return false;
	vector<TagPoint2f> TagPnts;
	if (DistortCenterPnt.size() != longaxisRadius.size())
		return false;
	int pntsSize = DistortCenterPnt.size();
	Mat distanceMatrix1 = Mat::zeros(pntsSize, pntsSize, CV_64F);
	Mat distanceMatrix = Mat::zeros(pntsSize, pntsSize, CV_64F);
	//step-2 计算椭圆中心之间的距离并除以主椭圆长轴半径，将其存储在矩阵中
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
	//根据邻近点个数分布情况，进行分类。
	//1号点
	if (fourVec.size() != 1)
		return false;
	TagPoint2f firstPoint;
	firstPoint[0] = TAG1;
	firstPoint[1] = centerPnt[fourVec[0].first].x;
	firstPoint[2] = centerPnt[fourVec[0].first].y;
	TagPnts.push_back(firstPoint);
	firstFeaturelength = longaxisRadius[fourVec[0].first];
	//4,5,8号点
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
	//识别4，5点
	if (fourFivePoints.size() != 2)
		return false;
	//向量AB＝（x1,y1,z1）, 向量CD＝（x2,y2,z2）
	//向量AB×向量CD＝（y1z2-z1y2，z1x2-x1z2，x1y2-y1x2）
	Vec3f norm_X, norm_Y1, norm_Z1;//norm_X为长轴的方向向量，定义为1到5点为正方向
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
	//2,3,6,7点
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
	//向量AB＝（x1,y1,z1）, 向量CD＝（x2,y2,z2）
	//向量AB×向量CD＝（y1z2-z1y2，z1x2-x1z2，x1y2-y1x2）
	Vec3f norm_Y2, norm_Z2, norm_Y3, norm_Z3;//norm_X为1,8的方向向量，定义为1到8点为正方向
	//2,3点
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
	//6.7点
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
	// step-6对标记好的点，按照标记点进行排序
	//由小到大排序//chengwei added
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

    //// 正则化　摄像机图像坐标
    normalizPixel(pointLeft,NormPixLeft,Lfc, Lcc, Lkc, Lalfpha_c);
    //// 正则化　投影机图像坐标
    normalizPixel(pointRight,NormPixRight,Rfc, Rcc, Rkc, Ralfpha_c);

    Mat Kp(3,3,CV_64F);

    //获得Mat类型的R、T
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

    //计算三维坐标,使用right相机的x坐标和y坐标
    for (size_t i=0;i<NormPixLeft.size();i++)
    {
        //给定系数矩阵，保存在Kp　３Ｘ３
        double temp = NormPixRight[i].x * T.at<double>(2,0) - T.at<double>(0,0);//这个值是什么含义？

        Kp.at<double>(0,0) = 1; Kp.at<double>(0,1) = 0; Kp.at<double>(0,2) = -NormPixLeft[i].x;
        Kp.at<double>(1,0) = 0; Kp.at<double>(1,1) = 1; Kp.at<double>(1,2) = -NormPixLeft[i].y;

        Kp.at<double>(2,0) = (R.at<double>(0,0) - NormPixRight[i].x * R.at<double>(2,0))/temp;
        Kp.at<double>(2,1) = (R.at<double>(0,1) - NormPixRight[i].x * R.at<double>(2,1))/temp;
        Kp.at<double>(2,2) = (R.at<double>(0,2) - NormPixRight[i].x * R.at<double>(2,2))/temp;

        Rvec.at<double>(2,0) = 1;

        Mat inverA(3,3,CV_64F);
        Mat point(3,1,CV_64F);

        invert(Kp,inverA);//求逆矩阵

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
	
    //step-1 计算左右相机的二维特征点点集
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
    //step-2 将特征点的标签信息去掉
    vector<Point2f> leftPnts2f,rightPnts2f;
    vector<int> TagVec;
    pntsTagConfirm(leftTagsPnts2f,rightTagsPnts2f,leftPnts2f,rightPnts2f,TagVec);
    //step-3 计算左相机摄像机坐标系下的三维点信息
    vector<Point3f> pnts3fVec;
    //第一种计算三维点信息的方法

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
	//STEP-1:中心点提取
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


	///对检测出的椭圆中心坐标求其质心
	if (!ClpMeasurement::calculateCentroid(CenterPnts2f_Left, CentroidPnts2f_Left))
	{
		return false;
	}
	///对检测出的椭圆中心坐标求其质心
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










////////重载函数getTagPoint3f()，锁定ROI区域。
bool CoreAlgorithm::getTagPoint3f(const Mat& img1, const cv::Rect maskLeftLightPen, const CamPara _campara1, const Mat& img2, const cv::Rect maskRightLightPen, const CamPara _campara2,
	const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& tagPnts3f)
{
	Mat cameraMatrixLeft = (cv::Mat_<double>(3, 3) << _campara1.CameraIntrinsic[0][0], _campara1.CameraIntrinsic[0][1], _campara1.CameraIntrinsic[0][2]
		, _campara1.CameraIntrinsic[1][0], _campara1.CameraIntrinsic[1][1], _campara1.CameraIntrinsic[1][2]
		, _campara1.CameraIntrinsic[2][0], _campara1.CameraIntrinsic[2][1], _campara1.CameraIntrinsic[2][2]);
	Mat cameraMatrixRight = (cv::Mat_<double>(3, 3) << _campara2.CameraIntrinsic[0][0], _campara2.CameraIntrinsic[0][1], _campara2.CameraIntrinsic[0][2]
		, _campara2.CameraIntrinsic[1][0], _campara2.CameraIntrinsic[1][1], _campara2.CameraIntrinsic[1][2]
		, _campara2.CameraIntrinsic[2][0], _campara2.CameraIntrinsic[2][1], _campara2.CameraIntrinsic[2][2]);

	//step-1 计算左右相机的二维特征点点集
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
	//step-2 将特征点的标签信息去掉
	vector<Point2f> leftPnts2f, rightPnts2f;
	vector<int> TagVec;
	pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);
	//step-3 计算左相机摄像机坐标系下的三维点信息
	vector<Point3f> pnts3fVec;
	//第一种计算三维点信息的方法

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




////////重载函数getTagPoint3f()，锁定ROI区域,输出左右图像中的光笔质心点
bool CoreAlgorithm::getTagPoint3f(const Mat& img1, const cv::Rect maskLeftLightPen, const CamPara _campara1, const Mat& img2, const cv::Rect maskRightLightPen, const CamPara _campara2,
	const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& tagPnts3f, Point2f &LightPenCentroidPnt_Left, Point2f &LightPenCentroidPnt_Right)
{
	Mat cameraMatrixLeft = (cv::Mat_<double>(3, 3) << _campara1.CameraIntrinsic[0][0], _campara1.CameraIntrinsic[0][1], _campara1.CameraIntrinsic[0][2]
		, _campara1.CameraIntrinsic[1][0], _campara1.CameraIntrinsic[1][1], _campara1.CameraIntrinsic[1][2]
		, _campara1.CameraIntrinsic[2][0], _campara1.CameraIntrinsic[2][1], _campara1.CameraIntrinsic[2][2]);
	Mat cameraMatrixRight = (cv::Mat_<double>(3, 3) << _campara2.CameraIntrinsic[0][0], _campara2.CameraIntrinsic[0][1], _campara2.CameraIntrinsic[0][2]
		, _campara2.CameraIntrinsic[1][0], _campara2.CameraIntrinsic[1][1], _campara2.CameraIntrinsic[1][2]
		, _campara2.CameraIntrinsic[2][0], _campara2.CameraIntrinsic[2][1], _campara2.CameraIntrinsic[2][2]);

	//step-1 计算左右相机的二维特征点点集
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
	
	//step-2 将特征点的标签信息去掉
	vector<Point2f> leftPnts2f, rightPnts2f;
	vector<int> TagVec;
	pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);
	//step-3 计算左相机摄像机坐标系下的三维点信息
	vector<Point3f> pnts3fVec;
	//第一种计算三维点信息的方法

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
	//求出圆中心投影点
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
		//求出一般方程
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
	//求出圆中心投影点
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
		//求出一般方程
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
	///////STEP1:图像二值化处理
	Mat thresholdImg;
	threshold(img, thresholdImg, 100, 255, 0);///此处阈值的设定如果较小的话，如果噪声添加较大，将影响最终结果。
	//////STEP2:图像的形态学处理（腐蚀和膨胀）(开运算：先腐蚀再膨胀)
	/////STEP2.1:先腐蚀；
	//int dilateKernelSize = 1;///定义膨胀的内核大小
	//Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2 * dilateKernelSize + 1, 2 * dilateKernelSize + 1), Point(-1, -1));///此处的Point（-1，-1）表示锚位于中心，为默认值，可以不写。
	//Mat erodeImg;
	//erode(thresholdImg, erodeImg, element);
	////////STEP2.2:再膨胀；
	//Mat dilateImg;
	//dilate(erodeImg, dilateImg, element);
	//////STEP3:寻找椭圆轮廓
	///将二值化后的图像绘制出来
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
		///////STEP4:用椭圆拟合二维点集
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

//////识别编码点
//编码标志点识别
bool CoreAlgorithm::findCircularMarker(const Mat img, vector<pair<unsigned int, Point2f>> &results_center)
{

	//将图像转化成灰度图
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

	//利用轮廓的椭圆检测
	vector<RotatedRect> ellipses0;
	CoreAlgorithm::detectEllipse(imgOriginal, ellipses0);

	//ellipses0中储存的长轴、短轴转换为半长轴、半短轴
	for (int i = 0; i < ellipses0.size(); i++)
	{
		ellipses0[i].size.width = ellipses0[i].size.width / 2;
		ellipses0[i].size.height = ellipses0[i].size.height / 2;
	}

	//第一次筛选椭圆
	vector<RotatedRect> ellipses1;
	for (int i = 0; i < ellipses0.size(); ++i) {
		if (ellipses0[i].size.height / ellipses0[i].size.width > 0 && ellipses0[i].size.height / ellipses0[i].size.width < 1.6 && ellipses0[i].size.height > 4)
		{
			ellipses1.push_back(ellipses0[i]);
		}
	}

	//第二次筛选：若两圆心距离小于大圆半径，剔除大圆
	vector<RotatedRect> ellipses2;
	bool WrongCircle = false;
	for (int i = 0; i < ellipses1.size(); ++i) {
		//判断是否有其他圆心在此圆内,而且此圆半径较大
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
		//若无，则记此圆为经过第二次筛选的圆
		if (!WrongCircle)
		{
			ellipses2.push_back(ellipses1[i]);
		}
		WrongCircle = false;
	}

	//第三次筛选：剔除太大的圆
	//根据偏离平均值的程度排除误检测的椭圆，由于第一次筛选椭圆时已剔除半长轴小于4个像素的椭圆，此处只对太大的圆进行剔除
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

	//第四次筛选：剔除不符合编码标志点形态特征的圆
	Mat img1, img2;
	vector<RotatedRect> ellipses4;
	cv::Rect_<float> region(0, 0, 0, 0);
	Point2f img1_center;
	vector<vector<Point>> contours1;

	for (int j = 0; j < ellipses3.size(); j++)
	{
		//选择周围区域
		region.x = ellipses3[j].center.x - 4 * ellipses3[j].size.height;
		region.y = ellipses3[j].center.y - 4 * ellipses3[j].size.height;
		region.width = 8 * ellipses3[j].size.height;
		region.height = 8 * ellipses3[j].size.height;

		//排除太靠近边缘的圆
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

		//二值化
		threshold(img1, img2, 100, 255, CV_THRESH_BINARY);

		//查找连通域
		findContours(img2, contours1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//排除周围连通域面积太小的圆
		for (int i = 0; i < contours1.size(); i++) {
			if (contourArea(contours1[i], false) >(0.35 * 3.14 * ellipses3[j].size.height * ellipses3[j].size.height))
			{
				ellipses4.push_back(ellipses3[j]);
				break;
			}
		}
	}

	//利用梯度的椭圆检测
	cv::Rect_<float> mask(0, 0, 0, 0);
	int kenelsize = 3;
	double precisionlevel = 0.04;
	bool multi = 0;
	vector<RotatedRect>  findResults;
	Point2f center_temp;
	vector<Point2f> vec_center_temp;//检测到的圆心（经过基于梯度的椭圆检测函数findEllipses最终确定的编码标志点中心圆）
	vector<float> vec_radius_temp;//半径（取半长轴为半径）（经过基于梯度的椭圆检测函数findEllipses最终确定的编码标志点中心圆）
	Mat imgROI = imgOriginal.clone();

	for (int i = 0; i < ellipses4.size(); i++)
	{
		if (ellipses4[i].center.x - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.y - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.x + 2 * ellipses4[i].size.height < imgOriginal.size().width && ellipses4[i].center.y + 2 * ellipses4[i].size.height < imgOriginal.size().height)
		{
			//确定每个进行椭圆检测的小区域
			mask.x = ellipses4[i].center.x - 2 * ellipses4[i].size.height;
			mask.y = ellipses4[i].center.y - 2 * ellipses4[i].size.height;
			mask.width = 4 * ellipses4[i].size.height;
			mask.height = 4 * ellipses4[i].size.height;

			//画出每个进行椭圆检测的小区域
			rectangle(imgROI, mask, 255, 3);

			//基于梯度的椭圆检测
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

	////显示所有进行椭圆检测的小区域
	//namedWindow("ROI", 0);
	//imshow("ROI", imgROI);
	//while (char(waitKey(1)) != ' ') {}

	//画出检测出的全部圆
	Mat img4;
	cvtColor(imgOriginal, img4, CV_GRAY2BGR);
	for (int i = 0; i < vec_radius_temp.size(); i++) {
		circle(img4, vec_center_temp[i], vec_radius_temp[i], Scalar(0, 0, 255), 2, 8, 0);
	}
	/*namedWindow("Circles", 0);
	imshow("Circles", img4);
	while (char(waitKey(1)) != ' ') {}*/


	//计算每个标志的编码
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
	vector<string> BinaryCode;//存放最终确定的二进制编码
	vector<int> Code;//存放最终确定的十进制编码
	cv::Rect_<float> ROI_marker(0, 0, 0, 0);
	cv::Rect_<float> ROI_circle(0, 0, 0, 0);
	Point2f img5_center, img7_center;
	int maxcontour;
	vector<Point2f> Coordinate;	//存放最终确定的编码标志点中心坐标

	for (int i = 0; i < vec_center_temp.size(); i++)//第i个编码标志
	{
		//对环形编码带进行处理

		//选择编码标志点所在区域
		ROI_marker.x = vec_center_temp[i].x - 4 * vec_radius_temp[i];
		ROI_marker.y = vec_center_temp[i].y - 4 * vec_radius_temp[i];
		ROI_marker.width = 8 * vec_radius_temp[i];
		ROI_marker.height = 8 * vec_radius_temp[i];

		//排除太靠近边缘的圆
		if (ROI_marker.x < 0 || ROI_marker.y < 0 || (vec_center_temp[i].x + 4 * vec_radius_temp[i]) > imgOriginal.cols || (vec_center_temp[i].y + 4 * vec_radius_temp[i]) > imgOriginal.rows)
		{
			continue;
		}

		//用黑色把中心圆和环形编码带外围覆盖
		img5 = imgOriginal.clone();
		img5 = Mat(img5, ROI_marker);
		img5_center.x = img5.cols / 2;
		img5_center.y = img5.rows / 2;
		//circle(img5, img5_center, 3.8 * vec_radius_temp[i] + 2 * vec_radius_temp[i], Scalar(0, 0, 0), 4 * vec_radius_temp[i], 8, 0);//环形编码带外围（似乎不需要涂黑）
		circle(img5, img5_center, vec_radius_temp[i] / 2, Scalar(0, 0, 0), 2 * vec_radius_temp[i], 8, 0);//中心圆
		//namedWindow("查看环形编码带", 0);
		//imshow("查看环形编码带", img5);
		//while (char(waitKey(1)) != ' ') {}

		//二值化
		threshold(img5, img6, 100, 255, CV_THRESH_BINARY);


		//查找连通域
		findContours(img6, contours_temp, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//求环形编码带面积
		for (int j = 0; j < contours_temp.size(); j++) {
			area_temp.push_back(contourArea(contours_temp[j], false));
		}

		//排除太小的连通域
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

		//若轮廓数为0，则不是编码标志点
		if (Num_Contours == 0)
		{
			continue;
		}

		//若有四个轮廓，则编码已确定
		if (Num_Contours == 4)
		{
			BinaryCode.push_back("01010101");
			Coordinate.push_back(vec_center_temp[i]);

			//清除本次循环储存的数据
			contours.clear();
			area.clear();
			//continue;
		}

		//若有三个轮廓，则编码已确定
		else if (Num_Contours == 3)
		{
			BinaryCode.push_back("01010111");
			Coordinate.push_back(vec_center_temp[i]);

			//清除本次循环储存的数据
			contours.clear();
			area.clear();
			//continue;
		}

		//若不是四个或三个轮廓，则继续进行以下运算

		//对中心圆采用连通域的方法求面积
		//截取包含中心圆的小区域
		else
		{
			ROI_circle.x = vec_center_temp[i].x - 1.5 * vec_radius_temp[i];
			ROI_circle.y = vec_center_temp[i].y - 1.5 * vec_radius_temp[i];
			ROI_circle.width = 3 * vec_radius_temp[i];
			ROI_circle.height = 3 * vec_radius_temp[i];

			img7 = imgOriginal.clone();
			img7 = Mat(img7, ROI_circle);

			//中心圆的周围涂黑
			img7_center.x = img7.cols / 2;
			img7_center.y = img7.rows / 2;
			circle(img7, img7_center, 1.5 * vec_radius_temp[i] + 0.75 * vec_radius_temp[i], Scalar(0, 0, 0), 1.5 * vec_radius_temp[i], 8, 0);

			//二值化
			threshold(img7, img8, 100, 255, CV_THRESH_BINARY);

			//查找连通域
			findContours(img8, contours_center, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

			//保留最大的一个连通域
			maxcontour = 0;
			for (int j = 1; j < contours_center.size(); j++)
			{
				if (contours_center[j].size() > contours_center[maxcontour].size())
				{
					maxcontour = j;
				}
			}
			contours_center_max = contours_center[maxcontour];

			//求中心圆面积
			ss = contourArea(contours_center_max, false);

			//确定每一块连通域对应的编码

			//设计编码标志点时，单个环形编码带只有1、3、5、7的情况
			for (int j = 0; j < Num_Contours; ++j)
			{
				if (area[j] >(0.388*ss) && area[j] < (1.550*ss))   SingleNumber.push_back(1);
				else if (area[j] > (1.550*ss) && area[j] < (3.100*ss)) SingleNumber.push_back(3);
				else if (area[j] > (3.100*ss) && area[j] < (4.650*ss)) SingleNumber.push_back(5);
				else if (area[j] > (4.650*ss) && area[j] < (6.200*ss)) SingleNumber.push_back(7);
			}

			//若只有一个轮廓，则编码已确定

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

				//清除本次循环储存的数据
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				SingleNumber.clear();
				area.clear();
				//continue;
			}

			//若不是一个轮廓，则继续进行以下运算

			//若有两个轮廓
			else if (Num_Contours == 2)
			{
				//计算轮廓矩
				for (int j = 0; j < Num_Contours; j++)
				{
					mu.push_back(moments(contours[j], false));
				}

				//计算轮廓的质心
				for (int j = 0; j < Num_Contours; j++)
				{
					mc.push_back(Point2f(mu[j].m10 / mu[j].m00, mu[j].m01 / mu[j].m00));
				}

				//计算连通域质心与中心圆圆心连线的角度
				for (int j = 0; j < Num_Contours; ++j) {
					Angle.push_back(atan2(mc[j].y - img5_center.y, mc[j].x - img5_center.x) / 3.1416 * 180);
				}

				//将角度范围转换为0-360度
				for (int j = 0; j < Num_Contours; ++j) {
					if (Angle[j] < 0) Angle[j] = Angle[j] + 360;
				}

				//根据两段环形编码带之间的夹角，确定它们之间相隔了几个单位
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

				//转换为唯一的二进制编码
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

				//清除本次循环储存的数据
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
	}//对每个编码标志处理的for循环结束


	//将二进制编码转换为十进制编码
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

	//输出检测结果：编码、中心圆的圆心坐标
	/*cout << "\n编码\t中心圆的圆心坐标" << endl;
	for (int j = 0; j < Code.size(); j++)
	{
	cout << Code[j] << "\t" << Coordinate[j] << endl;
	}*/
	//char *fname = "E:/Data/编码点坐标.txt";
	//ofstream fout(fname, ios::app);
	//fout << "\n编码\t中心圆的圆心坐标" << endl;
	//for (int j = 0; j < Code.size(); j++)
	//{
	//	fout << Code[j] << "\t" << Coordinate[j] << endl;
	//}
	//fout.close();

	pair<unsigned int, Point2f> marker;

	//储存结果
	for (int j = 0; j < Code.size(); j++)
	{
		marker.first = Code[j];
		marker.second = Coordinate[j];
		results_center.push_back(marker);
	}
	/////进行排序
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

	//将图像转化成灰度图
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

	//利用轮廓的椭圆检测
	vector<RotatedRect> ellipses0;
	CoreAlgorithm::detectEllipse(imgOriginal, ellipses0);

	//ellipses0中储存的长轴、短轴转换为半长轴、半短轴
	for (int i = 0; i < ellipses0.size(); i++)
	{
		ellipses0[i].size.width = ellipses0[i].size.width / 2;
		ellipses0[i].size.height = ellipses0[i].size.height / 2;
	}

	//第一次筛选椭圆
	vector<RotatedRect> ellipses1;
	for (int i = 0; i < ellipses0.size(); ++i) {
		if (ellipses0[i].size.height / ellipses0[i].size.width > 0 && ellipses0[i].size.height / ellipses0[i].size.width < 1.6 && ellipses0[i].size.height > 4)
		{
			ellipses1.push_back(ellipses0[i]);
		}
	}

	//第二次筛选：若两圆心距离小于大圆半径，剔除大圆
	vector<RotatedRect> ellipses2;
	bool WrongCircle = false;
	for (int i = 0; i < ellipses1.size(); ++i) {
		//判断是否有其他圆心在此圆内,而且此圆半径较大
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
		//若无，则记此圆为经过第二次筛选的圆
		if (!WrongCircle)
		{
			ellipses2.push_back(ellipses1[i]);
		}
		WrongCircle = false;
	}

	//第三次筛选：剔除太大的圆
	//根据偏离平均值的程度排除误检测的椭圆，由于第一次筛选椭圆时已剔除半长轴小于4个像素的椭圆，此处只对太大的圆进行剔除
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

	//第四次筛选：剔除不符合编码标志点形态特征的圆
	Mat img1, img2;
	vector<RotatedRect> ellipses4;
	cv::Rect_<float> region(0, 0, 0, 0);
	Point2f img1_center;
	vector<vector<Point>> contours1;

	for (int j = 0; j < ellipses3.size(); j++)
	{
		//选择周围区域
		region.x = ellipses3[j].center.x - 4 * ellipses3[j].size.height;
		region.y = ellipses3[j].center.y - 4 * ellipses3[j].size.height;
		region.width = 8 * ellipses3[j].size.height;
		region.height = 8 * ellipses3[j].size.height;

		//排除太靠近边缘的圆
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

		//二值化
		threshold(img1, img2, 100, 255, CV_THRESH_BINARY);

		//查找连通域
		findContours(img2, contours1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//排除周围连通域面积太小的圆
		for (int i = 0; i < contours1.size(); i++) {
			if (contourArea(contours1[i], false) >(0.35 * 3.14 * ellipses3[j].size.height * ellipses3[j].size.height))
			{
				ellipses4.push_back(ellipses3[j]);
				break;
			}
		}
	}

	//利用梯度的椭圆检测
	cv::Rect_<float> mask(0, 0, 0, 0);
	int kenelsize = 3;
	double precisionlevel = 0.04;
	bool multi = 0;
	vector<RotatedRect>  findResults;
	Point2f center_temp;
	vector<Point2f> vec_center_temp;//检测到的圆心（经过基于梯度的椭圆检测函数findEllipses最终确定的编码标志点中心圆）
	vector<float> vec_radius_temp;//半径（取半长轴为半径）（经过基于梯度的椭圆检测函数findEllipses最终确定的编码标志点中心圆）
	Mat imgROI = imgOriginal.clone();

	for (int i = 0; i < ellipses4.size(); i++)
	{
		if (ellipses4[i].center.x - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.y - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.x + 2 * ellipses4[i].size.height < imgOriginal.size().width && ellipses4[i].center.y + 2 * ellipses4[i].size.height < imgOriginal.size().height)
		{
			//确定每个进行椭圆检测的小区域
			mask.x = ellipses4[i].center.x - 2 * ellipses4[i].size.height;
			mask.y = ellipses4[i].center.y - 2 * ellipses4[i].size.height;
			mask.width = 4 * ellipses4[i].size.height;
			mask.height = 4 * ellipses4[i].size.height;

			//画出每个进行椭圆检测的小区域
			rectangle(imgROI, mask, 255, 3);

			//基于梯度的椭圆检测
			CoreAlgorithm::findEllipses(imgOriginal, mask, findResults, precisionlevel, multi, kenelsize);
			////qiyong added 
			////读取编码点中心的圆直径
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

	////显示所有进行椭圆检测的小区域
	//namedWindow("ROI", 0);
	//imshow("ROI", imgROI);
	//while (char(waitKey(1)) != ' ') {}

	//画出检测出的全部圆
	Mat img4;
	cvtColor(imgOriginal, img4, CV_GRAY2BGR);
	for (int i = 0; i < vec_radius_temp.size(); i++) {
		circle(img4, vec_center_temp[i], vec_radius_temp[i], Scalar(0, 0, 255), 2, 8, 0);
	}
	/*namedWindow("Circles", 0);
	imshow("Circles", img4);
	while (char(waitKey(1)) != ' ') {}*/


	//计算每个标志的编码
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
	vector<string> BinaryCode;//存放最终确定的二进制编码
	vector<int> Code;//存放最终确定的十进制编码
	cv::Rect_<float> ROI_marker(0, 0, 0, 0);
	cv::Rect_<float> ROI_circle(0, 0, 0, 0);
	Point2f img5_center, img7_center;
	int maxcontour;
	vector<Point2f> Coordinate;	//存放最终确定的编码标志点中心坐标

	for (int i = 0; i < vec_center_temp.size(); i++)//第i个编码标志
	{
		//对环形编码带进行处理

		//选择编码标志点所在区域
		ROI_marker.x = vec_center_temp[i].x - 4 * vec_radius_temp[i];
		ROI_marker.y = vec_center_temp[i].y - 4 * vec_radius_temp[i];
		ROI_marker.width = 8 * vec_radius_temp[i];
		ROI_marker.height = 8 * vec_radius_temp[i];

		//排除太靠近边缘的圆
		if (ROI_marker.x < 0 || ROI_marker.y < 0 || (vec_center_temp[i].x + 4 * vec_radius_temp[i]) > imgOriginal.cols || (vec_center_temp[i].y + 4 * vec_radius_temp[i]) > imgOriginal.rows)
		{
			continue;
		}

		//用黑色把中心圆和环形编码带外围覆盖
		img5 = imgOriginal.clone();
		img5 = Mat(img5, ROI_marker);
		img5_center.x = img5.cols / 2;
		img5_center.y = img5.rows / 2;
		//circle(img5, img5_center, 3.8 * vec_radius_temp[i] + 2 * vec_radius_temp[i], Scalar(0, 0, 0), 4 * vec_radius_temp[i], 8, 0);//环形编码带外围（似乎不需要涂黑）
		circle(img5, img5_center, vec_radius_temp[i] / 2, Scalar(0, 0, 0), 2 * vec_radius_temp[i], 8, 0);//中心圆
		//namedWindow("查看环形编码带", 0);
		//imshow("查看环形编码带", img5);
		//while (char(waitKey(1)) != ' ') {}

		//二值化
		threshold(img5, img6, 100, 255, CV_THRESH_BINARY);


		//查找连通域
		findContours(img6, contours_temp, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//求环形编码带面积
		for (int j = 0; j < contours_temp.size(); j++) {
			area_temp.push_back(contourArea(contours_temp[j], false));
		}

		//排除太小的连通域
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

		//若轮廓数为0，则不是编码标志点
		if (Num_Contours == 0)
		{
			continue;
		}

		//若有四个轮廓，则编码已确定
		if (Num_Contours == 4)
		{
			BinaryCode.push_back("01010101");
			Coordinate.push_back(vec_center_temp[i]);

			//清除本次循环储存的数据
			contours.clear();
			area.clear();
			//continue;
		}

		//若有三个轮廓，则编码已确定
		else if (Num_Contours == 3)
		{
			BinaryCode.push_back("01010111");
			Coordinate.push_back(vec_center_temp[i]);

			//清除本次循环储存的数据
			contours.clear();
			area.clear();
			//continue;
		}

		//若不是四个或三个轮廓，则继续进行以下运算

		//对中心圆采用连通域的方法求面积
		//截取包含中心圆的小区域
		else
		{
			ROI_circle.x = vec_center_temp[i].x - 1.5 * vec_radius_temp[i];
			ROI_circle.y = vec_center_temp[i].y - 1.5 * vec_radius_temp[i];
			ROI_circle.width = 3 * vec_radius_temp[i];
			ROI_circle.height = 3 * vec_radius_temp[i];

			img7 = imgOriginal.clone();
			img7 = Mat(img7, ROI_circle);

			//中心圆的周围涂黑
			img7_center.x = img7.cols / 2;
			img7_center.y = img7.rows / 2;
			circle(img7, img7_center, 1.5 * vec_radius_temp[i] + 0.75 * vec_radius_temp[i], Scalar(0, 0, 0), 1.5 * vec_radius_temp[i], 8, 0);

			//二值化
			threshold(img7, img8, 100, 255, CV_THRESH_BINARY);

			//查找连通域
			findContours(img8, contours_center, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

			//保留最大的一个连通域
			maxcontour = 0;
			for (int j = 1; j < contours_center.size(); j++)
			{
				if (contours_center[j].size() > contours_center[maxcontour].size())
				{
					maxcontour = j;
				}
			}
			contours_center_max = contours_center[maxcontour];

			//求中心圆面积
			ss = contourArea(contours_center_max, false);

			//确定每一块连通域对应的编码

			//设计编码标志点时，单个环形编码带只有1、3、5、7的情况
			for (int j = 0; j < Num_Contours; ++j)
			{
				if (area[j] >(0.388*ss) && area[j] < (1.550*ss))   SingleNumber.push_back(1);
				else if (area[j] > (1.550*ss) && area[j] < (3.100*ss)) SingleNumber.push_back(3);
				else if (area[j] > (3.100*ss) && area[j] < (4.650*ss)) SingleNumber.push_back(5);
				else if (area[j] > (4.650*ss) && area[j] < (6.200*ss)) SingleNumber.push_back(7);
			}

			//若只有一个轮廓，则编码已确定

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

				//清除本次循环储存的数据
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				SingleNumber.clear();
				area.clear();
				//continue;
			}

			//若不是一个轮廓，则继续进行以下运算

			//若有两个轮廓
			else if (Num_Contours == 2)
			{
				//计算轮廓矩
				for (int j = 0; j < Num_Contours; j++)
				{
					mu.push_back(moments(contours[j], false));
				}

				//计算轮廓的质心
				for (int j = 0; j < Num_Contours; j++)
				{
					mc.push_back(Point2f(mu[j].m10 / mu[j].m00, mu[j].m01 / mu[j].m00));
				}

				//计算连通域质心与中心圆圆心连线的角度
				for (int j = 0; j < Num_Contours; ++j) {
					Angle.push_back(atan2(mc[j].y - img5_center.y, mc[j].x - img5_center.x) / 3.1416 * 180);
				}

				//将角度范围转换为0-360度
				for (int j = 0; j < Num_Contours; ++j) {
					if (Angle[j] < 0) Angle[j] = Angle[j] + 360;
				}

				//根据两段环形编码带之间的夹角，确定它们之间相隔了几个单位
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

				//转换为唯一的二进制编码
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

				//清除本次循环储存的数据
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
	}//对每个编码标志处理的for循环结束


	//将二进制编码转换为十进制编码
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

	//输出检测结果：编码、中心圆的圆心坐标
	/*cout << "\n编码\t中心圆的圆心坐标" << endl;
	for (int j = 0; j < Code.size(); j++)
	{
	cout << Code[j] << "\t" << Coordinate[j] << endl;
	}*/
	//char *fname = "E:/Data/编码点坐标.txt";
	//ofstream fout(fname, ios::app);
	//fout << "\n编码\t中心圆的圆心坐标" << endl;
	//for (int j = 0; j < Code.size(); j++)
	//{
	//	fout << Code[j] << "\t" << Coordinate[j] << endl;
	//}
	//fout.close();

	pair<unsigned int, Point2f> marker;

	//储存结果
	for (int j = 0; j < Code.size(); j++)
	{
		marker.first = Code[j];
		marker.second = Coordinate[j];
		results_center.push_back(marker);
	}
	/////进行排序
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







////重载
bool CoreAlgorithm::findCircularMarker(const Mat img, const cv::Rect Mask, vector<pair<unsigned int, Point2f>> &results_center)
{

	//将图像转化成灰度图
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

	//利用轮廓的椭圆检测
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
	///将该ROI区域显示出来
	//cv::namedWindow("maskRoi", 1);
	//cv::imshow("maskRoi", imgOriginal);
	//cv::waitKey(10);
	
 	CoreAlgorithm::detectEllipse(imgOriginal, ellipses0); 

	//ellipses0中储存的长轴、短轴转换为半长轴、半短轴
	for (int i = 0; i < ellipses0.size(); i++)
	{
		ellipses0[i].size.width = ellipses0[i].size.width / 2;
		ellipses0[i].size.height = ellipses0[i].size.height / 2;
	}

	//第一次筛选椭圆
	vector<RotatedRect> ellipses1;
	for (int i = 0; i < ellipses0.size(); ++i) {
		if (ellipses0[i].size.height / ellipses0[i].size.width > 0 && ellipses0[i].size.height / ellipses0[i].size.width < 1.6 && ellipses0[i].size.height > 4)
		{
			ellipses1.push_back(ellipses0[i]);
		}
	}

	//第二次筛选：若两圆心距离小于大圆半径，剔除大圆
	vector<RotatedRect> ellipses2;
	bool WrongCircle = false;
	for (int i = 0; i < ellipses1.size(); ++i) {
		//判断是否有其他圆心在此圆内,而且此圆半径较大
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
		//若无，则记此圆为经过第二次筛选的圆
		if (!WrongCircle)
		{
			ellipses2.push_back(ellipses1[i]);
		}
		WrongCircle = false;
	}

	//第三次筛选：剔除太大的圆
	//根据偏离平均值的程度排除误检测的椭圆，由于第一次筛选椭圆时已剔除半长轴小于4个像素的椭圆，此处只对太大的圆进行剔除
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

	//第四次筛选：剔除不符合编码标志点形态特征的圆
	Mat img1, img2;
	vector<RotatedRect> ellipses4;
	cv::Rect_<float> region(0, 0, 0, 0);
	Point2f img1_center;
	vector<vector<Point>> contours1;

	for (int j = 0; j < ellipses3.size(); j++)
	{
		//选择周围区域
		region.x = ellipses3[j].center.x - 4 * ellipses3[j].size.height;
		region.y = ellipses3[j].center.y - 4 * ellipses3[j].size.height;
		region.width = 8 * ellipses3[j].size.height;
		region.height = 8 * ellipses3[j].size.height;

		//排除太靠近边缘的圆
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

		//二值化
		threshold(img1, img2, 100, 255, CV_THRESH_BINARY);

		//查找连通域
		findContours(img2, contours1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//排除周围连通域面积太小的圆
		for (int i = 0; i < contours1.size(); i++) {
			if (contourArea(contours1[i], false) >(0.35 * 3.14 * ellipses3[j].size.height * ellipses3[j].size.height))
			{
				ellipses4.push_back(ellipses3[j]);
				break;
			}
		}
	}

	//利用梯度的椭圆检测
	cv::Rect_<float> mask(0, 0, 0, 0);
	int kenelsize = 3;
	double precisionlevel = 0.04;
	bool multi = 0;
	vector<RotatedRect>  findResults;
	Point2f center_temp;
	vector<Point2f> vec_center_temp;//检测到的圆心（经过基于梯度的椭圆检测函数findEllipses最终确定的编码标志点中心圆）
	vector<float> vec_radius_temp;//半径（取半长轴为半径）（经过基于梯度的椭圆检测函数findEllipses最终确定的编码标志点中心圆）
	Mat imgROI = imgOriginal.clone();

	for (int i = 0; i < ellipses4.size(); i++)
	{
		if (ellipses4[i].center.x - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.y - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.x + 2 * ellipses4[i].size.height < imgOriginal.size().width && ellipses4[i].center.y + 2 * ellipses4[i].size.height < imgOriginal.size().height)
		{
			//确定每个进行椭圆检测的小区域
			mask.x = ellipses4[i].center.x - 2 * ellipses4[i].size.height;
			mask.y = ellipses4[i].center.y - 2 * ellipses4[i].size.height;
			mask.width = 4 * ellipses4[i].size.height;
			mask.height = 4 * ellipses4[i].size.height;

			//画出每个进行椭圆检测的小区域
			rectangle(imgROI, mask, 255, 3);

			//基于梯度的椭圆检测
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

	////显示所有进行椭圆检测的小区域
	//namedWindow("ROI", 0);
	//imshow("ROI", imgROI);
	//while (char(waitKey(1)) != ' ') {}

	//画出检测出的全部圆
	Mat img4;
	cvtColor(imgOriginal, img4, CV_GRAY2BGR);
	for (int i = 0; i < vec_radius_temp.size(); i++) {
		circle(img4, vec_center_temp[i], vec_radius_temp[i], Scalar(0, 0, 255), 2, 8, 0);
	}
	/*namedWindow("Circles", 0);
	imshow("Circles", img4);
	while (char(waitKey(1)) != ' ') {}*/


	//计算每个标志的编码
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
	vector<string> BinaryCode;//存放最终确定的二进制编码
	vector<int> Code;//存放最终确定的十进制编码
	cv::Rect_<float> ROI_marker(0, 0, 0, 0);
	cv::Rect_<float> ROI_circle(0, 0, 0, 0);
	Point2f img5_center, img7_center;
	int maxcontour;
	vector<Point2f> Coordinate;	//存放最终确定的编码标志点中心坐标

	for (int i = 0; i < vec_center_temp.size(); i++)//第i个编码标志
	{
		//对环形编码带进行处理

		//选择编码标志点所在区域
		ROI_marker.x = vec_center_temp[i].x - 4 * vec_radius_temp[i];
		ROI_marker.y = vec_center_temp[i].y - 4 * vec_radius_temp[i];
		ROI_marker.width = 8 * vec_radius_temp[i];
		ROI_marker.height = 8 * vec_radius_temp[i];

		//排除太靠近边缘的圆
		if (ROI_marker.x < 0 || ROI_marker.y < 0 || (vec_center_temp[i].x + 4 * vec_radius_temp[i]) > imgOriginal.cols || (vec_center_temp[i].y + 4 * vec_radius_temp[i]) > imgOriginal.rows)
		{
			continue;
		}

		//用黑色把中心圆和环形编码带外围覆盖
		img5 = imgOriginal.clone();
		img5 = Mat(img5, ROI_marker);
		img5_center.x = img5.cols / 2;
		img5_center.y = img5.rows / 2;
		//circle(img5, img5_center, 3.8 * vec_radius_temp[i] + 2 * vec_radius_temp[i], Scalar(0, 0, 0), 4 * vec_radius_temp[i], 8, 0);//环形编码带外围（似乎不需要涂黑）
		circle(img5, img5_center, vec_radius_temp[i] / 2, Scalar(0, 0, 0), 2 * vec_radius_temp[i], 8, 0);//中心圆
		//namedWindow("查看环形编码带", 0);
		//imshow("查看环形编码带", img5);
		//while (char(waitKey(1)) != ' ') {}

		//二值化
		threshold(img5, img6, 100, 255, CV_THRESH_BINARY);


		//查找连通域
		findContours(img6, contours_temp, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//求环形编码带面积
		for (int j = 0; j < contours_temp.size(); j++) {
			area_temp.push_back(contourArea(contours_temp[j], false));
		}

		//排除太小的连通域
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

		//若轮廓数为0，则不是编码标志点
		if (Num_Contours == 0)
		{
			continue;
		}

		//若有四个轮廓，则编码已确定
		if (Num_Contours == 4)
		{
			BinaryCode.push_back("01010101");
			Coordinate.push_back(vec_center_temp[i]);

			//清除本次循环储存的数据
			contours.clear();
			area.clear();
			//continue;
		}

		//若有三个轮廓，则编码已确定
		else if (Num_Contours == 3)
		{
			BinaryCode.push_back("01010111");
			Coordinate.push_back(vec_center_temp[i]);

			//清除本次循环储存的数据
			contours.clear();
			area.clear();
			//continue;
		}

		//若不是四个或三个轮廓，则继续进行以下运算

		//对中心圆采用连通域的方法求面积
		//截取包含中心圆的小区域
		else
		{
			ROI_circle.x = vec_center_temp[i].x - 1.5 * vec_radius_temp[i];
			ROI_circle.y = vec_center_temp[i].y - 1.5 * vec_radius_temp[i];
			ROI_circle.width = 3 * vec_radius_temp[i];
			ROI_circle.height = 3 * vec_radius_temp[i];

			img7 = imgOriginal.clone();
			img7 = Mat(img7, ROI_circle);

			//中心圆的周围涂黑
			img7_center.x = img7.cols / 2;
			img7_center.y = img7.rows / 2;
			circle(img7, img7_center, 1.5 * vec_radius_temp[i] + 0.75 * vec_radius_temp[i], Scalar(0, 0, 0), 1.5 * vec_radius_temp[i], 8, 0);

			//二值化
			threshold(img7, img8, 100, 255, CV_THRESH_BINARY);

			//查找连通域
			findContours(img8, contours_center, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

			//保留最大的一个连通域
			maxcontour = 0;
			for (int j = 1; j < contours_center.size(); j++)
			{
				if (contours_center[j].size() > contours_center[maxcontour].size())
				{
					maxcontour = j;
				}
			}
			contours_center_max = contours_center[maxcontour];

			//求中心圆面积
			ss = contourArea(contours_center_max, false);

			//确定每一块连通域对应的编码

			//设计编码标志点时，单个环形编码带只有1、3、5、7的情况
			for (int j = 0; j < Num_Contours; ++j)
			{
				if (area[j] >(0.388*ss) && area[j] < (1.550*ss))   SingleNumber.push_back(1);
				else if (area[j] > (1.550*ss) && area[j] < (3.100*ss)) SingleNumber.push_back(3);
				else if (area[j] > (3.100*ss) && area[j] < (4.650*ss)) SingleNumber.push_back(5);
				else if (area[j] > (4.650*ss) && area[j] < (6.200*ss)) SingleNumber.push_back(7);
			}

			//若只有一个轮廓，则编码已确定

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

				//清除本次循环储存的数据
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				SingleNumber.clear();
				area.clear();
				//continue;
			}

			//若不是一个轮廓，则继续进行以下运算

			//若有两个轮廓
			else if (Num_Contours == 2)
			{
				//计算轮廓矩
				for (int j = 0; j < Num_Contours; j++)
				{
					mu.push_back(moments(contours[j], false));
				}

				//计算轮廓的质心
				for (int j = 0; j < Num_Contours; j++)
				{
					mc.push_back(Point2f(mu[j].m10 / mu[j].m00, mu[j].m01 / mu[j].m00));
				}

				//计算连通域质心与中心圆圆心连线的角度
				for (int j = 0; j < Num_Contours; ++j) {
					Angle.push_back(atan2(mc[j].y - img5_center.y, mc[j].x - img5_center.x) / 3.1416 * 180);
				}

				//将角度范围转换为0-360度
				for (int j = 0; j < Num_Contours; ++j) {
					if (Angle[j] < 0) Angle[j] = Angle[j] + 360;
				}

				//根据两段环形编码带之间的夹角，确定它们之间相隔了几个单位
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

				//转换为唯一的二进制编码
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

				//清除本次循环储存的数据
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
	}//对每个编码标志处理的for循环结束


	//将二进制编码转换为十进制编码
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

	//输出检测结果：编码、中心圆的圆心坐标
	/*cout << "\n编码\t中心圆的圆心坐标" << endl;
	for (int j = 0; j < Code.size(); j++)
	{
	cout << Code[j] << "\t" << Coordinate[j] << endl;
	}*/
	//char *fname = "E:/Data/编码点坐标.txt";
	//ofstream fout(fname, ios::app);
	//fout << "\n编码\t中心圆的圆心坐标" << endl;
	//for (int j = 0; j < Code.size(); j++)
	//{
	//	fout << Code[j] << "\t" << Coordinate[j] << endl;
	//}
	//fout.close();

	pair<unsigned int, Point2f> marker;

	//储存结果
	for (int j = 0; j < Code.size(); j++)
	{
		marker.first = Code[j];
		marker.second = Coordinate[j];
		marker.second.x += MaskRoi.x;
		marker.second.y += MaskRoi.y;
		results_center.push_back(marker);
	}
	///// 对标记好的点，按照标记点进行排序
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











/////计算空间一条直线外一点到这条直线的垂足点坐标。
Point3f CoreAlgorithm::GetFootOfPerpendicular(
	const Point3f &pt,     // 直线外一点  
	const Point3f &begin,  // 直线开始点  
	const Point3f &end)   // 直线结束点  
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

//////创建编码点相对于相机的坐标系转换
//void CoreAlgorithm::createCodepntsCoordinateSVD(vector<TagPoint3f> tagPnts, vector<TagPoint3f>& CodepntsCenterToCam)
//{
//	//////将tagPnts中的标签取出，放在容器中
//	vector<int> tags;
//	for (size_t i = 0; i < tagPnts.size(); i++)
//	{
//		tags.push_back(int(tagPnts[i][0]));
//	}
//	///计算这些排好序的三维点的质心，作为编码点坐标系的原点
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
//   ///////将srcMat矩阵输出到txt文本中
//   //char *fname = "F:/Data/srcMat and R.txt.txt";
//   //ofstream fout(fname, ios::app);
//   //fout <<"srcMat== "<<srcMat << endl;
//   //fout.close();
//
//   /////对上述一系列三维点进行奇异值分解
//   Mat S, U, VT;
//   SVD::compute(srcMat,S,U,VT,SVD::FULL_UV);////VT此处为V的转置矩阵，也就是奇异向量的转置
//   //////选择最大的特征值的特征向量作为x轴，选择第二个特征值的特征向量与x轴垂直的矢量作为y轴，z轴是x,y轴的叉乘
//
//   Vec3f norm_X, norm_Y;
//   norm_X[0] = VT.at<float>(0, 0);
//   norm_X[1] = VT.at<float>(1, 0);
//   norm_X[2] = VT.at<float>(2, 0);
//   /////将三个向量分别与坐标系X,Y,Z,其中x=(1,0,0);y=(0,1,0);z=(0,0,1),相乘，如果为负，则取反。
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
//   ////三个单位向量保存为R，编码点坐标原点为T
//    Mat R(3, 3, CV_32F);
//  /*  T.at<float>(0, 0) = CentroidPoint.x;
//   T.at<float>(1, 0) = CentroidPoint.y;
//   T.at<float>(2, 0) = CentroidPoint.z;*/
//   R.at<float>(0, 0) = norm_X[0]; R.at<float>(1, 0) = norm_X[1]; R.at<float>(2, 0) = norm_X[2];
//   R.at<float>(0, 1) = norm_Y[0]; R.at<float>(1, 1) = norm_Y[1]; R.at<float>(2, 1) = norm_Y[2];
//   R.at<float>(0, 2) = norm_Z[0]; R.at<float>(1, 2) = norm_Z[1]; R.at<float>(2, 2) = norm_Z[2];
//
//   ///////将srcMat矩阵输出到txt文本中
//   //char *fname = "F:/Data/srcMat and R.txt";
//   //ofstream fout(fname, ios::app);
//   //fout <<"R== "<< R << endl;
//   //fout.close();
//
//   // /*  Mat R_invert;
//   //invert(R, R_invert);*/
//   ////STEP-5:将测量坐标系下的点转化到编码点坐标系下：P2 = R*P1 + T
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
//   ////将相机坐标系下的点坐标转化到编码点坐标系下，由于此处的R已经是逆矩阵了，由于opencv中的SVD性质可得。
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
	//////将tagPnts中的标签取出，放在容器中
	vector<int> tags;
	for (size_t i = 0; i < tagPnts.size(); i++)
	{
		tags.push_back(int(tagPnts[i][0]));
	}
	///计算这些排好序的三维点的质心，作为编码点坐标系的原点
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
	/////对上述一系列三维点进行奇异值分解
	Mat S, U, VT;
	cv::SVD::compute(srcMat, S, U, VT, cv::SVD::FULL_UV);////VT此处为V的转置矩阵，也就是奇异向量的转置
	//////选择最大的特征值的特征向量作为x轴，选择第二个特征值的特征向量与x轴垂直的矢量作为y轴，z轴是x,y轴的叉乘
	////将相机坐标系下的点坐标转化到编码点坐标系下，由于此处的R已经是逆矩阵了，由于opencv中的SVD性质可得。
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


//////创建编码点相对于相机的坐标系转换
//void CoreAlgorithm::createCodepntsCoordinate(vector<TagPoint3f> tagPnts, vector<TagPoint3f>& CodepntsCenterToCodeOrigin)
//{
//	//////将tagPnts中的标签取出，放在容器中
//	vector<int> tags;
//	for (size_t i = 0; i < tagPnts.size(); i++)
//	{
//		tags.push_back(int(tagPnts[i][0]));
//	}
//	Point3f CentroidPoint;
//	CoreAlgorithm::CalculateCentroid(tagPnts, CentroidPoint);
//	
//	//////建立X轴坐标
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
//	///////进行判断法线方向是否为从第二个点到第一个点的方向，第二个点减去第一个点的坐标
//	Vec3f norm_12;
//	norm_12[0] = Z_AxisPnts[1].x - Z_AxisPnts[0].x;
//	norm_12[1] = Z_AxisPnts[1].y - Z_AxisPnts[0].y;
//	norm_12[2] = Z_AxisPnts[1].z - Z_AxisPnts[0].z;
//	//////点乘
//	float temp_Z = norm_Z[0] * norm_12[0] + norm_Z[1] * norm_12[1] + norm_Z[2] * norm_12[2];
//	if (temp_Z < 0)///////方向不同，则添加负号
//	{
//		norm_Z = -1 * norm_Z;
//	}
//
//	//////求经过第三个编码点，且垂直于Z轴的单位向量，获得Y轴单位向量。
//
//	Point3f pt;/////pt0为直线上一点，pt1为直线外一点。
//	pt = Point3f(tagPnts[2][1], tagPnts[2][2], tagPnts[2][3]);
//	Point3f PerpendicularFoot = CoreAlgorithm::GetFootOfPerpendicular(pt, Z_AxisPnts[0], Z_AxisPnts[1]);
//    /////直线拟合Y轴
//	vector<Point3f> Y_AxisPnts;
//	Y_AxisPnts.push_back(PerpendicularFoot);
//	Y_AxisPnts.push_back(pt);
//
//	Vec6f Y_Line;
//	fitLine(Y_AxisPnts, Y_Line, CV_DIST_L2, 0, 0.01, 0.01);
//	Vec3f norm_Y;
//	norm_Y[0] = Y_Line[0]; norm_Y[1] = Y_Line[1]; norm_Y[2] = Y_Line[2];
//	///////进行判断法线方向是否为从第三个点到垂足点的方向，第三个点减去垂足点的坐标
//	Vec3f norm_3foot;
//	norm_3foot[0] = Y_AxisPnts[1].x - Y_AxisPnts[0].x;
//	norm_3foot[1] = Y_AxisPnts[1].y - Y_AxisPnts[0].y;
//	norm_3foot[2] = Y_AxisPnts[1].z - Y_AxisPnts[0].z;
//	float temp_Y = norm_Y[0] * norm_3foot[0] + norm_Y[1] * norm_3foot[1] + norm_Y[2] * norm_3foot[2];
//	if (temp_Y < 0)
//	{
//		norm_Y = -1 * norm_Y;
//	}
//     ////Y叉乘Z，获得X轴单位向量。由于此处是Z叉乘Y，得到的X应该取反，方向才能相同
//	Vec3f norm_X;
//	norm_X[0] = norm_Z[1] * norm_Y[2] - norm_Z[2] * norm_Y[1];
//	norm_X[1] = norm_Z[2] * norm_Y[0] - norm_Z[0] * norm_Y[2];
//	norm_X[2] = norm_Z[0] * norm_Y[1] - norm_Z[1] * norm_Y[0];
//	//////因为叉乘顺序反了，所以增加一个负号  
//	norm_X = -norm_X;
//
//	/////三个单位向量保存为R，第一个编码点中心为T;
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
//	//STEP-5:将测量坐标系下的点转化到编码点坐标系下：P2 = R*P1 + T
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
	//////将tagPnts中的标签取出，放在容器中
	vector<int> tags;
	for (size_t i = 0; i < tagPnts.size(); i++)
	{
		tags.push_back(int(tagPnts[i][0]));
	}
	Point3f CentroidPoint;
	CoreAlgorithm::CalculateCentroid(tagPnts, CentroidPoint);

	//////建立X轴坐标
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
	///////进行判断法线方向是否为从第二个点到第一个点的方向，第二个点减去第一个点的坐标
	Vec3f norm_12;
	norm_12[0] = X_AxisPnts[1].x - X_AxisPnts[0].x;
	norm_12[1] = X_AxisPnts[1].y - X_AxisPnts[0].y;
	norm_12[2] = X_AxisPnts[1].z - X_AxisPnts[0].z;
	//////点乘
	float temp_X = norm_X[0] * norm_12[0] + norm_X[1] * norm_12[1] + norm_X[2] * norm_12[2];
	if (temp_X < 0)///////方向不同，则添加负号
	{
		norm_X = -1 * norm_X;
	}

	//////求经过第三个编码点，且垂直于Z轴的单位向量，获得Y轴单位向量。

	Point3f pt;/////pt0为直线上一点，pt1为直线外一点。
	pt = Point3f(tagPnts[2][1], tagPnts[2][2], tagPnts[2][3]);
	Point3f PerpendicularFoot = CoreAlgorithm::GetFootOfPerpendicular(pt, X_AxisPnts[0], X_AxisPnts[1]);
	/////直线拟合Y轴
	vector<Point3f> Y_AxisPnts;
	Y_AxisPnts.push_back(PerpendicularFoot);
	Y_AxisPnts.push_back(pt);

	cv::Vec6f Y_Line;
	fitLine(Y_AxisPnts, Y_Line, CV_DIST_L2, 0, 0.01, 0.01);
	Vec3f norm_Y;
	norm_Y[0] = Y_Line[0]; norm_Y[1] = Y_Line[1]; norm_Y[2] = Y_Line[2];
	///////进行判断法线方向是否为从第三个点到垂足点的方向，第三个点减去垂足点的坐标
	Vec3f norm_3foot;
	norm_3foot[0] = Y_AxisPnts[1].x - Y_AxisPnts[0].x;
	norm_3foot[1] = Y_AxisPnts[1].y - Y_AxisPnts[0].y;
	norm_3foot[2] = Y_AxisPnts[1].z - Y_AxisPnts[0].z;
	float temp_Y = norm_Y[0] * norm_3foot[0] + norm_Y[1] * norm_3foot[1] + norm_Y[2] * norm_3foot[2];
	if (temp_Y < 0)
	{
		norm_Y = -1 * norm_Y;
	}

	////X叉乘Y，获得Z轴单位向量。
	Vec3f norm_Z;
	norm_Z[0] = norm_X[1] * norm_Y[2] - norm_X[2] * norm_Y[1];
	norm_Z[1] = norm_X[2] * norm_Y[0] - norm_X[0] * norm_Y[2];
	norm_Z[2] = norm_X[0] * norm_Y[1] - norm_X[1] * norm_Y[0];

	/////三个单位向量保存为R，第一个编码点中心为T;
	Mat T(3, 1, CV_32F); Mat R(3, 3, CV_32F);
	T.at<float>(0, 0) = CentroidPoint.x;
	T.at<float>(1, 0) = CentroidPoint.y;
	T.at<float>(2, 0) = CentroidPoint.z;

	R.at<float>(0, 0) = norm_X[0]; R.at<float>(1, 0) = norm_X[1]; R.at<float>(2, 0) = norm_X[2];
	R.at<float>(0, 1) = norm_Y[0]; R.at<float>(1, 1) = norm_Y[1]; R.at<float>(2, 1) = norm_Y[2];
	R.at<float>(0, 2) = norm_Z[0]; R.at<float>(1, 2) = norm_Z[1]; R.at<float>(2, 2) = norm_Z[2];
	Mat R_invert;
	invert(R, R_invert);
	//STEP-5:将测量坐标系下的点转化到编码点坐标系下：P2 = R*P1 + T
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




/////将5个TagPoint3f类型的数据转化成一个Mat
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

///////计算一系列三维点质心
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

 /////将两个函数相同的行Mat合并成一个大Mat
 Mat CoreAlgorithm::comMatR(Mat Matrix1, Mat Matrix2, Mat &MatrixCom)
 {
	 CV_Assert(Matrix1.rows == Matrix2.rows);//行数不相等，出现错误中断    
	 MatrixCom.create(Matrix1.rows, Matrix1.cols + Matrix2.cols, CV_64FC1);
	 Mat temp = MatrixCom.colRange(0, Matrix1.cols);
	 Matrix1.copyTo(temp);
	 Mat temp1 = MatrixCom.colRange(Matrix1.cols, Matrix1.cols + Matrix2.cols);
	 Matrix2.copyTo(temp1);
	 return MatrixCom;
 }
 ////将两个函数相同的列Mat合并成一个大Mat
 Mat CoreAlgorithm::comMatC(Mat Matrix1, Mat Matrix2, Mat &MatrixCom)
 {
	 CV_Assert(Matrix1.cols == Matrix2.cols);//列数不相等，出现错误中断    
	 MatrixCom.create(Matrix1.rows + Matrix2.rows, Matrix1.cols,CV_64FC1);
	 Mat temp = MatrixCom.rowRange(0, Matrix1.rows);
	 Matrix1.copyTo(temp);
	 Mat temp1 = MatrixCom.rowRange(Matrix1.rows, Matrix1.rows + Matrix2.rows);
	 Matrix2.copyTo(temp1);
	 return MatrixCom;
 }
  


 /////重载getCodePoint3f()函数
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

	 //step-1 计算左右相机的二维特征点点集
	 vector<Point3f> pntsw;
	 pntsTagConfirm(tagPnts3fw, pntsw);
	 ///////利用编码点识别算法检测识别出左图中编码点的坐标
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
	 ///////利用编码点识别算法检测识别出右图中编码点的坐标
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
	 /////将识别出的编码点放入容器中，便于后续处理
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
	 ////将特征点的标签信息去掉
	 vector<Point2f> leftPnts2f, rightPnts2f;
	 vector<int> TagVec;
	 pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);

	 ////计算左相机摄像机坐标系下的三维点信息
	 vector<Point3f> pnts3fVec;
	 //第一种计算三维点信息的方法
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

	 //step-1 计算左右相机的二维特征点点集
	 vector<Point3f> pntsw;
	 pntsTagConfirm(tagPnts3fw, pntsw);
	 ///////利用编码点识别算法检测识别出左图中编码点的坐标
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

	
	 
	 ///////利用编码点识别算法检测识别出右图中编码点的坐标
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
	
	 /////将识别出的编码点放入容器中，便于后续处理
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
	 ////将特征点的标签信息去掉
	 vector<Point2f> leftPnts2f, rightPnts2f;
	 vector<int> TagVec;
	 pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);

	 ////计算左相机摄像机坐标系下的三维点信息
	 vector<Point3f> pnts3fVec;
	 //第一种计算三维点信息的方法
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


 ///////利用编码标志点识别函数识别出图片中标志点位置，并重建出相机坐标系下的三维点。
 bool  CoreAlgorithm::getCodePoint3f(const Mat& imgLeft, const CamPara _campara1, const Mat& imgRight, const CamPara _campara2,
	 const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<pair<unsigned int, Point2f>> &LeftmarkerResults, vector<pair<unsigned int, Point2f>> &RightmarkerResults, vector<TagPoint3f>& tagPnts3f)
 {
	 cv::Mat cameraMatrixLeft = (cv::Mat_<double>(3, 3) << _campara1.CameraIntrinsic[0][0], _campara1.CameraIntrinsic[0][1], _campara1.CameraIntrinsic[0][2]
		 , _campara1.CameraIntrinsic[1][0], _campara1.CameraIntrinsic[1][1], _campara1.CameraIntrinsic[1][2]
		 , _campara1.CameraIntrinsic[2][0], _campara1.CameraIntrinsic[2][1], _campara1.CameraIntrinsic[2][2]);
	 cv::Mat cameraMatrixRight = (cv::Mat_<double>(3, 3) << _campara2.CameraIntrinsic[0][0], _campara2.CameraIntrinsic[0][1], _campara2.CameraIntrinsic[0][2]
		 , _campara2.CameraIntrinsic[1][0], _campara2.CameraIntrinsic[1][1], _campara2.CameraIntrinsic[1][2]
		 , _campara2.CameraIntrinsic[2][0], _campara2.CameraIntrinsic[2][1], _campara2.CameraIntrinsic[2][2]);

	 //step-1 计算左右相机的二维特征点点集
	 vector<Point3f> pntsw;
	 pntsTagConfirm(tagPnts3fw, pntsw);

	 ///////利用编码点识别算法检测识别出左图中编码点的坐标
	 LeftmarkerResults.clear();
	 bool leftResult = CoreAlgorithm::findCircularMarker(imgLeft, LeftmarkerResults);
	 ///////利用编码点识别算法检测识别出右图中编码点的坐标
	 RightmarkerResults.clear();
	 bool rightResult = CoreAlgorithm::findCircularMarker(imgRight, RightmarkerResults);
	 if (LeftmarkerResults.size() == RightmarkerResults.size() && LeftmarkerResults.size() ==3)
	 {
		 /////将识别出的编码点放入容器中，便于后续处理
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
		 ////将特征点的标签信息去掉
		 vector<Point2f> leftPnts2f, rightPnts2f;
		 vector<int> TagVec;
		 pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);

		 ////计算左相机摄像机坐标系下的三维点信息
		 vector<Point3f> pnts3fVec;
		 //第一种计算三维点信息的方法
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



















 ////重载getCodePoint3f()函数
 bool CoreAlgorithm::getCodePoint3f_First(const Mat imgLeft, cv::Rect maskLeftRoi_First, const Mat imgRight, cv::Rect maskRightRoi_First, const CamPara _campara1, const CamPara _campara2, const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& TagPnt_First, pair<unsigned int, Point2f> &CodeLeft_First, pair<unsigned int, Point2f> &CodeRight_First, int flag_First)
 {
	 cv::Mat cameraMatrixLeft = (cv::Mat_<double>(3, 3) << _campara1.CameraIntrinsic[0][0], _campara1.CameraIntrinsic[0][1], _campara1.CameraIntrinsic[0][2]
		 , _campara1.CameraIntrinsic[1][0], _campara1.CameraIntrinsic[1][1], _campara1.CameraIntrinsic[1][2]
		 , _campara1.CameraIntrinsic[2][0], _campara1.CameraIntrinsic[2][1], _campara1.CameraIntrinsic[2][2]);
	 cv::Mat cameraMatrixRight = (cv::Mat_<double>(3, 3) << _campara2.CameraIntrinsic[0][0], _campara2.CameraIntrinsic[0][1], _campara2.CameraIntrinsic[0][2]
		 , _campara2.CameraIntrinsic[1][0], _campara2.CameraIntrinsic[1][1], _campara2.CameraIntrinsic[1][2]
		 , _campara2.CameraIntrinsic[2][0], _campara2.CameraIntrinsic[2][1], _campara2.CameraIntrinsic[2][2]);

	 //step-1 计算左右相机的二维特征点点集
	 vector<Point3f> pntsw;
	 pntsTagConfirm(tagPnts3fw, pntsw);
	 ///////利用编码点识别算法检测识别出左图中编码点的坐标
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
	

	 ///////利用编码点识别算法检测识别出右图中编码点的坐标
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

	 /////将识别出的编码点放入容器中，便于后续处理
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
	 ////将特征点的标签信息去掉
	 vector<Point2f> leftPnts2f, rightPnts2f;
	 vector<int> TagVec;
	 pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);

	 ////计算左相机摄像机坐标系下的三维点信息
	 vector<Point3f> pnts3fVec;
	 //第一种计算三维点信息的方法
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
	 //step-1 计算左右相机的二维特征点点集
	 vector<Point3f> pntsw;
	 pntsTagConfirm(tagPnts3fw, pntsw);
	 ///////利用编码点识别算法检测识别出左图中编码点的坐标
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

	 ///////利用编码点识别算法检测识别出右图中编码点的坐标
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

	 /////将识别出的编码点放入容器中，便于后续处理
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
	 ////将特征点的标签信息去掉
	 vector<Point2f> leftPnts2f, rightPnts2f;
	 vector<int> TagVec;
	 pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);

	 ////计算左相机摄像机坐标系下的三维点信息
	 vector<Point3f> pnts3fVec;
	 //第一种计算三维点信息的方法
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
	 //step-1 计算左右相机的二维特征点点集
	 vector<Point3f> pntsw;
	 pntsTagConfirm(tagPnts3fw, pntsw);
	 ///////利用编码点识别算法检测识别出左图中编码点的坐标
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

	 ///////利用编码点识别算法检测识别出右图中编码点的坐标
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

	 /////将识别出的编码点放入容器中，便于后续处理
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
	 ////将特征点的标签信息去掉
	 vector<Point2f> leftPnts2f, rightPnts2f;
	 vector<int> TagVec;
	 pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);

	 ////计算左相机摄像机坐标系下的三维点信息
	 vector<Point3f> pnts3fVec;
	 //第一种计算三维点信息的方法
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




 /////重载getCodePoint3f()函数
 bool CoreAlgorithm::getCodePoint3f(const Mat imgLeft, const CamPara _campara1, const Mat imgRight, const CamPara _campara2,
	 const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3fw, vector<TagPoint3f>& tagPnts3f,
	 Point2f &LightPenCentroidPntLeft, Point2f &LightPenCentroidPntRight, float &CircleDiameter, vector<pair<unsigned int, Point2f>> &markers_Left, vector<pair<unsigned int, Point2f>> &markers_Right)
 {
	 ////////先识别出光笔中各特征点的中心坐标。

	 cv::Mat cameraMatrixLeft = (cv::Mat_<double>(3, 3) << _campara1.CameraIntrinsic[0][0], _campara1.CameraIntrinsic[0][1], _campara1.CameraIntrinsic[0][2]
		 , _campara1.CameraIntrinsic[1][0], _campara1.CameraIntrinsic[1][1], _campara1.CameraIntrinsic[1][2]
		 , _campara1.CameraIntrinsic[2][0], _campara1.CameraIntrinsic[2][1], _campara1.CameraIntrinsic[2][2]);
	 cv::Mat cameraMatrixRight = (cv::Mat_<double>(3, 3) << _campara2.CameraIntrinsic[0][0], _campara2.CameraIntrinsic[0][1], _campara2.CameraIntrinsic[0][2]
		 , _campara2.CameraIntrinsic[1][0], _campara2.CameraIntrinsic[1][1], _campara2.CameraIntrinsic[1][2]
		 , _campara2.CameraIntrinsic[2][0], _campara2.CameraIntrinsic[2][1], _campara2.CameraIntrinsic[2][2]);

	 //step-1 计算左右相机的二维特征点点集
	 vector<Point3f> pntsw;
	 pntsTagConfirm(tagPnts3fw, pntsw);
	 ///////利用编码点识别算法检测识别出左图中编码点的坐标
	 vector<pair<unsigned int, Point2f>> LeftmarkerResults, RightmarkerResults;
	 if (!CoreAlgorithm::findCircularMarker(imgLeft, LeftmarkerResults))
	 {
		 return false;
	 }
	 ////对识别出的编码点进行排序
	 if (LeftmarkerResults.size() == 3)
	 {
		 markers_Left = LeftmarkerResults;
	 }
	 else
	 {
		 return false;
	 }


	 ///////利用编码点识别算法检测识别出右图中编码点的坐标
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
	 /////将识别出的编码点放入容器中，便于后续处理
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
	 ////将特征点的标签信息去掉
	 vector<Point2f> leftPnts2f, rightPnts2f;
	 vector<int> TagVec;
	 pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);

	 ////计算左相机摄像机坐标系下的三维点信息
	 vector<Point3f> pnts3fVec;
	 //第一种计算三维点信息的方法
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