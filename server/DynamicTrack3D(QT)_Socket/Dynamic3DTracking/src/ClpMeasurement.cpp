#pragma once
#include "StdAfx.h"
#include "ClpMeasurement.h"
#include "XMLReader.h"
#include "CoreAlgorithm.h"
#include<opencv2/opencv.hpp>
#include "pcamera.h"
#include"SharedHead.h"
#include"fstream"
#include "Condensation.h"
#include<QMessageBox>
//using namespace cv;
ClpMeasurement::ClpMeasurement(void)
{
	err = OPERATION_OK;
	dataGet = false;
	_position = SProbePosition2(0.0,0.0,0.0,0.0,0.0,0.0);
}


ClpMeasurement::~ClpMeasurement(void)
{

}



void ClpMeasurement::dataOutput(SProbePosition2& _positionOut)
{
	_positionOut.dX = _position.dX;
	_positionOut.dY = _position.dY;
	_positionOut.dZ = _position.dZ;
	_positionOut.dZI = _position.dZI;
	_positionOut.dZJ = _position.dZJ;
	_positionOut.dZK = _position.dZK;
	dataGet = false;
}
cv::Mat ClpMeasurement::hBitmap2Mat(HBITMAP hBmp)
{
	BITMAP bmp;   
	GetObject(hBmp,sizeof(BITMAP),&bmp);
	int nChannels = bmp.bmBitsPixel == 1 ? 1 : bmp.bmBitsPixel/8 ;  
	int depth = bmp.bmBitsPixel == 1 ? IPL_DEPTH_1U : IPL_DEPTH_8U;   
	IplImage* img1 = cvCreateImage(cvSize(bmp.bmWidth,bmp.bmHeight),depth,nChannels);  
	memcpy(img1->imageData,(char*)(bmp.bmBits),bmp.bmHeight*bmp.bmWidth*nChannels);
	cvFlip(img1, NULL, 0);
	cv::Mat image = cv::cvarrToMat(img1,true);
	cvReleaseImage(&img1);
	return image;  
}



void ClpMeasurement::getMeasureData(Mat imageleft,Mat imageright)
{
	//step-1����ȡÿ��λ�˵�ͼƬ�Ĵ��б�ǩ��Ϣ�Ķ�ά������㼯�������centerPntsWithTag
	vector<TagPoint3f>  tagPnts; 
	/*try
	{
	if(!CoreAlgorithm::getTagPoint3f(imageleft,camparaleft,imageright,campararight,camGroupPara,lightpenfeatures,tagPnts))
	{
	err =  RECOGNITION_ERROR;
	return;
	}
	}
	catch(...)
	{
	err =  RECOGNITION_ERROR;
	return;
	}*/
	if (!CoreAlgorithm::getTagPoint3f(imageleft, camparaleft, imageright, campararight, camGroupPara, lightpenfeatures, tagPnts))
	{
		return;
	}
	RT PoseToCamleft;
	vector<Point3f> Pnts3fleft,Pnts3flightpen;
	pntsTagConfirm(tagPnts,Pnts3fleft);
	pntsTagConfirm(lightpenPara.FeatursPoints,Pnts3flightpen);
	//try
	//{
	//	CoreAlgorithm::rigidTransform(Pnts3flightpen,Pnts3fleft,PoseToCamleft.R,PoseToCamleft.T);
	//}
	//catch (...)
	//{
	//	err = RECOGNITION_ERROR;
	//	return;
	//}
	CoreAlgorithm::rigidTransform(Pnts3flightpen, Pnts3fleft, PoseToCamleft.R, PoseToCamleft.T);
	//�����Ϊ������ϵ
	if (PoseToCamleft.T.rows == 0 || PoseToCamleft.R.rows == 0)
	{

		err = RECOGNITION_ERROR;
		return;
	}
	_position.dX = PoseToCamleft.T.at<double>(0,0);
	_position.dY = PoseToCamleft.T.at<double>(1,0);
	_position.dZ = PoseToCamleft.T.at<double>(2,0);
	Mat aq = Mat::zeros(3,1,CV_64F);
	aq.at<double>(2,0) = 1;
	Mat rz = PoseToCamleft.R*aq;
	rz = rz/cv::norm(rz);
	_position.dZI =rz.at<double>(0,0);
	_position.dZJ =rz.at<double>(1,0);
	_position.dZK = rz.at<double>(2,0);
	dataGet = true;
	err=  OPERATION_OK;
}








bool ClpMeasurement::PutImgsToImgVec(vector<pair<Mat, Mat>>  &m_ImgVec,int m_Num)
{
	if (m_Num < 1)
	{
		return false;
	}
	char leftImg[500],rightImg[500];
	for (int i = 1; i < m_Num; i++)
	{
		sprintf(leftImg, "F:/Lab/30Frames_s/OnlyThreeCodes/All_in/Left/%d.bmp", i);
		sprintf(rightImg, "F:/Lab/30Frames_s/OnlyThreeCodes/All_in/Right/%d.bmp", i);
		Mat imgLeft = cv::imread(leftImg, 0); Mat imgRight = cv::imread(rightImg, 0);
		pair<Mat, Mat> imgPair;
		imgPair.first = imgLeft.clone();
		imgPair.second = imgRight.clone();
		m_ImgVec.push_back(imgPair);
	}
	return true;
}



bool ClpMeasurement::findCodeOriginalPosition2D(vector<pair<Mat, Mat>>  m_ImgVec, vector<pair<unsigned int, Point2f>> &markerResultsLeft, vector<pair<unsigned int, Point2f>> &markerResultsRight, int &m_OriNum,float &Cir_Diameter_Code)
{
	if (m_ImgVec.empty())
	{
		return false;
	}
	for (size_t j = m_OriNum; j < m_ImgVec.size(); j++)
	{
		markerResultsLeft.clear(); markerResultsRight.clear();
		CoreAlgorithm::findCircularMarker(m_ImgVec[j].first, markerResultsLeft,Cir_Diameter_Code);
		CoreAlgorithm::findCircularMarker(m_ImgVec[j].second, markerResultsRight);
		if (markerResultsLeft.size()>=3&&markerResultsRight.size()>=3&&
			markerResultsLeft[0].first == markerResultsRight[0].first&&
			markerResultsLeft[1].first== markerResultsRight[1].first&&
			markerResultsLeft[2].first==markerResultsRight[2].first)  /////����Ϊ�˱�֤������Ҫ���������־�㣬�Ա���Խ������������ϵ��
		{
			m_OriNum = j;
			return true;
		}
		else
		{
			continue;
		}


	}
	m_OriNum = -1;
	return false;
}


bool ClpMeasurement::findCodeOriginalPosition2D(vector<pair<Mat, Mat>>  m_ImgVec, vector<pair<unsigned int, Point2f>> &markerResultsLeft, vector<pair<unsigned int, Point2f>> &markerResultsRight, int &m_OriNum, float &Cir_Diameter_Code, int flag)
{
	if (m_ImgVec.empty())
	{
		return false;
	}
	for (size_t j = m_OriNum; j < m_ImgVec.size(); j++)
	{
		markerResultsLeft.clear(); markerResultsRight.clear();
		CoreAlgorithm::findCircularMarker(m_ImgVec[j].first, markerResultsLeft, Cir_Diameter_Code);
		CoreAlgorithm::findCircularMarker(m_ImgVec[j].second, markerResultsRight);
		if (markerResultsLeft.size() >= flag && markerResultsRight.size() >= flag &&
			markerResultsLeft[0].first == markerResultsRight[0].first&&
			markerResultsLeft[1].first == markerResultsRight[1].first&&
			markerResultsLeft[flag-1].first == markerResultsRight[flag-1].first)  /////����Ϊ�˱�֤������Ҫ���������־�㣬�Ա���Խ������������ϵ��
		{
			m_OriNum = j;
			return true;
		}
		else
		{
			continue;
		}


	}
	m_OriNum = -1;
	return false;
}









bool ClpMeasurement::calculateLightPenCentroid(const Mat img, const vector<pair<unsigned int, Point2f>>  markerResults, Point2f &LightPenCentroidPnt)
{

	float width_height_ratio = 4;
	Mat img_threshold;
	//STEP-1:��ֵ��
	threshold(img, img_threshold, 100, 255, cv::THRESH_BINARY);
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
			if (contoursOrigin[i].size()>10)
			{
				data.push_back(fitEllipse(contoursOrigin[i]));
			}
			else
			{
				continue;
			}
		}
	}
	///************��һ�δ����Ƿ�������õģ���Ҫ������ȷ��distanceMatrix.at<double>(i, j)��������ȡֵ��Χ********///
	///��������Ϻõ���Բ������ɫ���������Ƴ���  
	/////�Ƚ���ͨ����imgMatͼƬת����Ϊ��ͨ����ͼƬ
	Mat colorImg;
	cv::cvtColor(img, colorImg, CV_GRAY2BGR);
	for (size_t j = 0; j < data.size(); j++)
	{
		cv::ellipse(colorImg, data[j], Scalar(0, 255, 0), 1);
		////�����е���ԲԲ��������ֱ��
		string Num = to_string(j);
		cv::putText(colorImg, Num, Point(data[j].center.x, data[j].center.y), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0));
	}
	//////*******************************************************************************************//////

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
			distanceMatrix1.at<double>(i, j) = CoreAlgorithm::distancePoints2f(CenterPnts[i], CenterPnts[j]);
		}
	}
	cv::add(distanceMatrix1, distanceMatrix1.t(), distanceMatrix);
	vector<pair<int, vector<int>>>  LightPenPntsVec, OneVec, TwoVec, ThreeVec, FourVec;
	for (size_t i = 0; i < distanceMatrix.cols; i++)
	{
		pair<int, vector<int>> tempdata;
		tempdata.first = i;
		for (size_t j = 0; j < distanceMatrix.cols; j++)
		{
			if (distanceMatrix.at<double>(i, j)<50 && distanceMatrix.at<double>(i, j)>0) ////�˴��Ŀ�ȡ��ΧΪ��30~80
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
	//////�Գ��μ����Ķ�������������ɸѡ���ҵ���ʵİ˸��б�Բ����
	///////////////////////
	//STEP-3:��Բ���,������,������ɸѡ
	RotatedRect rotatedBox;
	vector<Point2f> centerPnts;
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
		n++;
		//�����ߵ����״������Ļ��������������Ҫ��һ��
		if ((height > width ? (height / width < width_height_ratio) : (width / height<width_height_ratio))&& height>5 && width>5)
		{
			centerPnts.push_back(rotatedBox.center);
		}

	}

	if (centerPnts.size()!= 8)
	{
		return false;
	}

	///�Լ�������Բ����������������
	if (!calculateCentroid(centerPnts, LightPenCentroidPnt))
	{
		return false;
	}
	return true;
	//STEP-4 �Լ��������ĵ���й��˺��޳�

}

/////���غ���calculateLightPenCentroid(),������������Բ��ֱ����
bool ClpMeasurement::calculateLightPenCentroid(const Mat img, const vector<pair<unsigned int, Point2f>>  markerResults, Point2f &LightPenCentroidPnt, float &CircleDiameter)
{
	float width_height_ratio = 4;
	Mat img_threshold;
	//STEP-1:��ֵ��
	threshold(img, img_threshold, 100, 255, cv::THRESH_BINARY);
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
			data.push_back(fitEllipse(contoursOrigin[i]));
		}
	}
	///************��һ�δ����Ƿ�������õģ���Ҫ������ȷ��distanceMatrix.at<double>(i, j)��������ȡֵ��Χ********///
	///��������Ϻõ���Բ������ɫ���������Ƴ���  
	/////�Ƚ���ͨ����imgMatͼƬת����Ϊ��ͨ����ͼƬ
	//Mat colorImg;
	//cv::cvtColor(img, colorImg, CV_GRAY2BGR);
	//for (size_t j = 0; j < data.size(); j++)
	//{
	//	cv::ellipse(colorImg, data[j], Scalar(0, 255, 0), 1);
	//	////�����е���ԲԲ��������ֱ��
	//	string Num = to_string(j);
	//	cv::putText(colorImg, Num, Point(data[j].center.x, data[j].center.y), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0));
	//}
	//////*******************************************************************************************//////

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
			distanceMatrix1.at<double>(i, j) = CoreAlgorithm::distancePoints2f(CenterPnts[i], CenterPnts[j]);
		}
	}
	cv::add(distanceMatrix1, distanceMatrix1.t(), distanceMatrix);
	vector<pair<int, vector<int>>>  LightPenPntsVec, OneVec, TwoVec, ThreeVec, FourVec;
	for (size_t i = 0; i < distanceMatrix.cols; i++)
	{
		pair<int, vector<int>> tempdata;
		tempdata.first = i;
		for (size_t j = 0; j < distanceMatrix.cols; j++)
		{
			if (distanceMatrix.at<double>(i, j)<30 && distanceMatrix.at<double>(i, j)>0) ////�˴��Ŀ�ȡ��ΧΪ��30~80
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
	//////�Գ��μ����Ķ�������������ɸѡ���ҵ���ʵİ˸��б�Բ����
	///////////////////////
	//STEP-3:��Բ���,������,������ɸѡ
	RotatedRect rotatedBox;
	vector<Point2f> centerPnts;
	vector<float> Diameters;
	float fDiameter;
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
		fDiameter = (height + width) / 2.0;
		Diameters.push_back(fDiameter);
		n++;
		//�����ߵ����״������Ļ��������������Ҫ��һ��
		if ((height > width ? (height / width < width_height_ratio) : (width / height<width_height_ratio)) && height>5 && width>5)
		{
			centerPnts.push_back(rotatedBox.center);
		}

	}

	if (centerPnts.size() != 8)
	{
		return false;
	}
	cv::Scalar meanValue = cv::mean(Diameters);
	CircleDiameter = meanValue.val[0];


	///�Լ�������Բ����������������
	if (!calculateCentroid(centerPnts, LightPenCentroidPnt))
	{
		return false;
	}
	return true;
	//STEP-4 �Լ��������ĵ���й��˺��޳�




}


bool ClpMeasurement::calculateLightPenCentroid(const Mat img, Point2f &LightPenCentroidPnt, float &CircleDiameter)
{

	float width_height_ratio = 4;
	Mat img_threshold;
	//STEP-1:��ֵ��
	threshold(img, img_threshold, 100, 255, cv::THRESH_BINARY);
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
			data.push_back(fitEllipse(contoursOrigin[i]));
		}
	}
	///************��һ�δ����Ƿ�������õģ���Ҫ������ȷ��distanceMatrix.at<double>(i, j)��������ȡֵ��Χ********///
	///��������Ϻõ���Բ������ɫ���������Ƴ���  
	/////�Ƚ���ͨ����imgMatͼƬת����Ϊ��ͨ����ͼƬ
	Mat colorImg;
	cv::cvtColor(img, colorImg, CV_GRAY2BGR);
	for (size_t j = 0; j < data.size(); j++)
	{
		cv::ellipse(colorImg, data[j], Scalar(0, 255, 0), 1);
		////�����е���ԲԲ��������ֱ��
		string Num = to_string(j);
		cv::putText(colorImg, Num, Point(data[j].center.x, data[j].center.y), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0));
	}
	//////*******************************************************************************************//////

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
			distanceMatrix1.at<double>(i, j) = CoreAlgorithm::distancePoints2f(CenterPnts[i], CenterPnts[j]);
		}
	}
	cv::add(distanceMatrix1, distanceMatrix1.t(), distanceMatrix);
	vector<pair<int, vector<int>>>  LightPenPntsVec, OneVec, TwoVec, ThreeVec, FourVec;
	for (size_t i = 0; i < distanceMatrix.cols; i++)
	{
		pair<int, vector<int>> tempdata;
		tempdata.first = i;
		for (size_t j = 0; j < distanceMatrix.cols; j++)
		{
			if (distanceMatrix.at<double>(i, j)<30 && distanceMatrix.at<double>(i, j)>0) ////�˴��Ŀ�ȡ��ΧΪ��30~80
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
	//////�Գ��μ����Ķ�������������ɸѡ���ҵ���ʵİ˸��б�Բ����
	///////////////////////
	//STEP-3:��Բ���,������,������ɸѡ
	RotatedRect rotatedBox;
	vector<Point2f> centerPnts;
	vector<float> Diameters;
	float fDiameter;
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
		fDiameter = (height + width) / 2.0;
		Diameters.push_back(fDiameter);
		n++;
		//�����ߵ����״������Ļ��������������Ҫ��һ��
		if ((height > width ? (height / width < width_height_ratio) : (width / height<width_height_ratio)) && height>5 && width>5)
		{
			centerPnts.push_back(rotatedBox.center);
		}

	}

	if (centerPnts.size() != 8)
	{
		return false;
	}
	cv::Scalar meanValue = cv::mean(Diameters);
	CircleDiameter = meanValue.val[0];


	///�Լ�������Բ����������������
	if (!ClpMeasurement::calculateCentroid(centerPnts, LightPenCentroidPnt))
	{
		return false;
	}
	return true;
	//STEP-4 �Լ��������ĵ���й��˺��޳�

}

bool ClpMeasurement::calculateLightPenCentroid(const Mat img, Point2f &LightPenCentroidPnt)
{
	float width_height_ratio = 4;
	Mat img_threshold;
	//STEP-1:��ֵ��
	threshold(img, img_threshold, 80, 255, cv::THRESH_BINARY);
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
			if (contoursOrigin[i].size() >= 5)
			{
				data.push_back(fitEllipse(contoursOrigin[i]));
			}
			else
			{
				continue;
			}
			
		}
	}
	///************��һ�δ����Ƿ�������õģ���Ҫ������ȷ��distanceMatrix.at<double>(i, j)��������ȡֵ��Χ********///
	///��������Ϻõ���Բ������ɫ���������Ƴ���  
	/////�Ƚ���ͨ����imgMatͼƬת����Ϊ��ͨ����ͼƬ
	//Mat colorImg;
	//cv::cvtColor(img, colorImg, CV_GRAY2BGR);
	//for (size_t j = 0; j < data.size(); j++)
	//{
	//	cv::ellipse(colorImg, data[j], Scalar(0, 255, 0), 1);
	//	////�����е���ԲԲ��������ֱ��
	//	string Num = to_string(j);
	//	cv::putText(colorImg, Num, Point(data[j].center.x, data[j].center.y), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0));
	//}
	//////*******************************************************************************************//////

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
			distanceMatrix1.at<double>(i, j) = CoreAlgorithm::distancePoints2f(CenterPnts[i], CenterPnts[j]);
		}
	}
	cv::add(distanceMatrix1, distanceMatrix1.t(), distanceMatrix);
	vector<pair<int, vector<int>>>  LightPenPntsVec, OneVec, TwoVec, ThreeVec, FourVec;
	for (size_t i = 0; i < distanceMatrix.cols; i++)
	{
		pair<int, vector<int>> tempdata;
		tempdata.first = i;
		for (size_t j = 0; j < distanceMatrix.cols; j++)
		{
			if (distanceMatrix.at<double>(i, j)<30 && distanceMatrix.at<double>(i, j)>0) ////�˴��Ŀ�ȡ��ΧΪ��30~80
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
	//////�Գ��μ����Ķ�������������ɸѡ���ҵ���ʵİ˸��б�Բ����
	///////////////////////
	//STEP-3:��Բ���,������,������ɸѡ
	RotatedRect rotatedBox;
	vector<Point2f> centerPnts;
	vector<float> Diameters;
	float fDiameter;
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
		fDiameter = (height + width) / 2.0;
		Diameters.push_back(fDiameter);
		n++;
		//�����ߵ����״������Ļ��������������Ҫ��һ��
		if ((height > width ? (height / width < width_height_ratio) : (width / height<width_height_ratio)) && height>5 && width>5)
		{
			centerPnts.push_back(rotatedBox.center);
		}

	}

	if (centerPnts.size() != 8)
	{
		return false;
	}

	///�Լ�������Բ����������������
	if (!ClpMeasurement::calculateCentroid(centerPnts, LightPenCentroidPnt))
	{
		return false;
	}
	return true;
	//STEP-4 �Լ��������ĵ���й��˺��޳�

}





bool ClpMeasurement::calculateCentroid(const vector<Point2f> centerPnts, Point2f &LightPenPntsCentroid)
{
	CV_Assert(centerPnts.size() != 0);
	double sum_x = 0;
	double sum_y = 0;
	for (size_t i = 0; i < centerPnts.size(); i++)
	{
		sum_x = sum_x + centerPnts[i].x;
		sum_y = sum_y + centerPnts[i].y;
	}
	LightPenPntsCentroid.x = sum_x / centerPnts.size();
	LightPenPntsCentroid.y = sum_y / centerPnts.size();

	return true;
}






bool ClpMeasurement::calculateCodesCentroid(const  vector<pair<unsigned int, Point2f>>  markerResults, Point2f &centroidPnt)
{
	CV_Assert(markerResults.size()!=0);
	double sum_x = 0;
	double sum_y = 0;
	for (size_t i = 0; i < markerResults.size(); i++)
	{
		sum_x = sum_x + markerResults[i].second.x;
		sum_y = sum_y + markerResults[i].second.y;
	}
	centroidPnt.x = sum_x / markerResults.size();
	centroidPnt.y = sum_y / markerResults.size();
	return true;
}

























