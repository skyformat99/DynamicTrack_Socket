#pragma once
#include "stdafx.h"
#include "XMLWriter.h"

void XMLWriter::writeCamPara( std::string filename, const CamPara& camparameter )
{
	char text[100];
	char* segmentation=" ";
	TiXmlDocument doc;
	TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	doc.LinkEndChild( decl );

	//创建内参数元素
	TiXmlElement * intrinsic_parameters_element = new TiXmlElement( "Intrinsic_parameters" );
	doc.LinkEndChild( intrinsic_parameters_element );

	TiXmlElement * fc_element = new TiXmlElement( "fc" );
	intrinsic_parameters_element->LinkEndChild(fc_element);

	TiXmlElement * cc_element = new TiXmlElement( "cc" );
	intrinsic_parameters_element->LinkEndChild(cc_element);

	sprintf_s(text, "%.9lf%s%.9lf", camparameter.CameraIntrinsic[0][0]
							,segmentation
							,camparameter.CameraIntrinsic[1][1]);

	TiXmlText * fc_text = new TiXmlText( text );
	fc_element->LinkEndChild( fc_text );

	sprintf_s(text, "%.9lf%s%.9lf", camparameter.CameraIntrinsic[0][2]
							,segmentation
							,camparameter.CameraIntrinsic[1][2]);

	TiXmlText * cc_text = new TiXmlText( text );
	cc_element->LinkEndChild( cc_text );

	//创建畸变参数元素
	TiXmlElement * Distortions_element = new TiXmlElement( "Distortions" );
	doc.LinkEndChild( Distortions_element );

	TiXmlElement * kc_element = new TiXmlElement( "kc" );
	Distortions_element->LinkEndChild(kc_element);

	sprintf_s(text, "%.9lf%s%.9lf%s%.9lf%s%.9lf", camparameter.DistortionCoeffs[0]
							,segmentation
							,camparameter.DistortionCoeffs[1]
							,segmentation
							,camparameter.DistortionCoeffs[2]
							,segmentation
							,camparameter.DistortionCoeffs[3]);

	TiXmlText * kc_text = new TiXmlText( text );
	kc_element->LinkEndChild( kc_text );

	//创建误差元素
	TiXmlElement * error_element = new TiXmlElement( "Error" );
	doc.LinkEndChild( error_element );

	//创建重投影误差元素
	TiXmlElement * pixel_norm_error_element = new TiXmlElement( "Repro_Norm_error" );
	error_element->LinkEndChild(pixel_norm_error_element);

	sprintf_s(text, "%lf", camparameter.totalReproNormErr);

	TiXmlText * pixel_norm_error_text = new TiXmlText( text );
	pixel_norm_error_element->LinkEndChild( pixel_norm_error_text );

	TiXmlElement * pixel_error_element = new TiXmlElement( "Repro_error" );
	error_element->LinkEndChild(pixel_error_element);

	sprintf_s(text, "%lf%s%lf", camparameter.ReprojectionError[0]
						,segmentation
						,camparameter.ReprojectionError[1]);

	TiXmlText * pixel_error_text = new TiXmlText( text );
	pixel_error_element->LinkEndChild( pixel_error_text );

	//创建内参数误差元素
	TiXmlElement * fc_error_element = new TiXmlElement( "fc_error" );
	error_element->LinkEndChild(fc_error_element);

	sprintf_s(text, "%lf%s%lf", camparameter.fcError[0]
						,segmentation
						,camparameter.fcError[1]);

	TiXmlText * fc_error_text = new TiXmlText( text );
	fc_error_element->LinkEndChild( fc_error_text );

	TiXmlElement * cc_error_element = new TiXmlElement( "cc_error" );
	error_element->LinkEndChild(cc_error_element);

	sprintf_s(text, "%lf%s%lf", camparameter.ccError[0]
						,segmentation
						,camparameter.ccError[1]);

	TiXmlText * cc_error_text = new TiXmlText( text );
	cc_error_element->LinkEndChild( cc_error_text );

	TiXmlElement * kc_error_element = new TiXmlElement( "kc_error" );
	error_element->LinkEndChild(kc_error_element);

	sprintf_s(text, "%lf%s%lf%s%lf%s%lf", camparameter.kcError[0]
						,segmentation,camparameter.kcError[1]
						,segmentation,camparameter.kcError[2]
						,segmentation,camparameter.kcError[3]);

	TiXmlText * kc_error_text = new TiXmlText( text );
	kc_error_element->LinkEndChild( kc_error_text );

	//创建标定板外参数
	writeImageRT(doc,camparameter);
	
	doc.SaveFile( filename.c_str() );
}

void XMLWriter::writeCamGroupPara(std::string filename, const CamGroupPara& camGroupParameter)
{
	//char text[100];
	char* segmentation = " ";
	TiXmlDocument doc;
	TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	doc.LinkEndChild( decl );

	TiXmlElement *left_mid_element = new TiXmlElement( "Left_Mid" );
	doc.LinkEndChild( left_mid_element );
	TiXmlElement *right_mid_element = new TiXmlElement( "Right_Mid" );
	doc.LinkEndChild( right_mid_element );
	TiXmlElement *right_left_element = new TiXmlElement( "Right_Left" );
	doc.LinkEndChild( right_left_element );

	writeGroupRT(left_mid_element,camGroupParameter.left2MidRotVector,camGroupParameter.left2MidTraVector);

	writeGroupRT(right_mid_element,camGroupParameter.right2MidRotVector,camGroupParameter.right2MidTraVector);

	writeGroupRT(right_left_element,camGroupParameter.right2LeftRotVector,camGroupParameter.right2LeftTraVector);
	doc.SaveFile( filename.c_str() );
}

void XMLWriter::writeCornerData( std::string filename, const CalibrationData& m_cornerData )
{
	char text[100];
	char* segmentation = " ";
	TiXmlDocument doc;
	TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0","","" );
	doc.LinkEndChild( decl );

	//保存图像尺寸
	TiXmlElement *ImageSize_element = new TiXmlElement( "ImageSize" );
	doc.LinkEndChild( ImageSize_element );
	TiXmlElement *Image_Height_element = new TiXmlElement( "Height" );
	ImageSize_element->LinkEndChild( Image_Height_element );
	sprintf_s(text, "%d", m_cornerData.imgHeight);
	TiXmlText * Height_text = new TiXmlText( text );
	Image_Height_element->LinkEndChild( Height_text );

	TiXmlElement *Image_Width_element = new TiXmlElement( "Width" );
	ImageSize_element->LinkEndChild( Image_Width_element );

	sprintf_s(text, "%d", m_cornerData.imgWidth);
	TiXmlText * width_text = new TiXmlText( text );
	Image_Width_element->LinkEndChild( width_text );

	//保存角点二维数据
	TiXmlElement *Corner_Points_element = new TiXmlElement( "Corner_Points" );
	doc.LinkEndChild( Corner_Points_element );
	for(unsigned int i=0;i<m_cornerData.plane2dPntsVec.size();i++)
	{
		for (unsigned int j = 0; j < m_cornerData.plane2dPntsVec[i].size(); j++)
		{
			sprintf_s(text, "%s%d", "p" ,i);
			TiXmlElement * index_element = new TiXmlElement( text );
			Corner_Points_element->LinkEndChild(index_element);
			sprintf_s(text, "%f%s%f", m_cornerData.plane2dPntsVec[i][j].x
									,segmentation
									,m_cornerData.plane2dPntsVec[i][j].y);
			TiXmlText * width_text = new TiXmlText( text );
			index_element->LinkEndChild( width_text );
		}
	}

	//保存角点特征点
	TiXmlElement *Feature_Points_element = new TiXmlElement( "Feature_Points" );
	doc.LinkEndChild( Feature_Points_element );
	for(unsigned int i=0;i<m_cornerData.plane3dPntsVec.size();i++)
	{
		for (unsigned int j = 0; j < m_cornerData.plane3dPntsVec[i].size(); j++)
		{
			sprintf_s(text, "%s%d", "p" ,i);
			TiXmlElement * index_element = new TiXmlElement( text );
			Feature_Points_element->LinkEndChild(index_element);
            float x = m_cornerData.plane3dPntsVec[i][j].x;
            float y = m_cornerData.plane3dPntsVec[i][j].y;
            float z = m_cornerData.plane3dPntsVec[i][j].z;
            sprintf_s(text, "%.3f%s%.3f%s%.3f", m_cornerData.plane3dPntsVec[i][j].x
								,segmentation,m_cornerData.plane3dPntsVec[i][j].y
								,segmentation,m_cornerData.plane3dPntsVec[i][j].z);
			TiXmlText * FeaturePoint_text = new TiXmlText( text );
			index_element->LinkEndChild( FeaturePoint_text );
		}
	}

	//保存每帧角点数
	TiXmlElement *CornerNumPerFrame_element = new TiXmlElement( "CornerNumPerFrame" );
	CornerNumPerFrame_element->SetAttribute("size",m_cornerData.plane2dPntsVec.size());
	doc.LinkEndChild( CornerNumPerFrame_element );
	for(unsigned int i=0;i<m_cornerData.plane2dPntsVec.size();i++)
	{
		sprintf_s(text, "%s%d", "n" ,i);
		TiXmlElement * index_element = new TiXmlElement( text );
		CornerNumPerFrame_element->LinkEndChild(index_element);

		sprintf_s(text, "%d", m_cornerData.plane2dPntsVec[i].size());
		TiXmlText * CornerNumPerFrame_text = new TiXmlText( text );
		index_element->LinkEndChild( CornerNumPerFrame_text );
	}

	//保存图像的索引
	TiXmlElement *FrameNumList_element = new TiXmlElement( "FrameNumList" );
	FrameNumList_element->SetAttribute("size",m_cornerData.frameNumList.size());
	doc.LinkEndChild( FrameNumList_element );
	for(unsigned int i=0;i<m_cornerData.frameNumList.size();i++)
	{
		sprintf_s(text, "%s%d", "n" ,i);
		TiXmlElement * index_element = new TiXmlElement( text );
		FrameNumList_element->LinkEndChild(index_element);

		sprintf_s(text, "%d", m_cornerData.frameNumList[i]);
		TiXmlText * FrameNumList_text = new TiXmlText( text );
		index_element->LinkEndChild( FrameNumList_text );
	}

	doc.SaveFile( filename.c_str() );
}

void XMLWriter::writePointsCloud( std::string filename, const std::vector<cv::Point3f>& points )
{
	char text[100];
	char* segmentation = " ";
	TiXmlDocument doc;
	TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	doc.LinkEndChild( decl );

	TiXmlElement *Points_element = new TiXmlElement( "Points" );
	Points_element->SetAttribute("size",points.size());
	doc.LinkEndChild( Points_element );

	for(unsigned int i=0;i<points.size();i++)
	{
		sprintf_s(text, "%s%d", "p",i+1);
		TiXmlElement * index_element = new TiXmlElement( text );
		Points_element->LinkEndChild(index_element);

		sprintf_s(text, "%f%s%f%s%f", points[i].x,segmentation
									,points[i].y,segmentation
									,points[i].z,segmentation);
		TiXmlText *point_text = new TiXmlText( text );
		index_element->LinkEndChild( point_text );
	}

	doc.SaveFile( filename.c_str() );
}

void XMLWriter::writeImageRT( TiXmlDocument& doc,const CamPara& camparameter )
{
	char text[100];
	char* segmentation = " ";
	TiXmlElement * Image_RT_element = new TiXmlElement( "Image_RT" );
	Image_RT_element->SetAttribute("size",camparameter.imgRTVec.size());
	doc.LinkEndChild( Image_RT_element );

	for(unsigned int i=0;i<camparameter.imgRTVec.size();i++)
	{
		sprintf_s(text, "%s%d", "n" ,i);
		TiXmlElement * index_element = new TiXmlElement( text );
		Image_RT_element->LinkEndChild(index_element);

		TiXmlElement * R_element = new TiXmlElement( "R" );
		index_element->LinkEndChild(R_element);
	
		sprintf_s(text, "%f%s%f%s%f", camparameter.imgRTVec[i].R.at<double>(0,0)
								,segmentation
								,camparameter.imgRTVec[i].R.at<double>(1,0)
								,segmentation
								,camparameter.imgRTVec[i].R.at<double>(2,0));


		TiXmlText * R_text = new TiXmlText( text );
		R_element->LinkEndChild( R_text );

		TiXmlElement * T_element = new TiXmlElement( "T" );
		index_element->LinkEndChild(T_element);

		sprintf_s(text, "%f%s%f%s%f", camparameter.imgRTVec[i].T.at<double>(0,0)
								,segmentation
								,camparameter.imgRTVec[i].T.at<double>(1,0)
								,segmentation
								,camparameter.imgRTVec[i].T.at<double>(2,0));

		TiXmlText * T_text = new TiXmlText( text );
		T_element->LinkEndChild( T_text );

		//单张图片重投影误差
		TiXmlElement * Repro_Norm_error_element = new TiXmlElement( "Repro_Norm_error" );
		index_element->LinkEndChild(Repro_Norm_error_element);

		sprintf_s(text, "%f", camparameter.reprojectNormErr[i]);

		TiXmlText * Repro_Norm_error_text = new TiXmlText( text );
		Repro_Norm_error_element->LinkEndChild( Repro_Norm_error_text );

	}
}

void XMLWriter::writeGroupRT( TiXmlElement* element,const double RotVector[3],const double TraVector[3] )
{
	char text[100];
	char* segmentation = " ";
	sprintf_s(text, "%f%s%f%s%f", RotVector[0],segmentation
								,RotVector[1],segmentation
								,RotVector[2]);
	TiXmlElement * R_element = new TiXmlElement( "R" );
	element->LinkEndChild(R_element);
	TiXmlText * R_text = new TiXmlText( text );
	R_element->LinkEndChild( R_text );

	sprintf_s(text, "%f%s%f%s%f", TraVector[0],segmentation
								,TraVector[1],segmentation
								,TraVector[2]);
	TiXmlElement * T_element = new TiXmlElement( "T" );
	element->LinkEndChild(T_element);
	TiXmlText * T_text = new TiXmlText( text );
	T_element->LinkEndChild( T_text );
}

void XMLWriter::writeLightPenPara(std::string filename, const std::vector<TagPoint3f>& pnts,const Point3f probeCenter)
{
	char text[100];
	char* segmentation = " ";
	TiXmlDocument doc;
	TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
	doc.LinkEndChild(decl);

	TiXmlElement *Points_element = new TiXmlElement("Points_LightPenCoordinate");
	Points_element->SetAttribute("size", pnts.size());
	doc.LinkEndChild(Points_element);

	for (unsigned int i = 0; i < pnts.size(); i++)
	{
		sprintf_s(text, "p%d",(int)pnts[i][0]);
		TiXmlElement * index_element = new TiXmlElement(text);
		Points_element->LinkEndChild(index_element);

		sprintf_s(text, "%f%s%f%s%f", pnts[i][1], segmentation
			, pnts[i][2], segmentation
			, pnts[i][3], segmentation);
		TiXmlText *point_text = new TiXmlText(text);
		index_element->LinkEndChild(point_text);
	}

	TiXmlElement *probeCenter_element = new TiXmlElement("probeCenter");
	sprintf_s(text, "_");
	TiXmlElement * _element = new TiXmlElement(text);
	probeCenter_element->LinkEndChild(_element);
	sprintf_s(text, "%f%s%f%s%f", probeCenter.x, segmentation
		, probeCenter.y, segmentation
		, probeCenter.z, segmentation);
	TiXmlText *point_text = new TiXmlText(text);
	_element->LinkEndChild(point_text);

	doc.LinkEndChild(probeCenter_element);

	doc.SaveFile(filename.c_str());
}
void XMLWriter::writeLightPenPara(std::string filename, const std::vector<TagPoint3f>& pnts)
{
	char text[100];
	char* segmentation = " ";
	TiXmlDocument doc;
	TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
	doc.LinkEndChild(decl);

	TiXmlElement *Points_element = new TiXmlElement("Points_LightPenCoordinate");
	Points_element->SetAttribute("size", pnts.size());
	doc.LinkEndChild(Points_element);

	for (unsigned int i = 0; i < pnts.size(); i++)
	{
		sprintf_s(text, "p%d",(int)pnts[i][0]);
		TiXmlElement * index_element = new TiXmlElement(text);
		Points_element->LinkEndChild(index_element);

		sprintf_s(text, "%f%s%f%s%f", pnts[i][1], segmentation
			, pnts[i][2], segmentation
			, pnts[i][3], segmentation);
		TiXmlText *point_text = new TiXmlText(text);
		index_element->LinkEndChild(point_text);
	}
	doc.SaveFile(filename.c_str());
}
void XMLWriter::writeMatData( std::string filename,const cv::Mat& matData )
{
	char text[100];
	char* segmentation = " ";
	TiXmlDocument doc;
	TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	doc.LinkEndChild( decl );

	TiXmlElement *Points_element = new TiXmlElement( "Mat" );
	//Points_element->SetAttribute("rows",matData);
	doc.LinkEndChild( Points_element );

	for(int i=0;i<matData.rows;i++)
	{
		for(int j = 0;j<matData.cols;j++)
		{
		sprintf_s(text, "%s%d", "p",i+1);
		TiXmlElement * index_element = new TiXmlElement( text );
		Points_element->LinkEndChild(index_element);

		sprintf_s(text, "%f%s", matData.at<double>(i,j),segmentation);
		TiXmlText *point_text = new TiXmlText( text );
		index_element->LinkEndChild( point_text );
		}
	}

	doc.SaveFile( filename.c_str() );
}

