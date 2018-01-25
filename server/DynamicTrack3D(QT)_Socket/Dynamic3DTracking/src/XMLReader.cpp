#include "stdafx.h"
#include "XMLReader.h"

bool XMLReader::readCamPara( std::string filename,CamPara& m_camPara )
{
	//加载文件
	TiXmlDocument doc;
	if( !doc.LoadFile( filename.c_str()) )
	{
		return false;
	}
	TiXmlHandle hDoc(&doc);

	if( !parseIntrinsicparametersElement(hDoc,m_camPara) )
		return false;
	if( !parseDistortionsElement(hDoc,m_camPara) )
		return false;
 	if( !parseImageRTElement(hDoc,m_camPara) )
		return false;

	return true;
}

bool XMLReader::readCamGroupPara( std::string filename,CamGroupPara& m_camGroupPara )
{
	TiXmlDocument doc;
	if( !doc.LoadFile( filename.c_str()) )
	{
		return false;
	}
	TiXmlHandle hDoc(&doc);

	if( !parseLeftMidElement(hDoc,m_camGroupPara) )
		return false;
	if( !parseRightMidElement(hDoc,m_camGroupPara) )
		return false;
	if (!parseRightLeftElement(hDoc, m_camGroupPara))
		return false;

	return true;
}

bool XMLReader::readCornerData( std::string filename,CalibrationData& m_cornerData )
{
	TiXmlDocument doc;
	if( !doc.LoadFile( filename.c_str()) )
	{
		return false;
	}
	TiXmlHandle hDoc(&doc);

	vector<int> CornerNumPerFrame;

	if( !parseImageSizeElement(hDoc,m_cornerData) )
		return false;
	if( !parseCornerNumPerFrameElement(hDoc,CornerNumPerFrame) )
		return false;
	if( !parseCornerPointsElement(hDoc,CornerNumPerFrame,m_cornerData) )
		return false;
	if( !parseFeaturePointsElement(hDoc,CornerNumPerFrame,m_cornerData) )
		return false;
	if( !parseFrameNumListElement(hDoc,m_cornerData) )
		return false;

	return true;
}

bool XMLReader::readPointsCloud( std::string filename,std::vector<cv::Point3f>& m_points )
{
	char* segmentation = " ";
	TiXmlDocument doc;
	if( !doc.LoadFile( filename.c_str()) )
	{
		return false;
	}
	TiXmlHandle hDoc(&doc);
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("Points").Element();
	if( pElem->ValueTStr() == "Points" )
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChildElement();
		const char* jj = tElem->Value();
		for(tElem;tElem;tElem=tElem->NextSiblingElement())
		{
			std::vector<std::string> nums = split( tElem->GetText(),segmentation); 
			float point[3];
			sscanf_s( nums[0].c_str(), "%f", &point[0] );
			sscanf_s( nums[1].c_str(), "%f", &point[1] );
			sscanf_s( nums[2].c_str(), "%f", &point[2] );
			m_points.push_back(cv::Point3f(point[0],point[1],point[2]));
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool XMLReader::parseIntrinsicparametersElement( TiXmlHandle hDoc, CamPara& m_camPara )
{
	char* segmentation = " ";
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("Intrinsic_parameters").Element();
	if( pElem->ValueTStr() == "Intrinsic_parameters" )
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChild("fc")->ToElement();
		if(  tElem->ValueTStr() == "fc" )
		{
			std::vector<std::string> nums = split( tElem->GetText(),segmentation); 
			float fc0,fc1;
			sscanf_s( nums[0].c_str(), "%f", &fc0 );
			sscanf_s( nums[1].c_str(), "%f", &fc1 );
			m_camPara.CameraIntrinsic[0][0] = fc0;
			m_camPara.CameraIntrinsic[1][1] = fc1;
		}
		tElem = pElem->FirstChild("cc")->ToElement();
		if(  tElem->ValueTStr() == "cc" )
		{
			std::vector<std::string> nums = split( tElem->GetText(),segmentation); 
			float cc0,cc1;
			sscanf_s( nums[0].c_str(), "%f", &cc0 );
			sscanf_s( nums[1].c_str(), "%f", &cc1 );
			m_camPara.CameraIntrinsic[0][2] = cc0;
			m_camPara.CameraIntrinsic[1][2] = cc1;
		}
	}
	else
	{
		return false;
	}

	return true;
}

bool XMLReader::parseDistortionsElement( TiXmlHandle hDoc, CamPara& m_camPara )
{
	char* segmentation = " ";
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("Distortions").Element();
	if( pElem->ValueTStr() == "Distortions" )
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChild("kc")->ToElement();
		if(  tElem->ValueTStr() == "kc" )
		{
			std::vector<std::string> nums = split( tElem->GetText(),segmentation); 
			float kc[4];
			for(int i=0;i<4;i++)
			{
				sscanf_s( nums[i].c_str(), "%f", &kc[i] );
				m_camPara.DistortionCoeffs[i] = kc[i];
			}
		}
	}
	else
	{
		return false;
	}

	return true;
}

bool XMLReader::parseImageRTElement(TiXmlHandle hDoc, CamPara& m_camPara)
{
	char* segmentation = " ";
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("Image_RT").Element();
	if( pElem->ValueTStr() == "Image_RT" )
	{
		int num = pElem->FirstAttribute()->IntValue();
		TiXmlElement* tElem;
		tElem = pElem->FirstChild()->ToElement();
		for (int i = 0; i < num; i++)
		{
			RT rt; rt.R = Mat(3, 1, CV_64F); rt.T = Mat(3, 1, CV_64F);
			TiXmlElement* _tElem;
			_tElem = tElem->FirstChild()->ToElement();
			for(_tElem;_tElem;_tElem=_tElem->NextSiblingElement())
			{
				std::vector<std::string> nums = split( _tElem->GetText(),segmentation); 
				float vec[3];
				for( int j=0;j<3;j++ )
				{
					sscanf_s( nums[j].c_str(), "%f", &vec[j] );
					if (_tElem->ValueTStr() == "R")
						rt.R.at<double>(j, 0) = vec[j];
					else if (_tElem->ValueTStr() == "T")
						rt.T.at<double>(j, 0) = vec[j];
					else
						break;
				}
			}
			m_camPara.imgRTVec.push_back(rt);
			tElem = tElem->NextSiblingElement();
		}
	}
	else
	{
		return false;
	}

	return true;
}

bool XMLReader::parseErrorElement( TiXmlHandle hDoc, CamPara& m_camPara )
{
	char* segmentation = " ";
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("Error").Element();
	std::string error_tag_string[4];
	error_tag_string[0]="pixel_error";error_tag_string[1]="fc_error";
	error_tag_string[2]="cc_error";error_tag_string[3]="kc_error";
	if( pElem->ValueTStr() == "Error" )
	{
		for(int i=0;i<4;i++)
		{
			TiXmlElement* tElem;
			tElem = pElem->FirstChild(error_tag_string[i].c_str())->ToElement();
			std::vector<std::string> nums = split( tElem->GetText(),segmentation); 
			int n=0;

			//判断是否为kc_error,因为它有4个参数，其他均两个参数
			if( tElem->ValueTStr() == "kc_error" ) n=4;
			else n=2;

			float error[5];
			for(int j=0;j<n;j++)
			{
				sscanf_s( nums[j].c_str(), "%f", &error[j] );
			}

			if(  tElem->ValueTStr() == "pixel_error" )
			{
				m_camPara.ReprojectionError[0] = error[0];m_camPara.ReprojectionError[1] = error[1];
			}
			else if( tElem->ValueTStr() == "fc_error" )
			{
				m_camPara.fcError[0]=error[0];m_camPara.fcError[1]=error[1];
			}
			else if( tElem->ValueTStr() == "cc_error" )
			{
				m_camPara.ccError[0]=error[0];m_camPara.ccError[1]=error[1];
			}
			else if( tElem->ValueTStr() == "kc_error" )
			{
				for(int j=0;j<4;j++) 
					m_camPara.kcError[j]=error[j];
			}
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool XMLReader::parseImageSizeElement( TiXmlHandle hDoc, CalibrationData& m_cornerData )
{
	char* segmentation = " ";
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("ImageSize").Element();
	if( pElem->ValueTStr() == "ImageSize" )
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChild("Height")->ToElement();
		if(  tElem->ValueTStr() == "Height" )
		{
			std::vector<std::string> nums = split( tElem->GetText(),segmentation); 
			float height;
			sscanf_s( nums[0].c_str(), "%f", &height );
			m_cornerData.imgHeight = int(height);
		}

		tElem = pElem->FirstChild("Width")->ToElement();
		if(  tElem->ValueTStr() == "Width" )
		{
			std::vector<std::string> nums = split( tElem->GetText(),segmentation); 
			float width;
			sscanf_s( nums[0].c_str(), "%f", &width );
			m_cornerData.imgWidth = int(width);
		}
	}
	else
	{
		return false;
	}

	return true;
}

bool XMLReader::parseCornerPointsElement( TiXmlHandle hDoc,const vector<int>& CornerNumPerFrame, CalibrationData& m_cornerData )
{
	char* segmentation = " ";
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("Corner_Points").Element();
	if( pElem->ValueTStr() == "Corner_Points" )
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChildElement();
		const char* jj = tElem->Value();
		for (unsigned int i = 0; i < CornerNumPerFrame.size();i++)
		{
			vector<Point2f> pnts2d;
			for (int j = 0; j<CornerNumPerFrame[i]; tElem = tElem->NextSiblingElement(),j++)
			{
				vector<string> nums = split( tElem->GetText(),segmentation); 
				float point[2];
				sscanf_s( nums[0].c_str(), "%f", &point[0] );
				sscanf_s( nums[1].c_str(), "%f", &point[1] );
				pnts2d.push_back(cv::Point2f(point[0],point[1]));
			}
			m_cornerData.plane2dPntsVec.push_back(pnts2d);
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool XMLReader::parseFeaturePointsElement( TiXmlHandle hDoc,const vector<int>& CornerNumPerFrame, CalibrationData& m_cornerData )
{
	char* segmentation = " ";
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("Feature_Points").Element();
	if( pElem->ValueTStr() == "Feature_Points" )
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChildElement();
		for (unsigned int i = 0; i < CornerNumPerFrame.size();i++)
		{
			vector<Point3f> pnts3d;
			for (int j = 0; j<CornerNumPerFrame[i]; tElem = tElem->NextSiblingElement(),j++)
			{
				vector<string> nums = split( tElem->GetText(),segmentation); 
				float point[3];
				sscanf_s( nums[0].c_str(), "%f", &point[0] );
				sscanf_s( nums[1].c_str(), "%f", &point[1] );
				sscanf_s( nums[2].c_str(), "%f", &point[2] );
				pnts3d.push_back(Point3f(point[0],point[1],point[2]));
			}
			m_cornerData.plane3dPntsVec.push_back(pnts3d);
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool XMLReader::parseCornerNumPerFrameElement( TiXmlHandle hDoc, vector<int>& CornerNumPerFrame )
{
	char* segmentation = " ";
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("CornerNumPerFrame").Element();
	if( pElem->ValueTStr() == "CornerNumPerFrame" )
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChildElement();
		const char* jj = tElem->Value();
		for(tElem;tElem;tElem=tElem->NextSiblingElement())
		{
			std::vector<std::string> nums = split( tElem->GetText(),segmentation); 
			int num;
			sscanf_s( nums[0].c_str(), "%d", &num );
			CornerNumPerFrame.push_back(num);
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool XMLReader::parseFrameNumListElement( TiXmlHandle hDoc, CalibrationData& m_cornerData )
{
	char* segmentation = " ";
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("FrameNumList").Element();
	int n = pElem->FirstAttribute()->IntValue();
	if( pElem->ValueTStr() == "FrameNumList" )
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChildElement();
		const char* jj = tElem->Value();
		for(tElem;tElem;tElem=tElem->NextSiblingElement())
		{
			std::vector<std::string> nums = split( tElem->GetText(),segmentation); 
			int index;
			sscanf_s( nums[0].c_str(), "%d", &index );
			m_cornerData.frameNumList.push_back(index);
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool XMLReader::parseLeftMidElement( TiXmlHandle hDoc, CamGroupPara& m_camGroupPara )
{
	char* segmentation = " ";
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("Left_Mid").Element();
	if( pElem->ValueTStr() == "Left_Mid" )
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChildElement();
		std::vector<std::string> nums = split( tElem->GetText(),segmentation); 
		for(tElem;tElem;tElem=tElem->NextSiblingElement())
		{
			std::vector<std::string> nums = split( tElem->GetText(),segmentation); 
			float vec[3];
			for( int j=0;j<3;j++ )
			{
				sscanf_s( nums[j].c_str(), "%f", &vec[j] );
				if( tElem->ValueTStr() == "R")
					m_camGroupPara.left2MidRotVector[j] = vec[j];
				else if( tElem->ValueTStr() == "T")
					m_camGroupPara.left2MidTraVector[j] = vec[j];
			}
		}
	}
	else
	{
		return false;
	}

	return true;
}

bool XMLReader::parseRightMidElement( TiXmlHandle hDoc, CamGroupPara& m_camGroupPara )
{
	char* segmentation = " ";
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("Right_Mid").Element();
	if( pElem->ValueTStr() == "Right_Mid" )
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChildElement();
		std::vector<std::string> nums = split( tElem->GetText(),segmentation); 
		for(tElem;tElem;tElem=tElem->NextSiblingElement())
		{
			std::vector<std::string> nums = split( tElem->GetText(),segmentation); 
			float vec[3];
			for( int j=0;j<3;j++ )
			{
				sscanf_s( nums[j].c_str(), "%f", &vec[j] );
				if( tElem->ValueTStr() == "R")
					m_camGroupPara.right2MidRotVector[j] = vec[j];
				else if( tElem->ValueTStr() == "T")
					m_camGroupPara.right2MidTraVector[j] = vec[j];
			}
		}
	}
	else
	{
		return false;
	}

	return true;
}

bool XMLReader::parseRightLeftElement( TiXmlHandle hDoc, CamGroupPara& m_camGroupPara )
{
	char* segmentation = " ";
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("Right_Left").Element();
	if( pElem->ValueTStr() == "Right_Left" )
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChildElement();
		std::vector<std::string> nums = split( tElem->GetText(),segmentation); 
		for(tElem;tElem;tElem=tElem->NextSiblingElement())
		{
			std::vector<std::string> nums = split( tElem->GetText(),segmentation); 
			float vec[3];
			for( int j=0;j<3;j++ )
			{
				sscanf_s( nums[j].c_str(), "%f", &vec[j] );
				if( tElem->ValueTStr() == "R")
					m_camGroupPara.right2LeftRotVector[j] = vec[j];
				else if( tElem->ValueTStr() == "T")
					m_camGroupPara.right2LeftTraVector[j] = vec[j];
			}
		}
	}
	else
	{
		return false;
	}

	return true;
}

bool XMLReader::readLightPenPara(std::string filename, std::vector<TagPoint3f>& pnts, cv::Point3f& pnt)
{
	char* segmentation = " ";
	TiXmlDocument doc;
	if (!doc.LoadFile(filename.c_str()))
	{
		return false;
	}
	TiXmlHandle hDoc(&doc);

	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("Points_LightPenCoordinate").Element();
	if (pElem->ValueTStr() == "Points_LightPenCoordinate")
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChildElement();
		std::vector<std::string> nums = split(tElem->GetText(), segmentation);
		for (tElem; tElem; tElem = tElem->NextSiblingElement())
		{
			std::vector<std::string> nums = split(tElem->GetText(), segmentation);
			float vec[3];
			for (int j = 0; j < 3; j++)
			{
				sscanf_s(nums[j].c_str(), "%f", &vec[j]);
			}
			TagPoint3f _pnt; int _tag;
			string str(tElem->Value());
			char o = str.at(1);
			sscanf_s(&o, "%d",&_tag);
			_pnt[0] = float(_tag);
			_pnt[1] = vec[0];
			_pnt[2] = vec[1];
			_pnt[3] = vec[2];
			pnts.push_back(_pnt);
		}
	}
	else
	{
		return false;
	}

	pElem = hDoc.FirstChild("probeCenter").Element();
	if (pElem->ValueTStr() == "probeCenter")
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChildElement();
		std::vector<std::string> nums = split(tElem->GetText(), segmentation);
		float vec[3];
		for (int j = 0; j < 3; j++)
		{
			sscanf_s(nums[j].c_str(), "%f", &vec[j]);
		}
		pnt = cv::Point3f(vec[0], vec[1], vec[2]);
	}
	else
	{
		return false;
	}

	return true;
}
//chengwei added
bool XMLReader::readLightPenDotPara(std::string filename,std::vector<TagPoint3f>& pnts)
{
	char* segmentation = " ";
	TiXmlDocument doc;
	if (!doc.LoadFile(filename.c_str()))
	{
		return false;
	}
	TiXmlHandle hDoc(&doc);

	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("Points_LightPenFeatures").Element();
	if (pElem->ValueTStr() == "Points_LightPenFeatures")
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChildElement();
		std::vector<std::string> nums = split(tElem->GetText(), segmentation);
		for (tElem; tElem; tElem = tElem->NextSiblingElement())
		{
			std::vector<std::string> nums = split(tElem->GetText(), segmentation);
			float vec[3];
			for (int j = 0; j < 3; j++)
			{
				sscanf_s(nums[j].c_str(), "%f", &vec[j]);
			}
			TagPoint3f _pnt; int _tag;
			string str(tElem->Value());
			char o = str.at(1);
			sscanf_s(&o, "%d",&_tag);
			_pnt[0] = float(_tag);
			_pnt[1] = vec[0];
			_pnt[2] = vec[1];
			_pnt[3] = vec[2];
			pnts.push_back(_pnt);
		}
	}
	else
	{
		return false;
	}
	return true;
}
bool XMLReader::readWorkplacePath(std::string filename,std::string& path)
{
	TiXmlDocument doc;
	if( !doc.LoadFile( filename.c_str()) )
	{
		return false;
	}
	TiXmlHandle hDoc(&doc);
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("Workplace_path").Element();
	if( pElem->ValueTStr() == "Workplace_path" )
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChild("Path")->ToElement();
		if(  tElem->ValueTStr() == "Path" )
		{
			path = tElem->GetText(); 
		}
	}
	else
	{
		return false;
	}

	return true;
}

