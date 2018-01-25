#pragma once
#include "SharedMethod.h"

class XMLReader
{
public:
	XMLReader(){  };
	
	///��ȡ����������ڲ�����������������������궨�������
	static bool readCamPara(std::string filename,CamPara& m_camPara);

	///��ȡ�������̬��������ȡ��Left_Mid��Right_Mid��Right_Left
	static bool readCamGroupPara(std::string filename,CamGroupPara& m_camGroupPara);

	///��ȡ�ǵ����ݣ�����ͼ��ߴ硢�ǵ��ά���ꡢ��ά���꣨z=0)��ÿ��ǵ�����ͼ������
	static bool readCornerData(std::string filename, CalibrationData& m_cornerData);

	///��ȡ��ά��������
	static bool readPointsCloud(std::string filename, std::vector<cv::Point3f>& m_points);
	
	///��ȡ��ʱ궨�Ľ��
	static bool readLightPenPara(std::string filename,std::vector<TagPoint3f>& pnts,cv::Point3f& pnt);
	//��ȡ�����Ʋ������������������λ��Ϣ
	//chengwei added
	//��ȡ�������������
	static bool readLightPenDotPara(std::string filename,std::vector<TagPoint3f>& pnts);
	//��ȡ����Ŀ¼��Ϣ
	static bool readWorkplacePath(std::string filename,std::string& path);
private:
	

	///�����ڲ���Ԫ��
	static bool parseIntrinsicparametersElement(TiXmlHandle hDoc, CamPara& m_camPara);

	///��������Ԫ��
	static bool parseDistortionsElement(TiXmlHandle hDoc, CamPara& m_camPara);

	///�������Ԫ�أ���������㲻��Ҫ����
	static bool parseErrorElement(TiXmlHandle hDoc, CamPara& m_camPara);

	///����ͼ��ߴ�Ԫ��
	static bool parseImageSizeElement(TiXmlHandle hDoc, CalibrationData& m_cornerData);

	///�����ǵ��ά����Ԫ��
	static bool parseCornerPointsElement(TiXmlHandle hDoc,const vector<int>& CornerNumPerFrame,CalibrationData& m_cornerData);

	///�����ǵ���ά�㣨z=0)Ԫ��
	static bool parseFeaturePointsElement(TiXmlHandle hDoc,const vector<int>& CornerNumPerFrame, CalibrationData& m_cornerData);

	///����ÿ��ǵ���Ԫ��
	static bool parseCornerNumPerFrameElement(TiXmlHandle hDoc, vector<int>& CornerNumPerFrame);

	///�����궨ͼƬ����Ԫ��
	static bool parseFrameNumListElement(TiXmlHandle hDoc, CalibrationData& m_cornerData);

	///����Left_MidԪ��
	static bool parseLeftMidElement(TiXmlHandle hDoc, CamGroupPara& m_camGroupPara);

	///����Right_MidԪ��
	static bool parseRightMidElement(TiXmlHandle hDoc, CamGroupPara& m_camGroupPara);

	///����Right_LeftԪ��
	static bool parseRightLeftElement(TiXmlHandle hDoc, CamGroupPara& m_camGroupPara);

	///�����궨�������Ԫ��
	static bool parseImageRTElement(TiXmlHandle hDoc, CamPara& m_camPara);
	
};

