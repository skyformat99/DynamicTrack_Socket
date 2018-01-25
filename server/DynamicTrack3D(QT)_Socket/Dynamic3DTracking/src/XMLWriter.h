#pragma once
#include "SharedMethod.h"

//ʹ��TinyXML
class XMLWriter 
{
public:
	XMLWriter(){};

	///���浥��������ڲ���������������������ͱ궨ͼƬ�����
	static void writeCamPara(std::string filename, const CamPara& camparameter);

	///�����������̬������������Left_Mid��Right_Mid
	static void writeCamGroupPara(std::string filename, const CamGroupPara& camGroupParameter);

	///����ǵ����ݣ�����ͼ��ߴ硢�ǵ��ά���ꡢ��ά���꣨z=0)��ÿ��ǵ�����ͼ������
	static void writeCornerData(std::string filename, const CalibrationData& m_cornerData);

	///�����������
	static void writePointsCloud(std::string filename, const std::vector<cv::Point3f>& points);

	///�����ʲ�ͷ�궨�����ݣ�1���������ϵ�µĹ�����ĵ㣻2����ͷ�궨���Ĳ�������ϵ�µĲ�ͷ����λ��
	static void writeLightPenPara(std::string filename, const std::vector<Vec4f>& pnts, const Point3f probeCenter);
	static void writeLightPenPara(std::string filename, const std::vector<Vec4f>& pnts);
	//chengwei added
	//���� : �� cv::Mat ����д�뵽 .xml �ļ�
	//���� : -1�����ļ�ʧ�ܣ�0��д�����ݳɹ���1������Ϊ��
    //* ���� : fileName	[in]	�ļ���
     // * ���� : matData	[in]	��������
	//ע�⣬Ŀǰֻ���double�;�����в���
	static void writeMatData(std::string fileName,const cv::Mat& matData);

private:

	///���浥������궨ͼƬ���������R�Ծ�����ʽ����
	static void writeImageRT(TiXmlDocument& doc,const CamPara& camparameter);

	///����һ���������̬������R��������ʽ����
	static void writeGroupRT(TiXmlElement* element,const double RotVector[3],const double TraVector[3]);
};

