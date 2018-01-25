#pragma once
#include "SharedHead.h"
#include "windows.h"
#include <vector>
#include <string>



///c++字符串分割函数
///以pattern为分隔符，返回vector<string>
static std::vector<std::string> split(std::string str, std::string pattern);

static void jointImages_row( Mat& dst,const vector<Mat>& images );//将几张图片组合成一列
static void jointImages_col(Mat& dst,const vector<Mat>& images);//将几张图片组合成一行
static void m_resize(Mat &image,float scale);

///数据类型转化
//static QImage mat2QImage(const Mat& src );
//static Mat QImage2Mat( const QImage& src );
//static void QPointF2Point2f(const vector<QPointF>& qPnts,vector<Point2f>& pnts);

///模板函数
///进行三维重建前确认点的tag匹配性，并进行数据类型转换
///进行刚体变换前确认点的tag匹配性，并进行数据类型转换
static void pntsTagConfirm(const vector<TagPoint3f>& pnts1, const vector<TagPoint3f>& pnts2,
							vector<Point3f>& out_pnts1, vector<Point3f>& out_pnts2, vector<int>& tags);
static void pntsTagConfirm(const vector<TagPoint3f>& pnts1, const vector<TagPoint3f>& pnts2,
							vector<Point3f>& out_pnts1, vector<Point3f>& out_pnts2);

static void pntsTagConfirm(const vector<TagPoint3f>& pnts,vector<Point3f>& out_pnts);//chengwei added
static void pntsTagConfirm(const vector<TagPoint3f>& pnts,vector<Point3f>& out_pnts,vector<int>& tags);//chengwei added

static void pntsTagConfirm(const vector<TagPoint2f>& pnts1, const vector<TagPoint2f>& pnts2,
							vector<Point2f>& out_pnts1, vector<Point2f>& out_pnts2, vector<int>& tags);
static void pntsTagConfirm(const vector<TagPoint2f>& pnts1, const vector<TagPoint2f>& pnts2,
							vector<Point2f>& out_pnts1, vector<Point2f>& out_pnts2);
static void pntsTagConfirm(const vector<TagPoint2f>& pnts1,vector<Point2f>& out_pnts1,vector<int>& tags);//chengwei added
static void pntsTagConfirm(const vector<TagPoint2f>& pnts1,vector<Point2f>& out_pnts1);//chengwei added
//给没有标签信息的三维点集增加标签信息
static void addPntsTag(const vector<vector<Point3f>> Pnts3fAll, const vector<int>& tags,vector<vector<TagPoint3f>> &TagPnts3fAll);//chengwei addded
static void addPntsTag(const vector<Point3f> Pnts3fAll, const vector<int>& tags,vector<TagPoint3f> &TagPnts3fAll);//chengwei addded
///判断标定图片的数量是否一致
//static int isReadedImgNumEqual(QMap<CamPosition, QVector<QString>> imgPathMap);

///判断标定图片的尺寸是否一致
//static bool isReadedImgAllSameSize(QMap<CamPosition, vector<QString>> imgPathMap, int& h, int& w);

void jointImages_row( Mat& dst,const vector<Mat>& images )
{
	int img_num = images.size();
	Mat src = images[0];
	dst = Mat::zeros(img_num*src.rows,src.cols,CV_8U);
	for( unsigned int i=0;i<img_num;i++ )
	{
		cv::Rect m_rect(0,i*src.rows,src.cols,src.rows);
		images[i].copyTo(dst(m_rect));
	}
}

void jointImages_col(Mat& dst,const vector<Mat>& images)
{
	int img_num = images.size();
	Mat src = images[0];
	dst = Mat::zeros(src.rows,img_num*src.cols,CV_8U);
	for( int i=0;i<img_num;i++ )
	{
		cv::Rect m_rect(i*src.cols,0,src.cols,src.rows);
		images[i].copyTo(dst(m_rect));
	}
}

void m_resize(Mat &image,float scale)
{
	int width = image.cols;
	int height = image.rows;
    //double img_scale=width/height;
    //chengwei changed

	cv::Size2f m_size = cv::Size2f(width/scale,height/scale);
	cv::resize(image, image,m_size);	
}
//static int isReadedImgNumEqual(QMap<CamPosition, vector<QString>> imgPathMap)
//{
//	QMap<CamPosition, vector<QString>>::Iterator imgPathMapIter;
//	imgPathMapIter = imgPathMap.begin();
//	int imgNum = (*imgPathMapIter).size();
//	for (; imgPathMapIter != imgPathMap.end(); imgPathMapIter++)
//	{
//		if (imgNum != (*imgPathMapIter).size())
//			return 0;
//	}
//
//	return imgNum;
//}

///模板函数
///进行三维重建前确认点的tag匹配性，并进行数据类型转换
///进行刚体变换前确认点的tag匹配性，并进行数据类型转换
void pntsTagConfirm(const vector<TagPoint3f>& pnts1, const vector<TagPoint3f>& pnts2,
					vector<Point3f>& out_pnts1, vector<Point3f>& out_pnts2, vector<int>& tags)
{
	for (unsigned int i = 0; i < pnts1.size(); i++)
	{
		for (unsigned int j = 0; j < pnts2.size(); j++)
		{
			if (pnts1[i][0] == pnts2[j][0])
			{
				out_pnts1.push_back(Point3f(pnts1[i][1], pnts1[i][2], pnts1[i][3]));
				out_pnts2.push_back(Point3f(pnts2[j][1], pnts2[j][2], pnts2[j][3]));
				tags.push_back(int(pnts1[i][0]));
			}
		}
	}
}

void pntsTagConfirm(const vector<TagPoint3f>& pnts1, const vector<TagPoint3f>& pnts2,
					vector<Point3f>& out_pnts1, vector<Point3f>& out_pnts2)
{
	for (unsigned int i = 0; i < pnts1.size(); i++)
	{
		for (unsigned int j = 0; j < pnts2.size(); j++)
		{
			if (pnts1[i][0] == pnts2[j][0])
			{
				out_pnts1.push_back(Point3f(pnts1[i][1], pnts1[i][2], pnts1[i][3]));
				out_pnts2.push_back(Point3f(pnts2[j][1], pnts2[j][2], pnts2[j][3]));
			}
		}
	}
}

void pntsTagConfirm(const vector<TagPoint3f>& pnts,vector<Point3f>& out_pnts)
{
	//本部分仅仅为了去除标签信息
	for (unsigned int i = 0; i < pnts.size(); i++)
	{
				out_pnts.push_back(Point3f(pnts[i][1], pnts[i][2], pnts[i][3]));
	}
}
void pntsTagConfirm(const vector<TagPoint3f>& pnts,vector<Point3f>& out_pnts,vector<int>& tags)
{
	//本部分仅仅为了去除标签信息
	for (unsigned int i = 0; i < pnts.size(); i++)
	{
				out_pnts.push_back(Point3f(pnts[i][1], pnts[i][2], pnts[i][3]));
				tags.push_back(int(pnts[i][0]));
	}
}

void pntsTagConfirm(const vector<TagPoint2f>& pnts1, const vector<TagPoint2f>& pnts2,
	vector<Point2f>& out_pnts1, vector<Point2f>& out_pnts2, vector<int>& tags)
{
	for (unsigned int i = 0; i < pnts1.size(); i++)
	{
		for (unsigned int j = 0; j < pnts2.size(); j++)
		{
			if (pnts1[i][0] == pnts2[j][0])
			{
				out_pnts1.push_back(Point2f(pnts1[i][1], pnts1[i][2]));
				out_pnts2.push_back(Point2f(pnts2[j][1], pnts2[j][2]));
				tags.push_back(int(pnts1[i][0]));
			}
		}
	}
}

void pntsTagConfirm(const vector<TagPoint2f>& pnts1, const vector<TagPoint2f>& pnts2,
	vector<Point2f>& out_pnts1, vector<Point2f>& out_pnts2)
{
	for (unsigned int i = 0; i < pnts1.size(); i++)
	{
		for (unsigned int j = 0; j < pnts2.size(); j++)
		{
			if (pnts1[i][0] == pnts2[j][0])
			{
				out_pnts1.push_back(Point2f(pnts1[i][1], pnts1[i][2]));
				out_pnts2.push_back(Point2f(pnts2[j][1], pnts2[j][2]));
			}
		}
	}
}
void pntsTagConfirm(const vector<TagPoint2f>& pnts1,vector<Point2f>& out_pnts1)
{
	//本部分仅仅为了去除标签信息
	for (unsigned int i = 0; i < pnts1.size(); i++)
	{
		out_pnts1.push_back(Point2f(pnts1[i][1], pnts1[i][2]));
	}
}
void pntsTagConfirm(const vector<TagPoint2f>& pnts1,vector<Point2f>& out_pnts1,vector<int>& tags)
{
	//本部分仅仅为了去除标签信息
	for (unsigned int i = 0; i < pnts1.size(); i++)
	{
		out_pnts1.push_back(Point2f(pnts1[i][1], pnts1[i][2]));
		tags.push_back(int(pnts1[i][0]));
	}
}

void addPntsTag(const vector<vector<Point3f>> Pnts3fAll, const vector<int>& tags,vector<vector<TagPoint3f>> &TagPnts3fAll)
{
	TagPoint3f TagPoint;
	TagPnts3fAll.resize(Pnts3fAll.size());
	//vector<Point3f> pointtemp;
	for (unsigned int i = 0; i < Pnts3fAll.size(); i++)
	{
		for(unsigned int k = 0;k < Pnts3fAll[i].size();k++)
		{
		TagPoint[0] = float(tags[k]);
		TagPoint[1] = Pnts3fAll[i][k].x;
		TagPoint[2] = Pnts3fAll[i][k].y;
		TagPoint[3] = Pnts3fAll[i][k].z;
		TagPnts3fAll[i].push_back(TagPoint);
		}
	}
}
void addPntsTag(const vector<Point3f> Pnts3fAll, const vector<int>& tags,vector<TagPoint3f> &TagPnts3fAll)
{
	for (unsigned int i = 0; i < Pnts3fAll.size(); i++)
	{
		TagPoint3f TagPoint;
		TagPoint[0] = float(tags[i]);
		TagPoint[1] = Pnts3fAll[i].x;
		TagPoint[2] = Pnts3fAll[i].y;
		TagPoint[3] = Pnts3fAll[i].z;
		TagPnts3fAll.push_back(TagPoint);
	}
}
std::vector<std::string> split(std::string str, std::string pattern)
{
	std::string::size_type pos;
	std::vector<std::string> result;
	str += pattern;            //扩展字符串以方便操作
	unsigned int size = str.size();

	for (unsigned int i = 0; i < size; i++)
	{
		pos = str.find(pattern, i);
		if (pos < size)
		{
			std::string s = str.substr(i, pos - i);
			result.push_back(s);
			i = pos + pattern.size() - 1;
		}
	}
	return result;
}

//static bool isReadedImgAllSameSize(QMap<CamPosition, vector<QString>> imgPathMap,int& h,int& w)
//{
//	int height=0, width=0;
//	bool fisrtImg = true;
//	QMap<CamPosition, vector<QString>>::Iterator iter;
//	for (iter = imgPathMap.begin(); iter != imgPathMap.end();iter++)
//	{
//		for (int i = 0; i < (*iter).size(); i++)
//		{
//			Mat img;
//			img = imread((*iter)[i].toStdString());
//			if (fisrtImg == true)
//			{
//				height = img.rows;
//				width = img.cols;
//				fisrtImg = false;
//				img.release();
//			}
//			else
//			{
//				if (height != img.rows || width != img.cols)
//				{
//					img.release();
//					return false;
//				}
//			}
//		}
//	}
//
//	h = height;
//	w = width;
//	return true;
//}
