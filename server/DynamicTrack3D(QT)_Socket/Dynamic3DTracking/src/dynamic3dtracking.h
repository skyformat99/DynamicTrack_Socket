#ifndef DYNAMIC3DTRACKING_H
#define DYNAMIC3DTRACKING_H
#include <QtWidgets/QMainWindow>
#include "ui_dynamic3dtracking.h"

#include<sstream>
//qt header
#include <QFileDialog>
#include <QSize>
#include<QGroupBox>
#include<QPixmap>

//Point Grey header
#include "pcamera.h"

//核心代码库
#include "ClpMeasurement.h"
#include "CamCapThread.h"  //封装好的多线程库
#include "GBK.h"

//工程中头文件
#include "dynamictrackdlg.h"

class Dynamic3DTracking : public QMainWindow
{
	Q_OBJECT

public:
	Dynamic3DTracking(QWidget *parent = 0);
	~Dynamic3DTracking();

private:
	Ui::Dynamic3DTrackingClass ui;

public:
	//////////////test/////////////
	CamCapThread* threadL;
	CamCapThread* threadR;
	
	//定义两个容器存放左右拍摄好的图片
	vector<Image> vecImgL1, vecImgR1;

	//左右图像路径
	QString imgPathL1, imgPathR1;
	//定义左右相机
	PCamera camL, camR;
	//定义PGR图像变量
	Image pImageL, pImageR;

	//存储当前显示的图像Mat
	Mat ImgMat_L, ImgMat_R;     //三通道
	Mat grayImg_L, grayImg_R;   //单通道图像

	//计时器
	QTimer *timerL, *timerR;

	bool camOpenFlagL, camOpenFlagR;

	//保存图像的序号
	int imgNum,imgNumL,imgNumR;


	//保存外触发下相机拍摄的图片总数目
	int NumRetrieveBuffer;

private slots:
    //设置保存图片路径
	void on_btnSavePath_clicked();
	//打开相机
	void on_btnOpenCams_clicked();
	//左相机拍摄图片
	void timerLout();
	//右相机拍摄图片
	void timerRout();
 
	//退出
	void on_btnExit_clicked();
	//单相机拍摄
	void on_btnSingleCamCapture_clicked();
	//双相机同时拍摄
	void on_btnTwoCamsCapture_clicked();
	////关闭相机
	void on_btnCloseCams_clicked();
	//设置曝光时间
	void on_btnSetExpTime_clicked();

	//设置外触发拍摄的图片张数
	void on_btnSetPicNum_clicked();

	//外部触发相机拍摄
	void on_externalTrigButton_clicked();

	//动态跟踪
	void on_btnDynamicTracking_clicked();

public:
	//绘制左右相机图片显示窗口
	void paintEvent(QPaintEvent *event);
	//删除文件夹内的内容
	void removefilesindir(const QString& path);

};

#endif // DYNAMIC3DTRACKING_H
