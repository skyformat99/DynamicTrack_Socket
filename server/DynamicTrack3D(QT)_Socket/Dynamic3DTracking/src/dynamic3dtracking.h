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

//���Ĵ����
#include "ClpMeasurement.h"
#include "CamCapThread.h"  //��װ�õĶ��߳̿�
#include "GBK.h"

//������ͷ�ļ�
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
	
	//�����������������������õ�ͼƬ
	vector<Image> vecImgL1, vecImgR1;

	//����ͼ��·��
	QString imgPathL1, imgPathR1;
	//�����������
	PCamera camL, camR;
	//����PGRͼ�����
	Image pImageL, pImageR;

	//�洢��ǰ��ʾ��ͼ��Mat
	Mat ImgMat_L, ImgMat_R;     //��ͨ��
	Mat grayImg_L, grayImg_R;   //��ͨ��ͼ��

	//��ʱ��
	QTimer *timerL, *timerR;

	bool camOpenFlagL, camOpenFlagR;

	//����ͼ������
	int imgNum,imgNumL,imgNumR;


	//�����ⴥ������������ͼƬ����Ŀ
	int NumRetrieveBuffer;

private slots:
    //���ñ���ͼƬ·��
	void on_btnSavePath_clicked();
	//�����
	void on_btnOpenCams_clicked();
	//���������ͼƬ
	void timerLout();
	//���������ͼƬ
	void timerRout();
 
	//�˳�
	void on_btnExit_clicked();
	//���������
	void on_btnSingleCamCapture_clicked();
	//˫���ͬʱ����
	void on_btnTwoCamsCapture_clicked();
	////�ر����
	void on_btnCloseCams_clicked();
	//�����ع�ʱ��
	void on_btnSetExpTime_clicked();

	//�����ⴥ�������ͼƬ����
	void on_btnSetPicNum_clicked();

	//�ⲿ�����������
	void on_externalTrigButton_clicked();

	//��̬����
	void on_btnDynamicTracking_clicked();

public:
	//�����������ͼƬ��ʾ����
	void paintEvent(QPaintEvent *event);
	//ɾ���ļ����ڵ�����
	void removefilesindir(const QString& path);

};

#endif // DYNAMIC3DTRACKING_H
