
#include "dynamic3dtracking.h"


Dynamic3DTracking::Dynamic3DTracking(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	//��ʼ�������־λ
	camOpenFlagL = 0;
	camOpenFlagR = 0;

	//����ͼƬ��ų�ʼ��
	imgNum = 0;
	imgNumL = 0;
	imgNumR = 0;

	//�ⴥ������ͼƬ������ʼ��
	NumRetrieveBuffer = 100;
	stringstream sNum;
	sNum << NumRetrieveBuffer;
	ui.lineEditPicNum->setText(sNum.str().c_str());
}


Dynamic3DTracking::~Dynamic3DTracking()
{
	
}

//ͼƬ������ʾ
void Dynamic3DTracking::paintEvent(QPaintEvent *event)
{
	QSize groupBoxLsize = ui.groupBoxL->size();
	QSize groupBoxRsize = ui.groupBoxR->size();
	QSize labelLsize(groupBoxLsize.width() - 20, groupBoxLsize.height() - 20);
	QSize labelRsize(groupBoxRsize.width() - 20, groupBoxRsize.height() - 20);
	ui.labelLeftCam->resize(labelLsize);
	ui.labelRightCam->resize(labelRsize);
}

//����ͼƬ����·��
void  Dynamic3DTracking::on_btnSavePath_clicked()
{
   ////���������������ͼ�����һ��Ŀ¼
	QString imgPath = QFileDialog::getExistingDirectory(this,GBK::ToUnicode("����ͼ��洢·��"),"./triger",QFileDialog::ShowDirsOnly);
	//ȷ������ͼ��·��
	imgPathL1 = imgPath + "/L";
	imgPathR1 = imgPath + "/R";
	ui.btnOpenCams->setEnabled(true);
}

//ɾ���ļ����ڵ�����
void Dynamic3DTracking::removefilesindir(const QString& path)
{
	QDir dir(path);
	QFileInfoList info_list = dir.entryInfoList(QDir::Files | QDir::Hidden | QDir::NoDotAndDotDot | QDir::NoSymLinks | QDir::AllDirs);
	foreach(QFileInfo file_info, info_list)
	{
		if (file_info.isDir())
		{
			removefilesindir(file_info.absoluteFilePath());
		}
		else if (file_info.isFile())
		{
			QFile file(file_info.absoluteFilePath());
			qDebug() << "remove file  : " << file_info.absoluteFilePath();
			file.remove();
		}
	}
	QDir temp_dir;
	//temp_dir.rmdir(path) ; //ɾ���ļ��м�������ļ� 
	temp_dir.remove(path); //ɾ���ļ���������ļ�,�������ļ��б���
	qDebug() << "remove empty dir : " << path;
}




//�����
void Dynamic3DTracking::on_btnOpenCams_clicked()
{
	
	//���ж�ѡ�е�����һ�����
	if (ui.radioBtnL->isChecked()) ///�����
	{
		if (!camL.ConnectToCamera(1))
		{
			QMessageBox::critical(this,GBK::ToUnicode("����Ϣ"),GBK::ToUnicode("���������ʧ�ܣ�����"));
			return;
		}
		timerL = new QTimer(this);
		connect(timerL, SIGNAL(timeout()), this, SLOT(timerLout()));
		timerL->start(10);
		camOpenFlagL = 1;
	}
	else
	{
		if (!camR.ConnectToCamera(0))
		{
			QMessageBox::critical(this,GBK::ToUnicode("����Ϣ"),GBK::ToUnicode("���������ʧ�ܣ�����"));
			return;
		}
		timerR = new QTimer(this);
		connect(timerR,SIGNAL(timeout()),this,SLOT(timerRout()));
		////����ÿ�δ���timeout()��ʱ����
		timerR->start(10);
		camOpenFlagR = 1;
	}
	//�����֮�󣬼���ر������ť
	ui.btnCloseCams->setEnabled(true);
	ui.btnSingleCamCapture->setEnabled(true);
	ui.btnTwoCamsCapture->setEnabled(true);

}

//���������ץ��ͼƬ
void Dynamic3DTracking::timerLout()
{
	camL.GrabAPicture(pImageL);
	//������ĺõ�ͼƬImageת��QT�е�QImage
	QImage qImageL = QImage((const uchar*)(pImageL.GetData()),pImageL.GetCols(),pImageL.GetRows(),QImage::Format_Indexed8); 
	//�ڿؼ�����ʾ
	ui.labelLeftCam->setPixmap(QPixmap::fromImage(qImageL));
	//������������ͼƬ���ݴ��ڴ�С��������Ӧ
	ui.labelLeftCam->resize(qImageL.size());
}

//���������ץ��ͼƬ
void Dynamic3DTracking::timerRout()
{
	camR.GrabAPicture(pImageR);
	//������ĺõ�ͼƬ��ʽImageת��QT�е�QImage.
	QImage qImageR = QImage((const uchar*)(pImageR.GetData()),pImageR.GetCols(),pImageR.GetRows(),QImage::Format_Indexed8);
	//�ڿؼ�����ʾ
	ui.labelRightCam->setPixmap(QPixmap::fromImage(qImageR));
	//������������ͼƬ���ݴ��ڴ�С��������Ӧ
	ui.labelRightCam->resize(qImageR.size());
}

//���������
void Dynamic3DTracking::on_btnSingleCamCapture_clicked()
{
	if (ui.radioBtnL->isChecked() && camOpenFlagL)  //�ж϶�ѡ��ť��û��ѡ������һ������������Ѿ��������ˡ�
	{
		QString imgNameL;
		imgNameL.sprintf("%s%d%s", "/imgL", imgNumL, ".bmp");
		imgNameL = imgPathL1 + imgNameL;

		QByteArray byteAL = imgNameL.toLatin1();//Converts a Latin-1 character to an 8-bit ASCII representation of the character.
		char *imgNameTemL = byteAL.data();

		pImageL.Save(imgNameTemL);
		imgNumL++;
	}
	if (ui.radioBtnR->isChecked() && camOpenFlagR)
	{
		QString imgNameR;
		imgNameR.sprintf("%s%d%s", "/imgR", imgNumR, ".bmp");
		imgNameR = imgPathR1 + imgNameR;

		QByteArray byteAR = imgNameR.toLatin1();
		char *imgNameTemR = byteAR.data();

		pImageR.Save(imgNameTemR);
		imgNumR++;
	}

}


//˫���ͬʱ����
void Dynamic3DTracking::on_btnTwoCamsCapture_clicked()
{
	if (camOpenFlagL&&camOpenFlagR)
	{
		QString imgNameL;
		imgNameL.sprintf("%s%d%s","/imgL",imgNum,".bmp");

		imgNameL = imgPathL1+ imgNameL;

		QByteArray byteAL = imgNameL.toLatin1();
		char *imgNameTemL = byteAL.data();
		pImageL.Save(imgNameTemL);

		QString imgNameR;
		imgNameR.sprintf("%s%d%s", "/imgR", imgNum, ".bmp");
	
		imgNameR = imgPathR1 + imgNameR;

		QByteArray byteAR = imgNameR.toLatin1();
		char *imgNameTemR = byteAR.data();
		pImageR.Save(imgNameTemR);
		
		++imgNum;

	}
	else
	{
		QMessageBox::critical(this, GBK::ToUnicode("������ʾ"),GBK::ToUnicode("���������ʧ�ܣ�����"));
		return;
	}
}

//�ر����
void Dynamic3DTracking::on_btnCloseCams_clicked()
{
	if (ui.radioBtnL->isChecked())
	{
		timerL->stop();
		if (!camL.DisconnectCamera())
		{
			QMessageBox::critical(this,GBK::ToUnicode("����Ϣ"),GBK::ToUnicode("������ر�ʧ�ܣ�����"),
			QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
		}
	}
	else
	{
		timerR->stop();
		if (!camR.DisconnectCamera())
		{
			QMessageBox::critical(this, GBK::ToUnicode("����Ϣ"), GBK::ToUnicode("������ر�ʧ�ܣ�����"),
				QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Ok);
		}
	}
}

//�����ع�ʱ��
void Dynamic3DTracking::on_btnSetExpTime_clicked()
{
	if (ui.radioBtnL->isChecked())
	{
		camL.SetExposureTime(ui.lineEditExpoTime->text().toFloat());
	}
	else
	{
		camR.SetExposureTime(ui.lineEditExpoTime->text().toFloat());
	}
}

//�����ⴥ�������ͼƬ����
void Dynamic3DTracking::on_btnSetPicNum_clicked()
{
	QString m_String = ui.lineEditPicNum->text();
	NumRetrieveBuffer = m_String.toInt();
}



//�ⴥ��
void Dynamic3DTracking::on_externalTrigButton_clicked()
{
	//���ⴥ����֮ͨǰ����Ҫ��triger�ļ����µ�����ͼƬ����յ�
	QString PathL, PathR;
	PathL = "./triger/L/";
	PathR = "./triger/R/";
	removefilesindir(PathL);
	removefilesindir(PathR);
	
	//��Ҫ�ж�����Ƿ����ӣ�������ӣ���Ҫ�ȹرա�
	if (camOpenFlagL)
	{
		timerL->stop();
		camL.DisconnectCamera();
	}
	if (camOpenFlagR)
	{
		timerR->stop();
		camR.DisconnectCamera();
	}

	threadL = new CamCapThread(LRThread::L);
	threadR = new CamCapThread(LRThread::R);
	threadL->start();
	threadR->start();

	threadL->run(LRThread::L,vecImgL1,NumRetrieveBuffer);
	threadR->run(LRThread::R,vecImgR1,NumRetrieveBuffer);

	//�ر��߳� ��Ҫ�Ⱥ�������������� quit() wait() delete thread
	threadL->quit();
	threadR->quit();
	threadL->wait();
	threadR->wait();
	delete threadL;
	delete threadR;

	//�������������
	if (!camL.ConnectToCamera(0))
	{
		QMessageBox::critical(this, GBK::ToUnicode("����Ϣ"), GBK::ToUnicode("���������ʧ�ܣ�����"));
		return;
	}
	timerL = new QTimer(this);
	connect(timerL, SIGNAL(timeout()), this, SLOT(timerLout()));
	timerL->start(10);

	camOpenFlagL = 1;

	if (!camR.ConnectToCamera(1))
	{
		QMessageBox::critical(this, GBK::ToUnicode("����Ϣ"), GBK::ToUnicode("���������ʧ�ܣ�����"));
		return;
	}
	timerR = new QTimer(this);
	connect(timerR, SIGNAL(timeout()), this, SLOT(timerRout()));
	timerR->start(10);
	camOpenFlagR = 1;


	ui.BtnDeformation->setEnabled(true);
	ui.btnXGraph->setEnabled(true);

}

//��̬����
void Dynamic3DTracking::on_btnDynamicTracking_clicked()
{
	DynamicTrackDlg  dynamicTrackModel(this);
	dynamicTrackModel.exec();
}

//�˳�����
void Dynamic3DTracking::on_btnExit_clicked()
{
	exit(0);
}


