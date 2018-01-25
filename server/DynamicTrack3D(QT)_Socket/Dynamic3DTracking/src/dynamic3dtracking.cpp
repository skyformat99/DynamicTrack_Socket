
#include "dynamic3dtracking.h"


Dynamic3DTracking::Dynamic3DTracking(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	//初始化软件标志位
	camOpenFlagL = 0;
	camOpenFlagR = 0;

	//拍摄图片序号初始化
	imgNum = 0;
	imgNumL = 0;
	imgNumR = 0;

	//外触发拍摄图片张数初始化
	NumRetrieveBuffer = 100;
	stringstream sNum;
	sNum << NumRetrieveBuffer;
	ui.lineEditPicNum->setText(sNum.str().c_str());
}


Dynamic3DTracking::~Dynamic3DTracking()
{
	
}

//图片不断显示
void Dynamic3DTracking::paintEvent(QPaintEvent *event)
{
	QSize groupBoxLsize = ui.groupBoxL->size();
	QSize groupBoxRsize = ui.groupBoxR->size();
	QSize labelLsize(groupBoxLsize.width() - 20, groupBoxLsize.height() - 20);
	QSize labelRsize(groupBoxRsize.width() - 20, groupBoxRsize.height() - 20);
	ui.labelLeftCam->resize(labelLsize);
	ui.labelRightCam->resize(labelRsize);
}

//设置图片保存路径
void  Dynamic3DTracking::on_btnSavePath_clicked()
{
   ////设置左右相机拍摄图像的上一级目录
	QString imgPath = QFileDialog::getExistingDirectory(this,GBK::ToUnicode("设置图像存储路径"),"./triger",QFileDialog::ShowDirsOnly);
	//确定左右图像路径
	imgPathL1 = imgPath + "/L";
	imgPathR1 = imgPath + "/R";
	ui.btnOpenCams->setEnabled(true);
}

//删除文件夹内的内容
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
	//temp_dir.rmdir(path) ; //删除文件夹及里面的文件 
	temp_dir.remove(path); //删除文件夹里面的文件,不包括文件夹本身
	qDebug() << "remove empty dir : " << path;
}




//打开相机
void Dynamic3DTracking::on_btnOpenCams_clicked()
{
	
	//先判断选中的是哪一个相机
	if (ui.radioBtnL->isChecked()) ///左相机
	{
		if (!camL.ConnectToCamera(1))
		{
			QMessageBox::critical(this,GBK::ToUnicode("坏消息"),GBK::ToUnicode("左相机连接失败，请检查"));
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
			QMessageBox::critical(this,GBK::ToUnicode("坏消息"),GBK::ToUnicode("右相机连接失败，请检查"));
			return;
		}
		timerR = new QTimer(this);
		connect(timerR,SIGNAL(timeout()),this,SLOT(timerRout()));
		////设置每次触发timeout()的时间间隔
		timerR->start(10);
		camOpenFlagR = 1;
	}
	//打开相机之后，激活关闭相机按钮
	ui.btnCloseCams->setEnabled(true);
	ui.btnSingleCamCapture->setEnabled(true);
	ui.btnTwoCamsCapture->setEnabled(true);

}

//左相机主动抓拍图片
void Dynamic3DTracking::timerLout()
{
	camL.GrabAPicture(pImageL);
	//将相机拍好的图片Image转成QT中的QImage
	QImage qImageL = QImage((const uchar*)(pImageL.GetData()),pImageL.GetCols(),pImageL.GetRows(),QImage::Format_Indexed8); 
	//在控件上显示
	ui.labelLeftCam->setPixmap(QPixmap::fromImage(qImageL));
	//将左相机拍摄的图片根据窗口大小进行自适应
	ui.labelLeftCam->resize(qImageL.size());
}

//右相机主动抓拍图片
void Dynamic3DTracking::timerRout()
{
	camR.GrabAPicture(pImageR);
	//将相机拍好的图片格式Image转成QT中的QImage.
	QImage qImageR = QImage((const uchar*)(pImageR.GetData()),pImageR.GetCols(),pImageR.GetRows(),QImage::Format_Indexed8);
	//在控件上显示
	ui.labelRightCam->setPixmap(QPixmap::fromImage(qImageR));
	//将右相机拍摄的图片根据窗口大小进行自适应
	ui.labelRightCam->resize(qImageR.size());
}

//单相机拍摄
void Dynamic3DTracking::on_btnSingleCamCapture_clicked()
{
	if (ui.radioBtnL->isChecked() && camOpenFlagL)  //判断多选按钮有没有选择其中一个，并且相机已经连接上了。
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


//双相机同时拍摄
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
		QMessageBox::critical(this, GBK::ToUnicode("友情提示"),GBK::ToUnicode("左右相机打开失败，请检查"));
		return;
	}
}

//关闭相机
void Dynamic3DTracking::on_btnCloseCams_clicked()
{
	if (ui.radioBtnL->isChecked())
	{
		timerL->stop();
		if (!camL.DisconnectCamera())
		{
			QMessageBox::critical(this,GBK::ToUnicode("坏消息"),GBK::ToUnicode("左相机关闭失败，请检查"),
			QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
		}
	}
	else
	{
		timerR->stop();
		if (!camR.DisconnectCamera())
		{
			QMessageBox::critical(this, GBK::ToUnicode("坏消息"), GBK::ToUnicode("右相机关闭失败，请检查"),
				QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Ok);
		}
	}
}

//设置曝光时间
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

//设置外触发拍摄的图片张数
void Dynamic3DTracking::on_btnSetPicNum_clicked()
{
	QString m_String = ui.lineEditPicNum->text();
	NumRetrieveBuffer = m_String.toInt();
}



//外触发
void Dynamic3DTracking::on_externalTrigButton_clicked()
{
	//在外触发接通之前，需要将triger文件夹下的所有图片都清空掉
	QString PathL, PathR;
	PathL = "./triger/L/";
	PathR = "./triger/R/";
	removefilesindir(PathL);
	removefilesindir(PathR);
	
	//先要判断相机是否连接，如果连接，则要先关闭。
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

	//关闭线程 需要先后调用这三个函数 quit() wait() delete thread
	threadL->quit();
	threadR->quit();
	threadL->wait();
	threadR->wait();
	delete threadL;
	delete threadR;

	//拍摄后打开两个相机
	if (!camL.ConnectToCamera(0))
	{
		QMessageBox::critical(this, GBK::ToUnicode("坏消息"), GBK::ToUnicode("左相机连接失败，请检查"));
		return;
	}
	timerL = new QTimer(this);
	connect(timerL, SIGNAL(timeout()), this, SLOT(timerLout()));
	timerL->start(10);

	camOpenFlagL = 1;

	if (!camR.ConnectToCamera(1))
	{
		QMessageBox::critical(this, GBK::ToUnicode("坏消息"), GBK::ToUnicode("右相机连接失败，请检查"));
		return;
	}
	timerR = new QTimer(this);
	connect(timerR, SIGNAL(timeout()), this, SLOT(timerRout()));
	timerR->start(10);
	camOpenFlagR = 1;


	ui.BtnDeformation->setEnabled(true);
	ui.btnXGraph->setEnabled(true);

}

//动态跟踪
void Dynamic3DTracking::on_btnDynamicTracking_clicked()
{
	DynamicTrackDlg  dynamicTrackModel(this);
	dynamicTrackModel.exec();
}

//退出界面
void Dynamic3DTracking::on_btnExit_clicked()
{
	exit(0);
}


