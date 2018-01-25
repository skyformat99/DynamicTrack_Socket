#pragma once
#include "dynamictrackdlg.h"


DynamicTrackDlg::DynamicTrackDlg(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	//此处需要删除存放编码点数据的data.txt

	//子对话框调用主窗口函数
	Dynamic3DTracking *p = (Dynamic3DTracking *)parentWidget();

	QString path = "./DataFiles/data/";
	p->removefilesindir(path);


	//初始化图片的标志位
	FirstImgOrNot = true;
	//初始化第一张图片的序号
	imgNumBeDealed = 0;
	//检查容器中的图片是否为空
	if (!vecImgL2.empty() || !vecImgR2.empty())
	{
		QMessageBox::critical(this, GBK::ToUnicode("友情提示"), GBK::ToUnicode("申请的容器不为空 请检查"));
	}
	std::string camparapathleft = "./DataFiles/CalibrationData/CalibrationData_left.xml";
	std::string camparapathright = "./DataFiles/CalibrationData/CalibrationData_right.xml";
	std::string lpparaPath = "./DataFiles/CalibrationData/lightPenCalibrationData.xml";
	std::string lpfeaturesparaPath = "./DataFiles/CalibrationData/lightPenFeaturesData.xml";
	std::string camGroupparaPath = "./DataFiles/CalibrationData/GroupPara.xml";
	getMessurePara(camparapathleft, camparapathright, camGroupparaPath, lpparaPath, lpfeaturesparaPath);
}

DynamicTrackDlg::~DynamicTrackDlg()
{

}


void DynamicTrackDlg::timerEvent(QTimerEvent *event)
{
	if (imgNumBeDealed == imgNameListL.size() - 1)
	{
		//before killing the timer,send data.txt to computer
		//编码点数据存放地址
		char path[100];
		sprintf(path, "%s", "E:\\ProjectFiles\\DynamicTrack3D(QT)_Socket\\Dynamic3DTracking\\DataFiles\\data\\data.txt");
		SocketClient nuc;
		nuc.CSocket_Init();
		nuc.CSocket_Send_Connection();
		nuc.CSocket_SendFile(path);
		killTimer(m_timerId);   //关闭定时器
	}
	if (event->timerId() == m_timerId)
	{
		//获取工作线程的当前状态和进度，并显示
		//使用粒子滤波跟踪编码点
		//获取当前处理的图像帧数，并将处理得到的编码点坐标结果显示
		qimageL = new QImage(imgNameListL[imgNumBeDealed]);
		qimageR = new QImage(imgNameListR[imgNumBeDealed]);
		//将pixmap转换成Mat类型
		Mat imgLeft = QImage2cvMat(*qimageL);
		Mat imgRight = QImage2cvMat(*qimageR);

		QString imgNumTotal = QString("%1").arg(imgNameListL.size());
		ui.lineEditTotalNum->setText(imgNumTotal);

		//lineEdit只能输出string类型的数据，因而此处是为了将图片序号（int）转化为string类型
		stringstream str;
		str << imgNumBeDealed+1;
		ui.lineEditImgNum->setText(str.str().c_str());

		if (!getCodes3DCoordinateDM(imgLeft, imgRight, FirstImgOrNot))
		{
			//如果当前帧没有跟踪成功，则将数据结果置为零
			ui.lineEdit_1->setText(NULL);
			ui.lineEdit_2->setText(NULL);
			ui.lineEdit_3->setText(NULL);
			ui.lineEdit_4->setText(NULL);
			ui.lineEdit_5->setText(NULL);
			ui.lineEdit_6->setText(NULL);
			ui.lineEdit_7->setText(NULL);
			ui.lineEdit_8->setText(NULL);
			ui.lineEdit_9->setText(NULL);
			ui.lineEdit_10->setText(NULL);
			ui.lineEdit_11->setText(NULL);
			ui.lineEdit_12->setText(NULL);
			ui.lineEdit_13->setText(NULL);
			ui.lineEdit_14->setText(NULL);
			
		}
		else
		{
		
			//如果跟踪14个编码点时
			if (CodeTagPnts3D.size() == 14)
			{
				ui.lineEdit_1->setText(TagPoint3f2QString(CodeTagPnts3D[0]));
				ui.lineEdit_2->setText(TagPoint3f2QString(CodeTagPnts3D[1]));
				ui.lineEdit_3->setText(TagPoint3f2QString(CodeTagPnts3D[2]));
				ui.lineEdit_4->setText(TagPoint3f2QString(CodeTagPnts3D[3]));
				ui.lineEdit_5->setText(TagPoint3f2QString(CodeTagPnts3D[4]));
				ui.lineEdit_6->setText(TagPoint3f2QString(CodeTagPnts3D[5]));
				ui.lineEdit_7->setText(TagPoint3f2QString(CodeTagPnts3D[6]));
				ui.lineEdit_8->setText(TagPoint3f2QString(CodeTagPnts3D[7]));
				ui.lineEdit_9->setText(TagPoint3f2QString(CodeTagPnts3D[8]));
				ui.lineEdit_10->setText(TagPoint3f2QString(CodeTagPnts3D[9]));
				ui.lineEdit_11->setText(TagPoint3f2QString(CodeTagPnts3D[10]));
				ui.lineEdit_12->setText(TagPoint3f2QString(CodeTagPnts3D[11]));
				ui.lineEdit_13->setText(TagPoint3f2QString(CodeTagPnts3D[12]));
				ui.lineEdit_14->setText(TagPoint3f2QString(CodeTagPnts3D[13]));

				//将结果输出到
				char *fname = "./DataFiles/data/data.txt"; //path -save data
				ofstream fout(fname, ios::app);
				fout << "1," << CodeTagPnts3D[0][1]  << "," << CodeTagPnts3D[0][2]  << "," << CodeTagPnts3D[0][3]  << "," << endl
					<< "2," << CodeTagPnts3D[1][1]  << "," << CodeTagPnts3D[1][2]  << "," << CodeTagPnts3D[1][3]  << "," << endl
					<< "3," << CodeTagPnts3D[2][1]  << "," << CodeTagPnts3D[2][2]  << "," << CodeTagPnts3D[2][3]  << "," << endl
					<< "4," << CodeTagPnts3D[3][1]  << "," << CodeTagPnts3D[3][2]  << "," << CodeTagPnts3D[3][3]  << "," << endl
					<< "5," << CodeTagPnts3D[4][1]  << "," << CodeTagPnts3D[4][2]  << "," << CodeTagPnts3D[4][3]  << "," << endl
					<< "6," << CodeTagPnts3D[5][1]  << "," << CodeTagPnts3D[5][2]  << "," << CodeTagPnts3D[5][3]  << "," << endl
					<< "7," << CodeTagPnts3D[6][1]  << "," << CodeTagPnts3D[6][2]  << "," << CodeTagPnts3D[6][3]  << "," << endl
					<< "8," << CodeTagPnts3D[7][1]  << "," << CodeTagPnts3D[7][2]  << "," << CodeTagPnts3D[7][3]  << "," << endl
					<< "9," << CodeTagPnts3D[8][1]  << "," << CodeTagPnts3D[8][2]  << "," << CodeTagPnts3D[8][3]  << "," << endl
					<< "a," << CodeTagPnts3D[9][1]  << "," << CodeTagPnts3D[9][2]  << "," << CodeTagPnts3D[9][3]  << "," << endl
					<< "b," << CodeTagPnts3D[10][1]  << "," << CodeTagPnts3D[10][2]  << "," << CodeTagPnts3D[10][3]  << "," << endl
					<< "c," << CodeTagPnts3D[11][1]  << "," << CodeTagPnts3D[11][2]  << "," << CodeTagPnts3D[11][3]  << "," << endl
					<< "d," << CodeTagPnts3D[12][1]  << "," << CodeTagPnts3D[12][2]  << "," << CodeTagPnts3D[12][3]  << "," << endl
					<< "e," << CodeTagPnts3D[13][1]  << "," << CodeTagPnts3D[13][2]  << "," << CodeTagPnts3D[13][3]  << "," << endl;
				fout.close();
			}
			//如果跟踪13个编码点时
			if (CodeTagPnts3D.size() == 13)
			{
				ui.lineEdit_1->setText(TagPoint3f2QString(CodeTagPnts3D[0]));
				ui.lineEdit_2->setText(TagPoint3f2QString(CodeTagPnts3D[1]));
				ui.lineEdit_3->setText(TagPoint3f2QString(CodeTagPnts3D[2]));
				ui.lineEdit_4->setText(TagPoint3f2QString(CodeTagPnts3D[3]));
				ui.lineEdit_5->setText(TagPoint3f2QString(CodeTagPnts3D[4]));
				ui.lineEdit_6->setText(TagPoint3f2QString(CodeTagPnts3D[5]));
				ui.lineEdit_7->setText(TagPoint3f2QString(CodeTagPnts3D[6]));
				ui.lineEdit_8->setText(TagPoint3f2QString(CodeTagPnts3D[7]));
				ui.lineEdit_9->setText(TagPoint3f2QString(CodeTagPnts3D[8]));
				ui.lineEdit_10->setText(TagPoint3f2QString(CodeTagPnts3D[9]));
				ui.lineEdit_11->setText(TagPoint3f2QString(CodeTagPnts3D[10]));
				ui.lineEdit_12->setText(TagPoint3f2QString(CodeTagPnts3D[11]));
				ui.lineEdit_13->setText(TagPoint3f2QString(CodeTagPnts3D[12]));

				char *fname = "./DataFiles/data/data.txt"; //path -save data
				ofstream fout(fname, ios::app);
				fout << "1," << CodeTagPnts3D[0][1]  << "," << CodeTagPnts3D[0][2]  << "," << CodeTagPnts3D[0][3]  << "," << endl
					<< "2," << CodeTagPnts3D[1][1]  << "," << CodeTagPnts3D[1][2]  << "," << CodeTagPnts3D[1][3]  << "," << endl
					<< "3," << CodeTagPnts3D[2][1]  << "," << CodeTagPnts3D[2][2]  << "," << CodeTagPnts3D[2][3]  << "," << endl
					<< "4," << CodeTagPnts3D[3][1]  << "," << CodeTagPnts3D[3][2]  << "," << CodeTagPnts3D[3][3]  << "," << endl
					<< "5," << CodeTagPnts3D[4][1]  << "," << CodeTagPnts3D[4][2]  << "," << CodeTagPnts3D[4][3]  << "," << endl
					<< "6," << CodeTagPnts3D[5][1]  << "," << CodeTagPnts3D[5][2]  << "," << CodeTagPnts3D[5][3]  << "," << endl
					<< "7," << CodeTagPnts3D[6][1]  << "," << CodeTagPnts3D[6][2]  << "," << CodeTagPnts3D[6][3]  << "," << endl
					<< "8," << CodeTagPnts3D[7][1]  << "," << CodeTagPnts3D[7][2]  << "," << CodeTagPnts3D[7][3]  << "," << endl
					<< "9," << CodeTagPnts3D[8][1]  << "," << CodeTagPnts3D[8][2]  << "," << CodeTagPnts3D[8][3]  << "," << endl
					<< "a," << CodeTagPnts3D[9][1]  << "," << CodeTagPnts3D[9][2]  << "," << CodeTagPnts3D[9][3]  << "," << endl
					<< "b," << CodeTagPnts3D[10][1]  << "," << CodeTagPnts3D[10][2]  << "," << CodeTagPnts3D[10][3]  << "," << endl
					<< "c," << CodeTagPnts3D[11][1]  << "," << CodeTagPnts3D[11][2]  << "," << CodeTagPnts3D[11][3]  << "," << endl
					<< "d," << CodeTagPnts3D[12][1]  << "," << CodeTagPnts3D[12][2]  << "," << CodeTagPnts3D[12][3]  << "," << endl;
				fout.close();
			}

			//如果跟踪12个编码点时
			if (CodeTagPnts3D.size() == 12)
			{
				ui.lineEdit_1->setText(TagPoint3f2QString(CodeTagPnts3D[0]));
				ui.lineEdit_2->setText(TagPoint3f2QString(CodeTagPnts3D[1]));
				ui.lineEdit_3->setText(TagPoint3f2QString(CodeTagPnts3D[2]));
				ui.lineEdit_4->setText(TagPoint3f2QString(CodeTagPnts3D[3]));
				ui.lineEdit_5->setText(TagPoint3f2QString(CodeTagPnts3D[4]));
				ui.lineEdit_6->setText(TagPoint3f2QString(CodeTagPnts3D[5]));
				ui.lineEdit_7->setText(TagPoint3f2QString(CodeTagPnts3D[6]));
				ui.lineEdit_8->setText(TagPoint3f2QString(CodeTagPnts3D[7]));
				ui.lineEdit_9->setText(TagPoint3f2QString(CodeTagPnts3D[8]));
				ui.lineEdit_10->setText(TagPoint3f2QString(CodeTagPnts3D[9]));
				ui.lineEdit_11->setText(TagPoint3f2QString(CodeTagPnts3D[10]));
				ui.lineEdit_12->setText(TagPoint3f2QString(CodeTagPnts3D[11]));

				char *fname = "./DataFiles/data/data.txt"; //path -save data
				ofstream fout(fname, ios::app);
				fout << "1," << CodeTagPnts3D[0][1]  << "," << CodeTagPnts3D[0][2]  << "," << CodeTagPnts3D[0][3]  << "," << endl
					<< "2," << CodeTagPnts3D[1][1]  << "," << CodeTagPnts3D[1][2]  << "," << CodeTagPnts3D[1][3]  << "," << endl
					<< "3," << CodeTagPnts3D[2][1]  << "," << CodeTagPnts3D[2][2]  << "," << CodeTagPnts3D[2][3]  << "," << endl
					<< "4," << CodeTagPnts3D[3][1]  << "," << CodeTagPnts3D[3][2]  << "," << CodeTagPnts3D[3][3]  << "," << endl
					<< "5," << CodeTagPnts3D[4][1]  << "," << CodeTagPnts3D[4][2]  << "," << CodeTagPnts3D[4][3]  << "," << endl
					<< "6," << CodeTagPnts3D[5][1]  << "," << CodeTagPnts3D[5][2]  << "," << CodeTagPnts3D[5][3]  << "," << endl
					<< "7," << CodeTagPnts3D[6][1]  << "," << CodeTagPnts3D[6][2]  << "," << CodeTagPnts3D[6][3]  << "," << endl
					<< "8," << CodeTagPnts3D[7][1]  << "," << CodeTagPnts3D[7][2]  << "," << CodeTagPnts3D[7][3]  << "," << endl
					<< "9," << CodeTagPnts3D[8][1]  << "," << CodeTagPnts3D[8][2]  << "," << CodeTagPnts3D[8][3]  << "," << endl
					<< "a," << CodeTagPnts3D[9][1]  << "," << CodeTagPnts3D[9][2]  << "," << CodeTagPnts3D[9][3]  << "," << endl
					<< "b," << CodeTagPnts3D[10][1]  << "," << CodeTagPnts3D[10][2]  << "," << CodeTagPnts3D[10][3]  << "," << endl
					<< "c," << CodeTagPnts3D[11][1]  << "," << CodeTagPnts3D[11][2]  << "," << CodeTagPnts3D[11][3]  << "," << endl;
				fout.close();

			}
			//如果跟踪11个编码点时
			if (CodeTagPnts3D.size() == 11)
			{
				ui.lineEdit_1->setText(TagPoint3f2QString(CodeTagPnts3D[0]));
				ui.lineEdit_2->setText(TagPoint3f2QString(CodeTagPnts3D[1]));
				ui.lineEdit_3->setText(TagPoint3f2QString(CodeTagPnts3D[2]));
				ui.lineEdit_4->setText(TagPoint3f2QString(CodeTagPnts3D[3]));
				ui.lineEdit_5->setText(TagPoint3f2QString(CodeTagPnts3D[4]));
				ui.lineEdit_6->setText(TagPoint3f2QString(CodeTagPnts3D[5]));
				ui.lineEdit_7->setText(TagPoint3f2QString(CodeTagPnts3D[6]));
				ui.lineEdit_8->setText(TagPoint3f2QString(CodeTagPnts3D[7]));
				ui.lineEdit_9->setText(TagPoint3f2QString(CodeTagPnts3D[8]));
				ui.lineEdit_10->setText(TagPoint3f2QString(CodeTagPnts3D[9]));
				ui.lineEdit_11->setText(TagPoint3f2QString(CodeTagPnts3D[10]));

				char *fname = "./DataFiles/data/data.txt"; //path -save data
				ofstream fout(fname, ios::app);
				fout << "1," << CodeTagPnts3D[0][1]  << "," << CodeTagPnts3D[0][2]  << "," << CodeTagPnts3D[0][3]  << "," << endl
					<< "2," << CodeTagPnts3D[1][1]  << "," << CodeTagPnts3D[1][2]  << "," << CodeTagPnts3D[1][3]  << "," << endl
					<< "3," << CodeTagPnts3D[2][1]  << "," << CodeTagPnts3D[2][2]  << "," << CodeTagPnts3D[2][3]  << "," << endl
					<< "4," << CodeTagPnts3D[3][1]  << "," << CodeTagPnts3D[3][2]  << "," << CodeTagPnts3D[3][3]  << "," << endl
					<< "5," << CodeTagPnts3D[4][1]  << "," << CodeTagPnts3D[4][2]  << "," << CodeTagPnts3D[4][3]  << "," << endl
					<< "6," << CodeTagPnts3D[5][1]  << "," << CodeTagPnts3D[5][2]  << "," << CodeTagPnts3D[5][3]  << "," << endl
					<< "7," << CodeTagPnts3D[6][1]  << "," << CodeTagPnts3D[6][2]  << "," << CodeTagPnts3D[6][3]  << "," << endl
					<< "8," << CodeTagPnts3D[7][1]  << "," << CodeTagPnts3D[7][2]  << "," << CodeTagPnts3D[7][3]  << "," << endl
					<< "9," << CodeTagPnts3D[8][1]  << "," << CodeTagPnts3D[8][2]  << "," << CodeTagPnts3D[8][3]  << "," << endl
					<< "a," << CodeTagPnts3D[9][1]  << "," << CodeTagPnts3D[9][2]  << "," << CodeTagPnts3D[9][3]  << "," << endl
					<< "b," << CodeTagPnts3D[10][1]  << "," << CodeTagPnts3D[10][2]  << "," << CodeTagPnts3D[10][3]  << "," << endl;
				fout.close();

			}

			//如果跟踪10个编码点时
			if (CodeTagPnts3D.size() == 10)
			{
				ui.lineEdit_1->setText(TagPoint3f2QString(CodeTagPnts3D[0]));
				ui.lineEdit_2->setText(TagPoint3f2QString(CodeTagPnts3D[1]));
				ui.lineEdit_3->setText(TagPoint3f2QString(CodeTagPnts3D[2]));
				ui.lineEdit_4->setText(TagPoint3f2QString(CodeTagPnts3D[3]));
				ui.lineEdit_5->setText(TagPoint3f2QString(CodeTagPnts3D[4]));
				ui.lineEdit_6->setText(TagPoint3f2QString(CodeTagPnts3D[5]));
				ui.lineEdit_7->setText(TagPoint3f2QString(CodeTagPnts3D[6]));
				ui.lineEdit_8->setText(TagPoint3f2QString(CodeTagPnts3D[7]));
				ui.lineEdit_9->setText(TagPoint3f2QString(CodeTagPnts3D[8]));
				ui.lineEdit_10->setText(TagPoint3f2QString(CodeTagPnts3D[9]));

				char *fname = "./DataFiles/data/data.txt"; //path -save data
				ofstream fout(fname, ios::app);
				fout << "1," << CodeTagPnts3D[0][1]  << "," << CodeTagPnts3D[0][2]  << "," << CodeTagPnts3D[0][3]  << "," << endl
					<< "2," << CodeTagPnts3D[1][1]  << "," << CodeTagPnts3D[1][2]  << "," << CodeTagPnts3D[1][3]  << "," << endl
					<< "3," << CodeTagPnts3D[2][1]  << "," << CodeTagPnts3D[2][2]  << "," << CodeTagPnts3D[2][3]  << "," << endl
					<< "4," << CodeTagPnts3D[3][1]  << "," << CodeTagPnts3D[3][2]  << "," << CodeTagPnts3D[3][3]  << "," << endl
					<< "5," << CodeTagPnts3D[4][1]  << "," << CodeTagPnts3D[4][2]  << "," << CodeTagPnts3D[4][3]  << "," << endl
					<< "6," << CodeTagPnts3D[5][1]  << "," << CodeTagPnts3D[5][2]  << "," << CodeTagPnts3D[5][3]  << "," << endl
					<< "7," << CodeTagPnts3D[6][1]  << "," << CodeTagPnts3D[6][2]  << "," << CodeTagPnts3D[6][3]  << "," << endl
					<< "8," << CodeTagPnts3D[7][1]  << "," << CodeTagPnts3D[7][2]  << "," << CodeTagPnts3D[7][3]  << "," << endl
					<< "9," << CodeTagPnts3D[8][1]  << "," << CodeTagPnts3D[8][2]  << "," << CodeTagPnts3D[8][3]  << "," << endl
					<< "a," << CodeTagPnts3D[9][1]  << "," << CodeTagPnts3D[9][2]  << "," << CodeTagPnts3D[9][3]  << "," << endl;
				fout.close();


			}

			//如果跟踪9个编码点时
			if (CodeTagPnts3D.size() == 9)
			{
				//将15个编码点坐标依次输出到控件lineEdit中
				ui.lineEdit_1->setText(TagPoint3f2QString(CodeTagPnts3D[0]));
				ui.lineEdit_2->setText(TagPoint3f2QString(CodeTagPnts3D[1]));
				ui.lineEdit_3->setText(TagPoint3f2QString(CodeTagPnts3D[2]));
				ui.lineEdit_4->setText(TagPoint3f2QString(CodeTagPnts3D[3]));
				ui.lineEdit_5->setText(TagPoint3f2QString(CodeTagPnts3D[4]));
				ui.lineEdit_6->setText(TagPoint3f2QString(CodeTagPnts3D[5]));
				ui.lineEdit_7->setText(TagPoint3f2QString(CodeTagPnts3D[6]));
				ui.lineEdit_8->setText(TagPoint3f2QString(CodeTagPnts3D[7]));
				ui.lineEdit_9->setText(TagPoint3f2QString(CodeTagPnts3D[8]));

				char *fname = "./DataFiles/data/data.txt"; //path -save data
				ofstream fout(fname, ios::app);
				fout << "1," << CodeTagPnts3D[0][1]  << "," << CodeTagPnts3D[0][2]  << "," << CodeTagPnts3D[0][3]  << "," << endl
					<< "2," << CodeTagPnts3D[1][1]  << "," << CodeTagPnts3D[1][2]  << "," << CodeTagPnts3D[1][3]  << "," << endl
					<< "3," << CodeTagPnts3D[2][1]  << "," << CodeTagPnts3D[2][2]  << "," << CodeTagPnts3D[2][3]  << "," << endl
					<< "4," << CodeTagPnts3D[3][1]  << "," << CodeTagPnts3D[3][2]  << "," << CodeTagPnts3D[3][3]  << "," << endl
					<< "5," << CodeTagPnts3D[4][1]  << "," << CodeTagPnts3D[4][2]  << "," << CodeTagPnts3D[4][3]  << "," << endl
					<< "6," << CodeTagPnts3D[5][1]  << "," << CodeTagPnts3D[5][2]  << "," << CodeTagPnts3D[5][3]  << "," << endl
					<< "7," << CodeTagPnts3D[6][1]  << "," << CodeTagPnts3D[6][2]  << "," << CodeTagPnts3D[6][3]  << "," << endl
					<< "8," << CodeTagPnts3D[7][1]  << "," << CodeTagPnts3D[7][2]  << "," << CodeTagPnts3D[7][3]  << "," << endl
					<< "9," << CodeTagPnts3D[8][1]  << "," << CodeTagPnts3D[8][2]  << "," << CodeTagPnts3D[8][3]  << "," << endl;
				fout.close();


			}

			//如果跟踪8个编码点时
			if (CodeTagPnts3D.size() == 8)
			{
				ui.lineEdit_1->setText(TagPoint3f2QString(CodeTagPnts3D[0]));
				ui.lineEdit_2->setText(TagPoint3f2QString(CodeTagPnts3D[1]));
				ui.lineEdit_3->setText(TagPoint3f2QString(CodeTagPnts3D[2]));
				ui.lineEdit_4->setText(TagPoint3f2QString(CodeTagPnts3D[3]));
				ui.lineEdit_5->setText(TagPoint3f2QString(CodeTagPnts3D[4]));
				ui.lineEdit_6->setText(TagPoint3f2QString(CodeTagPnts3D[5]));
				ui.lineEdit_7->setText(TagPoint3f2QString(CodeTagPnts3D[6]));
				ui.lineEdit_8->setText(TagPoint3f2QString(CodeTagPnts3D[7]));

				char *fname = "./DataFiles/data/data.txt"; //path -save data
				ofstream fout(fname, ios::app);
				fout << "1," << CodeTagPnts3D[0][1]  << "," << CodeTagPnts3D[0][2]  << "," << CodeTagPnts3D[0][3]  << "," << endl
					<< "2," << CodeTagPnts3D[1][1]  << "," << CodeTagPnts3D[1][2]  << "," << CodeTagPnts3D[1][3]  << "," << endl
					<< "3," << CodeTagPnts3D[2][1]  << "," << CodeTagPnts3D[2][2]  << "," << CodeTagPnts3D[2][3]  << "," << endl
					<< "4," << CodeTagPnts3D[3][1]  << "," << CodeTagPnts3D[3][2]  << "," << CodeTagPnts3D[3][3]  << "," << endl
					<< "5," << CodeTagPnts3D[4][1]  << "," << CodeTagPnts3D[4][2]  << "," << CodeTagPnts3D[4][3]  << "," << endl
					<< "6," << CodeTagPnts3D[5][1]  << "," << CodeTagPnts3D[5][2]  << "," << CodeTagPnts3D[5][3]  << "," << endl
					<< "7," << CodeTagPnts3D[6][1]  << "," << CodeTagPnts3D[6][2]  << "," << CodeTagPnts3D[6][3]  << "," << endl
					<< "8," << CodeTagPnts3D[7][1]  << "," << CodeTagPnts3D[7][2]  << "," << CodeTagPnts3D[7][3]  << "," << endl;
				fout.close();

			}

			//如果跟踪7个编码点时
			if (CodeTagPnts3D.size() == 7)
			{
				ui.lineEdit_1->setText(TagPoint3f2QString(CodeTagPnts3D[0]));
				ui.lineEdit_2->setText(TagPoint3f2QString(CodeTagPnts3D[1]));
				ui.lineEdit_3->setText(TagPoint3f2QString(CodeTagPnts3D[2]));
				ui.lineEdit_4->setText(TagPoint3f2QString(CodeTagPnts3D[3]));
				ui.lineEdit_5->setText(TagPoint3f2QString(CodeTagPnts3D[4]));
				ui.lineEdit_6->setText(TagPoint3f2QString(CodeTagPnts3D[5]));
				ui.lineEdit_7->setText(TagPoint3f2QString(CodeTagPnts3D[6]));

				char *fname = "./DataFiles/data/data.txt"; //path -save data
				ofstream fout(fname, ios::app);
				fout << "1," << CodeTagPnts3D[0][1]  << "," << CodeTagPnts3D[0][2]  << "," << CodeTagPnts3D[0][3]  << "," << endl
					<< "2," << CodeTagPnts3D[1][1]  << "," << CodeTagPnts3D[1][2]  << "," << CodeTagPnts3D[1][3]  << "," << endl
					<< "3," << CodeTagPnts3D[2][1]  << "," << CodeTagPnts3D[2][2]  << "," << CodeTagPnts3D[2][3]  << "," << endl
					<< "4," << CodeTagPnts3D[3][1]  << "," << CodeTagPnts3D[3][2]  << "," << CodeTagPnts3D[3][3]  << "," << endl
					<< "5," << CodeTagPnts3D[4][1]  << "," << CodeTagPnts3D[4][2]  << "," << CodeTagPnts3D[4][3]  << "," << endl
					<< "6," << CodeTagPnts3D[5][1]  << "," << CodeTagPnts3D[5][2]  << "," << CodeTagPnts3D[5][3]  << "," << endl
					<< "7," << CodeTagPnts3D[6][1]  << "," << CodeTagPnts3D[6][2]  << "," << CodeTagPnts3D[6][3]  << "," << endl;
				fout.close();

			}
			//如果跟踪6个编码点时
			if (CodeTagPnts3D.size() == 6)
			{
				ui.lineEdit_1->setText(TagPoint3f2QString(CodeTagPnts3D[0]));
				ui.lineEdit_2->setText(TagPoint3f2QString(CodeTagPnts3D[1]));
				ui.lineEdit_3->setText(TagPoint3f2QString(CodeTagPnts3D[2]));
				ui.lineEdit_4->setText(TagPoint3f2QString(CodeTagPnts3D[3]));
				ui.lineEdit_5->setText(TagPoint3f2QString(CodeTagPnts3D[4]));
				ui.lineEdit_6->setText(TagPoint3f2QString(CodeTagPnts3D[5]));

				char *fname = "./DataFiles/data/data.txt"; //path -save data
				ofstream fout(fname, ios::app);
				fout << "1," << CodeTagPnts3D[0][1]  << "," << CodeTagPnts3D[0][2]  << "," << CodeTagPnts3D[0][3]  << "," << endl
					<< "2," << CodeTagPnts3D[1][1]  << "," << CodeTagPnts3D[1][2]  << "," << CodeTagPnts3D[1][3]  << "," << endl
					<< "3," << CodeTagPnts3D[2][1]  << "," << CodeTagPnts3D[2][2]  << "," << CodeTagPnts3D[2][3]  << "," << endl
					<< "4," << CodeTagPnts3D[3][1]  << "," << CodeTagPnts3D[3][2]  << "," << CodeTagPnts3D[3][3]  << "," << endl
					<< "5," << CodeTagPnts3D[4][1]  << "," << CodeTagPnts3D[4][2]  << "," << CodeTagPnts3D[4][3]  << "," << endl
					<< "6," << CodeTagPnts3D[5][1]  << "," << CodeTagPnts3D[5][2]  << "," << CodeTagPnts3D[5][3]  << "," << endl;
				fout.close();

			}
			//如果跟踪5个编码点时
			if (CodeTagPnts3D.size() == 5)
			{
				ui.lineEdit_1->setText(TagPoint3f2QString(CodeTagPnts3D[0]));
				ui.lineEdit_2->setText(TagPoint3f2QString(CodeTagPnts3D[1]));
				ui.lineEdit_3->setText(TagPoint3f2QString(CodeTagPnts3D[2]));
				ui.lineEdit_4->setText(TagPoint3f2QString(CodeTagPnts3D[3]));
				ui.lineEdit_5->setText(TagPoint3f2QString(CodeTagPnts3D[4]));

				char *fname = "./DataFiles/data/data.txt"; //path -save data
				ofstream fout(fname, ios::app);
				fout << "1," << CodeTagPnts3D[0][1]  << "," << CodeTagPnts3D[0][2]  << "," << CodeTagPnts3D[0][3]  << "," << endl
					<< "2," << CodeTagPnts3D[1][1]  << "," << CodeTagPnts3D[1][2]  << "," << CodeTagPnts3D[1][3]  << "," << endl
					<< "3," << CodeTagPnts3D[2][1]  << "," << CodeTagPnts3D[2][2]  << "," << CodeTagPnts3D[2][3]  << "," << endl
					<< "4," << CodeTagPnts3D[3][1]  << "," << CodeTagPnts3D[3][2]  << "," << CodeTagPnts3D[3][3]  << "," << endl
					<< "5," << CodeTagPnts3D[4][1]  << "," << CodeTagPnts3D[4][2]  << "," << CodeTagPnts3D[4][3]  << "," << endl;
				fout.close();


			}
			//如果跟踪4个编码点时
			if (CodeTagPnts3D.size() == 4)
			{
				ui.lineEdit_1->setText(TagPoint3f2QString(CodeTagPnts3D[0]));
				ui.lineEdit_2->setText(TagPoint3f2QString(CodeTagPnts3D[1]));
				ui.lineEdit_3->setText(TagPoint3f2QString(CodeTagPnts3D[2]));
				ui.lineEdit_4->setText(TagPoint3f2QString(CodeTagPnts3D[3]));

				char *fname = "./DataFiles/data/data.txt"; //path -save data
				ofstream fout(fname, ios::app);
				fout << "1," << CodeTagPnts3D[0][1]  << "," << CodeTagPnts3D[0][2]  << "," << CodeTagPnts3D[0][3]  << "," << endl
					<< "2," << CodeTagPnts3D[1][1]  << "," << CodeTagPnts3D[1][2]  << "," << CodeTagPnts3D[1][3]  << "," << endl
					<< "3," << CodeTagPnts3D[2][1]  << "," << CodeTagPnts3D[2][2]  << "," << CodeTagPnts3D[2][3]  << "," << endl
					<< "4," << CodeTagPnts3D[3][1]  << "," << CodeTagPnts3D[3][2]  << "," << CodeTagPnts3D[3][3]  << "," << endl;
				fout.close();

			}
			//如果跟踪3个编码点时
			if (CodeTagPnts3D.size() == 3)
			{
				ui.lineEdit_1->setText(TagPoint3f2QString(CodeTagPnts3D[0]));
				ui.lineEdit_2->setText(TagPoint3f2QString(CodeTagPnts3D[1]));
				ui.lineEdit_3->setText(TagPoint3f2QString(CodeTagPnts3D[2]));

				char *fname = "./DataFiles/data/data.txt"; //path -save data
				ofstream fout(fname, ios::app);
				fout << "1," << CodeTagPnts3D[0][1]+Rand(0,50)  << "," << CodeTagPnts3D[0][2]+Rand(0,50)  << "," << CodeTagPnts3D[0][3]+Rand(0,50)  << "," << endl
					<< "2," << CodeTagPnts3D[1][1]+Rand(0, 50) << "," << CodeTagPnts3D[1][2]+Rand(0,50) << "," << CodeTagPnts3D[1][3]+Rand(0,50) << "," << endl
					<< "3," << CodeTagPnts3D[2][1]+Rand(0, 50) << "," << CodeTagPnts3D[2][2]+Rand(0,50) << "," << CodeTagPnts3D[2][3]+Rand(0,50) << "," << endl;
				fout.close();

			}
			//如果跟踪2个编码点时
			if (CodeTagPnts3D.size() == 2)
			{
				ui.lineEdit_1->setText(TagPoint3f2QString(CodeTagPnts3D[0]));
				ui.lineEdit_2->setText(TagPoint3f2QString(CodeTagPnts3D[1]));

				char *fname = "./DataFiles/data/data.txt"; //path -save data
				ofstream fout(fname, ios::app);
				fout << "1," << CodeTagPnts3D[0][1]  << "," << CodeTagPnts3D[0][2]  << "," << CodeTagPnts3D[0][3]  << "," << endl
					<< "2," << CodeTagPnts3D[1][1]  << "," << CodeTagPnts3D[1][2]  << "," << CodeTagPnts3D[1][3]  << "," << endl;
				fout.close();

			}
			//如果跟踪1个编码点时
			if (CodeTagPnts3D.size() == 1)
			{
				ui.lineEdit_1->setText(TagPoint3f2QString(CodeTagPnts3D[0]));

				char *fname = "./DataFiles/data/data.txt"; //path -save data
				ofstream fout(fname, ios::app);
				fout << "1," << CodeTagPnts3D[0][1]  << "," << CodeTagPnts3D[0][2]  << "," << CodeTagPnts3D[0][3]  << "," << endl
					<< "2," << CodeTagPnts3D[1][1]  << "," << CodeTagPnts3D[1][2]  << "," << CodeTagPnts3D[1][3]  << "," << endl;
				fout.close();

			}
		}
	}
	imgNumBeDealed++;
}


void DynamicTrackDlg::on_btnToTrack_clicked()
{
	//启动定时器，设置检查定时器的时间间隔，单位ms



	m_timerId = startTimer(20);
}


void DynamicTrackDlg::on_btnStopTracking_clicked()
{
	this->close();

}


void DynamicTrackDlg::on_btnSetTrackPath_clicked()
{
	//实现如何读取一个文件夹中的所有图片
	QString imgPath = QFileDialog::getExistingDirectory(this, GBK::ToUnicode("设置跟踪的图像路径"), "./triger", QFileDialog::Option::ShowDirsOnly);
	imgPathL2 = imgPath + "/L";
	imgPathR2 = imgPath + "/R";
	QDir dirL(imgPathL2);QDir dirR(imgPathR2);
	QStringList nameFilters;
	nameFilters << "*.bmp";
	dirL.setNameFilters(nameFilters); dirR.setNameFilters(nameFilters);
	dirL.setSorting(QDir::Name); dirR.setSorting(QDir::Name);

	//获取路径中图像名列表
	imgNameListL = dirL.entryList(); imgNameListR = dirR.entryList();
	QString temL = imgPathL2; QString temR = imgPathR2;
	temL = temL + "/";	temR = temR + "/";

	for (size_t i = 0; i < imgNameListL.size();i++)
	{
		imgNameListL[i].prepend(temL);
		imgNameListR[i].prepend(temR);
	}
	ui.btnToTrack->setEnabled(true);
}




QString DynamicTrackDlg::TagPoint3f2QString(const TagPoint3f CodeTagPnts3D)
{
	QString Code_Xvalue = QString("%1").arg(CodeTagPnts3D[1]);
	QString Code_Yvalue = QString("%1").arg(CodeTagPnts3D[2]);
	QString Code_Zvalue = QString("%1").arg(CodeTagPnts3D[3]);
	QString Code_Data = "X: " + Code_Xvalue + "  " + "Y:" + Code_Yvalue + "  " + "Z:" + Code_Zvalue;
	return Code_Data;
}


void DynamicTrackDlg::getMessurePara(const std::string CameraParaPathleft, const std::string CameraParaPathright, const std::string camgroupPara, const std::string LightPenParaPath, const std::string LightpenFeatures)
{
	//读入相机及光笔参数
	if (!(XMLReader::readCamPara(CameraParaPathleft, camparaleft)) || !(XMLReader::readCamGroupPara(camgroupPara, camGroupPara)) ||
		!(XMLReader::readCamPara(CameraParaPathright, campararight)) ||
		!(XMLReader::readLightPenPara(LightPenParaPath, lightpenPara.FeatursPoints, lightpenPara.CenterPoint)) ||
		!(XMLReader::readLightPenDotPara(LightpenFeatures, lightpenfeatures)))
	{
		err = READ_MEASUREMENT_PARA_ERROR;
	}
	err = OPERATION_OK;
}


bool DynamicTrackDlg::getCodes3DCoordinateDM(const Mat imgLeft, const Mat imgRight)
{
	if (FirstFrameOkOrNot == true)     //判断是否为第一帧图片
	{
		if (!findCodesOriPos2DDM(imgLeft, imgRight, markerOriLeft, markerOriRight, Cir_Diameter_Code, FlagNum))
		{
		
			return false;
		}
		FirstFrameOkOrNot = false;
	}

	if (FirstFrameOkOrNot == false)   //判断是否为非第一帧图片
	{
		if (!TrackALLCodesDM(imgLeft, imgRight, measurementVec, markerCenterVec, FlagNum, Cir_Diameter_Code, Flags_Code, CodeTagPnts3D))
		{
			FirstFrameOkOrNot = true;
	
			return false;
		}
	}
	return true;
}

bool DynamicTrackDlg::getCodes3DCoordinateDM(const Mat imgLeft, const Mat imgRight, bool &FirstFrameOkOrNot)
{
	if (FirstFrameOkOrNot == true)     //判断是否为第一帧图片
	{
		if (!findCodesOriPos2DDM(imgLeft, imgRight, markerOriLeft, markerOriRight, Cir_Diameter_Code, FlagNum))
		{

			return false;
		}
		FirstFrameOkOrNot = false;
	}

	if (FirstFrameOkOrNot == false)   //判断是否为非第一帧图片
	{
		if (!TrackALLCodesDM(imgLeft, imgRight, measurementVec, markerCenterVec, FlagNum, Cir_Diameter_Code, Flags_Code, CodeTagPnts3D))
		{
			FirstFrameOkOrNot = true;

			return false;
		}
	}
	return true;
}


bool DynamicTrackDlg::findCodesOriPos2DDM(const Mat imgLeft, const Mat imgRight, vector<pair<unsigned int, Point2f>>  &markerOriLeft, vector<pair<unsigned int, Point2f>> &markerOriRight,
	float &Code_Diameter, int &flag)
{
	//对于初始帧，需要进行全局检测
	markerOriLeft.clear(); markerOriRight.clear();
	cv::Rect maskRoiLeft = cv::Rect(Point2f(0, 0), imgLeft.size());
	cv::Rect maskRoiRight = cv::Rect(Point2f(0, 0), imgRight.size());
	if (!findCircularMarkerDM(imgLeft, maskRoiLeft, markerOriLeft, Code_Diameter)
		|| !findCircularMarkerDM(imgRight, maskRoiRight, markerOriRight, Code_Diameter))
	{
		return false;
	}
	if (markerOriLeft.size() == markerOriRight.size() && markerOriLeft[0].first == markerOriRight[0].first) //这是为了保证视野中必须要有编码点，并且顺序要对应
	{
		flag = markerOriLeft.size();
		//如果二维点全部检测成功，则进行三维重建。
		vector<pair<unsigned int, Point2f>> LeftmarkerResults, RightmarkerResults;
		pair<unsigned int, Point2f> markerResult_Left, markerResult_Right;

		for (size_t k = 0; k < markerOriLeft.size(); k++)
		{
			markerResult_Left.first = k; markerResult_Right.first = k;
			markerResult_Left.second = markerOriLeft[k].second; markerResult_Right.second = markerOriRight[k].second;
			LeftmarkerResults.push_back(markerResult_Left); RightmarkerResults.push_back(markerResult_Right);
		}

		//对于上述检测正确的编码点二维坐标，进行三维重建。
		CalculateCodeCenter3D(LeftmarkerResults, RightmarkerResults, CodeTagPnts3D);       ////对当前第一个编码点进行三维重建
		getCodeFlags(markerOriLeft, Flags_Code);//将编码点的编号拿出来
		if (!SetInitialPositionDM(
			markerOriLeft, markerOriRight,
			measurementVec))
		{
			return false;
		}
	}

	else
	{
		return false;
	}

	return true;
}

bool DynamicTrackDlg::TrackALLCodesDM(const Mat imgLeft, const Mat imgRight, vector<pair<Mat, Mat>> &measurementVec,
	vector<pair<Point2f, Point2f>>  &markerCenterVec, int flag_Num, float &Cir_Diameter, vector<unsigned int> Flags_Code, vector<TagPoint3f> &CodeTagPnts3D)
{
	CodeTagPnts3D.clear(); markerCenterVec.clear();
	if (flag_Num<0 || flag_Num>measurementVec.size())
	{
		std::cerr << "设置的追踪编码点个数过少或者过多了，请重新设置编码点追踪数目。" << endl;
		return false;
	}
	//利用Particle filter进行编码点跟踪
	for (size_t k = 0; k < flag_Num; k++) //设置追踪的编码点个数
	{

		Point2f markerCenter_Left, markerCenter_Right; pair<Point2f, Point2f>  markerCenter;
		if (TrackCodeTrace(imgLeft, measurementVec[k].first, markerCenter_Left, Flags_Code[k], Cir_Diameter)
			&&TrackCodeTrace(imgRight, measurementVec[k].second, markerCenter_Right, Flags_Code[k], Cir_Diameter)) //// 需要注意的是，此处的flag为编码点的标志位数值，而非编码点数目的多少。
		{
			markerCenter.first = markerCenter_Left; markerCenter.second = markerCenter_Right;
			markerCenterVec.push_back(markerCenter);
			continue;
		}
		else
		{
			std::cout << "编码点跟踪失败，返回" << endl;
			return false;
		}
	}
	//如果二维点全部检测成功，则进行三维重建。
	vector<pair<unsigned int, Point2f>> LeftmarkerResults, RightmarkerResults;
	pair<unsigned int, Point2f> markerResult_Left, markerResult_Right;
	if (markerCenterVec.size() < 1)
	{
		return false;
	}
	else
	{
		for (size_t k = 0; k < markerCenterVec.size(); k++)
		{
			markerResult_Left.first = k; markerResult_Right.first = k;
			markerResult_Left.second = markerCenterVec[k].first; markerResult_Right.second = markerCenterVec[k].second;
			LeftmarkerResults.push_back(markerResult_Left); RightmarkerResults.push_back(markerResult_Right);
		}
	}

	CalculateCodeCenter3D(LeftmarkerResults, RightmarkerResults, CodeTagPnts3D);       ////对当前第一个编码点进行三维重建
	//更新预判位置
	if (!UpdataPositionDM(markerCenterVec,
		measurementVec))
	{
		return false;
	}

	return true;
}

bool  DynamicTrackDlg::findCircularMarkerDM(const Mat img, vector<pair<unsigned int, Point2f>> &results_center)
{
	//将图像转化成灰度图
	Mat imgOriginal;
	if (img.channels() == 1)
	{
		imgOriginal = img;
	}
	else
	{
		cvtColor(img, imgOriginal, CV_BGR2GRAY);
	}
	//namedWindow("Original", 0);
	//imshow("Original", imgOriginal);
	//while (char(waitKey(1)) != ' ') {}

	//利用轮廓的椭圆检测
	vector<RotatedRect> ellipses0;
	CoreAlgorithm::detectEllipse(imgOriginal, ellipses0);

	//ellipses0中储存的长轴、短轴转换为半长轴、半短轴
	for (int i = 0; i < ellipses0.size(); i++)
	{
		ellipses0[i].size.width = ellipses0[i].size.width / 2;
		ellipses0[i].size.height = ellipses0[i].size.height / 2;
	}

	//第一次筛选椭圆
	vector<RotatedRect> ellipses1;
	for (int i = 0; i < ellipses0.size(); ++i) {
		if (ellipses0[i].size.height / ellipses0[i].size.width > 0 && ellipses0[i].size.height / ellipses0[i].size.width < 1.6 && ellipses0[i].size.height > 4)
		{
			ellipses1.push_back(ellipses0[i]);
		}
	}

	//第二次筛选：若两圆心距离小于大圆半径，剔除大圆
	vector<RotatedRect> ellipses2;
	bool WrongCircle = false;
	for (int i = 0; i < ellipses1.size(); ++i) {
		//判断是否有其他圆心在此圆内,而且此圆半径较大
		for (int j = 0; j < i; ++j)
		{
			if (CoreAlgorithm::get_distance(ellipses1[i].center, ellipses1[j].center) < ellipses1[i].size.height && ellipses1[i].size.height > ellipses1[j].size.height)
			{
				WrongCircle = true;
				break;
			}
		}
		if (!WrongCircle)
		{
			for (int j = i + 1; j < ellipses1.size(); ++j)
			{
				if (CoreAlgorithm::get_distance(ellipses1[i].center, ellipses1[j].center) < ellipses1[i].size.height && ellipses1[i].size.height > ellipses1[j].size.height)
				{
					WrongCircle = true;
					break;
				}
			}
		}
		//若无，则记此圆为经过第二次筛选的圆
		if (!WrongCircle)
		{
			ellipses2.push_back(ellipses1[i]);
		}
		WrongCircle = false;
	}

	//第三次筛选：剔除太大的圆
	//根据偏离平均值的程度排除误检测的椭圆，由于第一次筛选椭圆时已剔除半长轴小于4个像素的椭圆，此处只对太大的圆进行剔除
	vector<RotatedRect> ellipses3;

	float sum = 0, average;
	for (int i = 0; i < ellipses2.size(); ++i)
	{
		sum = sum + ellipses2[i].size.height;
	}
	average = sum / ellipses2.size();

	for (int i = 0; i < ellipses2.size(); ++i)
	{
		if (ellipses2[i].size.height < (1.6*average))
		{
			ellipses3.push_back(ellipses2[i]);
		}
	}

	//第四次筛选：剔除不符合编码标志点形态特征的圆
	Mat img1, img2;
	vector<RotatedRect> ellipses4;
	cv::Rect_<float> region(0, 0, 0, 0);
	Point2f img1_center;
	vector<vector<Point>> contours1;

	for (int j = 0; j < ellipses3.size(); j++)
	{
		//选择周围区域
		region.x = ellipses3[j].center.x - 4 * ellipses3[j].size.height;
		region.y = ellipses3[j].center.y - 4 * ellipses3[j].size.height;
		region.width = 8 * ellipses3[j].size.height;
		region.height = 8 * ellipses3[j].size.height;

		//排除太靠近边缘的圆
		if (region.x < 0 || region.y < 0 || (ellipses3[j].center.y + 4 * ellipses3[j].size.height) > imgOriginal.rows || (ellipses3[j].center.x + 4 * ellipses3[j].size.height) > imgOriginal.cols)
		{
			continue;
		}

		img1 = imgOriginal.clone();
		img1 = Mat(img1, region);
		img1_center.x = img1.cols / 2;
		img1_center.y = img1.rows / 2;
		circle(img1, img1_center, 3.8 * ellipses3[j].size.height + 2 * ellipses3[j].size.height, Scalar(0, 0, 0), 4 * ellipses3[j].size.height, 8, 0);
		circle(img1, img1_center, ellipses3[j].size.height / 2, Scalar(0, 0, 0), 2 * ellipses3[j].size.height, 8, 0);

		//二值化
		threshold(img1, img2, 100, 255, CV_THRESH_BINARY);

		//查找连通域
		findContours(img2, contours1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//排除周围连通域面积太小的圆
		for (int i = 0; i < contours1.size(); i++) {
			if (contourArea(contours1[i], false) >(0.35 * 3.14 * ellipses3[j].size.height * ellipses3[j].size.height))
			{
				ellipses4.push_back(ellipses3[j]);
				break;
			}
		}
	}

	//利用梯度的椭圆检测
	cv::Rect_<float> mask(0, 0, 0, 0);
	int kenelsize = 3;
	double precisionlevel = 0.04;
	bool multi = 0;
	vector<RotatedRect>  findResults;
	Point2f center_temp;
	vector<Point2f> vec_center_temp;//检测到的圆心（经过基于梯度的椭圆检测函数findEllipses最终确定的编码标志点中心圆）
	vector<float> vec_radius_temp;//半径（取半长轴为半径）（经过基于梯度的椭圆检测函数findEllipses最终确定的编码标志点中心圆）
	Mat imgROI = imgOriginal.clone();

	for (int i = 0; i < ellipses4.size(); i++)
	{
		if (ellipses4[i].center.x - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.y - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.x + 2 * ellipses4[i].size.height < imgOriginal.size().width && ellipses4[i].center.y + 2 * ellipses4[i].size.height < imgOriginal.size().height)
		{
			//确定每个进行椭圆检测的小区域
			mask.x = ellipses4[i].center.x - 2 * ellipses4[i].size.height;
			mask.y = ellipses4[i].center.y - 2 * ellipses4[i].size.height;
			mask.width = 4 * ellipses4[i].size.height;
			mask.height = 4 * ellipses4[i].size.height;

			//画出每个进行椭圆检测的小区域
			rectangle(imgROI, mask, 255, 3);

			//基于梯度的椭圆检测
			CoreAlgorithm::findEllipses(imgOriginal, mask, findResults, precisionlevel, multi, kenelsize);

			for (int i = 0; i < findResults.size(); ++i)
			{
				if (findResults[i].size.width / findResults[i].size.height > 0 && findResults[i].size.width / findResults[i].size.height < 1.6 && findResults[i].size.width > 4) {
					center_temp.x = findResults[i].center.x;
					center_temp.y = findResults[i].center.y;
					vec_center_temp.push_back(center_temp);
					vec_radius_temp.push_back(findResults[i].size.width);
				}
			}
		}
	}

	////显示所有进行椭圆检测的小区域
	//namedWindow("ROI", 0);
	//imshow("ROI", imgROI);
	//while (char(waitKey(1)) != ' ') {}

	//画出检测出的全部圆
	Mat img4;
	cv::cvtColor(imgOriginal, img4, CV_GRAY2BGR);
	for (int i = 0; i < vec_radius_temp.size(); i++) {
		circle(img4, vec_center_temp[i], vec_radius_temp[i], Scalar(0, 0, 255), 2, 8, 0);
	}
	/*namedWindow("Circles", 0);
	imshow("Circles", img4);
	while (char(waitKey(1)) != ' ') {}*/


	//计算每个标志的编码
	Mat img5, img6, img7, img8;
	vector<vector<Point>> contours, contours_temp, contours_center;
	vector<Point> contours_center_max;
	double ss;
	vector<double> area_temp, area;
	int Num_Contours;
	vector<cv::Moments> mu;
	vector<Point2f> mc;
	vector<int> SingleNumber;
	vector<float> Angle;
	float Angle_Between;
	float min, max;
	int Num_Between1, Num_Between2;
	string TempCode;
	vector<string> BinaryCode;//存放最终确定的二进制编码
	vector<int> Code;//存放最终确定的十进制编码
	cv::Rect_<float> ROI_marker(0, 0, 0, 0);
	cv::Rect_<float> ROI_circle(0, 0, 0, 0);
	Point2f img5_center, img7_center;
	int maxcontour;
	vector<Point2f> Coordinate;	//存放最终确定的编码标志点中心坐标

	for (int i = 0; i < vec_center_temp.size(); i++)//第i个编码标志
	{
		//对环形编码带进行处理

		//选择编码标志点所在区域
		ROI_marker.x = vec_center_temp[i].x - 4 * vec_radius_temp[i];
		ROI_marker.y = vec_center_temp[i].y - 4 * vec_radius_temp[i];
		ROI_marker.width = 8 * vec_radius_temp[i];
		ROI_marker.height = 8 * vec_radius_temp[i];

		//排除太靠近边缘的圆
		if (ROI_marker.x < 0 || ROI_marker.y < 0 || (vec_center_temp[i].x + 4 * vec_radius_temp[i]) > imgOriginal.cols || (vec_center_temp[i].y + 4 * vec_radius_temp[i]) > imgOriginal.rows)
		{
			continue;
		}

		//用黑色把中心圆和环形编码带外围覆盖
		img5 = imgOriginal.clone();
		img5 = cv::Mat(img5, ROI_marker);
		img5_center.x = img5.cols / 2;
		img5_center.y = img5.rows / 2;
		//circle(img5, img5_center, 3.8 * vec_radius_temp[i] + 2 * vec_radius_temp[i], Scalar(0, 0, 0), 4 * vec_radius_temp[i], 8, 0);//环形编码带外围（似乎不需要涂黑）
		circle(img5, img5_center, vec_radius_temp[i] / 2, cv::Scalar(0, 0, 0), 2 * vec_radius_temp[i], 8, 0);//中心圆
		//namedWindow("查看环形编码带", 0);
		//imshow("查看环形编码带", img5);
		//while (char(waitKey(1)) != ' ') {}

		//二值化
		threshold(img5, img6, 100, 255, CV_THRESH_BINARY);
		//namedWindow("查看二值化", 0);
		//imshow("查看二值化", img6);
		//while (char(waitKey(1)) != ' ') {}

		//查找连通域
		findContours(img6, contours_temp, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//求环形编码带面积
		for (int j = 0; j < contours_temp.size(); j++) {
			area_temp.push_back(contourArea(contours_temp[j], false));
		}

		//排除太小的连通域
		for (int j = 0; j < contours_temp.size(); j++)
		{
			if (area_temp[j] >(0.35 * 3.14 * vec_radius_temp[i] * vec_radius_temp[i]))
			{
				area.push_back(area_temp[j]);
				contours.push_back(contours_temp[j]);
			}
		}
		Num_Contours = contours.size();
		contours_temp.clear();
		area_temp.clear();

		//若轮廓数为0，则不是编码标志点
		if (Num_Contours == 0)
		{
			continue;
		}

		//若有四个轮廓，则编码已确定
		if (Num_Contours == 4)
		{
			BinaryCode.push_back("01010101");
			Coordinate.push_back(vec_center_temp[i]);

			//清除本次循环储存的数据
			contours.clear();
			area.clear();
			//continue;
		}

		//若不是四个轮廓，则继续进行以下运算

		//对中心圆采用连通域的方法求面积
		//截取包含中心圆的小区域
		else
		{
			ROI_circle.x = vec_center_temp[i].x - 1.5 * vec_radius_temp[i];
			ROI_circle.y = vec_center_temp[i].y - 1.5 * vec_radius_temp[i];
			ROI_circle.width = 3 * vec_radius_temp[i];
			ROI_circle.height = 3 * vec_radius_temp[i];

			img7 = imgOriginal.clone();
			img7 = cv::Mat(img7, ROI_circle);

			//中心圆的周围涂黑
			img7_center.x = img7.cols / 2;
			img7_center.y = img7.rows / 2;
			circle(img7, img7_center, 1.5 * vec_radius_temp[i] + 0.75 * vec_radius_temp[i], cv::Scalar(0, 0, 0), 1.5 * vec_radius_temp[i], 8, 0);

			//二值化
			threshold(img7, img8, 100, 255, CV_THRESH_BINARY);

			//查找连通域
			findContours(img8, contours_center, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

			//保留最大的一个连通域
			maxcontour = 0;
			for (int j = 1; j < contours_center.size(); j++)
			{
				if (contours_center[j].size() > contours_center[maxcontour].size())
				{
					maxcontour = j;
				}
			}
			contours_center_max = contours_center[maxcontour];

			//求中心圆面积
			ss = contourArea(contours_center_max, false);

			//确定每一块连通域对应的编码

			//设计编码标志点时，单个环形编码带只有1、3、5、7的情况
			for (int j = 0; j < Num_Contours; ++j)
			{
				if (area[j] >(0.388*ss) && area[j] < (1.550*ss))   SingleNumber.push_back(1);
				else if (area[j] > (1.550*ss) && area[j] < (3.100*ss)) SingleNumber.push_back(3);
				else if (area[j] > (3.100*ss) && area[j] < (4.650*ss)) SingleNumber.push_back(5);
				else if (area[j] > (4.650*ss) && area[j] < (6.200*ss)) SingleNumber.push_back(7);
			}

			//若只有一个轮廓，则编码已确定

			if (Num_Contours == 1)
			{
				for (int j = 0; j < 8 - SingleNumber[0]; j++)
				{
					TempCode.push_back('0');
				}
				for (int j = 0; j < SingleNumber[0]; j++)
				{
					TempCode.push_back('1');
				}

				BinaryCode.push_back(TempCode);
				Coordinate.push_back(vec_center_temp[i]);

				//清除本次循环储存的数据
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				SingleNumber.clear();
				area.clear();
				//continue;
			}


			//若有三个轮廓
			else if (Num_Contours == 3)
			{
				if (SingleNumber[0] == 3 || SingleNumber[1] == 3 || SingleNumber[2] == 3)
				{
					BinaryCode.push_back("01010111");
					Coordinate.push_back(vec_center_temp[i]);
				}

				else
				{
					BinaryCode.push_back("00010101");
					Coordinate.push_back(vec_center_temp[i]);
				}

				//清除本次循环储存的数据
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				SingleNumber.clear();
				area.clear();
			}

			//若不是一个或三个轮廓，则继续进行以下运算

			//若有两个轮廓
			else if (Num_Contours == 2)
			{
				//计算轮廓矩
				for (int j = 0; j < Num_Contours; j++)
				{
					mu.push_back(moments(contours[j], false));
				}

				//计算轮廓的质心
				for (int j = 0; j < Num_Contours; j++)
				{
					mc.push_back(cv::Point2f(mu[j].m10 / mu[j].m00, mu[j].m01 / mu[j].m00));
				}

				//计算连通域质心与中心圆圆心连线的角度
				for (int j = 0; j < Num_Contours; ++j) {
					Angle.push_back(atan2(mc[j].y - img5_center.y, mc[j].x - img5_center.x) / 3.1416 * 180);
				}

				//将角度范围转换为0-360度
				for (int j = 0; j < Num_Contours; ++j) {
					if (Angle[j] < 0) Angle[j] = Angle[j] + 360;
				}

				//根据两段环形编码带之间的夹角，确定它们之间相隔了几个单位
				int Num_Between;
				if (Angle[0] < Angle[1])
				{
					Angle_Between = Angle[1] - Angle[0] - (SingleNumber[0] + SingleNumber[1])*22.5;
					if (Angle_Between < 67.5) Num_Between = 1;
					else if (Angle_Between > 67.5 && Angle_Between < 112.5) Num_Between = 2;
					else if (Angle_Between > 112.5 && Angle_Between < 157.5) Num_Between = 3;
					else if (Angle_Between > 157.5 && Angle_Between < 202.5) Num_Between = 4;
					else if (Angle_Between > 202.5 && Angle_Between < 247.5) Num_Between = 5;

					for (int j = 0; j < SingleNumber[0]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < Num_Between; j++)
					{
						TempCode.push_back('0');
					}
					for (int j = 0; j < SingleNumber[1]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < 8 - SingleNumber[0] - Num_Between - SingleNumber[1]; j++)
					{
						TempCode.push_back('0');
					}
				}
				else
				{
					Angle_Between = Angle[0] - Angle[1] - (SingleNumber[0] + SingleNumber[1])*22.5;
					if (Angle_Between < 67.5) Num_Between = 1;
					else if (Angle_Between > 67.5 && Angle_Between < 112.5) Num_Between = 2;
					else if (Angle_Between > 112.5 && Angle_Between < 157.5) Num_Between = 3;
					else if (Angle_Between > 157.5 && Angle_Between < 202.5) Num_Between = 4;
					else if (Angle_Between > 202.5 && Angle_Between < 247.5) Num_Between = 5;

					for (int j = 0; j < SingleNumber[1]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < Num_Between; j++)
					{
						TempCode.push_back('0');
					}
					for (int j = 0; j < SingleNumber[0]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < 8 - SingleNumber[0] - Num_Between - SingleNumber[1]; j++)
					{
						TempCode.push_back('0');
					}
				}

				//转换为唯一的二进制编码
				if (TempCode == "10011100" || TempCode == "11100100")
				{
					BinaryCode.push_back("00100111");
				}
				else if (TempCode == "11101110")
				{
					BinaryCode.push_back("01110111");
				}
				else if (TempCode == "10001000")
				{
					BinaryCode.push_back("00010001");
				}
				else if (TempCode == "10111110" || TempCode == "11111010")
				{
					BinaryCode.push_back("01011111");
				}

				else if (TempCode == "10100000" || TempCode == "10000010")
				{
					BinaryCode.push_back("00000101");
				}
				else if (TempCode == "10010000" || TempCode == "10000100")
				{
					BinaryCode.push_back("00001001");
				}
				else if (TempCode == "10111000" || TempCode == "10001110" || TempCode == "11100010" || TempCode == "11101000")
				{
					BinaryCode.push_back("00010111");
				}

				Coordinate.push_back(vec_center_temp[i]);

				//清除本次循环储存的数据
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				area.clear();
				mu.clear();
				mc.clear();
				SingleNumber.clear();
				Angle.clear();
				//continue;
			}
		}
	}//对每个编码标志处理的for循环结束


	//将二进制编码转换为十进制编码
	for (int j = 0; j < BinaryCode.size(); j++)
	{
		if (BinaryCode[j] == "01111111")
		{
			Code.push_back(127);
		}
		else if (BinaryCode[j] == "00000001")
		{
			Code.push_back(1);
		}
		else if (BinaryCode[j] == "00100111")
		{
			Code.push_back(39);
		}
		else if (BinaryCode[j] == "01011111")
		{
			Code.push_back(95);
		}
		else if (BinaryCode[j] == "01010111")
		{
			Code.push_back(87);
		}
		else if (BinaryCode[j] == "00011111")
		{
			Code.push_back(31);
		}
		else if (BinaryCode[j] == "00000111")
		{
			Code.push_back(7);
		}
		else if (BinaryCode[j] == "01110111")
		{
			Code.push_back(119);
		}
		else if (BinaryCode[j] == "00010001")
		{
			Code.push_back(17);
		}
		else if (BinaryCode[j] == "01010101")
		{
			Code.push_back(85);
		}

		else if (BinaryCode[j] == "00000101")
		{
			Code.push_back(5);
		}
		else if (BinaryCode[j] == "00001001")
		{
			Code.push_back(9);
		}
		else if (BinaryCode[j] == "00010101")
		{
			Code.push_back(21);
		}
		else if (BinaryCode[j] == "00010111")
		{
			Code.push_back(23);
		}
	}

	//输出检测结果：编码、中心圆的圆心坐标
	/*cout << "\n编码\t中心圆的圆心坐标" << endl;
	for (int j = 0; j < Code.size(); j++)
	{
	cout << Code[j] << "\t" << Coordinate[j] << endl;
	}*/
	//char *fname = "E:/Data/编码点坐标.txt";
	//ofstream fout(fname, ios::app);
	//fout << "\n编码\t中心圆的圆心坐标" << endl;
	//for (int j = 0; j < Code.size(); j++)
	//{
	//	fout << Code[j] << "\t" << Coordinate[j] << endl;
	//}
	//fout.close();

	pair<unsigned int, Point2f> marker;

	//储存结果
	for (int j = 0; j < Code.size(); j++)
	{
		marker.first = Code[j];
		marker.second = Coordinate[j];
		results_center.push_back(marker);
	}

	//////
	if (results_center.size() >= 1)
	{
		///// 对标记好的点，按照标记点进行排序
		for (unsigned int i = 0; i < results_center.size() - 1; i++)
		{
			for (unsigned int j = i + 1; j<results_center.size(); j++)
			{
				if (results_center[i].first > results_center[j].first)
				{
					swap(results_center[i], results_center[j]);
				}
			}
		}

	}

	return true;



}


//编码标志点识别
bool DynamicTrackDlg::findCircularMarkerDM(const Mat img, const cv::Rect Mask, vector<pair<unsigned int, Point2f>> &results_center)
{
	//将图像转化成灰度图
	Mat imgOriginal;
	if (img.channels() == 1)
	{
		imgOriginal = img;
	}
	else
	{
		cvtColor(img, imgOriginal, CV_BGR2GRAY);
	}
	//namedWindow("Original", 0);
	//imshow("Original", imgOriginal);
	//while (char(waitKey(1)) != ' ') {}

	//利用轮廓的椭圆检测
	vector<RotatedRect> ellipses0;
	cv::Rect MaskRoi = Mask;
	if (MaskRoi.x < 0 || MaskRoi.y < 0)
	{
		MaskRoi.x = 0;
		MaskRoi.y = 0;
	}
	if (MaskRoi.x>img.cols || MaskRoi.y>img.rows)
	{
		MaskRoi.x = img.cols - MaskRoi.width;
		MaskRoi.y = img.rows - MaskRoi.height;
	}
	if ((MaskRoi.x + MaskRoi.width) > img.cols || (MaskRoi.y + MaskRoi.height) > img.rows)
	{
		MaskRoi.width = img.cols - MaskRoi.x;
		MaskRoi.height = img.rows - MaskRoi.y;
	}
	imgOriginal = Mat(imgOriginal, MaskRoi);

	//cv::namedWindow("MaskRoi", CV_WINDOW_NORMAL);
	//cv::imshow("MaskRoi", imgOriginal);
	//cv::waitKey(15);


	CoreAlgorithm::detectEllipse(imgOriginal, ellipses0);

	//ellipses0中储存的长轴、短轴转换为半长轴、半短轴
	for (int i = 0; i < ellipses0.size(); i++)
	{
		ellipses0[i].size.width = ellipses0[i].size.width / 2;
		ellipses0[i].size.height = ellipses0[i].size.height / 2;
	}

	//第一次筛选椭圆
	vector<RotatedRect> ellipses1;
	for (int i = 0; i < ellipses0.size(); ++i) {
		if (ellipses0[i].size.height / ellipses0[i].size.width > 0 && ellipses0[i].size.height / ellipses0[i].size.width < 1.6 && ellipses0[i].size.height > 4)
		{
			ellipses1.push_back(ellipses0[i]);
		}
	}

	//第二次筛选：若两圆心距离小于大圆半径，剔除大圆
	vector<RotatedRect> ellipses2;
	bool WrongCircle = false;
	for (int i = 0; i < ellipses1.size(); ++i) {
		//判断是否有其他圆心在此圆内,而且此圆半径较大
		for (int j = 0; j < i; ++j)
		{
			if (CoreAlgorithm::get_distance(ellipses1[i].center, ellipses1[j].center) < ellipses1[i].size.height && ellipses1[i].size.height > ellipses1[j].size.height)
			{
				WrongCircle = true;
				break;
			}
		}
		if (!WrongCircle)
		{
			for (int j = i + 1; j < ellipses1.size(); ++j)
			{
				if (CoreAlgorithm::get_distance(ellipses1[i].center, ellipses1[j].center) < ellipses1[i].size.height && ellipses1[i].size.height > ellipses1[j].size.height)
				{
					WrongCircle = true;
					break;
				}
			}
		}
		//若无，则记此圆为经过第二次筛选的圆
		if (!WrongCircle)
		{
			ellipses2.push_back(ellipses1[i]);
		}
		WrongCircle = false;
	}

	//第三次筛选：剔除太大的圆
	//根据偏离平均值的程度排除误检测的椭圆，由于第一次筛选椭圆时已剔除半长轴小于4个像素的椭圆，此处只对太大的圆进行剔除
	vector<RotatedRect> ellipses3;

	float sum = 0, average;
	for (int i = 0; i < ellipses2.size(); ++i)
	{
		sum = sum + ellipses2[i].size.height;
	}
	average = sum / ellipses2.size();

	for (int i = 0; i < ellipses2.size(); ++i)
	{
		if (ellipses2[i].size.height < (1.6*average))
		{
			ellipses3.push_back(ellipses2[i]);
		}
	}

	//第四次筛选：剔除不符合编码标志点形态特征的圆
	Mat img1, img2;
	vector<RotatedRect> ellipses4;
	cv::Rect_<float> region(0, 0, 0, 0);
	Point2f img1_center;
	vector<vector<Point>> contours1;

	for (int j = 0; j < ellipses3.size(); j++)
	{
		//选择周围区域
		region.x = ellipses3[j].center.x - 4 * ellipses3[j].size.height;
		region.y = ellipses3[j].center.y - 4 * ellipses3[j].size.height;
		region.width = 8 * ellipses3[j].size.height;
		region.height = 8 * ellipses3[j].size.height;

		//排除太靠近边缘的圆
		if (region.x < 0 || region.y < 0 || (ellipses3[j].center.y + 4 * ellipses3[j].size.height) > imgOriginal.rows || (ellipses3[j].center.x + 4 * ellipses3[j].size.height) > imgOriginal.cols)
		{
			continue;
		}

		img1 = imgOriginal.clone();
		img1 = Mat(img1, region);
		img1_center.x = img1.cols / 2;
		img1_center.y = img1.rows / 2;
		circle(img1, img1_center, 3.8 * ellipses3[j].size.height + 2 * ellipses3[j].size.height, Scalar(0, 0, 0), 4 * ellipses3[j].size.height, 8, 0);
		circle(img1, img1_center, ellipses3[j].size.height / 2, Scalar(0, 0, 0), 2 * ellipses3[j].size.height, 8, 0);

		//二值化
		threshold(img1, img2, 100, 255, CV_THRESH_BINARY);

		//查找连通域
		findContours(img2, contours1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//排除周围连通域面积太小的圆
		for (int i = 0; i < contours1.size(); i++) {
			if (contourArea(contours1[i], false) >(0.35 * 3.14 * ellipses3[j].size.height * ellipses3[j].size.height))
			{
				ellipses4.push_back(ellipses3[j]);
				break;
			}
		}
	}

	//利用梯度的椭圆检测
	cv::Rect_<float> mask(0, 0, 0, 0);
	int kenelsize = 3;
	double precisionlevel = 0.04;
	bool multi = 0;
	vector<RotatedRect>  findResults;
	Point2f center_temp;
	vector<Point2f> vec_center_temp;//检测到的圆心（经过基于梯度的椭圆检测函数findEllipses最终确定的编码标志点中心圆）
	vector<float> vec_radius_temp;//半径（取半长轴为半径）（经过基于梯度的椭圆检测函数findEllipses最终确定的编码标志点中心圆）
	Mat imgROI = imgOriginal.clone();

	for (int i = 0; i < ellipses4.size(); i++)
	{
		if (ellipses4[i].center.x - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.y - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.x + 2 * ellipses4[i].size.height < imgOriginal.size().width && ellipses4[i].center.y + 2 * ellipses4[i].size.height < imgOriginal.size().height)
		{
			//确定每个进行椭圆检测的小区域
			mask.x = ellipses4[i].center.x - 2 * ellipses4[i].size.height;
			mask.y = ellipses4[i].center.y - 2 * ellipses4[i].size.height;
			mask.width = 4 * ellipses4[i].size.height;
			mask.height = 4 * ellipses4[i].size.height;

			//画出每个进行椭圆检测的小区域
			rectangle(imgROI, mask, 255, 3);

			//基于梯度的椭圆检测
			CoreAlgorithm::findEllipses(imgOriginal, mask, findResults, precisionlevel, multi, kenelsize);

			for (int i = 0; i < findResults.size(); ++i)
			{
				if (findResults[i].size.width / findResults[i].size.height > 0 && findResults[i].size.width / findResults[i].size.height < 1.6 && findResults[i].size.width > 4) {
					center_temp.x = findResults[i].center.x;
					center_temp.y = findResults[i].center.y;
					vec_center_temp.push_back(center_temp);
					vec_radius_temp.push_back(findResults[i].size.width);
				}
			}
		}
	}

	////显示所有进行椭圆检测的小区域
	//namedWindow("ROI", 0);
	//imshow("ROI", imgROI);
	//while (char(waitKey(1)) != ' ') {}

	//画出检测出的全部圆
	Mat img4;
	cvtColor(imgOriginal, img4, CV_GRAY2BGR);
	for (int i = 0; i < vec_radius_temp.size(); i++) {
		circle(img4, vec_center_temp[i], vec_radius_temp[i], Scalar(0, 0, 255), 2, 8, 0);
	}
	/*namedWindow("Circles", 0);
	imshow("Circles", img4);
	while (char(waitKey(1)) != ' ') {}*/


	//计算每个标志的编码
	Mat img5, img6, img7, img8;
	vector<vector<Point>> contours, contours_temp, contours_center;
	vector<Point> contours_center_max;
	double ss;
	vector<double> area_temp, area;
	int Num_Contours;
	vector<cv::Moments> mu;
	vector<Point2f> mc;
	vector<int> SingleNumber;
	vector<float> Angle;
	float Angle_Between;
	float min, max;
	int Num_Between1, Num_Between2;
	string TempCode;
	vector<string> BinaryCode;//存放最终确定的二进制编码
	vector<int> Code;//存放最终确定的十进制编码
	cv::Rect_<float> ROI_marker(0, 0, 0, 0);
	cv::Rect_<float> ROI_circle(0, 0, 0, 0);
	Point2f img5_center, img7_center;
	int maxcontour;
	vector<Point2f> Coordinate;	//存放最终确定的编码标志点中心坐标

	for (int i = 0; i < vec_center_temp.size(); i++)//第i个编码标志
	{
		//对环形编码带进行处理

		//选择编码标志点所在区域
		ROI_marker.x = vec_center_temp[i].x - 4 * vec_radius_temp[i];
		ROI_marker.y = vec_center_temp[i].y - 4 * vec_radius_temp[i];
		ROI_marker.width = 8 * vec_radius_temp[i];
		ROI_marker.height = 8 * vec_radius_temp[i];

		//排除太靠近边缘的圆
		if (ROI_marker.x < 0 || ROI_marker.y < 0 || (vec_center_temp[i].x + 4 * vec_radius_temp[i]) > imgOriginal.cols || (vec_center_temp[i].y + 4 * vec_radius_temp[i]) > imgOriginal.rows)
		{
			continue;
		}

		//用黑色把中心圆和环形编码带外围覆盖
		img5 = imgOriginal.clone();
		img5 = cv::Mat(img5, ROI_marker);
		img5_center.x = img5.cols / 2;
		img5_center.y = img5.rows / 2;
		//circle(img5, img5_center, 3.8 * vec_radius_temp[i] + 2 * vec_radius_temp[i], Scalar(0, 0, 0), 4 * vec_radius_temp[i], 8, 0);//环形编码带外围（似乎不需要涂黑）
		circle(img5, img5_center, vec_radius_temp[i] / 2, cv::Scalar(0, 0, 0), 2 * vec_radius_temp[i], 8, 0);//中心圆
		//namedWindow("查看环形编码带", 0);
		//imshow("查看环形编码带", img5);
		//while (char(waitKey(1)) != ' ') {}

		//二值化
		threshold(img5, img6, 100, 255, CV_THRESH_BINARY);
		//namedWindow("查看二值化", 0);
		//imshow("查看二值化", img6);
		//while (char(waitKey(1)) != ' ') {}

		//查找连通域
		findContours(img6, contours_temp, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//求环形编码带面积
		for (int j = 0; j < contours_temp.size(); j++) {
			area_temp.push_back(contourArea(contours_temp[j], false));
		}

		//排除太小的连通域
		for (int j = 0; j < contours_temp.size(); j++)
		{
			if (area_temp[j] >(0.35 * 3.14 * vec_radius_temp[i] * vec_radius_temp[i]))
			{
				area.push_back(area_temp[j]);
				contours.push_back(contours_temp[j]);
			}
		}
		Num_Contours = contours.size();
		contours_temp.clear();
		area_temp.clear();

		//若轮廓数为0，则不是编码标志点
		if (Num_Contours == 0)
		{
			continue;
		}

		//若有四个轮廓，则编码已确定
		if (Num_Contours == 4)
		{
			BinaryCode.push_back("01010101");
			Coordinate.push_back(vec_center_temp[i]);

			//清除本次循环储存的数据
			contours.clear();
			area.clear();
			//continue;
		}

		//若不是四个轮廓，则继续进行以下运算

		//对中心圆采用连通域的方法求面积
		//截取包含中心圆的小区域
		else
		{
			ROI_circle.x = vec_center_temp[i].x - 1.5 * vec_radius_temp[i];
			ROI_circle.y = vec_center_temp[i].y - 1.5 * vec_radius_temp[i];
			ROI_circle.width = 3 * vec_radius_temp[i];
			ROI_circle.height = 3 * vec_radius_temp[i];

			img7 = imgOriginal.clone();
			img7 = cv::Mat(img7, ROI_circle);

			//中心圆的周围涂黑
			img7_center.x = img7.cols / 2;
			img7_center.y = img7.rows / 2;
			circle(img7, img7_center, 1.5 * vec_radius_temp[i] + 0.75 * vec_radius_temp[i], cv::Scalar(0, 0, 0), 1.5 * vec_radius_temp[i], 8, 0);

			//二值化
			threshold(img7, img8, 100, 255, CV_THRESH_BINARY);

			//查找连通域
			findContours(img8, contours_center, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

			//保留最大的一个连通域
			maxcontour = 0;
			for (int j = 1; j < contours_center.size(); j++)
			{
				if (contours_center[j].size() > contours_center[maxcontour].size())
				{
					maxcontour = j;
				}
			}
			contours_center_max = contours_center[maxcontour];

			//求中心圆面积
			ss = contourArea(contours_center_max, false);

			//确定每一块连通域对应的编码

			//设计编码标志点时，单个环形编码带只有1、3、5、7的情况
			for (int j = 0; j < Num_Contours; ++j)
			{
				if (area[j] >(0.388*ss) && area[j] < (1.550*ss))   SingleNumber.push_back(1);
				else if (area[j] > (1.550*ss) && area[j] < (3.100*ss)) SingleNumber.push_back(3);
				else if (area[j] > (3.100*ss) && area[j] < (4.650*ss)) SingleNumber.push_back(5);
				else if (area[j] > (4.650*ss) && area[j] < (6.200*ss)) SingleNumber.push_back(7);
			}

			//若只有一个轮廓，则编码已确定

			if (Num_Contours == 1)
			{
				for (int j = 0; j < 8 - SingleNumber[0]; j++)
				{
					TempCode.push_back('0');
				}
				for (int j = 0; j < SingleNumber[0]; j++)
				{
					TempCode.push_back('1');
				}

				BinaryCode.push_back(TempCode);
				Coordinate.push_back(vec_center_temp[i]);

				//清除本次循环储存的数据
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				SingleNumber.clear();
				area.clear();
				//continue;
			}


			//若有三个轮廓
			else if (Num_Contours == 3)
			{
				if (SingleNumber[0] == 3 || SingleNumber[1] == 3 || SingleNumber[2] == 3)
				{
					BinaryCode.push_back("01010111");
					Coordinate.push_back(vec_center_temp[i]);
				}

				else
				{
					BinaryCode.push_back("00010101");
					Coordinate.push_back(vec_center_temp[i]);
				}

				//清除本次循环储存的数据
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				SingleNumber.clear();
				area.clear();
			}

			//若不是一个或三个轮廓，则继续进行以下运算

			//若有两个轮廓
			else if (Num_Contours == 2)
			{
				//计算轮廓矩
				for (int j = 0; j < Num_Contours; j++)
				{
					mu.push_back(moments(contours[j], false));
				}

				//计算轮廓的质心
				for (int j = 0; j < Num_Contours; j++)
				{
					mc.push_back(cv::Point2f(mu[j].m10 / mu[j].m00, mu[j].m01 / mu[j].m00));
				}

				//计算连通域质心与中心圆圆心连线的角度
				for (int j = 0; j < Num_Contours; ++j) {
					Angle.push_back(atan2(mc[j].y - img5_center.y, mc[j].x - img5_center.x) / 3.1416 * 180);
				}

				//将角度范围转换为0-360度
				for (int j = 0; j < Num_Contours; ++j) {
					if (Angle[j] < 0) Angle[j] = Angle[j] + 360;
				}

				//根据两段环形编码带之间的夹角，确定它们之间相隔了几个单位
				int Num_Between;
				if (Angle[0] < Angle[1])
				{
					Angle_Between = Angle[1] - Angle[0] - (SingleNumber[0] + SingleNumber[1])*22.5;
					if (Angle_Between < 67.5) Num_Between = 1;
					else if (Angle_Between > 67.5 && Angle_Between < 112.5) Num_Between = 2;
					else if (Angle_Between > 112.5 && Angle_Between < 157.5) Num_Between = 3;
					else if (Angle_Between > 157.5 && Angle_Between < 202.5) Num_Between = 4;
					else if (Angle_Between > 202.5 && Angle_Between < 247.5) Num_Between = 5;

					for (int j = 0; j < SingleNumber[0]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < Num_Between; j++)
					{
						TempCode.push_back('0');
					}
					for (int j = 0; j < SingleNumber[1]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < 8 - SingleNumber[0] - Num_Between - SingleNumber[1]; j++)
					{
						TempCode.push_back('0');
					}
				}
				else
				{
					Angle_Between = Angle[0] - Angle[1] - (SingleNumber[0] + SingleNumber[1])*22.5;
					if (Angle_Between < 67.5) Num_Between = 1;
					else if (Angle_Between > 67.5 && Angle_Between < 112.5) Num_Between = 2;
					else if (Angle_Between > 112.5 && Angle_Between < 157.5) Num_Between = 3;
					else if (Angle_Between > 157.5 && Angle_Between < 202.5) Num_Between = 4;
					else if (Angle_Between > 202.5 && Angle_Between < 247.5) Num_Between = 5;

					for (int j = 0; j < SingleNumber[1]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < Num_Between; j++)
					{
						TempCode.push_back('0');
					}
					for (int j = 0; j < SingleNumber[0]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < 8 - SingleNumber[0] - Num_Between - SingleNumber[1]; j++)
					{
						TempCode.push_back('0');
					}
				}

				//转换为唯一的二进制编码
				if (TempCode == "10011100" || TempCode == "11100100")
				{
					BinaryCode.push_back("00100111");
				}
				else if (TempCode == "11101110")
				{
					BinaryCode.push_back("01110111");
				}
				else if (TempCode == "10001000")
				{
					BinaryCode.push_back("00010001");
				}
				else if (TempCode == "10111110" || TempCode == "11111010")
				{
					BinaryCode.push_back("01011111");
				}

				else if (TempCode == "10100000" || TempCode == "10000010")
				{
					BinaryCode.push_back("00000101");
				}
				else if (TempCode == "10010000" || TempCode == "10000100")
				{
					BinaryCode.push_back("00001001");
				}
				else if (TempCode == "10111000" || TempCode == "10001110" || TempCode == "11100010" || TempCode == "11101000")
				{
					BinaryCode.push_back("00010111");
				}

				Coordinate.push_back(vec_center_temp[i]);

				//清除本次循环储存的数据
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				area.clear();
				mu.clear();
				mc.clear();
				SingleNumber.clear();
				Angle.clear();
				//continue;
			}
		}
	}//对每个编码标志处理的for循环结束


	//将二进制编码转换为十进制编码
	for (int j = 0; j < BinaryCode.size(); j++)
	{
		if (BinaryCode[j] == "01111111")
		{
			Code.push_back(127);
		}
		else if (BinaryCode[j] == "00000001")
		{
			Code.push_back(1);
		}
		else if (BinaryCode[j] == "00100111")
		{
			Code.push_back(39);
		}
		else if (BinaryCode[j] == "01011111")
		{
			Code.push_back(95);
		}
		else if (BinaryCode[j] == "01010111")
		{
			Code.push_back(87);
		}
		else if (BinaryCode[j] == "00011111")
		{
			Code.push_back(31);
		}
		else if (BinaryCode[j] == "00000111")
		{
			Code.push_back(7);
		}
		else if (BinaryCode[j] == "01110111")
		{
			Code.push_back(119);
		}
		else if (BinaryCode[j] == "00010001")
		{
			Code.push_back(17);
		}
		else if (BinaryCode[j] == "01010101")
		{
			Code.push_back(85);
		}

		else if (BinaryCode[j] == "00000101")
		{
			Code.push_back(5);
		}
		else if (BinaryCode[j] == "00001001")
		{
			Code.push_back(9);
		}
		else if (BinaryCode[j] == "00010101")
		{
			Code.push_back(21);
		}
		else if (BinaryCode[j] == "00010111")
		{
			Code.push_back(23);
		}
	}

	//输出检测结果：编码、中心圆的圆心坐标
	/*cout << "\n编码\t中心圆的圆心坐标" << endl;
	for (int j = 0; j < Code.size(); j++)
	{
	cout << Code[j] << "\t" << Coordinate[j] << endl;
	}*/
	//char *fname = "E:/Data/编码点坐标.txt";
	//ofstream fout(fname, ios::app);
	//fout << "\n编码\t中心圆的圆心坐标" << endl;
	//for (int j = 0; j < Code.size(); j++)
	//{
	//	fout << Code[j] << "\t" << Coordinate[j] << endl;
	//}
	//fout.close();

	pair<unsigned int, Point2f> marker;

	//储存结果
	for (int j = 0; j < Code.size(); j++)
	{
		marker.first = Code[j];
		marker.second = Coordinate[j];
		marker.second.x += MaskRoi.x;
		marker.second.y += MaskRoi.y;
		results_center.push_back(marker);
	}

	//////
	if (results_center.size() >= 1)
	{
		///// 对标记好的点，按照标记点进行排序
		for (unsigned int i = 0; i < results_center.size() - 1; i++)
		{
			for (unsigned int j = i + 1; j<results_center.size(); j++)
			{
				if (results_center[i].first > results_center[j].first)
				{
					swap(results_center[i], results_center[j]);
				}
			}
		}

	}

	return true;
}

bool DynamicTrackDlg::findCircularMarkerDM(const Mat img, const cv::Rect Mask, vector<pair<unsigned int, Point2f>> &results_center, float &Code_Diameter)
{
	//将图像转化成灰度图
	Mat imgOriginal;
	if (img.channels() == 1)
	{
		imgOriginal = img;
	}
	else
	{
		cvtColor(img, imgOriginal, CV_BGR2GRAY);
	}
	//namedWindow("Original", 0);
	//imshow("Original", imgOriginal);
	//while (char(waitKey(1)) != ' ') {}

	//利用轮廓的椭圆检测
	vector<RotatedRect> ellipses0;
	cv::Rect MaskRoi = Mask;
	if (MaskRoi.x < 0 || MaskRoi.y < 0)
	{
		MaskRoi.x = 0;
		MaskRoi.y = 0;
	}
	if (MaskRoi.x>img.cols || MaskRoi.y>img.rows)
	{
		MaskRoi.x = img.cols - MaskRoi.width;
		MaskRoi.y = img.rows - MaskRoi.height;
	}
	if ((MaskRoi.x + MaskRoi.width) > img.cols || (MaskRoi.y + MaskRoi.height) > img.rows)
	{
		MaskRoi.width = img.cols - MaskRoi.x;
		MaskRoi.height = img.rows - MaskRoi.y;
	}
	imgOriginal = Mat(imgOriginal, MaskRoi);

	CoreAlgorithm::detectEllipse(imgOriginal, ellipses0);

	//ellipses0中储存的长轴、短轴转换为半长轴、半短轴
	for (int i = 0; i < ellipses0.size(); i++)
	{
		ellipses0[i].size.width = ellipses0[i].size.width / 2;
		ellipses0[i].size.height = ellipses0[i].size.height / 2;
	}

	//第一次筛选椭圆
	vector<RotatedRect> ellipses1;
	for (int i = 0; i < ellipses0.size(); ++i) {
		if (ellipses0[i].size.height / ellipses0[i].size.width > 0 && ellipses0[i].size.height / ellipses0[i].size.width < 1.6 && ellipses0[i].size.height > 4)
		{
			ellipses1.push_back(ellipses0[i]);
		}
	}

	//第二次筛选：若两圆心距离小于大圆半径，剔除大圆
	vector<RotatedRect> ellipses2;
	bool WrongCircle = false;
	for (int i = 0; i < ellipses1.size(); ++i) {
		//判断是否有其他圆心在此圆内,而且此圆半径较大
		for (int j = 0; j < i; ++j)
		{
			if (CoreAlgorithm::get_distance(ellipses1[i].center, ellipses1[j].center) < ellipses1[i].size.height && ellipses1[i].size.height > ellipses1[j].size.height)
			{
				WrongCircle = true;
				break;
			}
		}
		if (!WrongCircle)
		{
			for (int j = i + 1; j < ellipses1.size(); ++j)
			{
				if (CoreAlgorithm::get_distance(ellipses1[i].center, ellipses1[j].center) < ellipses1[i].size.height && ellipses1[i].size.height > ellipses1[j].size.height)
				{
					WrongCircle = true;
					break;
				}
			}
		}
		//若无，则记此圆为经过第二次筛选的圆
		if (!WrongCircle)
		{
			ellipses2.push_back(ellipses1[i]);
		}
		WrongCircle = false;
	}

	//第三次筛选：剔除太大的圆
	//根据偏离平均值的程度排除误检测的椭圆，由于第一次筛选椭圆时已剔除半长轴小于4个像素的椭圆，此处只对太大的圆进行剔除
	vector<RotatedRect> ellipses3;

	float sum = 0, average;
	for (int i = 0; i < ellipses2.size(); ++i)
	{
		sum = sum + ellipses2[i].size.height;
	}
	average = sum / ellipses2.size();

	for (int i = 0; i < ellipses2.size(); ++i)
	{
		if (ellipses2[i].size.height < (1.6*average))
		{
			ellipses3.push_back(ellipses2[i]);
		}
	}

	//第四次筛选：剔除不符合编码标志点形态特征的圆
	Mat img1, img2;
	vector<RotatedRect> ellipses4;
	cv::Rect_<float> region(0, 0, 0, 0);
	Point2f img1_center;
	vector<vector<Point>> contours1;

	Code_Diameter = 8 * ellipses3[0].size.height;
	for (int j = 0; j < ellipses3.size(); j++)
	{
		//选择周围区域
		region.x = ellipses3[j].center.x - 4 * ellipses3[j].size.height;
		region.y = ellipses3[j].center.y - 4 * ellipses3[j].size.height;
		region.width = 8 * ellipses3[j].size.height;
		region.height = 8 * ellipses3[j].size.height;


		//排除太靠近边缘的圆
		if (region.x < 0 || region.y < 0 || (ellipses3[j].center.y + 4 * ellipses3[j].size.height) > imgOriginal.rows || (ellipses3[j].center.x + 4 * ellipses3[j].size.height) > imgOriginal.cols)
		{
			continue;
		}

		img1 = imgOriginal.clone();
		img1 = Mat(img1, region);
		img1_center.x = img1.cols / 2;
		img1_center.y = img1.rows / 2;
		circle(img1, img1_center, 3.8 * ellipses3[j].size.height + 2 * ellipses3[j].size.height, Scalar(0, 0, 0), 4 * ellipses3[j].size.height, 8, 0);
		circle(img1, img1_center, ellipses3[j].size.height / 2, Scalar(0, 0, 0), 2 * ellipses3[j].size.height, 8, 0);

		//二值化
		cv::threshold(img1, img2, 100, 255, CV_THRESH_BINARY);

		//查找连通域
		cv::findContours(img2, contours1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//排除周围连通域面积太小的圆
		for (int i = 0; i < contours1.size(); i++) {
			if (contourArea(contours1[i], false) >(0.35 * 3.14 * ellipses3[j].size.height * ellipses3[j].size.height))
			{
				ellipses4.push_back(ellipses3[j]);
				break;
			}
		}
	}

	//利用梯度的椭圆检测
	cv::Rect_<float> mask(0, 0, 0, 0);
	int kenelsize = 3;
	double precisionlevel = 0.04;
	bool multi = 0;
	vector<RotatedRect>  findResults;
	Point2f center_temp;
	vector<Point2f> vec_center_temp;//检测到的圆心（经过基于梯度的椭圆检测函数findEllipses最终确定的编码标志点中心圆）
	vector<float> vec_radius_temp;//半径（取半长轴为半径）（经过基于梯度的椭圆检测函数findEllipses最终确定的编码标志点中心圆）
	Mat imgROI = imgOriginal.clone();

	for (int i = 0; i < ellipses4.size(); i++)
	{
		if (ellipses4[i].center.x - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.y - 2 * ellipses4[i].size.height > 0 && ellipses4[i].center.x + 2 * ellipses4[i].size.height < imgOriginal.size().width && ellipses4[i].center.y + 2 * ellipses4[i].size.height < imgOriginal.size().height)
		{
			//确定每个进行椭圆检测的小区域
			mask.x = ellipses4[i].center.x - 2 * ellipses4[i].size.height;
			mask.y = ellipses4[i].center.y - 2 * ellipses4[i].size.height;
			mask.width = 4 * ellipses4[i].size.height;
			mask.height = 4 * ellipses4[i].size.height;

			//画出每个进行椭圆检测的小区域
			rectangle(imgROI, mask, 255, 3);

			//基于梯度的椭圆检测
			CoreAlgorithm::findEllipses(imgOriginal, mask, findResults, precisionlevel, multi, kenelsize);

			for (int i = 0; i < findResults.size(); ++i)
			{
				if (findResults[i].size.width / findResults[i].size.height > 0 && findResults[i].size.width / findResults[i].size.height < 1.6 && findResults[i].size.width > 4) {
					center_temp.x = findResults[i].center.x;
					center_temp.y = findResults[i].center.y;
					vec_center_temp.push_back(center_temp);
					vec_radius_temp.push_back(findResults[i].size.width);
				}
			}
		}
	}

	////显示所有进行椭圆检测的小区域
	//namedWindow("ROI", 0);
	//imshow("ROI", imgROI);
	//while (char(waitKey(1)) != ' ') {}

	//画出检测出的全部圆
	Mat img4;
	cvtColor(imgOriginal, img4, CV_GRAY2BGR);
	for (int i = 0; i < vec_radius_temp.size(); i++) {
		circle(img4, vec_center_temp[i], vec_radius_temp[i], Scalar(0, 0, 255), 2, 8, 0);
	}
	/*namedWindow("Circles", 0);
	imshow("Circles", img4);
	while (char(waitKey(1)) != ' ') {}*/


	//计算每个标志的编码
	Mat img5, img6, img7, img8;
	vector<vector<Point>> contours, contours_temp, contours_center;
	vector<Point> contours_center_max;
	double ss;
	vector<double> area_temp, area;
	int Num_Contours;
	vector<cv::Moments> mu;
	vector<Point2f> mc;
	vector<int> SingleNumber;
	vector<float> Angle;
	float Angle_Between;
	float min, max;
	int Num_Between1, Num_Between2;
	string TempCode;
	vector<string> BinaryCode;//存放最终确定的二进制编码
	vector<int> Code;//存放最终确定的十进制编码
	cv::Rect_<float> ROI_marker(0, 0, 0, 0);
	cv::Rect_<float> ROI_circle(0, 0, 0, 0);
	Point2f img5_center, img7_center;
	int maxcontour;
	vector<Point2f> Coordinate;	//存放最终确定的编码标志点中心坐标

	for (int i = 0; i < vec_center_temp.size(); i++)//第i个编码标志
	{
		//对环形编码带进行处理

		//选择编码标志点所在区域
		ROI_marker.x = vec_center_temp[i].x - 4 * vec_radius_temp[i];
		ROI_marker.y = vec_center_temp[i].y - 4 * vec_radius_temp[i];
		ROI_marker.width = 8 * vec_radius_temp[i];
		ROI_marker.height = 8 * vec_radius_temp[i];

		//排除太靠近边缘的圆
		if (ROI_marker.x < 0 || ROI_marker.y < 0 || (vec_center_temp[i].x + 4 * vec_radius_temp[i]) > imgOriginal.cols || (vec_center_temp[i].y + 4 * vec_radius_temp[i]) > imgOriginal.rows)
		{
			continue;
		}

		//用黑色把中心圆和环形编码带外围覆盖
		img5 = imgOriginal.clone();
		img5 = cv::Mat(img5, ROI_marker);
		img5_center.x = img5.cols / 2;
		img5_center.y = img5.rows / 2;
		//circle(img5, img5_center, 3.8 * vec_radius_temp[i] + 2 * vec_radius_temp[i], Scalar(0, 0, 0), 4 * vec_radius_temp[i], 8, 0);//环形编码带外围（似乎不需要涂黑）
		circle(img5, img5_center, vec_radius_temp[i] / 2, cv::Scalar(0, 0, 0), 2 * vec_radius_temp[i], 8, 0);//中心圆
		//namedWindow("查看环形编码带", 0);
		//imshow("查看环形编码带", img5);
		//while (char(waitKey(1)) != ' ') {}

		//二值化
		threshold(img5, img6, 100, 255, CV_THRESH_BINARY);
		//namedWindow("查看二值化", 0);
		//imshow("查看二值化", img6);
		//while (char(waitKey(1)) != ' ') {}

		//查找连通域
		findContours(img6, contours_temp, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		//求环形编码带面积
		for (int j = 0; j < contours_temp.size(); j++) {
			area_temp.push_back(contourArea(contours_temp[j], false));
		}

		//排除太小的连通域
		for (int j = 0; j < contours_temp.size(); j++)
		{
			if (area_temp[j] >(0.35 * 3.14 * vec_radius_temp[i] * vec_radius_temp[i]))
			{
				area.push_back(area_temp[j]);
				contours.push_back(contours_temp[j]);
			}
		}
		Num_Contours = contours.size();
		contours_temp.clear();
		area_temp.clear();

		//若轮廓数为0，则不是编码标志点
		if (Num_Contours == 0)
		{
			continue;
		}

		//若有四个轮廓，则编码已确定
		if (Num_Contours == 4)
		{
			BinaryCode.push_back("01010101");
			Coordinate.push_back(vec_center_temp[i]);

			//清除本次循环储存的数据
			contours.clear();
			area.clear();
			//continue;
		}

		//若不是四个轮廓，则继续进行以下运算

		//对中心圆采用连通域的方法求面积
		//截取包含中心圆的小区域
		else
		{
			ROI_circle.x = vec_center_temp[i].x - 1.5 * vec_radius_temp[i];
			ROI_circle.y = vec_center_temp[i].y - 1.5 * vec_radius_temp[i];
			ROI_circle.width = 3 * vec_radius_temp[i];
			ROI_circle.height = 3 * vec_radius_temp[i];

			img7 = imgOriginal.clone();
			img7 = cv::Mat(img7, ROI_circle);

			//中心圆的周围涂黑
			img7_center.x = img7.cols / 2;
			img7_center.y = img7.rows / 2;
			circle(img7, img7_center, 1.5 * vec_radius_temp[i] + 0.75 * vec_radius_temp[i], cv::Scalar(0, 0, 0), 1.5 * vec_radius_temp[i], 8, 0);

			//二值化
			threshold(img7, img8, 100, 255, CV_THRESH_BINARY);

			//查找连通域
			findContours(img8, contours_center, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

			//保留最大的一个连通域
			maxcontour = 0;
			for (int j = 1; j < contours_center.size(); j++)
			{
				if (contours_center[j].size() > contours_center[maxcontour].size())
				{
					maxcontour = j;
				}
			}
			contours_center_max = contours_center[maxcontour];

			//求中心圆面积
			ss = contourArea(contours_center_max, false);

			//确定每一块连通域对应的编码

			//设计编码标志点时，单个环形编码带只有1、3、5、7的情况
			for (int j = 0; j < Num_Contours; ++j)
			{
				if (area[j] >(0.388*ss) && area[j] < (1.550*ss))   SingleNumber.push_back(1);
				else if (area[j] > (1.550*ss) && area[j] < (3.100*ss)) SingleNumber.push_back(3);
				else if (area[j] > (3.100*ss) && area[j] < (4.650*ss)) SingleNumber.push_back(5);
				else if (area[j] > (4.650*ss) && area[j] < (6.200*ss)) SingleNumber.push_back(7);
			}

			//若只有一个轮廓，则编码已确定

			if (Num_Contours == 1)
			{
				for (int j = 0; j < 8 - SingleNumber[0]; j++)
				{
					TempCode.push_back('0');
				}
				for (int j = 0; j < SingleNumber[0]; j++)
				{
					TempCode.push_back('1');
				}

				BinaryCode.push_back(TempCode);
				Coordinate.push_back(vec_center_temp[i]);

				//清除本次循环储存的数据
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				SingleNumber.clear();
				area.clear();
				//continue;
			}


			//若有三个轮廓
			else if (Num_Contours == 3)
			{
				if (SingleNumber[0] == 3 || SingleNumber[1] == 3 || SingleNumber[2] == 3)
				{
					BinaryCode.push_back("01010111");
					Coordinate.push_back(vec_center_temp[i]);
				}

				else
				{
					BinaryCode.push_back("00010101");
					Coordinate.push_back(vec_center_temp[i]);
				}

				//清除本次循环储存的数据
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				SingleNumber.clear();
				area.clear();
			}

			//若不是一个或三个轮廓，则继续进行以下运算

			//若有两个轮廓
			else if (Num_Contours == 2)
			{
				//计算轮廓矩
				for (int j = 0; j < Num_Contours; j++)
				{
					mu.push_back(moments(contours[j], false));
				}

				//计算轮廓的质心
				for (int j = 0; j < Num_Contours; j++)
				{
					mc.push_back(cv::Point2f(mu[j].m10 / mu[j].m00, mu[j].m01 / mu[j].m00));
				}

				//计算连通域质心与中心圆圆心连线的角度
				for (int j = 0; j < Num_Contours; ++j) {
					Angle.push_back(atan2(mc[j].y - img5_center.y, mc[j].x - img5_center.x) / 3.1416 * 180);
				}

				//将角度范围转换为0-360度
				for (int j = 0; j < Num_Contours; ++j) {
					if (Angle[j] < 0) Angle[j] = Angle[j] + 360;
				}

				//根据两段环形编码带之间的夹角，确定它们之间相隔了几个单位
				int Num_Between;
				if (Angle[0] < Angle[1])
				{
					Angle_Between = Angle[1] - Angle[0] - (SingleNumber[0] + SingleNumber[1])*22.5;
					if (Angle_Between < 67.5) Num_Between = 1;
					else if (Angle_Between > 67.5 && Angle_Between < 112.5) Num_Between = 2;
					else if (Angle_Between > 112.5 && Angle_Between < 157.5) Num_Between = 3;
					else if (Angle_Between > 157.5 && Angle_Between < 202.5) Num_Between = 4;
					else if (Angle_Between > 202.5 && Angle_Between < 247.5) Num_Between = 5;

					for (int j = 0; j < SingleNumber[0]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < Num_Between; j++)
					{
						TempCode.push_back('0');
					}
					for (int j = 0; j < SingleNumber[1]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < 8 - SingleNumber[0] - Num_Between - SingleNumber[1]; j++)
					{
						TempCode.push_back('0');
					}
				}
				else
				{
					Angle_Between = Angle[0] - Angle[1] - (SingleNumber[0] + SingleNumber[1])*22.5;
					if (Angle_Between < 67.5) Num_Between = 1;
					else if (Angle_Between > 67.5 && Angle_Between < 112.5) Num_Between = 2;
					else if (Angle_Between > 112.5 && Angle_Between < 157.5) Num_Between = 3;
					else if (Angle_Between > 157.5 && Angle_Between < 202.5) Num_Between = 4;
					else if (Angle_Between > 202.5 && Angle_Between < 247.5) Num_Between = 5;

					for (int j = 0; j < SingleNumber[1]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < Num_Between; j++)
					{
						TempCode.push_back('0');
					}
					for (int j = 0; j < SingleNumber[0]; j++)
					{
						TempCode.push_back('1');
					}
					for (int j = 0; j < 8 - SingleNumber[0] - Num_Between - SingleNumber[1]; j++)
					{
						TempCode.push_back('0');
					}
				}

				//转换为唯一的二进制编码
				if (TempCode == "10011100" || TempCode == "11100100")
				{
					BinaryCode.push_back("00100111");
				}
				else if (TempCode == "11101110")
				{
					BinaryCode.push_back("01110111");
				}
				else if (TempCode == "10001000")
				{
					BinaryCode.push_back("00010001");
				}
				else if (TempCode == "10111110" || TempCode == "11111010")
				{
					BinaryCode.push_back("01011111");
				}

				else if (TempCode == "10100000" || TempCode == "10000010")
				{
					BinaryCode.push_back("00000101");
				}
				else if (TempCode == "10010000" || TempCode == "10000100")
				{
					BinaryCode.push_back("00001001");
				}
				else if (TempCode == "10111000" || TempCode == "10001110" || TempCode == "11100010" || TempCode == "11101000")
				{
					BinaryCode.push_back("00010111");
				}

				Coordinate.push_back(vec_center_temp[i]);

				//清除本次循环储存的数据
				TempCode.clear();
				contours.clear();
				contours_center.clear();
				area.clear();
				mu.clear();
				mc.clear();
				SingleNumber.clear();
				Angle.clear();
				//continue;
			}
		}
	}//对每个编码标志处理的for循环结束


	//将二进制编码转换为十进制编码
	for (int j = 0; j < BinaryCode.size(); j++)
	{
		if (BinaryCode[j] == "01111111")
		{
			Code.push_back(127);
		}
		else if (BinaryCode[j] == "00000001")
		{
			Code.push_back(1);
		}
		else if (BinaryCode[j] == "00100111")
		{
			Code.push_back(39);
		}
		else if (BinaryCode[j] == "01011111")
		{
			Code.push_back(95);
		}
		else if (BinaryCode[j] == "01010111")
		{
			Code.push_back(87);
		}
		else if (BinaryCode[j] == "00011111")
		{
			Code.push_back(31);
		}
		else if (BinaryCode[j] == "00000111")
		{
			Code.push_back(7);
		}
		else if (BinaryCode[j] == "01110111")
		{
			Code.push_back(119);
		}
		else if (BinaryCode[j] == "00010001")
		{
			Code.push_back(17);
		}
		else if (BinaryCode[j] == "01010101")
		{
			Code.push_back(85);
		}

		else if (BinaryCode[j] == "00000101")
		{
			Code.push_back(5);
		}
		else if (BinaryCode[j] == "00001001")
		{
			Code.push_back(9);
		}
		else if (BinaryCode[j] == "00010101")
		{
			Code.push_back(21);
		}
		else if (BinaryCode[j] == "00010111")
		{
			Code.push_back(23);
		}
	}

	//输出检测结果：编码、中心圆的圆心坐标
	/*cout << "\n编码\t中心圆的圆心坐标" << endl;
	for (int j = 0; j < Code.size(); j++)
	{
	cout << Code[j] << "\t" << Coordinate[j] << endl;
	}*/
	//char *fname = "E:/Data/编码点坐标.txt";
	//ofstream fout(fname, ios::app);
	//fout << "\n编码\t中心圆的圆心坐标" << endl;
	//for (int j = 0; j < Code.size(); j++)
	//{
	//	fout << Code[j] << "\t" << Coordinate[j] << endl;
	//}
	//fout.close();

	pair<unsigned int, Point2f> marker;

	//储存结果
	for (int j = 0; j < Code.size(); j++)
	{
		marker.first = Code[j];
		marker.second = Coordinate[j];
		marker.second.x += MaskRoi.x;
		marker.second.y += MaskRoi.y;
		results_center.push_back(marker);
	}

	//////
	if (results_center.size() >= 1)
	{
		///// 对标记好的点，按照标记点进行排序
		for (unsigned int i = 0; i < results_center.size() - 1; i++)
		{
			for (unsigned int j = i + 1; j<results_center.size(); j++)
			{
				if (results_center[i].first > results_center[j].first)
				{
					swap(results_center[i], results_center[j]);
				}
			}
		}

	}

	return true;
}

bool DynamicTrackDlg::findCircularMarkerDM(const Mat img, const cv::Rect Mask, vector<pair<unsigned int, Point2f>> &results_center, int flag)
{
	results_center.clear();

	if (!findCircularMarkerDM(img, Mask, results_center))
	{
		return false;
	}

	if (results_center[0].first == flag)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool DynamicTrackDlg::UpdataPositionDM(const vector<pair<Point2f, Point2f>> markerCenterVec,  //Input
	vector<pair<Mat, Mat>>  &measurementVec)   //Output
{
	//更新编码点中心预判位置
	measurementVec.clear();
	if (markerCenterVec.empty())
	{
		return false;
	}
	for (size_t k = 0; k < markerCenterVec.size(); k++)
	{
		measurementPair.first = (cv::Mat_<float>(1, 2) << markerCenterVec[k].first.x, markerCenterVec[k].first.y);
		measurementPair.second = (cv::Mat_<float>(1, 2) << markerCenterVec[k].second.x, markerCenterVec[k].second.y);
		measurementVec.push_back(measurementPair);
	}
	return true;

}
void DynamicTrackDlg::getCodeFlags(const vector<pair<unsigned int, Point2f>> markerOriLeft, vector<unsigned int> &CodeFlags)
{
	//此处将编码点的标志位数值存放到一个容器中，以便后面调用。
	if (markerOriLeft.empty())
	{
		return;
	}
	for (size_t j = 0; j < markerOriLeft.size(); j++)
	{
		Flags_Code.push_back(markerOriLeft[j].first);
	}

}

bool DynamicTrackDlg::SetInitialPositionDM(
	const vector<pair<unsigned int, Point2f>>  markerOriLeft, const vector<pair<unsigned int, Point2f>> markerOriRight,
	vector<pair<Mat, Mat>> &measurementVec)
{
	measurementVec.clear();

	if (markerOriLeft.size() == 0 || markerOriRight.size() == 0)
	{
		return false;
	}

	for (size_t k = 0; k < markerOriLeft.size(); k++)
	{
		measurementPair.first = (cv::Mat_<float>(1, 2) << markerOriLeft[k].second.x, markerOriLeft[k].second.y);
		measurementPair.second = (cv::Mat_<float>(1, 2) << markerOriRight[k].second.x, markerOriRight[k].second.y);
		measurementVec.push_back(measurementPair);
	}

	return true;
}


bool  DynamicTrackDlg::CalculateCodeCenter3D(vector<pair<unsigned int, Point2f>> LeftmarkerResults,
	vector<pair<unsigned int, Point2f>> RightmarkerResults, vector<TagPoint3f>& CodeTagPnts3f)
{
	/////将识别出的编码点放入容器中，便于后续处理
	vector<TagPoint2f> leftTagsPnts2f;
	vector<TagPoint2f> rightTagsPnts2f;
	for (size_t k = 0; k < LeftmarkerResults.size(); k++)
	{
		TagPoint2f  LeftTagPnts, RightTagPnts;
		LeftTagPnts[0] = LeftmarkerResults[k].first;
		LeftTagPnts[1] = LeftmarkerResults[k].second.x;
		LeftTagPnts[2] = LeftmarkerResults[k].second.y;
		RightTagPnts[0] = RightmarkerResults[k].first;
		RightTagPnts[1] = RightmarkerResults[k].second.x;
		RightTagPnts[2] = RightmarkerResults[k].second.y;
		leftTagsPnts2f.push_back(LeftTagPnts);
		rightTagsPnts2f.push_back(RightTagPnts);
	}
	////将特征点的标签信息去掉
	vector<Point2f> leftPnts2f, rightPnts2f;
	vector<int> TagVec;
	pntsTagConfirm(leftTagsPnts2f, rightTagsPnts2f, leftPnts2f, rightPnts2f, TagVec);

	////计算左相机摄像机坐标系下的三维点信息
	vector<Point3f> pnts3fVec;
	//第一种计算三维点信息的方法
	RT CAMG;
	Mat rotation = Mat::zeros(3, 1, CV_64FC1);
	Mat translation = Mat::zeros(3, 1, CV_64FC1);
	for (int i = 0; i < 3; i++)
	{
		rotation.at<double>(i, 0) = camGroupPara.right2LeftRotVector[i];
		translation.at<double>(i, 0) = camGroupPara.right2LeftTraVector[i];
	}
	CAMG.R = rotation; CAMG.T = translation;
	vector<Point3f> pntsl;
	bool isok;
	isok = CoreAlgorithm::Cal3dPoint(leftPnts2f, camparaleft, rightPnts2f, campararight, CAMG.R, CAMG.T, pntsl);
	vector<TagPoint3f> result;
	addPntsTag(pntsl, TagVec, result);
	CodeTagPnts3f = result;

	return true;

}


bool DynamicTrackDlg::TrackCodeTrace(const Mat img, Mat &measurement, Point2f &markerCenterPnt, int flag, float &Cir_Diameter)
{
	///粒子滤波初始化
	int DP = 2; ////状态向量维数
	int nParticles = 62;
	//float xRange = img.cols;
	//float minRange[] = {0.0f,0.0f};
	//float maxRange[] = {xRange,xRange};
	float xMaxRange_x = measurement.at<float>(0, 0) + 10;
	float xMaxRange_y = measurement.at<float>(0, 1) + 10;
	float xMinRange_x = measurement.at<float>(0, 0) - 10;
	float xMinRange_y = measurement.at<float>(0, 1) - 10;
	float minRange[] = { xMinRange_x, xMinRange_y };
	float maxRange[] = { xMaxRange_x, xMaxRange_y };

	cv::Mat_<float> LB(1, DP, minRange);   ///粒子初始化时，取值下界
	cv::Mat_<float> UB(1, DP, maxRange);   ///粒子初始化时，取值上界
	cv::Mat_<float> dyna(cv::Mat_<float>::eye(2, 2)); ///转移矩阵
	ConDensation condens(DP, nParticles);
	condens.initSampleSet(LB, UB, dyna);
	Point2f f_StartPoint, f_Pnt;
	int width = 1.5*Cir_Diameter;        ///≈108pixels
	int front_gap = 0.75*Cir_Diameter;  ///≈54pixels
	int up_gap = 0.75*Cir_Diameter;     ///≈54pixels
	//std::cout <<"measure== "<< measurement << endl;    
	const cv::Mat_<float> &pred = condens.correct(measurement);
	cv::Point2f statePt(pred(0), pred(1));
	//std::cout << "statePt== " << statePt << endl;
	//将原图显示出来
	//cv::namedWindow("Origin Picture", CV_WINDOW_NORMAL);
	//cv::imshow("Origin Picture", img);
	//cv::waitKey(10);
	cv::Rect maskRoi = cv::Rect(cv::Point2i((int)(statePt.x - front_gap), (int)(statePt.y - up_gap)), Size(width, width));

	vector<pair<unsigned int, Point2f>> markerResults;
	findCircularMarkerDM(img, maskRoi, markerResults, flag);
	if (markerResults.size() == 0)
	{
		maskRoi = cv::Rect(cv::Point2i((int)(statePt.x - 1.5*front_gap), (int)(statePt.y - 1.5*up_gap)), Size(1.5*width, 1.5*width));
		findCircularMarkerDM(img, maskRoi, markerResults, flag);
		if (markerResults.size() == 0)
		{
			maskRoi = cv::Rect(cv::Point2i((int)(statePt.x - 3 * front_gap), (int)(statePt.y - 3 * up_gap)), Size(3 * width, 3 * width));
			findCircularMarkerDM(img, maskRoi, markerResults, flag);
			if (markerResults.size() == 0)
			{
				maskRoi = cv::Rect(cv::Point2i((int)(statePt.x - 6 * front_gap), (int)(statePt.y - 6 * up_gap)), Size(6 * width, 6 * width));
				findCircularMarkerDM(img, maskRoi, markerResults, flag);
			}

		}

	}

	if (markerResults.size() >= 1)
	{
		for (size_t m = 0; m < markerResults.size(); m++)
		{
			if (markerResults[m].first == flag)
			{
				markerCenterPnt = markerResults[m].second;
				measurement.at<float>(0, 0) = markerCenterPnt.x;
				measurement.at<float>(0, 1) = markerCenterPnt.y;

				return true;
			}
			else
			{
				continue;
			}
		}
	}
	Point2f Pnt = Point2f(-1, -1);
	markerCenterPnt = Pnt;
	return false;
}

bool DynamicTrackDlg::TrackCodeTraceALL(const Mat imgLeft, const Mat imgRight, vector<pair<Mat, Mat>> &measurementVec, vector<pair<Point2f, Point2f>>  &markerCenterVec, int flag_Num, float &Cir_Diameter, vector<unsigned int> Flags_Code, vector<TagPoint3f> &CodeTagPnts3D)
{
	if (flag_Num <= 0 || flag_Num > measurementVec.size())
	{
		std::cerr << "设置的追踪编码点个数过少或者过多了,请重新设置编码点追踪数目。" << endl;
	}

	for (size_t k = 0; k < flag_Num; k++)  ////设置追踪的编码点个数
	{
		Point2f markerCenter_Left, markerCenter_Right; pair<Point2f, Point2f>  markerCenter;
		if (TrackCodeTrace(imgLeft, measurementVec[k].first, markerCenter_Left, Flags_Code[k], Cir_Diameter)
			&& TrackCodeTrace(imgRight, measurementVec[k].second, markerCenter_Right, Flags_Code[k], Cir_Diameter)) //// 需要注意的是，此处的flag为编码点的标志位数值，而非编码点数目的多少。
		{
			markerCenter.first = markerCenter_Left; markerCenter.second = markerCenter_Right;
			markerCenterVec.push_back(markerCenter);
			continue;
		}
		else
		{
			return false;
		}
	}
	///////如果二维点全部检测成功，则进行三维重建。
	vector<pair<unsigned int, Point2f>> LeftmarkerResults, RightmarkerResults;
	pair<unsigned int, Point2f> markerResult_Left, markerResult_Right;
	if (markerCenterVec.size() < 1)
	{
		return false;
	}
	else
	{

		for (size_t k = 0; k < markerCenterVec.size(); k++)
		{
			markerResult_Left.first = k; markerResult_Right.first = k;
			markerResult_Left.second = markerCenterVec[k].first; markerResult_Right.second = markerCenterVec[k].second;
			LeftmarkerResults.push_back(markerResult_Left); RightmarkerResults.push_back(markerResult_Right);
		}
	}

	/////对于上述检测正确的编码点二维坐标，进行三维重建。
	CalculateCodeCenter3D(LeftmarkerResults, RightmarkerResults, CodeTagPnts3D);       ////对当前第一个编码点进行三维重建
	return true;
}


double DynamicTrackDlg::Rand(double dMin, double dMax)
{
	double dVal = (double)rand() / RAND_MAX;
	return dMin + dVal*(dMax - dMin);
}

QImage DynamicTrackDlg::cvMat2QImage(const cv::Mat& mat)
{
	// 8-bits unsigned, NO. OF CHANNELS = 1  
	if (mat.type() == CV_8UC1)
	{
		QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
		// Set the color table (used to translate colour indexes to qRgb values)  
		image.setColorCount(256);
		for (int i = 0; i < 256; i++)
		{
			image.setColor(i, qRgb(i, i, i));
		}
		// Copy input Mat  
		uchar *pSrc = mat.data;
		for (int row = 0; row < mat.rows; row++)
		{
			uchar *pDest = image.scanLine(row);
			memcpy(pDest, pSrc, mat.cols);
			pSrc += mat.step;
		}
		return image;
	}
	// 8-bits unsigned, NO. OF CHANNELS = 3  
	else if (mat.type() == CV_8UC3)
	{
		// Copy input Mat  
		const uchar *pSrc = (const uchar*)mat.data;
		// Create QImage with same dimensions as input Mat  
		QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
		return image.rgbSwapped();
	}
	else if (mat.type() == CV_8UC4)
	{
		qDebug() << "CV_8UC4";
		// Copy input Mat  
		const uchar *pSrc = (const uchar*)mat.data;
		// Create QImage with same dimensions as input Mat  
		QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
		return image.copy();
	}
	else
	{
		qDebug() << "ERROR: Mat could not be converted to QImage.";
		return QImage();
	}
}

cv::Mat DynamicTrackDlg::QImage2cvMat(QImage image)
{
	cv::Mat mat;
	qDebug() << image.format();
	switch (image.format())
	{
	case QImage::Format_ARGB32:
	case QImage::Format_RGB32:
	case QImage::Format_ARGB32_Premultiplied:
		mat = cv::Mat(image.height(), image.width(), CV_8UC4, (void*)image.constBits(), image.bytesPerLine());
		break;
	case QImage::Format_RGB888:
		mat = cv::Mat(image.height(), image.width(), CV_8UC3, (void*)image.constBits(), image.bytesPerLine());
		cv::cvtColor(mat, mat, CV_BGR2RGB);
		break;
	case QImage::Format_Indexed8:
		mat = cv::Mat(image.height(), image.width(), CV_8UC1, (void*)image.constBits(), image.bytesPerLine());
		break;
	}
	return mat;
}