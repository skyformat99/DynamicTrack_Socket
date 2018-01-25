/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the Qt Data Visualization module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:GPL$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3 or (at your option) any later version
** approved by the KDE Free Qt Foundation. The licenses are as published by
** the Free Software Foundation and appearing in the file LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "data.h"

#include <QtDataVisualization/QScatterDataProxy>
#include <QtDataVisualization/Q3DScene>
#include <QtDataVisualization/Q3DCamera>
#include <QtDataVisualization/QScatter3DSeries>
#include <QtDataVisualization/Q3DTheme>

using namespace QtDataVisualization;

const int itemCount = 500;

Data::Data(Q3DScatter *scatter)
: m_graph(scatter),
//! [1]
m_inputHandler(new AxesInputHandler(scatter)),
//! [1]
m_autoAdjust(false)
{
	m_graph->activeTheme()->setType(Q3DTheme::ThemeEbony);
	m_graph->activeTheme()->setLabelBorderEnabled(true);
	m_graph->activeTheme()->setLabelBackgroundColor(QColor(QRgb(0x151550)));
	m_graph->activeTheme()->setLabelTextColor(Qt::lightGray);
	m_graph->activeTheme()->setFont(QFont("Arial Black", 30));
	m_graph->setShadowQuality(QAbstract3DGraph::ShadowQualityNone); //此处是为了显示是否显示阴影
	m_graph->scene()->activeCamera()->setCameraPreset(Q3DCamera::CameraPresetIsometricRight);

	m_graph->axisX()->setRange(-200.0f, 1000.0f);
	m_graph->axisY()->setRange(-200.0f, 1500.0f);
	m_graph->axisZ()->setRange(1000.0f, 2500.0f);

	//! [0]
	// Give ownership of the handler to the graph and make it the active handler
	m_graph->setActiveInputHandler(m_inputHandler);
	//! [0]

	//! [2]
	// Give our axes to the input handler
	m_inputHandler->setAxes(m_graph->axisX(), m_graph->axisZ(), m_graph->axisY());
	//! [2]

	addData();
}

Data::~Data()
{
	delete m_graph;
}

void Data::toggleRanges()
{
	if (!m_autoAdjust) {
		m_graph->axisX()->setAutoAdjustRange(true);
		m_graph->axisZ()->setAutoAdjustRange(true);
		m_graph->axisY()->setAutoAdjustRange(true);
		m_inputHandler->setDragSpeedModifier(1.5f);
		m_autoAdjust = true;
	}
	else {
		m_graph->axisX()->setRange(-200.0f, 1000.0f);
		m_graph->axisY()->setRange(-200.0f, 1500.0f);
		m_graph->axisZ()->setRange(1000.0f, 2500.0f);
		m_inputHandler->setDragSpeedModifier(15.0f);
		m_autoAdjust = false;
	}
}

void Data::addData()
{
	QScatter3DSeries *series = new QScatter3DSeries;
	series->setItemLabelFormat(QStringLiteral("One- X:@xLabel,Y:@yLabel,Z:@zLabel"));
	series->setMesh(QAbstract3DSeries::MeshPoint);
	series->setItemSize(0.05f);
	series->setMeshSmooth(true);
	m_graph->addSeries(series);

	QScatter3DSeries *series2 = new QScatter3DSeries;
	series2->setItemLabelFormat(QStringLiteral("Two- X:@xLabel,Y:@yLabel,Z:@zLabel"));
	series2->setMesh(QAbstract3DSeries::MeshMinimal);
	series2->setItemSize(0.05f);
	series2->setMeshSmooth(true);
	m_graph->addSeries(series2);

	QScatter3DSeries *series3 = new QScatter3DSeries;
	series3->setItemLabelFormat(QStringLiteral("Three- X:@xLabel,Y:@yLabel,Z:@zLabel"));
	series3->setMesh(QAbstract3DSeries::MeshSphere);
	series3->setItemSize(0.05f);
	series3->setMeshSmooth(true);
	m_graph->addSeries(series3);

	QScatter3DSeries *series4 = new QScatter3DSeries;
	series4->setItemLabelFormat(QStringLiteral("Four- X:@xLabel,Y:@yLabel,Z:@zLabel"));
	series4->setMesh(QAbstract3DSeries::MeshBevelCube);
	series4->setItemSize(0.05f);
	series4->setMeshSmooth(true);
	m_graph->addSeries(series4);

	QScatter3DSeries *series5 = new QScatter3DSeries;
	series5->setItemLabelFormat(QStringLiteral("Five- X:@xLabel,Y:@yLabel,Z:@zLabel"));
	series5->setMesh(QAbstract3DSeries::MeshCylinder);
	series5->setItemSize(0.05f);
	m_graph->addSeries(series5);

	QScatter3DSeries *series6 = new QScatter3DSeries;
	series6->setItemLabelFormat(QStringLiteral("Six- X:@xLabel,Y:@yLabel,Z:@zLabel"));
	series6->setMesh(QAbstract3DSeries::MeshCone);
	series6->setItemSize(0.05f);
	m_graph->addSeries(series6);

	QScatter3DSeries *series7 = new QScatter3DSeries;
	series7->setItemLabelFormat(QStringLiteral("Seven- X:@xLabel,Y:@yLabel,Z:@zLabel"));
	series7->setMesh(QAbstract3DSeries::MeshBar);
	series7->setItemSize(0.05f);
	m_graph->addSeries(series7);

	QScatter3DSeries *series8 = new QScatter3DSeries;
	series8->setItemLabelFormat(QStringLiteral("Eight- X:@xLabel,Y:@yLabel,Z:@zLabel"));
	series8->setMesh(QAbstract3DSeries::MeshBevelBar);
	series8->setItemSize(0.05f);
	m_graph->addSeries(series8);

	QScatter3DSeries *series9 = new QScatter3DSeries;
	series9->setItemLabelFormat(QStringLiteral("Nine- X:@xLabel,Y:@yLabel,Z:@zLabel"));
	series9->setMesh(QAbstract3DSeries::MeshArrow);
	series9->setItemSize(0.05f);
	m_graph->addSeries(series9);

	QScatter3DSeries *series10 = new QScatter3DSeries;
	series10->setItemLabelFormat(QStringLiteral("Ten- X:@xLabel,Y:@yLabel,Z:@zLabel"));
	series10->setMesh(QAbstract3DSeries::MeshPyramid);
	series10->setItemSize(0.05f);
	m_graph->addSeries(series10);


	QScatter3DSeries *series11 = new QScatter3DSeries;
	series11->setItemLabelFormat(QStringLiteral("Eleven- X:@xLabel,Y:@yLabel,Z:@zLabel"));
	series11->setMesh(QAbstract3DSeries::MeshCube);
	series11->setItemSize(0.05f);
	m_graph->addSeries(series11);

	QScatter3DSeries *series12 = new QScatter3DSeries;
	series12->setItemLabelFormat(QStringLiteral("Twelve- X:@xLabel,Y:@yLabel,Z:@zLabel"));
	series12->setMesh(QAbstract3DSeries::MeshPoint);
	series12->setItemSize(0.15f);
	m_graph->addSeries(series12);

	QScatter3DSeries *series13 = new QScatter3DSeries;
	series13->setItemLabelFormat(QStringLiteral("Thirteen- X:@xLabel,Y:@yLabel,Z:@zLabel"));
	series13->setMesh(QAbstract3DSeries::MeshPoint);
	series13->setItemSize(0.10f);
	m_graph->addSeries(series13);

	QScatter3DSeries *series14 = new QScatter3DSeries;
	series14->setItemLabelFormat(QStringLiteral("Fourteen- X:@xLabel,Y:@yLabel,Z:@zLabel"));
	series14->setMesh(QAbstract3DSeries::MeshSphere);
	series14->setItemSize(0.12f);
	m_graph->addSeries(series14);


	//以下为从data.txt中读取数据
	QVector<QVector3D> itemList1;
	QVector<QVector3D> itemList2;
	QVector<QVector3D> itemList3;
	QVector<QVector3D> itemList4;
	QVector<QVector3D> itemList5;
	QVector<QVector3D> itemList6;
	QVector<QVector3D> itemList7;
	QVector<QVector3D> itemList8;
	QVector<QVector3D> itemList9;
	QVector<QVector3D> itemList10;
	QVector<QVector3D> itemList11;
	QVector<QVector3D> itemList12;
	QVector<QVector3D> itemList13;
	QVector<QVector3D> itemList14;

	//Read data items from the file to QVector
	QTextStream stream;
	QFile dataFile("./data.txt");
	if (dataFile.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		stream.setDevice(&dataFile);
		while (!stream.atEnd())
		{
			QString line = stream.readLine();
			if (line.startsWith("//")) //Ignore comments
			{
				continue;
			}
			if (line.startsWith("1"))
			{
				QStringList strList1 = line.split(",", QString::SkipEmptyParts);
				if (strList1.size() < 4)
				{
					qWarning() << "Invalid row read from data:" << line;
					continue;
				}
				itemList1.append(QVector3D(
					strList1.at(1).trimmed().toFloat(),
					strList1.at(2).trimmed().toFloat(),
					strList1.at(3).trimmed().toFloat()));
			}

			if (line.startsWith("2"))
			{
				QStringList strList2 = line.split(",", QString::SkipEmptyParts);
				if (strList2.size() < 4)
				{
					qWarning() << "Invalid row read from data:" << line;
					continue;
				}
				itemList2.append(QVector3D(
					strList2.at(1).trimmed().toFloat(),
					strList2.at(2).trimmed().toFloat(),
					strList2.at(3).trimmed().toFloat()));
			}
			if (line.startsWith("3"))
			{
				QStringList strList3 = line.split(",", QString::SkipEmptyParts);
				if (strList3.size() < 4)
				{
					qWarning() << "Invalid row read from data:" << line;
					continue;
				}
				itemList3.append(QVector3D(
					strList3.at(1).trimmed().toFloat(),
					strList3.at(2).trimmed().toFloat(),
					strList3.at(3).trimmed().toFloat()));
			}
			if (line.startsWith("4"))
			{
				QStringList strList4 = line.split(",", QString::SkipEmptyParts);
				if (strList4.size() < 4)
				{
					qWarning() << "Invalid row read from data:" << line;
					continue;
				}
				itemList4.append(QVector3D(
					strList4.at(1).trimmed().toFloat(),
					strList4.at(2).trimmed().toFloat(),
					strList4.at(3).trimmed().toFloat()));
			}
			if (line.startsWith("5"))
			{
				QStringList strList5 = line.split(",", QString::SkipEmptyParts);
				if (strList5.size() < 4)
				{
					qWarning() << "Invalid row read from data:" << line;
					continue;
				}
				itemList5.append(QVector3D(
					strList5.at(1).trimmed().toFloat(),
					strList5.at(2).trimmed().toFloat(),
					strList5.at(3).trimmed().toFloat()));
			}
			if (line.startsWith("6"))
			{
				QStringList strList6 = line.split(",", QString::SkipEmptyParts);
				if (strList6.size() < 4)
				{
					qWarning() << "Invalid row read from data:" << line;
					continue;
				}
				itemList6.append(QVector3D(
					strList6.at(1).trimmed().toFloat(),
					strList6.at(2).trimmed().toFloat(),
					strList6.at(3).trimmed().toFloat()));
			}
			if (line.startsWith("7"))
			{
				QStringList strList7 = line.split(",", QString::SkipEmptyParts);
				if (strList7.size() < 4)
				{
					qWarning() << "Invalid row read from data:" << line;
					continue;
				}
				itemList7.append(QVector3D(
					strList7.at(1).trimmed().toFloat(),
					strList7.at(2).trimmed().toFloat(),
					strList7.at(3).trimmed().toFloat()));
			}
			if (line.startsWith("8"))
			{
				QStringList strList8 = line.split(",", QString::SkipEmptyParts);
				if (strList8.size() < 4)
				{
					qWarning() << "Invalid row read from data:" << line;
					continue;
				}
				itemList8.append(QVector3D(
					strList8.at(1).trimmed().toFloat(),
					strList8.at(2).trimmed().toFloat(),
					strList8.at(3).trimmed().toFloat()));
			}
			if (line.startsWith("9"))
			{
				QStringList strList9 = line.split(",", QString::SkipEmptyParts);
				if (strList9.size() < 4)
				{
					qWarning() << "Invalid row read from data:" << line;
					continue;
				}
				itemList9.append(QVector3D(
					strList9.at(1).trimmed().toFloat(),
					strList9.at(2).trimmed().toFloat(),
					strList9.at(3).trimmed().toFloat()));
			}
			if (line.startsWith("a"))
			{
				QStringList strList10 = line.split(",", QString::SkipEmptyParts);
				if (strList10.size() < 4)
				{
					qWarning() << "Invalid row read from data:" << line;
					continue;
				}
				itemList10.append(QVector3D(
					strList10.at(1).trimmed().toFloat(),
					strList10.at(2).trimmed().toFloat(),
					strList10.at(3).trimmed().toFloat()));
			}
			if (line.startsWith("b"))
			{
				QStringList strList11 = line.split(",", QString::SkipEmptyParts);
				if (strList11.size() < 4)
				{
					qWarning() << "Invalid row read from data:" << line;
					continue;
				}
				itemList11.append(QVector3D(
					strList11.at(1).trimmed().toFloat(),
					strList11.at(2).trimmed().toFloat(),
					strList11.at(3).trimmed().toFloat()));
			}
			if (line.startsWith("c"))
			{
				QStringList strList12 = line.split(",", QString::SkipEmptyParts);
				if (strList12.size() < 4)
				{
					qWarning() << "Invalid row read from data:" << line;
					continue;
				}
				itemList12.append(QVector3D(
					strList12.at(1).trimmed().toFloat(),
					strList12.at(2).trimmed().toFloat(),
					strList12.at(3).trimmed().toFloat()));
			}
			if (line.startsWith("d"))
			{
				QStringList strList13 = line.split(",", QString::SkipEmptyParts);
				if (strList13.size() < 4)
				{
					qWarning() << "Invalid row read from data:" << line;
					continue;
				}
				itemList13.append(QVector3D(
					strList13.at(1).trimmed().toFloat(),
					strList13.at(2).trimmed().toFloat(),
					strList13.at(3).trimmed().toFloat()));
			}
			if (line.startsWith("e"))
			{
				QStringList strList14 = line.split(",", QString::SkipEmptyParts);
				if (strList14.size() < 4)
				{
					qWarning() << "Invalid row read from data:" << line;
					continue;
				}
				itemList14.append(QVector3D(
					strList14.at(1).trimmed().toFloat(),
					strList14.at(2).trimmed().toFloat(),
					strList14.at(3).trimmed().toFloat()));
			}
		}

	}
	else	{
		qWarning() << "Unable to open data file:" << dataFile.fileName();
	}
	if (itemList1.isEmpty())
	{
		qWarning() << "No data received" << endl;
		return ;
	}
	
	QScatterDataArray *dataArray = new QScatterDataArray;
	dataArray->resize(itemList1.size());
	QScatterDataItem *ptrToDataArray = &dataArray->first();
	for (int i = 0; i < itemList1.size(); i++) {
		ptrToDataArray->setPosition(itemList1.at(i));
		ptrToDataArray++;
	}
	m_graph->seriesList().at(0)->dataProxy()->resetArray(dataArray);

	if (!itemList2.isEmpty())
	{
		QScatterDataArray *dataArray2 = new QScatterDataArray;
		dataArray2->resize(itemList2.size());
		ptrToDataArray = &dataArray2->first();
		for (int i = 0; i < itemList2.size(); i++) {
			ptrToDataArray->setPosition(itemList2.at(i));
			ptrToDataArray++;
		}
		m_graph->seriesList().at(1)->dataProxy()->resetArray(dataArray2);
	}

	if (!itemList3.isEmpty())
	{
		QScatterDataArray *dataArray3 = new QScatterDataArray;
		dataArray3->resize(itemList3.size());
		ptrToDataArray = &dataArray3->first();
		for (int i = 0; i < itemList3.size(); i++) {
			ptrToDataArray->setPosition(itemList3.at(i));
			ptrToDataArray++;
		}
		m_graph->seriesList().at(2)->dataProxy()->resetArray(dataArray3);
	}

	if (!itemList4.isEmpty())
	{
		QScatterDataArray *dataArray4 = new QScatterDataArray;
		dataArray4->resize(itemList4.size());
		ptrToDataArray = &dataArray4->first();
		for (int i = 0; i < itemList4.size(); i++) {
			ptrToDataArray->setPosition(itemList4.at(i));
			ptrToDataArray++;
		}
		m_graph->seriesList().at(3)->dataProxy()->resetArray(dataArray4);
	}

	if (!itemList5.isEmpty())
	{
		QScatterDataArray *dataArray5 = new QScatterDataArray;
		dataArray5->resize(itemList5.size());
		ptrToDataArray = &dataArray5->first();
		for (int i = 0; i < itemList5.size(); i++) {
			ptrToDataArray->setPosition(itemList5.at(i));
			ptrToDataArray++;
		}
		m_graph->seriesList().at(4)->dataProxy()->resetArray(dataArray5);
	}
	
	if (!itemList6.isEmpty())
	{
		QScatterDataArray *dataArray6 = new QScatterDataArray;
		dataArray6->resize(itemList6.size());
		ptrToDataArray = &dataArray6->first();
		for (int i = 0; i < itemList6.size(); i++) {
			ptrToDataArray->setPosition(itemList6.at(i));
			ptrToDataArray++;
		}
		m_graph->seriesList().at(5)->dataProxy()->resetArray(dataArray6);
	}

	if (!itemList7.isEmpty())
	{
		QScatterDataArray *dataArray7 = new QScatterDataArray;
		dataArray7->resize(itemList7.size());
		ptrToDataArray = &dataArray7->first();
		for (int i = 0; i < itemList7.size(); i++) {
			ptrToDataArray->setPosition(itemList7.at(i));
			ptrToDataArray++;
		}
		m_graph->seriesList().at(6)->dataProxy()->resetArray(dataArray7);
	}

	if (!itemList8.isEmpty())
	{
		QScatterDataArray *dataArray8 = new QScatterDataArray;
		dataArray8->resize(itemList8.size());
		ptrToDataArray = &dataArray8->first();
		for (int i = 0; i < itemList8.size(); i++) {
			ptrToDataArray->setPosition(itemList8.at(i));
			ptrToDataArray++;
		}
		m_graph->seriesList().at(7)->dataProxy()->resetArray(dataArray8);
	}

	if (!itemList9.isEmpty())
	{
		QScatterDataArray *dataArray9 = new QScatterDataArray;
		dataArray9->resize(itemList9.size());
		ptrToDataArray = &dataArray9->first();
		for (int i = 0; i < itemList9.size(); i++) {
			ptrToDataArray->setPosition(itemList9.at(i));
			ptrToDataArray++;
		}
		m_graph->seriesList().at(8)->dataProxy()->resetArray(dataArray9);
	}

	if (!itemList10.isEmpty())
	{
		QScatterDataArray *dataArray10 = new QScatterDataArray;
		dataArray10->resize(itemList10.size());
		ptrToDataArray = &dataArray10->first();
		for (int i = 0; i < itemList10.size(); i++) {
			ptrToDataArray->setPosition(itemList10.at(i));
			ptrToDataArray++;
		}
		m_graph->seriesList().at(9)->dataProxy()->resetArray(dataArray10);

	}

	if (!itemList11.isEmpty())
	{
		QScatterDataArray *dataArray11 = new QScatterDataArray;
		dataArray11->resize(itemList11.size());
		ptrToDataArray = &dataArray11->first();
		for (int i = 0; i < itemList11.size(); i++) {
			ptrToDataArray->setPosition(itemList11.at(i));
			ptrToDataArray++;
		}
		m_graph->seriesList().at(10)->dataProxy()->resetArray(dataArray11);
	}

	if (!itemList12.isEmpty())
	{
		QScatterDataArray *dataArray12 = new QScatterDataArray;
		dataArray12->resize(itemList12.size());
		ptrToDataArray = &dataArray12->first();
		for (int i = 0; i < itemList12.size(); i++) {
			ptrToDataArray->setPosition(itemList12.at(i));
			ptrToDataArray++;
		}
		m_graph->seriesList().at(11)->dataProxy()->resetArray(dataArray12);
	}


	if (!itemList13.isEmpty())
	{
		QScatterDataArray *dataArray13 = new QScatterDataArray;
		dataArray13->resize(itemList13.size());
		ptrToDataArray = &dataArray13->first();
		for (int i = 0; i < itemList13.size(); i++) {
			ptrToDataArray->setPosition(itemList13.at(i));
			ptrToDataArray++;
		}
		m_graph->seriesList().at(12)->dataProxy()->resetArray(dataArray13);
	}

	if (!itemList14.isEmpty())
	{
		QScatterDataArray *dataArray14 = new QScatterDataArray;
		dataArray14->resize(itemList14.size());
		ptrToDataArray = &dataArray14->first();
		for (int i = 0; i < itemList14.size(); i++) {
			ptrToDataArray->setPosition(itemList14.at(i));
			ptrToDataArray++;
		}
		m_graph->seriesList().at(13)->dataProxy()->resetArray(dataArray14);
	}
}

QVector3D Data::randVector()
{
	return QVector3D(
		(float)(rand() % 100) / 2.0f - (float)(rand() % 100) / 2.0f,
		(float)(rand() % 100) / 2.0f - (float)(rand() % 100) / 2.0f,
		(float)(rand() % 100) / 2.0f - (float)(rand() % 100) / 2.0f);
}
