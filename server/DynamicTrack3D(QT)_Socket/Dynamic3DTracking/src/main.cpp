#include "dynamic3dtracking.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);    
	Dynamic3DTracking w;
	w.show();
	return a.exec();
}
