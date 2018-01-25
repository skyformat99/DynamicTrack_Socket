#ifndef DRAGGABLEAXES_SOCKET_H
#define DRAGGABLEAXES_SOCKET_H

#include <QtWidgets/QMainWindow>
#include "ui_draggableaxes_socket.h"

class Draggableaxes_Socket : public QMainWindow
{
	Q_OBJECT

public:
	Draggableaxes_Socket(QWidget *parent = 0);
	~Draggableaxes_Socket();

private:
	Ui::Draggableaxes_SocketClass ui;
};

#endif // DRAGGABLEAXES_SOCKET_H
