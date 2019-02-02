#ifndef MAINDIALOG_H
#define MAINDIALOG_H

#include <iostream>
#include <QtWidgets/QDialog>
#include <QtWidgets/QPushButton>
#include "ros/ros.h"
#include "std_msgs/String.h"

class MainDialog : public QDialog{
	Q_OBJECT
	public:
		MainDialog(QWidget *parent = 0);
	public slots:
		void avanzar();
		void parar();
		void retroceder();

	private:
		QPushButton *bAvanzar;
		QPushButton *bRetroceder;
		ros::Publisher publicador;
		void enviarRos(std::string);
};


#endif
