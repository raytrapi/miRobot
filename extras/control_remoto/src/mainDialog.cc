//#include <QtGui/QtGui>

#include "mainDialog.h"

MainDialog::MainDialog(QWidget *parent): QDialog(parent){

	this->bAvanzar = new QPushButton(this);
	connect(bAvanzar, SIGNAL(pressed()), this, SLOT(avanzar()));
	connect(bAvanzar, SIGNAL(released()), this, SLOT(parar()));
	this->bAvanzar->setText(tr("avanzar"));
	this->bAvanzar->move(0, 0);
	this->bAvanzar->show();
	this->bRetroceder = new QPushButton(this);
		connect(bRetroceder, SIGNAL(pressed()), this, SLOT(retroceder()));
		connect(bRetroceder, SIGNAL(released()), this, SLOT(parar()));
		this->bRetroceder->setText(tr("retroceder"));
		this->bRetroceder->move(110, 0);
		this->bRetroceder->show();


	this->bGirar = new QPushButton(this);
	connect(bGirar, SIGNAL(keypress()), this, SLOT(girar()));
	this->bGirar->setText(tr("Girar tortuga"));
	this->bGirar->move(210, 0);
	this->bGirar->show();

	setWindowTitle(tr("Robotica"));
	//setFixedHeight(sizeHint().height());


	/**ZONA ROS**/
	if(!ros::isInitialized()){
		int argc=0;
		char **argv=NULL;
		ros::init(argc,argv, "MI_APP", ros::init_options::NoSigintHandler);
	};
	ros::NodeHandle n;
	publicador = n.advertise<std_msgs::String>("tortuga", 1000);
}

void MainDialog::avanzar(){
	enviarRos("e /home/ray/robot/src/miRobot/programas/avanzar");
}
void MainDialog::girar(){
	enviarRos("e /home/ray/robot/src/miRobot/programas/girarDerechaTortuga");
}
void MainDialog::parar(){
	enviarRos("e /home/ray/robot/src/miRobot/programas/parar");
}
void MainDialog::retroceder(){
	enviarRos("e /home/ray/robot/src/miRobot/programas/retroceder");
}
void MainDialog::enviarRos(std::string mensaje){
	//ROS_INFO("%s", mensaje);
	std_msgs::String msg;
	msg.data = mensaje;
	publicador.publish(msg);
}
