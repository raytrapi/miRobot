/*
 * camara.cpp
 *
 *  Created on: 10 feb. 2018
 *      Author: ray
 */

#include "camara.h"

namespace ray {
	
	Camara::Camara(int id) {
		cap.open(id);
		dispositivo=0;

		msgInfoCamara.height=cap.get(cv::CAP_PROP_FRAME_HEIGHT);
		msgInfoCamara.width=cap.get(cv::CAP_PROP_FRAME_WIDTH);
		msgInfoCamara.distortion_model="plumb_bob";
		//gazebo::rendering::CameraPtr camara=this->sensorPtr->Camera();
		msgInfoCamara.D.resize(5);
		//gazebo::rendering::DistortionPtr distorsion=camara->LensDistortion();;
		//double hfov = camara->HFOV().Radian();
		ignition::math::Vector2d centro;
		/*if(distorsion!=NULL){
			centro=	distorsion->GetCenter();
			infoCamara.D[0]=distorsion->GetK1();
			infoCamara.D[1]=(double) distorsion->GetK2();
			infoCamara.D[2]=(double) distorsion->GetP1();
			infoCamara.D[3]=(double) distorsion->GetP2();
			infoCamara.D[4]=(double) distorsion->GetK3();
		}else{/**/
			msgInfoCamara.D[0]=0;
			msgInfoCamara.D[1]=0;
			msgInfoCamara.D[2]=0;
			msgInfoCamara.D[3]=0;
			msgInfoCamara.D[4]=0;
			centro.x=(msgInfoCamara.width /2.0);
			centro.y=(msgInfoCamara.height /2.0);
		//}

		/**/

		//std::cout<<camara->ImageFormat()<<"\r\n";
		//std::cout<<sensor_msgs::image_encodings::RGB8<<"\r\n";
		//formatoImagen="rgb8";
		double longitudFocal = cap.get(cv::CAP_PROP_FOCUS);
		if(longitudFocal==-1){
			//Calculamos a mano.
			//Mi cámara web es una SPC620NC y en sus especificaciones indica que la lente es de 32mmx32mmx16mm = Pi/2 de HFOV
			longitudFocal=(msgInfoCamara.width/2.0)*(1/tan(M_PI/4));
			//longitudFocal=(msgInfoCamara.width)*((2.0 * tan((M_PI/2) / 2.0)));
		}
		std::cout<<longitudFocal<<"\r\n";
		double x=centro.x;
		double y=centro.y;

		msgInfoCamara.K[0] = longitudFocal;
		msgInfoCamara.K[1] = 0.0;
		msgInfoCamara.K[2] = x;
		msgInfoCamara.K[3] = 0.0;
		msgInfoCamara.K[4] = longitudFocal;
		msgInfoCamara.K[5] = y;
		msgInfoCamara.K[6] = 0.0;
		msgInfoCamara.K[7] = 0.0;
		msgInfoCamara.K[8] = 1.0;
		  // rectification
		msgInfoCamara.R[0] = 1.0;
		msgInfoCamara.R[1] = 0.0;
		msgInfoCamara.R[2] = 0.0;
		msgInfoCamara.R[3] = 0.0;
		msgInfoCamara.R[4] = 1.0;
		msgInfoCamara.R[5] = 0.0;
		msgInfoCamara.R[6] = 0.0;
		msgInfoCamara.R[7] = 0.0;
		msgInfoCamara.R[8] = 1.0;
		  // camera_ projection matrix (same as camera_ matrix due
		  // to lack of distortion/rectification) (is this generated?)
		msgInfoCamara.P[0] = longitudFocal;
		msgInfoCamara.P[1] = 0.0;
		msgInfoCamara.P[2] = x;
		msgInfoCamara.P[3] = -longitudFocal * 0.07;
		msgInfoCamara.P[4] = 0.0;
		msgInfoCamara.P[5] = longitudFocal;
		msgInfoCamara.P[6] = y;
		msgInfoCamara.P[7] = 0.0;
		msgInfoCamara.P[8] = 0.0;
		msgInfoCamara.P[9] = 0.0;
		msgInfoCamara.P[10] = 1.0;
		msgInfoCamara.P[11] = 0.0;


		//Datos de imagen
		msgImagen.width=msgInfoCamara.width;
		msgImagen.height=msgInfoCamara.height;
		msgImagen.encoding="bgr8";
		msgImagen.is_bigendian=true;
		msgImagen.step=msgInfoCamara.width*3;
		msgImagen.data.resize(msgImagen.height*msgImagen.step);//Reservamos memoria
	}
	Camara::~Camara() {
		if(cap.isOpened()){
			cap.release();
		}
	}
	cv::Mat Camara::obtenerImagen() {
		if(!cap.isOpened()){

			throw 1;
		}
		cv::Mat frame;
		cap.read(frame);
		return frame;
	}

	sensor_msgs::CameraInfo Camara::getInfoCamara(gazebo::common::Time& ahora) {
		msgInfoCamara.header.frame_id = "camara";
		msgInfoCamara.header.stamp.sec = ahora.sec;
		msgInfoCamara.header.stamp.nsec = ahora.nsec;
		return msgInfoCamara;
	}

	sensor_msgs::Image Camara::getImgCamara(gazebo::common::Time& ahora) {
		msgImagen.header.frame_id = "camara";
		msgImagen.header.stamp.sec = ahora.sec;
		msgImagen.header.stamp.nsec = ahora.nsec;
		cv::Mat imagen=obtenerImagen();
		uchar * ptrImagen=imagen.data;
		uchar * ptrMsgImagen=(uchar *)(&msgImagen.data[0]);
		for(int i=0;i<msgImagen.height;i++){
			memcpy(ptrMsgImagen,ptrImagen,msgImagen.step);
			//Aumentamos las columnas
			ptrMsgImagen+=msgImagen.step;
			ptrImagen+=msgImagen.step;
		}
		return msgImagen;
	}

}


