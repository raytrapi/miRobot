/*
 * camara.h
 *
 *  Created on: 10 feb. 2018
 *      Author: ray
 */

#ifndef SRC_CAMARA_H_
#define SRC_CAMARA_H_

#include <opencv2/opencv.hpp>
#include <gazebo/gazebo.hh>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
//#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <ignition/math4/ignition/math.hh>

#include <iostream>
#include <stdio.h>
#include <thread>
namespace ray {
	class Camara {
		public:
			Camara(int);
			~Camara();
			cv::Mat obtenerImagen();
			sensor_msgs::CameraInfo getInfoCamara(gazebo::common::Time& ahora);
			sensor_msgs::Image getImgCamara(gazebo::common::Time& ahora);
		private:
			cv::VideoCapture cap;
			int dispositivo;
			sensor_msgs::CameraInfo msgInfoCamara;
			sensor_msgs::Image msgImagen;
			//std::string formatoImagen;
	};
	
}

#endif /* SRC_CAMARA_H_ */
