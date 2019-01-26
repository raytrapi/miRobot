/*
 * Camara.h
 *
 *  Created on: 6 feb. 2018
 *      Author: ray
 */

#ifndef SRC_SENSORES_CAMARA_H_
#define SRC_SENSORES_CAMARA_H_
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>


#include <image_transport/image_transport.h>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo/rendering/rendering.hh> //Para distorsion

#include <ignition/math4/ignition/math.hh>

#include <gazebo_plugins/GazeboRosCameraConfig.h>
#include <dynamic_reconfigure/server.h>

#include <thread>
#include <iostream>

namespace gazebo {

class Camara: public CameraPlugin {
	public:
		Camara();
		virtual ~Camara();
		void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

		ros::Subscriber subscriber;

		ros::CallbackQueue cola;
		ros::CallbackQueue cola2;
		std::thread threadColas;
	protected:
		virtual void OnNewFrame(const unsigned char *imagen,
						unsigned int alto, unsigned int ancho,
						unsigned int profundidad, const std::string &formato);
	private:


		void conexion();
		void desconexion();

		void thread();
		sensors::CameraSensorPtr sensorPtr;
		int conectados;
		std::string topic="";
		std::string nombreMundo;

		//gazebo::transport::NodePtr nodoSensor;
		//gazebo::transport::SubscriberPtr subcriptorSensor;
		std::unique_ptr<ros::NodeHandle> nodo;
		image_transport::ImageTransport* nodoImagen;
		ros::Publisher pubDatosCamara;
		image_transport::Publisher pubImagen;
		sensor_msgs::Image msgImagen;
		dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig> *servirImagen;
		void configCallback(gazebo_plugins::GazeboRosCameraConfig &config, uint32_t level);
		double frecuencia=0;
		common::Time frameAnterior;
		common::Time imagenAnterior;
		sensor_msgs::CameraInfo infoCamara;
		void enviarImagen(const unsigned char *fuente,	common::Time &anterior);
		void enviarInformacionCamara(common::Time &anterior);
		std::string formatoImagen;

};

}

#endif /* SRC_SENSORES_CAMARA_H_ */
