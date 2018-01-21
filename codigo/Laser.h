/*
 * laser.h
 *
 *  Created on: 16 ene. 2018
 *      Author: ray
 */

#ifndef CODIGO_LASER_H_
#define CODIGO_LASER_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/LaserScan.h>
#include <gazebo/gazebo.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
namespace gazebo {
	class Laser: public RayPlugin {
	public:
		Laser();
		virtual ~Laser();
		void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
		std::unique_ptr<ros::NodeHandle> nodo;
		ros::Subscriber subscriber;
		ros::Publisher publisher;
		ros::CallbackQueue cola;
		ros::CallbackQueue cola2;
		std::thread threadColas;


	private:
		void onScan(ConstLaserScanStampedPtr &_msg);
		void conexion();
		void desconexion();
		void configurar(sensor_msgs::LaserScan::ConstPtr &_msg);
		sensors::RaySensorPtr sensorPtr;
		int conectados;
		std::string topic="";
		std::string nombreMundo;
		//std::string frame;
		//physics::WorldPtr mundo;

	 	gazebo::transport::NodePtr nodoLaser_;
		gazebo::transport::SubscriberPtr subcriptorLaser;
	};

} /* namespace gazebo */

#endif /* CODIGO_LASER_H_ */
