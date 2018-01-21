/*
 * laser.cpp
 *
 *  Created on: 16 ene. 2018
 *      Author: ray
 */

#include "Laser.h"

namespace gazebo {

	Laser::Laser() {

	}

	Laser::~Laser() {
	}

	void Laser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
		this->nombreMundo = _parent->WorldName();
		//this->mundo = physics::get_world(this->nombreMundo);
		if(!ros::isInitialized()){
			int argc=0;
			char **argv=NULL;
			ros::init(argc,argv, "ray_laser", ros::init_options::NoSigintHandler);
		}
		using boost::dynamic_pointer_cast;
		this->sensorPtr=dynamic_pointer_cast<sensors::RaySensor>(_parent);
		this->topic="/"+((_sdf->HasElement("topic"))?_sdf->GetElement("topic")->GetValue()->GetAsString():_parent->Name());
		this->nodo.reset(new ros::NodeHandle("ray_laser"));


		ros::AdvertiseOptions ad=ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
						this->topic+"_pub",
						1,
						boost::bind(&Laser::conexion, this),
						boost::bind(&Laser::desconexion, this),
						ros::VoidPtr(),
						&this->cola2
						);
		this->publisher=this->nodo->advertise(ad);
		this->nodoLaser_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
				this->nodoLaser_->Init(this->nombreMundo);
		this->sensorPtr->SetActive(false);
		this->subcriptorLaser=this->nodoLaser_->Subscribe(this->sensorPtr->Topic(),&Laser::onScan, this);

	}

	void Laser::onScan(ConstLaserScanStampedPtr &_msg){

	  sensor_msgs::LaserScan laser_msg;
	  laser_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
	  laser_msg.header.frame_id = "laser";
	  laser_msg.angle_min = _msg->scan().angle_min();
	  laser_msg.angle_max = _msg->scan().angle_max();
	  laser_msg.angle_increment = _msg->scan().angle_step();
	  laser_msg.time_increment = 0;
	  laser_msg.scan_time = 0;
	  laser_msg.range_min = _msg->scan().range_min();
	  laser_msg.range_max = _msg->scan().range_max();
	  laser_msg.ranges.resize(_msg->scan().ranges_size());
	  std::copy(_msg->scan().ranges().begin(),
	            _msg->scan().ranges().end(),
	            laser_msg.ranges.begin());
	  laser_msg.intensities.resize(_msg->scan().intensities_size());
	  std::copy(_msg->scan().intensities().begin(),
	            _msg->scan().intensities().end(),
	            laser_msg.intensities.begin());
	  this->publisher.publish(laser_msg);
	}



	void Laser::conexion() {
		this->conectados++;
		gzdbg()<<this->conectados;
		if(this->conectados==1){
			this->subcriptorLaser=this->nodoLaser_->Subscribe(this->sensorPtr->Topic(),&Laser::onScan, this);
		}
	}

	void Laser::desconexion() {
		this->conectados--;
		gzdbg()<<this->conectados;
		if (this->conectados == 0)
			this->subcriptorLaser.reset();
	}

	void Laser::configurar(sensor_msgs::LaserScan::ConstPtr& _msg) {
	}

GZ_REGISTER_SENSOR_PLUGIN(Laser);



} /* namespace gazebo */
