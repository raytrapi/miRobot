#include "../src/listener.h"

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <gazebo/gazebo.hh>

#include "../src/Comandos.h"
namespace gazebo{
	void Listener::init(MiRobot * robot, const std::string topic){
		this->robot=robot;
		//Listener::contadorConexiones=0;
		if(!ros::isInitialized()){
			int argc=0;
			char **argv=NULL;
			ros::init(argc,argv, "gazebo_client", ros::init_options::NoSigintHandler);
		}

		this->nodo.reset(new ros::NodeHandle("gazebo_client"));

		ros::SubscribeOptions so=ros::SubscribeOptions::create<std_msgs::String>(
				topic,
				1,
				boost::bind(&Listener::listener, this, _1),
				ros::VoidPtr(),
				&this->cola
				);
		this->subscriber=this->nodo->subscribe(so);

		ros::AdvertiseOptions ad=ros::AdvertiseOptions::create<std_msgs::String>(
						topic+"_m",
						1,
						boost::bind(&Listener::conexion,this),
								boost::bind(&Listener::desconexion,this),
						ros::VoidPtr(),
						&this->cola2
						);
		this->publisher=this->nodo->advertise(ad);

		this->threadColas=std::thread(std::bind(&Listener::thread, this));
	}
	void Listener::listener(const std_msgs::String::ConstPtr& msg){
		std::string m=msg->data.c_str();
		Comandos::procesar(m, this->robot);
		//gzdbg<<m<<"\r\n";

	}

	void Listener::conexion(){
		ROS_INFO("Me conecto");
		gzdbg<<"Me conecto"<<"\r\n";
		//Listner::contadorConexiones++;
	}
	void Listener::desconexion(){
		ROS_INFO("Me desconecto");
		//Listner::contadorConexiones--;
	}
	void Listener::thread(){
		static const double timeout=0.01;
		while(this->nodo->ok()){
			this->cola.callAvailable(ros::WallDuration(timeout));
			///if(Listner::contadorConexiones>0){
				std_msgs::String m;
				std::stringstream ms;
				//ms<<"Hay conectados \r\n";
				ms<<this->robot->getEstado()<<"\r\n";
				m.data=ms.str();

				this->publisher.publish(m);
			//}
		}
	}
}
