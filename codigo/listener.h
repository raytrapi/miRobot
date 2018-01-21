#ifndef LISTNER
#define LISTNER

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <thread>
#include "miRobot.h"


namespace gazebo{
	class MiRobot;
	class Listener{
		private:
			std::unique_ptr<ros::NodeHandle> nodo;
			ros::Subscriber subscriber;
			ros::Publisher publisher;
			ros::CallbackQueue cola;
			ros::CallbackQueue cola2;
			std::thread threadColas;
			//static int contadorConexiones;
			MiRobot * robot;

		public:
			void init(MiRobot *,const std::string);
			void listener(const std_msgs::String::ConstPtr& msg);
			void conexion();
			void desconexion();
			void thread();
	};
}
#endif
