#ifndef LISTNER
#define LISTNER

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <thread>


namespace gazebo{
	class Listner{
		private:
			std::unique_ptr<ros::NodeHandle> nodo;
			ros::Subscriber subscriber;
			ros::Publisher publisher;
			ros::CallbackQueue cola;
			ros::CallbackQueue cola2;
			std::thread threadColas;
			//static int contadorConexiones;

		public:
			void init();
			void listener(const std_msgs::String::ConstPtr& msg);
			static void conexion(const ros::SingleSubscriberPublisher&);
			static void desconexion(const ros::SingleSubscriberPublisher&);
			void thread();
	};
}
#endif
