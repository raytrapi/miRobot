/*
 * Comandos.h
 *
 *  Created on: 12 mar. 2017
 *      Author: ray
 */
#ifndef COMANDOS_H_
#define COMANDOS_H_
#include <stdio.h>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <unistd.h>
#include "std_msgs/String.h"

#include "../src/miRobot.h"

namespace gazebo {

	class Comandos {

		public:

			static bool procesar(std::string &comando, MiRobot *);
			static std::string ruta;
	};

}

#endif
