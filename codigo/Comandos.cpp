/*
 * Comandos.cpp
 *
 *  Created on: 12 mar. 2017
 *      Author: ray
 */

#include "Comandos.h"
#include <stdio.h>
#include "std_msgs/String.h"
#include <vector>
#include "miRobot.h"
namespace gazebo {
	bool Comandos::procesar(std::string &comando, MiRobot * robot ){
		std::vector<std::string> partes=split(comando, ' ');

		if(partes.size()==0){
			gzerr<<"No se ha indicado un comando válido."<<"\r\n";
		}else{
			switch(partes[0][0]){
				case 'd':
					if(partes.size()>1){
						robot->pintar(partes[1]);
					}
					break;
			}
		}
		return false;
	}

} /* namespace gazebo */

std::vector<std::string> split(const std::string &c, char d){
	std::vector<std::string> resultado;

	std::stringstream cs(c);
	std::string parte;
	while(std::getline(cs, parte, d)){
		resultado.push_back(parte);
	}

	return resultado;
}
