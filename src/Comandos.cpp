/*
 * Comandos.cpp
 *
 *  Created on: 12 mar. 2017
 *      Author: ray
 */

#include "../src/Comandos.h"

#include <stdio.h>
#include "std_msgs/String.h"
#include <vector>

#include "../src/miRobot.h"
namespace gazebo {
	bool Comandos::procesar(std::string &comando, MiRobot * robot ){
		std::vector<std::string> partes=split(comando, ' ');

		if(partes.size()==0){
			//gzerr<<"No se ha indicado un comando válido."<<"\r\n";
		}else{
			switch(partes[0][0]){
				case 'd':
					if(partes.size()>1){
						robot->pintar(partes[1]);
					}
					break;
				case 'p':
					//p union tipo valor
					if(partes.size()>3){
						robot->parametrizar(partes[1],partes[2],std::stod(partes[3]));
					}
					break;
				case 'm':
					//m union posicion_radianes
					if(partes.size()>2){
						robot->mover(partes[1],std::stod(partes[2]));
					}
					break;
				case 'e':
					//e fichero
					std::ifstream stream(partes[1]);
					if(stream.good()){
						std::string linea;
						while(!stream.eof()){
							linea.clear();
							std::getline(stream, linea);
							Comandos::procesar(linea, robot);
						}
						stream.close();
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
