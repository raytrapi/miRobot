/*
 * Comandos.cpp
 *
 *  Created on: 12 mar. 2017
 *      Author: ray
 */

#include "comandos.h"

#include <stdio.h>
#include "std_msgs/String.h"
#include <vector>

#include "../src/miRobot.h"
namespace gazebo {
	std::string Comandos::ruta="";
	bool Comandos::procesar(std::string &comando, MiRobot * robot ){
		std::vector<std::string> partes=split(comando, ' ');
		bool valido=true;
		bool mostrar=false;
		if(partes.size()==0){
			//gzerr<<"No se ha indicado un comando válido."<<"\r\n";
		}else{
			switch(partes[0][0]){
				case 'd':
					robot->pintar(comando);
					mostrar=!mostrar;
					if(mostrar){
						ROS_INFO("Mostrando");
					}else{
						ROS_INFO("No mostrando");
					}
					break;
				case 'p':
					//p union tipo valor
					//Parametriza la unión con el valor indicado V=Velocidad constante, VM=Velocidada con PID, F=Fuerza, FT= Fuezas por tiempo
					if(partes.size()>3){
						robot->parametrizar(partes[1],partes[2],std::stod(partes[3]),partes);
					}

					break;
				case 'm':
					//m union posicion_radianes
					//Mueve la unión hasta la posición en radianes indicada
					if(partes.size()>2){
						robot->mover(partes[1],std::stod(partes[2]));
					}

					break;
				case 'e':
					//e fichero
					//Ejecuta un fichero
					{
					if( ruta==""){
						//Aún no tenemos una ruta. La cogemos del fichero
						ROS_INFO("Estas en %s",boost::filesystem::initial_path().string().c_str());
						boost::filesystem::path p=boost::filesystem::complete(partes[1]);
						ruta=p.parent_path().string();
						ROS_INFO("Ruta fija %s",ruta.c_str());
						ROS_INFO("Fichero: %s",partes[1].c_str());
						/*ROS_INFO("Ruta 1: %s", p.relative_path().string().c_str());
						ROS_INFO("Ruta 2: %s", p.parent_path().string().c_str());
						ROS_INFO("Ruta 3: %s", p.root_path().string().c_str());/**/
					}else{
						//Como tenemos una ruta se la añadimos al contenido
						partes[1]=ruta+"/"+partes[1];
					}/**/


					std::ifstream stream(partes[1]);
					if(stream.good()){
						std::string linea;
						while(!stream.eof()){
							linea.clear();
							std::getline(stream, linea);
							ROS_INFO("Proceso %s",linea.c_str());
							Comandos::procesar(linea, robot);
						}

					}else{
						ROS_INFO("No se ha podido abrir el fichero %s",partes[1].c_str());

					}
					stream.close();
					}

					break;
				case 'w':
					//w [union]
					//Espera hasta que la unión halla llegado a su destino. Si no se indica la unión se espera hata terminar todas
					ROS_INFO("esperando");

					break;
				case 's':
					{
						//s milisegundos
						//Duerme los milisegundos indicados.

						//de momento nos dormimos durante el tiempo indicado.
						std::string tiempo(partes[1]);
						ROS_INFO("durmiendo %d",std::stoi(tiempo));
						boost::this_thread::sleep(boost::posix_time::milliseconds(std::stoi(tiempo)));
						//usleep(std::stoi(tiempo));
						ROS_INFO("despierto");
					}

					break;/**/
				default:
					valido=false;
					//break;
			}
		}
		if(valido){
			robot->ultimoComando=comando;
			robot->mostrar=mostrar;

		}
		return false;
	}

} /* namespace gazebo */

