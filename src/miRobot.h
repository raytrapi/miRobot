#ifndef MIROBOT
#define MIROBOT
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Vector3.hh>
#include <map>
#include <vector>
#include <iostream>
#include <fstream>

#include "../src/listener.h"
std::vector<std::string> split(const std::string &c, char d);
std::string trim(const std::string &);
namespace gazebo{
   struct Vector_Fuerza{
	   double tiempo;
	   common::Time ultimaRevision;
	   std::vector<double> fuerzas;
   };
   class _Sensor;
   struct Union{
	   enum TIPOMOVIMIENTO {NINGUNO=0, VELOCIDAD=1, FUERZA=2} ;
	   physics::JointPtr joint;
	   common::PID pidVelocidad;
	   //common::PID pidFuerza;

	   common::Time tiempoActual;

	   double velocidad;
	   double anguloFinal;
	   //double fuerza;
	   //double fuerzaFinal;
	   Vector_Fuerza fuerzas;

	   bool moviendo=false;


	   TIPOMOVIMIENTO tipoMovimiento=NINGUNO;
   };

    class Listener;
	class MiRobot: public ModelPlugin{
			Listener *listener;
			physics::ModelPtr modelo;
			sdf::ElementPtr sdf;
			std::map<std::string,Union> uniones;
			std::map<std::string,_Sensor *> sensores;
			event::ConnectionPtr conexionUpdate;
			std::string estado;

			bool frecuencia=5000;
			common::Time siguienteVisualizacion;
			common::Time tiempoSensorAnterior;
			std::map<std::string,physics::BasePtr> buscar(physics::BasePtr contenedor, const physics::Entity::EntityType &);
		public:
			void Load (physics::ModelPtr _model,sdf::ElementPtr _sdf);
			void OnUpdate(const common::UpdateInfo & _info);
			const std::string& getEstado() const {
				return estado;
			}
			void mover(std::string laUnion, double valor);
			void parametrizar(std::string laUnion,std::string tipo, double valor,std::vector<std::string>&);
			void pintar(std::string);
			void cargarUniones();
			void cargarSensores();
			void leerSensores(common::Time);
			std::string ultimoComando="";
			bool mostrar=false;
			~MiRobot();
	};
	class _Sensor{
			private:
				std::ofstream fichero;
				common::Time ultimoTiempo=0;
				bool abierto=false;
				double frecuencia;
			public:
				gazebo::sensors::SensorPtr sensor;
				_Sensor(std::string ruta, gazebo::sensors::SensorPtr sensor):sensor(sensor){
					fichero.open(ruta.c_str());
					abierto=true;
					frecuencia=1/sensor->UpdateRate();
				}
				~_Sensor(){
					if(abierto){
						fichero.close();
					}
				}
				void log(common::Time tiempo, std::string log){
					if((tiempo-ultimoTiempo)>frecuencia){
						ultimoTiempo=tiempo;
						fichero<<tiempo.Double()<<",";
						fichero<<log;
						fichero<<"\n";
					}
				}
	};
}
#endif
