#ifndef MIROBOT
#define MIROBOT
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <map>

#include "../src/listener.h"
std::vector<std::string> split(const std::string &c, char d);
namespace gazebo{

   struct Union{
	   enum TIPOMOVIMIENTO {NINGUNO=0, VELOCIDAD=1, FUERZA=2} ;
	   physics::JointPtr joint;
	   common::PID pidVelocidad;
	   //common::PID pidFuerza;

	   common::Time tiempoActual;

	   double velocidad;
	   double anguloFinal;
	   double fuerza;
	   //double fuerzaFinal;

	   bool moviendo=false;


	   TIPOMOVIMIENTO tipoMovimiento=NINGUNO;
   };
    class Listener;
	class MiRobot: public ModelPlugin{
			Listener *listener;
			physics::ModelPtr modelo;
			sdf::ElementPtr sdf;
			std::map<std::string,Union> uniones;
			event::ConnectionPtr conexionUpdate;
			std::string estado;

			bool frecuencia=5000;
			common::Time siguienteVisualizacion;
			std::map<std::string,physics::BasePtr> buscar(physics::BasePtr contenedor, const physics::Entity::EntityType &);
		public:
			void Load (physics::ModelPtr _model,sdf::ElementPtr _sdf);
			void OnUpdate(const common::UpdateInfo & _info);
			const std::string& getEstado() const {
				return estado;
			}
			void mover(std::string laUnion, double valor);
			void parametrizar(std::string laUnion,std::string tipo, double valor);
			void pintar(std::string);
			void cargarUniones();
			std::string ultimoComando="";
			bool mostrar=false;
	};
}
#endif
