#ifndef MIROBOT
#define MIROBOT
#include "listner.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
namespace gazebo{
    class Listner;
	class MiRobot: public ModelPlugin{
		Listner *listner;
		physics::ModelPtr modelo;
		sdf::ElementPtr sdf;

		std::string estado;
		void buscar(physics::BasePtr contenedor, const physics::Entity::EntityType &);
		public:
		void Load (physics::ModelPtr _model,sdf::ElementPtr _sdf);
		const std::string& getEstado() const {
			return estado;
		}
		void pintar(std::string);
};
}
#endif
