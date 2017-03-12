#include "miRobot.h"
#include "listner.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <map>
namespace gazebo{
	 void MiRobot::Load (physics::ModelPtr _model,sdf::ElementPtr _sdf){
		 this->modelo=_model;
		 this->sdf=_sdf;
		 gzdbg<<"Hola Mundo\r\n";
		 listner=new Listner();
		 this->estado="inciado";
		 listner->init(this);
	 }
	 void MiRobot::pintar(std::string tipo){
		 if(tipo=="uniones"){
			 this->estado=" Elementos \r\n";
			 //Buscar los elementos de unión
			 buscar(this->modelo,physics::Entity::EntityType::JOINT);
		 }
	 }
	 void MiRobot::buscar(physics::BasePtr contenedor, const physics::Entity::EntityType & t){
		 unsigned int n=contenedor->GetChildCount();
		 for(int i=0;i<n; i++){
			 if(contenedor->GetChild(i)->HasType(t)){
				 this->estado+=contenedor->GetChild(i)->GetName()+"\r\n";
			 }
			 buscar(contenedor->GetChild(i),t);
		 }
	 }

	GZ_REGISTER_MODEL_PLUGIN(MiRobot);
}
