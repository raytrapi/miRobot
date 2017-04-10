#include "miRobot.h"
#include "listner.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <map>

namespace gazebo{
	 void MiRobot::Load (physics::ModelPtr _model,sdf::ElementPtr _sdf){
		 this->modelo=_model;
		 this->sdf=_sdf;
		 cargarUniones();
		 gzdbg<<"Hola Mundo\r\n";
		 listner=new Listner();
		 this->estado="inciado";
		 listner->init(this);

		 this->conexionUpdate=event::Events::ConnectWorldUpdateBegin(boost::bind(&MiRobot::OnUpdate,this,_1));

	 }

	 void MiRobot::OnUpdate(const common::UpdateInfo & _info){
		 common::Time actual=modelo->GetWorld()->GetSimTime();
		 this->estado=" Elementos \r\n";
		 //Buscar los elementos de unión
		 for(auto it=uniones.begin();it!=uniones.end();it++){
			 Union *_union=&it->second;
			 this->estado+=it->first+": A:"+std::to_string(_union->joint->GetAngle(0).Radian());
			 this->estado+=" V:"+std::to_string(_union->joint->GetVelocity(0));
			 this->estado+="\r\n";
			 if(_union->moviendo){
				 common::Time ticActual=actual-_union->tiempoActual;
				 double error=_union->joint->GetAngle(0).Radian()-_union->anguloFinal;

				 double absError=(error>0)?error:-1*error;
				 double velocidad=0;
				 ///if(absError>0.001){
					 velocidad=_union->pid.Update(error, ticActual);

				 /*}else{
					 _union->moviendo=false;
				 }*/
				 _union->velocidad=velocidad;
			 }

			 _union->joint->SetVelocity(0,_union->velocidad);
		 }
	 }
	 void MiRobot::mover(std::string laUnion, double valor){
		 Union *_union=&uniones[laUnion];
		 if(_union){
			 _union->anguloFinal=valor;
			 _union->tiempoActual=0;
			 _union->moviendo=true;
		 }
	 }
	 void MiRobot::parametrizar(std::string laUnion,std::string tipo, double valor){
		 Union *_union=&uniones[laUnion];
		 if(_union){
			 if(tipo=="V"){
				 _union->velocidad=valor;
				 _union->moviendo=false;
			 }else if(tipo=="VM"){
				 _union->pid.SetCmdMax(valor);
				 _union->pid.SetCmdMin(-1*valor);
			 }

		 }
	 }
	 void MiRobot::pintar(std::string tipo){
		 if(tipo=="uniones"){
			 this->estado=" Elementos \r\n";
			 //Buscar los elementos de unión
			 std::map<std::string,physics::BasePtr> uniones=buscar(this->modelo,physics::Entity::EntityType::JOINT);
			 for(auto it=uniones.begin();it!=uniones.end();it++){
				 this->estado+=it->first+"\r\n";
			 }
		 }
	 }
	 std::map<std::string,physics::BasePtr> MiRobot::buscar(physics::BasePtr contenedor, const physics::Entity::EntityType & t){
		 unsigned int n=contenedor->GetChildCount();
		 std::map<std::string,physics::BasePtr> resultado;
		 for(int i=0;i<n; i++){
			 if(contenedor->GetChild(i)->HasType(t)){
				 resultado[contenedor->GetChild(i)->GetName()]=contenedor->GetChild(i);
			 }
			 std::map<std::string,physics::BasePtr> resultado2=buscar(contenedor->GetChild(i),t);
			 resultado.insert(resultado2.begin(),resultado2.end());
		 }
		 return resultado;
	 }
	 void MiRobot::cargarUniones(){
		 std::map<std::string,physics::BasePtr> uniones=buscar(this->modelo,physics::Entity::EntityType::JOINT);
		 for(auto it=uniones.begin();it!=uniones.end();it++){
			 Union _union;
			 _union.joint=boost::static_pointer_cast<physics::Joint>(it->second);
			 _union.pid.Init(2, 1, 1, 0, 0, _union.joint->GetVelocityLimit(0), -1*_union.joint->GetVelocityLimit(0));
			 _union.tiempoActual=0;
			 this->uniones[it->first]=_union;
		 }
	 }

	GZ_REGISTER_MODEL_PLUGIN(MiRobot);
}
