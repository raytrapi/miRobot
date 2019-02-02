#include "listener.h"
#include "../src/miRobot.h"

#include "../src/listener.h"

namespace gazebo{
	 void MiRobot::Load (physics::ModelPtr _model,sdf::ElementPtr _sdf){
		 this->modelo=_model;
		 this->sdf=_sdf;
		 cargarUniones();
		 //gzdbg<<"Hola Mundo\r\n";
		 listener=new Listener();
		 std::string m="Iniciado";
		 listener->insertarMensaje(m);
		 //this->estado="Iniciado";
		 listener->init(this,"/"+((_sdf->HasElement("topic"))?_sdf->GetElement("topic")->GetValue()->GetAsString():_model->GetName()));

		 this->conexionUpdate=event::Events::ConnectWorldUpdateBegin(boost::bind(&MiRobot::OnUpdate,this,_1));

	 }

	 void MiRobot::OnUpdate(const common::UpdateInfo & _info){
		 common::Time actual=modelo->GetWorld()->SimTime();
		 //this->estado="";
		 //Buscar los elementos de unión
		 for(auto it=uniones.begin();it!=uniones.end();it++){
			 Union *_union=&it->second;
			 switch(_union->tipoMovimiento){
				 case Union::VELOCIDAD:
					 if(_union->moviendo){
						 common::Time ticActual=actual-_union->tiempoActual;
						 double error=_union->joint->Position(0)-_union->anguloFinal;

						 double absError=(error>0)?error:-1*error;
						 double velocidad=0;
						 ///if(absError>0.001){
							 velocidad=_union->pidVelocidad.Update(error, ticActual);

						 /*}else{
							 _union->moviendo=false;
						 }*/
						 _union->velocidad=velocidad;
					 }

					 _union->joint->SetVelocity(0,_union->velocidad);
					 break;
				 case Union::FUERZA:
					 //gzdbg<<"Fuerza"<<_union->fuerza<<"\r\n";
					 //gzdbg<<"Actual Fuerza"<<_union->joint->GetForce(0)<<"\r\n";
					 _union->joint->SetForce(0,_union->fuerza);
					 break;
				 default:
					 _union->joint->SetVelocity(0,0);
					 _union->joint->SetForce(0,0);
					 break;

			 }

		 }
		 if(mostrar){
			 if(siguienteVisualizacion<actual){
				 pintar(ultimoComando);
				 siguienteVisualizacion=actual+frecuencia;
			 }
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
				 //mostramos la velocidad actual de la uni�n
				 std::stringstream ss;
				 ss<<"Velocidad de "<<laUnion<<"= ["<<std::to_string(_union->joint->GetVelocity(0));
				 ss<<", "<<std::to_string(_union->joint->GetVelocity(1));
				 ss<<", "<<std::to_string(_union->joint->GetVelocity(2));
				 ss<<"]";
				 listener->insertarMensaje(ss.str());
				 _union->tipoMovimiento=Union::VELOCIDAD;
				 _union->velocidad=valor;
				 _union->moviendo=false;
			 }else if(tipo=="VM"){
				 _union->tipoMovimiento=Union::VELOCIDAD;
				 _union->pidVelocidad.SetCmdMax(valor);
				 _union->pidVelocidad.SetCmdMin(-1*valor);
			 }else if(tipo=="F"){
				 _union->tipoMovimiento=Union::FUERZA;
				 std::stringstream ss;
				 ss<<"Fuerza de "<<laUnion<<"= ["<<std::to_string(_union->joint->GetForce(0));
				 ss<<"]";
				 _union->fuerza=valor;
				 listener->insertarMensaje(ss.str());
				 /*_union->pidFuerza.SetCmdMax(valor);
				 _union->pidFuerza.SetCmdMin(-1*valor);/**/

			 }else{
				 _union->tipoMovimiento=Union::NINGUNO;
			 }

		 }
	 }
	 void MiRobot::pintar(std::string comando){
		 std::string m="";
		 std::vector<std::string> partes=split(comando, ' ');
		 std::string tipo="";
		 if(partes.size()>1){
			 tipo=partes[1];
		 }
		 if(tipo=="uniones"){
			 //Buscar los elementos de unión
			 std::map<std::string,physics::BasePtr> uniones=buscar(this->modelo,physics::Entity::EntityType::JOINT);
			 for(auto it=uniones.begin();it!=uniones.end();it++){
				 m+=it->first+"\r\n";
			 }

		 }else{
			 //this->estado="";
			 //Buscar los elementos de unión
			 for(auto it=uniones.begin();it!=uniones.end();it++){
				 Union *_union=&it->second;
				 m+=it->first+": A:"+std::to_string(_union->joint->Position(0));
				 m+=" V:"+std::to_string(_union->joint->GetVelocity(0));
				 m+="\r\n";

			 }

		 }

		 listener->insertarMensaje(m);
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
			 _union.pidVelocidad.Init(2, 1, 1, 0, 0, _union.joint->GetVelocityLimit(0), -1*_union.joint->GetVelocityLimit(0));
			 _union.tiempoActual=0;
			 this->uniones[it->first]=_union;
		 }
	 }

	GZ_REGISTER_MODEL_PLUGIN(MiRobot);
}
std::vector<std::string> split(const std::string &c, char d){
	std::vector<std::string> resultado;

	std::stringstream cs(c);
	std::string parte;
	while(std::getline(cs, parte, d)){
		resultado.push_back(parte);
	}

	return resultado;
}
