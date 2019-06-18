#include "listener.h"
#include "../src/miRobot.h"

#include "../src/listener.h"

namespace gazebo{
	 void MiRobot::Load (physics::ModelPtr _model,sdf::ElementPtr _sdf){
		 this->modelo=_model;
		 this->sdf=_sdf;
		 cargarUniones();
		 cargarSensores();
		 //gzdbg<<"Hola Mundo\r\n";
		 listener=new Listener();
		 std::string m="Iniciado";
		 listener->insertarMensaje(m);
		 //this->estado="Iniciado";
		 listener->init(this,"/"+((_sdf->HasElement("topic"))?_sdf->GetElement("topic")->GetValue()->GetAsString():_model->GetName()));

		 this->conexionUpdate=event::Events::ConnectWorldUpdateBegin(boost::bind(&MiRobot::OnUpdate,this,_1));
		 this->tiempoSensorAnterior=modelo->GetWorld()->SimTime();
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
					 if(_union->fuerzas.tiempo!=0 && _union->fuerzas.fuerzas.size()>1 &&
						actual>=_union->fuerzas.ultimaRevision+_union->fuerzas.tiempo){
						 //double fuerza=_union->fuerzas.fuerzas[0];
						 _union->fuerzas.fuerzas.erase(_union->fuerzas.fuerzas.begin());
						 ROS_INFO("Fuerza para la unión %s = %f",_union->joint->GetName().c_str(),_union->fuerzas.fuerzas[0]);
						 _union->fuerzas.ultimaRevision=actual;
					 }
					 _union->joint->SetForce(0,_union->fuerzas.fuerzas[0]);
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
		 leerSensores(actual);
	 }
	 void MiRobot::mover(std::string laUnion, double valor){
		 if(uniones.find(laUnion)==uniones.end()){
			 ROS_INFO("No se ha encontrado al unión %s",laUnion);
		 }
		 Union *_union=&uniones[laUnion];
		 if(_union){
			 _union->anguloFinal=valor;
			 _union->tiempoActual=0;
			 _union->moviendo=true;
			 _union->tipoMovimiento=Union::VELOCIDAD;
		 }
	 }
	 void MiRobot::parametrizar(std::string laUnion,std::string tipo, double valor, std::vector<std::string> &partes){
		 if(uniones.find(laUnion)==uniones.end()){
			 ROS_INFO("No se ha encontrado la unión %s",laUnion.c_str());
		 }else{
			 ROS_INFO("Encuentro la unión %s",laUnion.c_str());
		 }
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
				 _union->fuerzas.tiempo=0;
				 _union->fuerzas.fuerzas.clear();
				 _union->fuerzas.fuerzas.push_back(valor);
				 _union->fuerzas.ultimaRevision=modelo->GetWorld()->SimTime();
				 listener->insertarMensaje(ss.str());
				 /*_union->pidFuerza.SetCmdMax(valor);
				 _union->pidFuerza.SetCmdMin(-1*valor);/**/

			 }else if(tipo=="FT"){
				 //Obtenemos cada elemento
				 //Vector_Fuerza vF;
				 _union->fuerzas.tiempo=valor;
				 _union->fuerzas.fuerzas.clear();
				 for (int i = 4; i < partes.size(); i++) { //Las fuerzas están a partir del 4 parámetro
					 std::string str=trim(partes[i]);
					 if(str!=""){
						 _union->fuerzas.fuerzas.push_back(std::stod(str));
					 }
				}
				_union->fuerzas.ultimaRevision=modelo->GetWorld()->SimTime();
				_union->tipoMovimiento=Union::FUERZA;

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
	 void MiRobot::cargarSensores(){

		 std::vector<sensors::SensorPtr> vSensores=sensors::SensorManager::Instance()->GetSensors();
	 	 for(int i=0; i<vSensores.size();i++){

	 		 this->sensores[vSensores[i]->Name()]=new _Sensor("/home/ray/robot/src/miRobot/salidas/"+vSensores[i]->Name()+".txt", vSensores[i]);
	 		 gzerr<<vSensores[i]->Name().c_str()<<"\r\n";
	 	 }
	 }

	 void MiRobot::leerSensores(common::Time tiempo){
		 common::Time lapso=tiempo-this->tiempoSensorAnterior;
		 for(auto it=this->sensores.begin();it!=this->sensores.end();it++){


			 if(it->second->sensor->Type()=="imu"){
				 //sensors::ImuSensor imu=(sensors::ImuSensor)(boost::static_pointer_cast<sensors::Sensor>(it->second->sensor));
				 sensors::SensorPtr p=it->second->sensor;
				 sensors::ImuSensorPtr imu=std::static_pointer_cast<sensors::ImuSensor>(p);
				 if(imu->IsActive()){
					 ignition::math::Vector3d velocidades=imu->AngularVelocity();
					 ignition::math::Vector3d aceleraciones=imu->LinearAcceleration();
					 std::string log="";
				 	 log+=""+std::to_string(velocidades.X())+","+std::to_string(velocidades.Y())+","+std::to_string(velocidades.Z());
				 	 log+=","+std::to_string(aceleraciones.X())+","+std::to_string(aceleraciones.Y())+","+std::to_string(aceleraciones.Z());
					 (it->second)->log(tiempo,log);
				 }
				 /**/

			 }
			 if(it->second->sensor->Type()=="force_torque"){
				 sensors::SensorPtr p=it->second->sensor;
				 sensors::ForceTorqueSensorPtr fuerza=std::static_pointer_cast<sensors::ForceTorqueSensor>(p);
				 if(fuerza->IsActive()){
					 ignition::math::Vector3d fuerzas=fuerza->Force();
					 std::string log="";
					 log+=""+std::to_string(fuerzas.X())+","+std::to_string(fuerzas.Y())+","+std::to_string(fuerzas.Z());
					 (it->second)->log(tiempo,log);
				 }

			 }

		 }

	 }

	 MiRobot::~MiRobot(){
		 for(auto it=this->sensores.begin();it!=this->sensores.end();it++){
			 delete it->second;
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
std::string trim(const std::string &c){
	std::string s=c;
	while( !s.empty() && std::isspace( s.back() ) ) s.pop_back() ;

	// return residue after leading white space
	std::size_t pos = 0 ;
	while( pos < s.size() && std::isspace( s[pos] ) ) ++pos ;
	return s.substr(pos) ;
}
