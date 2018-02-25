/*
 * Camara.cpp
 *
 *  Created on: 6 feb. 2018
 *      Author: ray
 */

#include "camara.h"

namespace gazebo {

	Camara::Camara() {
		this->frameAnterior = common::Time(0);

	}

	Camara::~Camara() {
		// TODO Auto-generated destructor stub
	}
	void Camara::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
		CameraPlugin::Load(_parent, _sdf);
		this->nombreMundo = _parent->WorldName();
		//this->mundo = physics::get_world(this->nombreMundo);
		if(!ros::isInitialized()){
			int argc=0;
			char **argv=NULL;
			ros::init(argc,argv, "miRobot_camera", ros::init_options::NoSigintHandler);
		}
		using boost::dynamic_pointer_cast;
		this->sensorPtr=dynamic_pointer_cast<sensors::CameraSensor>(_parent);
		this->topic="/"+((_sdf->HasElement("topic"))?_sdf->GetElement("topic")->GetValue()->GetAsString():_parent->Name());
		this->nodo.reset(new ros::NodeHandle("miRobot_camera"));

		infoCamara.height=this->sensorPtr->ImageHeight();
		infoCamara.width=this->sensorPtr->ImageWidth();
		infoCamara.distortion_model="plumb_bob";
		gazebo::rendering::CameraPtr camara=this->sensorPtr->Camera();
		infoCamara.D.resize(5);
		gazebo::rendering::DistortionPtr distorsion=camara->LensDistortion();;
		double hfov = camara->HFOV().Radian();
		gazebo::math::Vector2d centro;
		if(distorsion!=NULL){
			centro=	distorsion->GetCenter();
			infoCamara.D[0]=distorsion->GetK1();
			infoCamara.D[1]=distorsion->GetK2();
			infoCamara.D[2]=distorsion->GetP1();
			infoCamara.D[3]=distorsion->GetP2();
			infoCamara.D[4]=distorsion->GetK3();
		}else{
			infoCamara.D[0]=0;
			infoCamara.D[1]=0;
			infoCamara.D[2]=0;
			infoCamara.D[3]=0;
			infoCamara.D[4]=0;
			centro.x=(static_cast<double>(infoCamara.width) /2.0);
			centro.y=(static_cast<double>(infoCamara.height) /2.0);
		}

		/**/

		//std::cout<<camara->ImageFormat()<<"\r\n";
		//std::cout<<sensor_msgs::image_encodings::RGB8<<"\r\n";
		formatoImagen=sensor_msgs::image_encodings::RGB8;
		double longitudFocal = (static_cast<double>(infoCamara.width)) / (2.0 * tan(hfov / 2.0));
		std::cout<<"hfov: "<<hfov<<"  long: "<<longitudFocal<<"\r\n";
		double x=centro.x;//(static_cast<double>(infoCamara.width=this->sensorPtr->ImageWidth()) + 1.0) /2.0;
		double y=centro.y;//(static_cast<double>(infoCamara.width=this->sensorPtr->ImageHeight()) + 1.0) /2.0;

		infoCamara.K[0] = longitudFocal;
		infoCamara.K[1] = 0.0;
		infoCamara.K[2] = x;
		infoCamara.K[3] = 0.0;
		infoCamara.K[4] = longitudFocal;
		infoCamara.K[5] = y;
		infoCamara.K[6] = 0.0;
		infoCamara.K[7] = 0.0;
		infoCamara.K[8] = 1.0;
		  // rectification
		infoCamara.R[0] = 1.0;
		infoCamara.R[1] = 0.0;
		infoCamara.R[2] = 0.0;
		infoCamara.R[3] = 0.0;
		infoCamara.R[4] = 1.0;
		infoCamara.R[5] = 0.0;
		infoCamara.R[6] = 0.0;
		infoCamara.R[7] = 0.0;
		infoCamara.R[8] = 1.0;
		  // camera_ projection matrix (same as camera_ matrix due
		  // to lack of distortion/rectification) (is this generated?)
		infoCamara.P[0] = longitudFocal;
		infoCamara.P[1] = 0.0;
		infoCamara.P[2] = x;
		infoCamara.P[3] = -longitudFocal * 0.07;
		infoCamara.P[4] = 0.0;
		infoCamara.P[5] = longitudFocal;
		infoCamara.P[6] = y;
		infoCamara.P[7] = 0.0;
		infoCamara.P[8] = 0.0;
		infoCamara.P[9] = 0.0;
		infoCamara.P[10] = 1.0;
		infoCamara.P[11] = 0.0;


		servirImagen =  new dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>(*this->nodo);
		dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>::CallbackType f =boost::bind(&Camara::configCallback, this, _1, _2);
		servirImagen->setCallback(f);

		this->nodoImagen = new image_transport::ImageTransport(*this->nodo);
		this->pubImagen = this->nodoImagen->advertise(
		    this->topic+"_pub_img", 2,
		    boost::bind(&Camara::conexion, this),
		    boost::bind(&Camara::desconexion, this),
			ros::VoidPtr(), true);

		ros::AdvertiseOptions ad=ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
						"/camera_info",
						2,
						boost::bind(&Camara::conexion, this),
						boost::bind(&Camara::desconexion, this),
						ros::VoidPtr(),
						&this->cola2
						);
		this->pubDatosCamara=this->nodo->advertise(ad);
		//this->nodoSensor= gazebo::transport::NodePtr(new gazebo::transport::Node());
		//		this->nodoSensor->Init(this->nombreMundo);
		//this->threadColas=std::thread(std::bind(&Laser::configurar, this));
		this->sensorPtr->SetActive(true);
		//this->subcriptorSensor=this->nodoSensor->Subscribe(this->sensorPtr->Topic(),&Camara::onFrame, this);/**/
		//this->threadColas = std::thread(std::bind(&Camara::thread, this));

	}

	void Camara::OnNewFrame(const unsigned char* imagen, unsigned int alto,
			unsigned int ancho, unsigned int profundidad,
			const std::string& formato) {

		common::Time actual = this->sensorPtr->LastMeasurementTime();
		if (actual - this->frameAnterior >= this->frecuencia){
			this->enviarImagen(imagen, actual);
			//this->enviarInformacionCamara(actual);
			this->frameAnterior = actual;

		}
	}

	void Camara::conexion() {
		std::cout<<"Conecto";
	}

	void Camara::desconexion() {
		std::cout<<"Desconecto";
	}


	void Camara::thread() {
		static const double timeout = 0.001;
		while (this->nodo->ok()){
		    /// take care of callback queue
		    this->cola2.callAvailable(ros::WallDuration(timeout));
		}
	}

	void Camara::configCallback(gazebo_plugins::GazeboRosCameraConfig& config, uint32_t level) {
		this->sensorPtr->SetUpdateRate(config.imager_rate);
	}

	void Camara::enviarImagen(const unsigned char* fuente,	common::Time& anterior) {
		//this->imagenAnterior = anterior;
		this->msgImagen.header.frame_id = "camara";
		this->msgImagen.header.stamp.sec = this->imagenAnterior.sec;
		this->msgImagen.header.stamp.nsec = this->imagenAnterior.nsec;

		fillImage(this->msgImagen, formatoImagen, this->sensorPtr->ImageHeight(), this->sensorPtr->ImageWidth(),
		        3*this->sensorPtr->ImageWidth(), reinterpret_cast<const void*>(fuente));

		this->pubImagen.publish(this->msgImagen);
	}

	void Camara::enviarInformacionCamara(common::Time& ahora) {
		//common::Time frameActual = this->sensorPtr->LastMeasurementTime();
		//if (frameActual - this->frameAnterior >= 10){

			//this->sensorPtr->
			//std::cout<<"lanzo";
			infoCamara.header.stamp.sec = ahora.sec;
			infoCamara.header.stamp.nsec = ahora.nsec;
			try{
				pubDatosCamara.publish(infoCamara);
			}catch (int e) {
				std::cout << "Error "<< e;
			}
			//frameAnterior = frameActual;
		//}
	}

}
GZ_REGISTER_SENSOR_PLUGIN(gazebo::Camara);
