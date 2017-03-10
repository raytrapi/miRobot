#include "miRobot.h"
#include "listner.h"
#include <gazebo/gazebo.hh>
namespace gazebo{
	class MiRobot: public ModelPlugin{
		Listner *listner;
		virtual void Load (physics::ModelPtr _model,sdf::ElementPtr _sdf){
			gzdbg<<"Hola Mundo\r\n";
			listner=new Listner();
			listner->init();
		}
	};

	GZ_REGISTER_MODEL_PLUGIN(MiRobot)
}
