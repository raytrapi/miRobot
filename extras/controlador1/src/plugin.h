/*
 * plugin.h
 *
 *  Created on: 21 ene. 2018
 *      Author: ray
 */

#ifndef SRC_PLUGIN_H_
#define SRC_PLUGIN_H_
#ifdef WIN32
	#ifdef PLUGIN_EXPORTS
	   #define PLUGIN_EXPORTS   __declspec(dllexport)
	#else
	  #define PLUGIN_EXPORTS   __declspec(dllimport)
	#endif
#else
	#define PLUGIN_EXPORTS
#endif


#include <opencv2/opencv.hpp>
#include <utility>
#include <string>
#include <ctime>
#include <math.h>
#include <map>
namespace vision {
	
	class PLUGIN_EXPORTS Plugin {
		private:
			bool conLog=false;
			std::string *_log;
			//std::map<std::string,void *> _metodos;
		protected:
			void debug( std::string l){if(conLog){*_log+=l;*_log+="\r\n";};};
			void debug( char *l){if(conLog){*_log+=l;*_log+="\r\n";};};
			void debug( int l){if(conLog){*_log+=l;*_log+="\r\n";};};

		public:
			virtual ~Plugin(){};
			virtual const char * getNombre() const=0;
			virtual const char * getTipo() const=0;
			virtual cv::Mat procesar(const cv::Mat &)=0;//TODO: Mejorar con rvaue
			void setDebug(std::string *l){
				conLog=true;
				_log=l;};
			const char * getDebug(){

				return (*_log).c_str();
			};
	};

} /* namespace vision */
#define REGISTRAR_PLUGIN(classname) \
	extern "C" vision::Plugin * crearInstancia() {\
			return (vision::Plugin *) new classname();\
	}\
	extern "C" void destroy(vision::Plugin* p) {\
	    delete p;\
	}
#ifdef _WIN32
	extern "C" __declspec(dllexport) vision::Plugin * crearInstancia();
	extern "C" __declspec(dllexport) void destroy(vision::Plugin *);
#endif
#endif /* SRC_PLUGIN_H_ */
