/*
 * librerias.h
 *
 *  Created on: 8 ene. 2018
 *      Author: ray
 */

#ifndef SRC_DLL_LIBRERIAS_H_
#define SRC_DLL_LIBRERIAS_H_
#ifdef _WIN32
	#include <windows.h>
#elif __linux__
	#include <stdlib.h>
	#include <dlfcn.h>
#endif
#include <boost/shared_ptr.hpp>
//#include <boost/dll.hpp>
#include <boost/regex.hpp>
//#include <boost/poly_collection/base_collection.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include <stdlib.h>
#include <map>


namespace ray {
	namespace dll {
		template<class T>
		class Librerias {
			#ifdef _WIN32
				typedef T * (WINAPI * PCTOR) ();
			#endif
			private:
				 std::map <std::string, std::map<std::string,boost::shared_ptr<T>>> libreriasCargadas;


			public:
				 Librerias(){numLibrerias=0;};
				 ~Librerias();
				 boost::shared_ptr<T> * librerias;
				 int numLibrerias;
				 void ok();
				 void leer(const char *);
				 void * cargarDLL(const char * fichero);
				 T * cargarClase(void * &hDLL, const char * clase);
				 /*template<class k>
				 void cargar(const boost::filesystem::path &ruta,boost::shared_ptr<k> clase,bool recargar=false);/**/
		};


	} /* namespace dll */
} /* namespace vision */

#include <dll/librerias.cpp>

#endif /* SRC_DLL_LIBRERIAS_H_ */
