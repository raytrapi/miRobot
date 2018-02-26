/*
 * librerias.cpp
 *
 *  Created on: 8 ene. 2018
 *      Author: ray
 */

#include "librerias.h"

namespace ray {
	namespace dll {
		//ray::dll::Librerias<Plu> librerias;

		template<class T>
		inline void * Librerias<T>::cargarDLL(const char * fichero){
			#ifdef WIN32
				HINSTANCE hDLL = LoadLibrary(fichero);
				return hDLL;
			#elif __linux__
				void * manejador;
				//std::cout<<"intento\r\n";
				manejador=dlopen(fichero, RTLD_LAZY);
				if (!manejador) {
					std::cerr << "No ha podido cargar la librería: " << dlerror() << '\n';

					}
				return manejador;
			#endif
			return NULL;
		}
		template<class T>
		inline T * Librerias<T>::cargarClase(void * &hDLL, const char * clase){
			#ifdef WIN32
				PCTOR instancia= (PCTOR)GetProcAddress((HINSTANCE)hDLL,clase);
				if(instancia!=NULL){
					return instancia();
				}
			#elif __linux__
				T * (*instancia)();
				*(void **) (&instancia)=dlsym(hDLL, clase);
				if(instancia!=NULL){
					return (*instancia)();
				}
			#endif
			return NULL;
		}
		
		template<class T>
		inline void ray::dll::Librerias<T>::ok() {
			std::cout<<"llegue\r\n";
		}
		
		template<class T>
		inline ray::dll::Librerias<T>::~Librerias() {
			/*for(int i=0;i<numLibrerias;i++){
				if(librerias[i]!=NULL){
					delete librerias[i];
				}
			}*/
		}

		/**
		 * Localizamos las librer�as
		 */
		template<class T>
		inline void Librerias<T>::leer(const char * ruta) {
				//char * clase="una";
			boost::filesystem::path p(ruta);
			if (boost::filesystem::exists(p) && boost::filesystem::is_directory(p)){
				//Cogemos los ficheros
				#ifdef WIN32
					boost::regex filtro(".*\\.dll$");
				#else
					boost::regex filtro(".*\\.so$");
				#endif
				boost::smatch coincide;
				numLibrerias=0;
				for(boost::filesystem::directory_entry& f:boost::filesystem::directory_iterator(p)){
					const std::string fichero=f.path().generic_string();
					if(boost::regex_match(fichero,coincide, filtro)){
						numLibrerias++;

					}
				}
				std::string log="Se han encontrado ";
				log+=numLibrerias;
				log+=" librerias";
				librerias=new boost::shared_ptr<T>[numLibrerias];
				int iPlugin=0;
				for(boost::filesystem::directory_entry& f:boost::filesystem::directory_iterator(p)){
					const std::string fichero=f.path().generic_string();
					if(boost::regex_match(fichero,coincide, filtro)){
						try{
							void *hDLL=cargarDLL((char *)f.path().generic_string().c_str());
							if(hDLL!=NULL){
								T *p=cargarClase(hDLL, "crearInstancia");
								if(p!=NULL){
									boost::shared_ptr<T> plugin(p);
									librerias[iPlugin++]=plugin;
								}else{
									std::cout<<"Fallo\r\n";
								}
							}else{
								std::cout<<"Fracaso cargar libreria\r\n";
							}
						}catch (int e) {

						}
					}
				}

			}
			//ray::utiles::Log::consola("Terminamos");
		}
		/*template<class T, class K>
		inline void cargar(const boost::filesystem::path &ruta,boost::shared_ptr<K> clase,bool recargar=false) {
		}*/
	
	} /* namespace dll */
} /* namespace ray */


