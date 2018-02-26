/*
 * algoritmoPlugin.h
 *
 *  Created on: 21 ene. 2018
 *      Author: ray
 */

#ifndef SRC_ALGORITMOPLUGIN_H_
#define SRC_ALGORITMOPLUGIN_H_


#include <string>
#include "plugin.h"

namespace vision {
	
	class PLUGIN_EXPORTS Algoritmo_plugin: public Plugin {

		public:
			virtual const char * getNombre() const{return "__ALGOTITMO__";};
			const char * getTipo() const{return "ALGORITMO";} ;

	};

} /* namespace vision */

#endif /* SRC_ALGORITMOPLUGIN_H_ */
