/*
 * sobel.h
 *
 *  Created on: 21 ene. 2018
 *      Author: ray
 */

#ifndef ALGORITMOS_SRC_SOBEL_H_
#define ALGORITMOS_SRC_SOBEL_H_
#include <string>

#include "../src/algoritmoPlugin.h"
namespace vision {
	
	class Sobel: public Algoritmo_plugin {
		public:
			~Sobel(){};
			const char * getNombre()const{return "Sobel";};
			cv::Mat procesar(const cv::Mat & imgOriginal);

	};


} /* namespace vision */

#endif /* ALGORITMOS_SRC_SOBEL_H_ */
