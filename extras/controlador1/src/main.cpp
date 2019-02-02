#include "main.h"


ray::dll::Librerias<vision::Plugin> librerias;
vision::Plugin *plugin;
image_transport::Publisher pubImagen;
bool enMovimiento=false;
//sensor_msgs::Image msgImagen;
int main(int argc, char* argv[]){
	if(argc<3){
		std::cout<<"Se ha de indicar el topic de imagen y el plugen de procesamiento de imagen"<<"\r\n";
		std::cout<<"controlador1 topico libreria.so"<<"\r\n";
		return -1;
	}
	//Iniciamos la librería
	void *hDLL=librerias.cargarDLL(argv[2]);
	if(hDLL!=NULL){
		plugin=librerias.cargarClase(hDLL, "crearInstancia");
		if(plugin==NULL){
			std::cout<<"Fallo\r\n";
			return -1;
		}
	}else{
		std::cout<<"Fracaso cargar libreria\r\n";
		return -1;
	}
	if(!ros::isInitialized()){
		int argc=0;
		char **argv=NULL;
		ros::init(argc,argv, "CONTROLADOR_1", ros::init_options::NoSigintHandler);
	};
	ros::NodeHandle n;

	//Generamos resultado
	image_transport::ImageTransport * imgNodo = new image_transport::ImageTransport(n);
	pubImagen = imgNodo->advertise(
				"/controlador_pub_img", 2, true);

	//Escuchamos
	ros::Subscriber sub = n.subscribe(argv[1], 30, cogerFoto);

	ros::spin();
	return 0;
}
void cogerFoto(const sensor_msgs::Image::ConstPtr& img){
	//La imagen que tenemos no está en formato MAT por lo que tendremos que hacer una conversión
	cv::Mat imagenCV=Img_Mat(img);
	//pubImagen.publish(img);
	cv::Mat resultado=plugin->procesar(imagenCV);
	//pubImagen.publish(img);
	//pubImagen.publish(Mat_Img(imagenCV, "rgb8" ,3));
	pubImagen.publish(Mat_Img(resultado, "mono8" ,1));/**/

	/*cv::Mat canalRojo(img->width, img->height,CV_8U);
	int cantidadRojo=0;
	for(int x=0; x<imagenCV.cols; x++){
		for(int y=0; y<imagenCV.rows; y++){
			cv::Vec3b pixel = imagenCV.at<cv::Vec3b>(cv::Point(x,y));
			int color=pixel.val[2];
			if(color>200){
				cantidadRojo++;
				canalRojo.at<uchar>(cv::Point(x,y))=255;
			}else{
				canalRojo.at<uchar>(cv::Point(x,y))=0;
			}
		}
	}
	if(cantidadRojo>(imagenCV.cols*imagenCV.rows)/75){
		//Movemos el robot
		if(!enMovimiento){
			ROS_INFO("Podemos mover");
			enMovimiento=true;
		}
	}else{
		if(enMovimiento){
			ROS_INFO("Paramos");
			enMovimiento=false;
		}
	}
	pubImagen.publish(Mat_Img(canalRojo, "mono8" ,1));/**/
}
sensor_msgs::Image Mat_Img(const cv::Mat img, char * codificacion, int pasos) {
	sensor_msgs::Image respuesta;
	gazebo::common::Time actual = gazebo::common::Time::GetWallTime();
	respuesta.header.frame_id = "camara";
	respuesta.header.stamp.sec = actual.sec;
	respuesta.header.stamp.nsec = actual.nsec;
	respuesta.width=img.rows;
	respuesta.height=img.cols;
	respuesta.encoding=codificacion;
	respuesta.is_bigendian=false;
	respuesta.step=respuesta.width*pasos;
	respuesta.data.resize(respuesta.height*respuesta.step);
 	uchar * ptrImagen=img.data;
	uchar * ptrMsgImagen=(uchar *)(&respuesta.data[0]);
	for(int i=0;i<respuesta.height;i++){
		memcpy(ptrMsgImagen,ptrImagen,respuesta.step);
		//Aumentamos las columnas
		ptrMsgImagen+=respuesta.step;
		ptrImagen+=respuesta.step;
	}
	return respuesta;
}
cv::Mat Img_Mat(const sensor_msgs::Image::ConstPtr& img) {
	/*cv::Mat respuesta(img->width, img->height,CV_8UC3, const_cast<uchar*>(&img->data[0]), img->step);
	return respuesta;/**/
	cv::Mat respuesta(img->width, img->height,CV_8UC3);//, const_cast<uchar*>(&img->data[0]), img->step);
	uchar * ptrImagen=respuesta.data;
	uchar * ptrMsgImagen=(uchar *)(&img->data[0]);
	for(int i=0;i<img->height;i++){
		memcpy(ptrImagen, ptrMsgImagen,img->step);
		ptrMsgImagen+=img->step;
		ptrImagen+=img->step;
	}
	return respuesta;
	/**/
}
