#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "camara.h"
ros::Publisher pubDatosCamara;
image_transport::Publisher pubImagen;

int main(int argc, char *argv[]){
	//ros::CallbackQueue cola;
	if(!ros::isInitialized()){
		int argc=0;
		char **argv=NULL;
		ros::init(argc,argv, "TE_VEO", ros::init_options::NoSigintHandler);
	};/**/
   ros::NodeHandle n;
   ros::Rate loop_rate(30); //Hz
   ray::Camara camara(0);


   image_transport::ImageTransport * imgNodo = new image_transport::ImageTransport(n);
   pubImagen = imgNodo->advertise(
   		    "/teVeo_pub_img", 2, true);

  // pubDatosCamara=n.advertise<sensor_msgs::CameraInfo>("/camera_info",2);


   while (ros::ok()){
	   //ROS_INFO("Mensaje");
	   try{
		   //cv::Mat fotograma=camara.obtenerImagen();
		   //std::cout<<fotograma.data;
		   //cv::imshow("Camara", fotograma);
		   gazebo::common::Time actual = gazebo::common::Time::GetWallTime();
		   pubImagen.publish(camara.getImgCamara(actual));
	//	   pubDatosCamara.publish(camara.getInfoCamara(actual));
	   }catch(int e){
		   ROS_ERROR("no hay cámara");
		   break;
	   }
	   ros::spinOnce();
	   loop_rate.sleep();
   }
   std::cout<<"Se acabo";
}
