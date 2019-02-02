
#include <iostream>

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

#include <gazebo/gazebo.hh>

#include "std_msgs/String.h"
#include "plugin.h"
#include "dll/librerias.h"


void cogerFoto(const sensor_msgs::Image::ConstPtr& img);
sensor_msgs::Image Mat_Img(const cv::Mat img, char * , int);
cv::Mat Img_Mat(const sensor_msgs::Image::ConstPtr&);
