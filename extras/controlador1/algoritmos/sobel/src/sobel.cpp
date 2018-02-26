/*
 * sobel.cpp
 *
 *  Created on: 21 ene. 2018
 *      Author: ray
 */

#include "sobel.h"

namespace vision {
	cv::Mat Sobel::procesar(const cv::Mat& imgOriginal) {

		//log("Llego");
		cv::Mat img(imgOriginal.rows,imgOriginal.cols,CV_8U);
		cv::Mat imgGris(imgOriginal.rows,imgOriginal.cols,CV_8U);
		int limiteX=imgOriginal.cols-1;
		int limiteY=imgOriginal.rows-1;
		for(int x=0; x<imgOriginal.cols; x++){
			for(int y=0; y<imgOriginal.rows; y++){

				int gx=0;
				int gy=0;
				int color=0;
				if(x>0){
					gx-=(imgGris.at<uchar>(cv::Point(x-1,y))<<1);

				}
				if(x<limiteX){
					uchar color=0;
					if(y==0){
						cv::Vec3b pixel = imgOriginal.at<cv::Vec3b>(cv::Point(x+1,y));
						color=rint(0.21*pixel.val[0]+0.72*pixel.val[1]+0.07*pixel.val[2]);
						imgGris.at<uchar>(cv::Point(x+1,y))=color;
					}else{
						color=imgGris.at<uchar>(cv::Point(x+1,y));
					}
					gx+=(color<<1);
				}
				if(x==0 && y==0){

					cv::Vec3b pixel = imgOriginal.at<cv::Vec3b>(cv::Point(x,y));
					uchar color=rint(0.21*pixel.val[0]+0.72*pixel.val[1]+0.07*pixel.val[2]);
					imgGris.at<uchar>(cv::Point(x,y))=color;
				}
				if(y>0){
					if(x>0){
						gx-=(imgGris.at<uchar>(cv::Point(x-1,y-1)));
						gy-=(imgGris.at<uchar>(cv::Point(x-1,y-1)));
					}
					if(x<limiteX){
						gx+=(imgGris.at<uchar>(cv::Point(x+1,y-1)));
						gy-=(imgGris.at<uchar>(cv::Point(x+1,y-1)));
					}
					gy-=(imgGris.at<uchar>(cv::Point(x,y-1))<<1);
				}
				if(y<limiteY){
					if(x>0){
						gx-=(imgGris.at<uchar>(cv::Point(x-1,y+1))<<1);
						gy+=(imgGris.at<uchar>(cv::Point(x-1,y+1)));
					}
					if(x<limiteX){
						cv::Vec3b pixel = imgOriginal.at<cv::Vec3b>(cv::Point(x+1,y+1));
						uchar color=rint(0.21*pixel.val[0]+0.72*pixel.val[1]+0.07*pixel.val[2]);
						imgGris.at<uchar>(cv::Point(x+1,y+1))=color;
						gx+=(color);
						gy+=(color);
					}
					if(x==0){
						cv::Vec3b pixel = imgOriginal.at<cv::Vec3b>(cv::Point(x,y+1));
						uchar color=rint(0.21*pixel.val[0]+0.72*pixel.val[1]+0.07*pixel.val[2]);
						imgGris.at<uchar>(cv::Point(x,y+1))=color;
						gy+=(color<<1);
					}
				}/**/
				int c=sqrt(gx*gx-gy*gy);
				if(c>100){
					c=255;
				}else{
					c=0;
				}
				img.at<uchar>(cv::Point(x,y))=c;
				//(*img.ptr(x,y))=0;
				//img.at<cv::Vec3b>(cv::Point(x,y)).val[0]=0;//0.21*pixel.val[0]+0.72*pixel.val[1]+0.07*pixel.val[2];
			}
		}
		//return img;
		return imgGris;
	}

} /* namespace vision */

REGISTRAR_PLUGIN(vision::Sobel);


