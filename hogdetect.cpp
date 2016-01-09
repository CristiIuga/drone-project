#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <cctype>
#include <iostream>
#include <iterator>
#include <stdio.h>

using namespace std;
using namespace cv;

HOGDescriptor hog;

int frameCount=0;int contor=0;int detIt=0;int drRect=0;
double scale=1,diff=0;
void detection( Mat& img, HOGDescriptor& hog);
double RealHeight=177,RealDistance=320,dstFocala=198.870056497;

cv::Mat decodeImage(const sensor_msgs::ImageConstPtr& msg_ptr)
{
	cv_bridge::CvImagePtr src_rgb;
	try
	{	//decode the image into cv::RGB		
		src_rgb = cv_bridge::toCvCopy(msg_ptr, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& ex)
	{
		ROS_ERROR("cv_bridge exception: %s", ex.what());
		cv::Mat img_src;
		return img_src;
	}	
	return src_rgb->image;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{
	cv::Mat img_resz,img_rgb = decodeImage(msg_ptr);
	cv::resize(img_rgb, img_resz, cv::Size(), 0.5, 0.5, INTER_LINEAR);
	detection(img_resz,hog);
	contor++;
	//printf( "contor %d \n detectate %d \n", contor,drRect);
	//cv::imshow("People detect",img_rgb);
	cv::waitKey(1);
	return;
}

double distanceTo(double hReal,double dFocal,double height)
{	
	//hReal is the person real height,dFocal is the focal distance and height is the height in pixels
	return (hReal*dFocal)/height;
}
void detection( Mat& img, HOGDescriptor& hog)
{
    	vector<Rect> found, found_filtered;
        double t = (double)getTickCount();
	double distanta;
        // run the detector with default parameters. to get a higher hit-rate
        // (and more false alarms, respectively), decrease the hitThreshold and
        // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
        hog.detectMultiScale(img, found, 0, Size(8,8), Size(32,32), 1.01, 2);
        t = (double)getTickCount() - t;
       // printf("tdetection time = %gms\n", t*1000./cv::getTickFrequency());
        int i, j;
        for( i = 0; i < found.size(); i++ )
        {
            Rect r = found[i];
            for( j = 0; j < found.size(); j++ )
                if( j != i && (r & found[j]) == r)
                    break;
            if( j == found.size() )
                found_filtered.push_back(r);
        }
        for( i = 0; i < found_filtered.size(); i++ )
        {
	    
            Rect r = found_filtered[i];
            // the HOG detector returns slightly larger rectangles than the real objects.
            // so we slightly shrink the rectangles to get a nicer output.
            r.x += cvRound(r.width*0.1);
            r.width = cvRound(r.width*0.7)*scale;
            r.y += cvRound(r.height*0.07);
            r.height = cvRound(r.height*0.7)*scale;
            rectangle(img, r.tl(), r.br(), cv::Scalar(0,0,255), 3,8,0);
	    Point d=r.br()-r.tl();
	    distanta=distanceTo(RealHeight,dstFocala,d.y);
            printf("pixels= (%d, %d) \t %g\n", d.x,d.y,distanta);
	    //printf("list=%g\n",box.size.width);
	    //drRect++;
	    //break;
        }
	cv::imshow( "result", img );
}



int main(int argc, char **argv)
{
	//get args
	ros::init(argc, argv, "hog_detect_process");

	//Subscribe to the ARdrone image raw topic
	ros::NodeHandle imgtr_nh;
	
	hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
	cv::namedWindow("result", CV_WINDOW_AUTOSIZE);

	image_transport::ImageTransport imgtr(imgtr_nh);
	image_transport::Subscriber sub = imgtr.subscribe("/ardrone/image_rect_color", 1, imageCallback);

	ros::Rate r(30);	

	//spin ros core
	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
