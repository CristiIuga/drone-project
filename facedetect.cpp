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

// for Kalman filter
#include <opencv/cv.h>
#include "opencv2/video/tracking.hpp"
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

int frameCount=0;int contor=0;int detIt=0;int drRect=0;
double scale=1;
string cascadeName = "../ws/src/agitr/haarcascadeFD.xml";
bool tryflip = false;
CascadeClassifier cascade,nestedCascade;
void detection( Mat& img, CascadeClassifier& cascade, CascadeClassifier& nestedCascade,double scale, bool tryflip );
double RealHeight=26,RealDistance=650,dstFocala=1175;

int stateSize = 6;
int measSize = 4;
int contrSize = 0;
unsigned int type = CV_32F;
cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]

void initializaKalmanFilter()
{
	 	//cv::Mat procNoise(stateSize, 1, type)
   	// [E_x,E_y,E_v_x,E_v_y,E_w,E_h]
 
   	// Transition State Matrix A
   	// Note: set dT at each processing step!
   	// [ 1 0 dT 0  0 0 ]
   	// [ 0 1 0  dT 0 0 ]
   	// [ 0 0 1  0  0 0 ]
   	// [ 0 0 0  1  0 0 ]
   	// [ 0 0 0  0  1 0 ]
   	// [ 0 0 0  0  0 1 ]
   	cv::setIdentity(kf.transitionMatrix);
 
   	// Measure Matrix H
   	// [ 1 0 0 0 0 0 ]
   	// [ 0 1 0 0 0 0 ]
  	// [ 0 0 0 0 1 0 ]
   	// [ 0 0 0 0 0 1 ]
   	//kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
   	kf.measurementMatrix.at<float>(0) = 1.0f;
   	kf.measurementMatrix.at<float>(7) = 1.0f;
   	kf.measurementMatrix.at<float>(16) = 1.0f;
   	kf.measurementMatrix.at<float>(23) = 1.0f;
 
   	// Process Noise Covariance Matrix Q
  	// [ Ex 0  0    0 0    0 ]
   	// [ 0  Ey 0    0 0    0 ]
   	// [ 0  0  Ev_x 0 0    0 ]
   	// [ 0  0  0    1 Ev_y 0 ]
   	// [ 0  0  0    0 1    Ew ]
   	// [ 0  0  0    0 0    Eh ]
   	cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
  	kf.processNoiseCov.at<float>(0) = 1e-2;
  	kf.processNoiseCov.at<float>(7) = 1e-2;
   	kf.processNoiseCov.at<float>(14) = 2.0f;
  	kf.processNoiseCov.at<float>(21) = 1.0f;
   	kf.processNoiseCov.at<float>(28) = 1e-2;
   	kf.processNoiseCov.at<float>(35) = 1e-2;
 
   	// Measures Noise Covariance Matrix R
   	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

	  cout << "M state = "<< endl << " "  << state << endl << endl;
	  cout << "M meas = "<< endl << " "  << meas << endl << endl;
	  cout << "M transition= "<< endl << " "  << kf.transitionMatrix << endl << endl;
	  cout << "M measurement= "<< endl << " "  << kf.measurementMatrix << endl << endl;
  	cout << "M processCOV= "<< endl << " "  << kf.processNoiseCov << endl << endl;
	  cout << "M measurementCOV= "<< endl << " "  << kf.measurementNoiseCov << endl << endl;
}

//function to apply a small transform to the received frame from the callback function
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

//the callback function
void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{
	cv::Mat img_resz,img_rgb = decodeImage(msg_ptr);
	//cv::resize(img_rgb, img_resz, cv::Size(), 0.5, 0.5, INTER_LINEAR);
	//send to the detection function
	detection(img_rgb,cascade,nestedCascade,scale,tryflip);
	cv::waitKey(1);
	return;
}

//function that gets the real height of object to be tracked,the initially computed focal distance and the height of the drawn rectangle in the detection
//it returns the distance between the camera and the object
double distanceTo(double hReal,double dFocal,double height)
{
	return (hReal*dFocal)/height;
}

double ticks = 0;
bool found = false;      
int notFoundCount = 0;

void detection( Mat& img, CascadeClassifier& cascade,CascadeClassifier& nestedCascade,double scale, bool tryflip )
{	
    double precTick = ticks;
    ticks = (double) cv::getTickCount();
    double deltaT = (ticks - precTick) / cv::getTickFrequency(); //seconds
    double distanta=0,distantaPred=0;
    int i = 0;
    double t = 0;
    vector<Rect> persons, objs;
    const static Scalar colors[] =  { CV_RGB(0,0,255),
        CV_RGB(0,128,255),
        CV_RGB(0,255,255),
        CV_RGB(0,255,0),
        CV_RGB(255,128,0),
        CV_RGB(255,255,0),
        CV_RGB(255,0,0),
        CV_RGB(255,0,255)} ;
    Mat gray, smallImg( cvRound (img.rows/scale), cvRound(img.cols/scale), CV_8UC1 );

    cvtColor( img, gray, CV_BGR2GRAY );
    resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
    equalizeHist( smallImg, smallImg );

    if (found)
    {
        //Matrix A
        kf.transitionMatrix.at<float>(2) = deltaT;
        kf.transitionMatrix.at<float>(9) = deltaT;
 
        //cout << "dT:" << endl << deltaT << endl;
 
        state = kf.predict();
        //cout << "State post:" << endl << state << endl;            
	      cv::Rect predRect;          
	      predRect.width = state.at<float>(4);          
	      predRect.height = state.at<float>(5)+15;          
	      predRect.x = state.at<float>(0) - predRect.width / 2;          
	      predRect.y = state.at<float>(1) - predRect.height / 2;
	      cv::rectangle(img, predRect, CV_RGB(255,255,255), 2);      
        distantaPred=distanceTo(RealHeight,dstFocala,predRect.height); //the distance calculated based on the predicted location
	      char str1[20];
	      sprintf(str1,"%g",distantaPred);
        putText(img, str1, Point(10,300), 2, 2, Scalar(255,255,255));
    }
 
    vector<int> reject_levels;
    vector<double> level_weights;

    t = (double)cvGetTickCount();
    //the folowing line is a slightly modified and undocumented version of the detectMultiScale function
    //cascade.detectMultiScale(img, persons, reject_levels, level_weights, 1.3, 3, 0, Size(15,15), Size(), true);
    cascade.detectMultiScale( smallImg, persons,
        1.3, 3, 0
        //|CV_HAAR_FIND_BIGGEST_OBJECT
        //|CV_HAAR_DO_ROUGH_SEARCH
        |CV_HAAR_SCALE_IMAGE
        ,
        Size(15, 15) );
   
    t = (double)cvGetTickCount() - t;
    //printf( "detection time = %g ms\n", t/((double)cvGetTickFrequency()*1000.) );
    //this is an alternate possibility using the slightly modified version of the detectMultiScale function
    /*
    for (int n = 0; n < objs.size(); n++) 
    {
          rectangle(img, objs[n], Scalar(255,0,0), 8);
		      char str[200];
		      sprintf(str,"%d",reject_levels[n]);
          putText(img, str, Point(objs[n].x, objs[n].y), 1, 1, Scalar(0,0,255));
    }
    */

    //printf( "semafoare = %ld \n", persons.size());
    //loop through the vector of detection from the detectMultiScale function
    for( vector<Rect>::const_iterator r = persons.begin(); r != persons.end(); r++, i++ )
    {
        //Mat smallImgROI;
        //vector<Rect> nestedObjects;
        //Point center;
        Scalar color = colors[i%8];
        //int radius;
        rectangle( img, cvPoint(cvRound((r->x+10)*scale), cvRound(r->y*scale)),
                       cvPoint(cvRound((r->x + r->width-10)*scale), cvRound((r->y + r->height-15)*scale)),
                       color, 3, 8, 0);
	      objs.push_back(Rect(cvPoint(cvRound((r->x+10)*scale), cvRound(r->y*scale)),cvPoint(cvRound((r->x + r->width-10)*scale), cvRound((r->y + r->height-15)*scale))));
	      cv::Point center;
        center.x = objs[i].x + objs[i].width / 2;
        center.y = objs[i].y + objs[i].height / 2;
        cv::circle(img, center, 2, CV_RGB(0,0,255), -1);
	      rectangle(img,objs[0],CV_RGB(0,255,0),3,8,0);
	      Point d=r->br()-r->tl();
	      distanta=distanceTo(RealHeight,dstFocala,d.y);//the distance as it results from the detection
	      char str2[20];
	      sprintf(str2,"%g",distanta);
        putText(img, str2, Point(0, 340), 2, 2, Scalar(255,255,0));
	      printf("pixels= (%d, %d) \t %d \t dist=%g cm\n", d.x,d.y,r->height,distanta);
        //printf( "desenat dreptunghi %d \n", drRect);

	      //update
      	if (persons.size() == 0)
      	{
        	notFoundCount++;
         	cout << "notFoundCount:" << notFoundCount << endl;          
		      if( notFoundCount >= 10 )
           	 	  found = false;
       	  else
            		kf.statePost = state;
        }
        else
        {
         notFoundCount = 0;
 
         meas.at<float>(0) = objs[0].x + objs[0].width / 2;
         meas.at<float>(1) = objs[0].y + objs[0].height / 2;
         meas.at<float>(2) = (float)objs[0].width;
         meas.at<float>(3) = (float)objs[0].height;
 
         if (!found) //detection
         {
            // Initialization
            kf.errorCovPre.at<float>(0) = 1; // px
            kf.errorCovPre.at<float>(7) = 1; // px
            kf.errorCovPre.at<float>(14) = 1;
            kf.errorCovPre.at<float>(21) = 1;
            kf.errorCovPre.at<float>(28) = 1; // px
            kf.errorCovPre.at<float>(35) = 1; // px
 
            state.at<float>(0) = meas.at<float>(0);
            state.at<float>(1) = meas.at<float>(1);
            state.at<float>(2) = 0;
            state.at<float>(3) = 0;
            state.at<float>(4) = meas.at<float>(2);
            state.at<float>(5) = meas.at<float>(3);
            
            found = true;
         }
         else
            kf.correct(meas); // Kalman Correction
 
         //cout << "Measure matrix:" << endl << meas << endl;
       }

	    // }
	      //cv::imshow( "result", img );
	      break;        
    }
    
    cv::imshow( "result", img );
    //this last part is if you want to save your frames with the detection and later make a video
    /*
    /string savePath  = "../ws/src/agitr/pp/frame" + boost::lexical_cast<std::string>(frameCount+100)+ ".jpg";
    cv::imwrite( savePath, img);
    frameCount++;
    */
}


int main(int argc, char **argv)
{
	//get args
	ros::init(argc, argv, "face_detect_process");

	initializaKalmanFilter();	

	//Subscribe to the ARdrone image raw topic
	ros::NodeHandle imgtr_nh;
	
	cv::namedWindow("result", CV_WINDOW_AUTOSIZE);

	image_transport::ImageTransport imgtr(imgtr_nh);
	image_transport::Subscriber sub = imgtr.subscribe("/ardrone/image_rect_color", 1, imageCallback);

	if( !cascade.load( cascadeName ) )
    	{
        cerr << "ERROR: Could not load classifier cascade" << endl;
        return -1;
   	 }
	printf( "Se intampla ceva? \n");
	ros::Rate r(30);	

	//spin ros core
	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}

