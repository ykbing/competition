//opencv_line_detect.cpp
//-I/usr/local/include/opencv4/opencv -I/usr/local/include/opencv4

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h> 
#include <string.h> 
#include <stdio.h>
#include <unistd.h>   
#include <stdint.h>   
#include <stdlib.h>  
#include <errno.h>


//multi thread
#include <pthread.h>


using namespace cv;
using namespace std;

typedef unsigned char BYTE;

#define IMG_Width   1280
#define IMG_Height  720

#define USE_DEBUG  1   // 1 Debug  사용
#define USE_CAMERA 1   // 1 CAMERA 사용  0 CAMERA 미사용

#define NO_LINE 20
#define DEG2RAD(x) (M_PI/180.0)*x
#define RAD2DEG(x) (180.0/M_PI)*x
#define Limit_Line_Angle 140

Mat mat_image_org_color;  // Image 저장하기 위한 변수
Mat cv_image;  // Image 저장하기 위한 변수
Mat mat_image_display_color;
Mat mat_image_org_gray;
Mat mat_image_roi_gray;
Mat mat_image_roi_gray_left1;
Mat mat_image_roi_gray_right1;
Mat mat_image_roi_threshold_left1;
Mat mat_image_roi_threshold_right1;

Mat mat_image_roi;
Mat mat_image_roi_left1;
Mat mat_image_roi_right1;
Mat mat_image_canny_edge;
Mat mat_image_canny_edge_roi;
Mat mat_image_canny_edge_roi_left1;
Mat mat_image_canny_edge_roi_right1;

Scalar GREEN(0,255,0);
Scalar RED(0,0,255);
Scalar BLUE(255,0,0);
Scalar YELLOW(0,255,255);

////////////////////////// Yellow Color Detection ////////////////////////
int iLowH = 20;  int iHighH = 30;
int iLowS = 100;  int iHighS = 255;
int iLowV = 100;  int iHighV = 255;
////////////////////////// Yellow Color Detection ////////////////////////

int  img_width  = 640;
int  img_height = 360;
int  lane_distance = 0;
int  m_roi_center, m_roi_height, m_roi_width,m_roi_width_large;
float  l_line_center, l_line_center_old;
float  r_line_center, r_line_center_old;
int  line_count_r = 0;
int  line_count_l = 0;
int  left_line_detect_flag = 0;
int  right_line_detect_flag = 0;
int steer_angle_new =0;
//int steer_angle_new_old = 0;
int steer_angle_old; 
float lane_center_x = 0.0;
float lane_center_x_old = 0.0;     
int    no_blob = 0, mission_flag =0, lane_count = 0;
float sonar_l_dis = 0.0;
float sonar_r_dis = 0.0;
float sonar_avg_dis = 0.0;
float sonar_error = 0.0;
float steer_angle = 0.0;
struct Rect_Region
{
    int left;
	int right;
    int top;
	int bottom;
		
};


struct line_detection_data
{
	
   int x1;
   int y1;
   int x2;
   int y2;
	
};

struct Rect_Region ROI_lane_left1;
struct Rect_Region ROI_lane_right1;

struct line_detection_data line_data_r[NO_LINE],line_data_l[NO_LINE];
int r_line_intersect[NO_LINE] = {0,};
int l_line_intersect[NO_LINE] = {0,};

Mat Canny_Edge_Detection_r(Mat img);
Mat Canny_Edge_Detection_l(Mat img);
void read_roi_data(void);
Mat Region_of_Interest(Mat image, Point *points);
Mat Region_of_Interest_crop(Mat image, Point *points);


void Init_lane_detect_ROI_l(void);
void Init_lane_detect_ROI_r(void);


void Update_lane_detect_ROI_l(int l_center_x);
void Update_lane_detect_ROI_r(int r_center_x);

//////////////////////////////////////// PID control ////////////////////////////////////////////////
float  Kp = 0.2;  float Ki = 0; float  Kd = 0.1;
float  p_gain =0.4; float i_gain=0.2; float d_gain = 0.1;
float  error_lane    = 0; 
float  error_lane_delta    = 0; 
float  error_lane_old = 0;

int so(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

 
void read_roi_data(void)
{
	
	FILE *fp;
	int result = 0;
	fp = fopen("//home//agilex//AA_data//vision_roi.txt","r");
	
	if(fp == NULL) 
	{
		ROS_INFO("ROI File does not exit ~~ \n\n");
		exit(0);
	}
	
	result = fscanf(fp,"%d", &m_roi_center);	//430
	result = fscanf(fp,"%d", &m_roi_height);	//20
	result = fscanf(fp,"%d", &m_roi_width);		//100
	result = fscanf(fp,"%d", &m_roi_width_large);	//200
	result = fscanf(fp,"%d", &lane_distance);	//362
	
	
	 ROI_lane_left1.top        = m_roi_center - m_roi_height;
	 ROI_lane_left1.bottom     = m_roi_center + m_roi_height;
	 ROI_lane_left1.left       =  0 ;
	 ROI_lane_left1.right      = ROI_lane_left1.left + m_roi_width;
	 
	 ROI_lane_right1.top       = m_roi_center - m_roi_height;
	 ROI_lane_right1.bottom    = m_roi_center + m_roi_height;
	 ROI_lane_right1.right     = img_width;
	 ROI_lane_right1.left      = ROI_lane_right1.right  - m_roi_width;
	 
	 fclose(fp);
	 
	 Update_lane_detect_ROI_l(img_width/2 - lane_distance/2);
	 Update_lane_detect_ROI_r(img_width/2 + lane_distance/2);
	// printf("img_width = %d\n",img_width);
	 printf("%3d %3d %3d %3d \n",ROI_lane_left1.left, ROI_lane_left1.top, ROI_lane_left1.right , ROI_lane_left1.bottom);
	 printf("%3d %3d %3d %3d \n",ROI_lane_right1.left, ROI_lane_right1.top, ROI_lane_right1.right , ROI_lane_right1.bottom);	 
	 
}
void vision_right_lane_detection(void)
{
	//Point points[4];  // ROI(Region of Interest)	

	float  c[NO_LINE] = { 0.0, };
	float  d[NO_LINE] = { 0.0, };
	float  r_line_center_x = 0.0;
	int inter_sect_x[NO_LINE] = { 0, };

	right_line_detect_flag = 0;
	r_line_center_old;

    Mat imgHSV_r;  
       	
	//cvtColor(mat_image_roi_right1, imgHSV_r, COLOR_BGR2HSV); 	
	//inRange(imgHSV_r, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), mat_image_roi_gray_right1);
	 
	cvtColor(mat_image_roi_right1, mat_image_roi_gray_right1, cv::COLOR_RGB2GRAY);        // color to gray conversion 		 
		
	mat_image_canny_edge_roi_right1 = Canny_Edge_Detection_r(mat_image_roi_gray_right1);


	vector<Vec4i> linesP_r;
	HoughLinesP(mat_image_canny_edge_roi_right1, linesP_r, 1, CV_PI / 180, m_roi_height, m_roi_height, m_roi_height);
	//printf("Right Line Number : %3d\n", (int)linesP_r.size());


	int offset_crop_y = ROI_lane_right1.top;
	int offset_crop_x = ROI_lane_right1.left;

	line_count_r = 0;

	for (int i = 0; i<linesP_r.size(); i++)
	{
		float intersect = 0.0;

		if (i >= NO_LINE) break;
		Vec4i R = linesP_r[i];
		
		if(fabs(R[3]-R[1])>1.0e-7)  		   c[i] =  (R[2]-R[0])/(R[3]-R[1]);
        else                                   c[i] = 1.0e7;
        
		d[i] = R[0]-c[i] *R[1] ;	

		//c[i] = (float)(R[2] - R[0]) / (float)(R[3] - R[1]);		   d[i] = R[0] - c[i] * R[1];

		if (fabs(c[i])< DEG2RAD(Limit_Line_Angle))
		{
			intersect = c[i] * (float)(m_roi_height/1.) + d[i];
			r_line_intersect[i] = (int)intersect + offset_crop_x;
			//printf("%3d %3d %3d %3d %.2lf %.2lf %.2lf\n\n", R[0], R[1], R[2], R[3], c[i], d[i], intersect);
			r_line_center_x += intersect;

			line_data_r[line_count_r].x1 = R[0] + offset_crop_x;  line_data_r[line_count_r].y1 = R[1] + offset_crop_y;
			line_data_r[line_count_r].x2 = R[2] + offset_crop_x;  line_data_r[line_count_r].y2 = R[3] + offset_crop_y;

			line_count_r++;

		}
	}

    if (line_count_r != 0)
	{	
	  r_line_center = r_line_center_x / (float)line_count_r + offset_crop_x;
	  r_line_center_old = r_line_center;
	  Update_lane_detect_ROI_r((int)r_line_center);
	  right_line_detect_flag = line_count_r;
	}	
	else
	{
     				
	  threshold(mat_image_roi_gray_right1,mat_image_roi_threshold_right1 , 225, 255, THRESH_BINARY);
	  imshow("Right Edge Image",mat_image_roi_threshold_right1);
	  
	  Mat img_labels,stats, centroids;  
	  
	  int numOfLables = connectedComponentsWithStats(mat_image_roi_threshold_right1, img_labels, stats, centroids, 8,CV_32S);  
      
      for (int j = 1; j < numOfLables; j++) 
      { 
		    int left=stats.at<int>(j,CC_STAT_LEFT);
		    int width=stats.at<int>(j,CC_STAT_WIDTH); 
		    
		    //printf("R Line Center(Blob) : %.d %d %d %d \n\n", j,left,width,left+width/2 );
		    r_line_center = left+width/2  + offset_crop_x;
	  } 
	  
	  Init_lane_detect_ROI_r();   
       
	 }

      //printf("R Line Center : %.2lf %d %.2lf \n\n", line_center_x,line_count_r, r_line_center );
	
}


void vision_left_lane_detection(void)
{
	//Point points[4];  // ROI(Region of Interest)	

	float  c[NO_LINE] = { 0.0, };
	float  d[NO_LINE] = { 0.0, };
	float  line_center_x = 0.0;
	int inter_sect_x[NO_LINE] = { 0, };

	left_line_detect_flag = 0;
    Mat imgHSV_l;  
	//cvtColor(mat_image_roi_left1, imgHSV_l, COLOR_BGR2HSV); 	
	//inRange(imgHSV_l, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), mat_image_roi_gray_left1);
	
	cvtColor(mat_image_roi_left1, mat_image_roi_gray_left1, cv::COLOR_RGB2GRAY);        // color to gray conversion 
	
	mat_image_canny_edge_roi_left1 = Canny_Edge_Detection_r(mat_image_roi_gray_left1);


	vector<Vec4i> linesP_l;
	HoughLinesP(mat_image_canny_edge_roi_left1, linesP_l, 1, CV_PI / 180, m_roi_height, m_roi_height, m_roi_height);
	//printf("Left Line Number : %3d\n", (int)linesP_l.size());


	int offset_crop_y = ROI_lane_left1.top;
	int offset_crop_x = ROI_lane_left1.left;

	line_count_l = 0;

	for (int i = 0; i<linesP_l.size(); i++)
	{
		float intersect = 0.0;

		if (i>=NO_LINE) break;
		Vec4i L = linesP_l[i];

        if(fabs(L[3]-L[1])>1.0e-7)  		   c[i] =  (L[2]-L[0])/(L[3]-L[1]);
        else                                   c[i] = 1.0e7;
        
		d[i] = L[0]-c[i] *L[1] ;		  
         
      //  printf("%3d %3d %3d %3d %3d %.2lf %.2lf %.2lf\n\n", offset_crop_x,L[0], L[1], L[2], L[3], RAD2DEG(c[i]), d[i], intersect);
		if (fabs(c[i])< DEG2RAD(Limit_Line_Angle))
		{
			intersect = c[i] * (float)(m_roi_height/1.) + d[i];
			l_line_intersect[i] = (int)intersect + offset_crop_x;
			//printf("%3d %3d %3d %3d %.2lf %.2lf %.2lf\n\n", L[0], L[1], L[2], L[3], c[i], d[i], intersect);
			line_center_x += intersect;
           // printf("%3.1f ", intersect)   ; 
			line_data_l[line_count_l].x1 = L[0] + offset_crop_x;  line_data_l[line_count_l].y1 = L[1] + offset_crop_y;
			line_data_l[line_count_l].x2 = L[2] + offset_crop_x;  line_data_l[line_count_l].y2 = L[3] + offset_crop_y;

			line_count_l++;

		}

	}

	if (line_count_l != 0)
	{
		l_line_center = line_center_x / (float)line_count_l + offset_crop_x;
		l_line_center_old = l_line_center;
		Update_lane_detect_ROI_l(l_line_center);
		left_line_detect_flag = line_count_l;
	}
	else
	{
		
		threshold(mat_image_roi_gray_left1,mat_image_roi_threshold_left1 , 225, 255, THRESH_BINARY);
	  
	  
        imshow("Left Edge Image",mat_image_roi_threshold_left1);
      
	    Mat img_labels,stats, centroids;  
	  
	    int numOfLables = connectedComponentsWithStats(mat_image_roi_threshold_left1, img_labels, stats, centroids, 8,CV_32S);  
      
        for (int j = 1; j < numOfLables; j++) 
        {  
		    int left=stats.at<int>(j,CC_STAT_LEFT);
		    int width=stats.at<int>(j,CC_STAT_WIDTH); 
		    
		    printf("L Line Center(Blob) : %.d %d %d %d \n\n", j,left,width,left+width/2 );
		    l_line_center = left+width/2  + offset_crop_x;
    	} 
    	Init_lane_detect_ROI_l();   
    	     
	}

	//printf("L Line Center : %.2lf %d %.2lf \n\n", line_center_x,line_count_l, l_line_center );
}


Mat Region_of_Interest_crop(Mat image, Point *points)
{
   Mat img_roi_crop;	
  
   Rect bounds(0,0,image.cols,image.rows);	 
   Rect r(points[0].x, points[0].y, points[2].x-points[0].x, points[2].y-points[0].y);    
  //printf("%d %d %d %d\n",points[0].x,points[0].y,points[2].x-points[0].x, points[2].y-points[0].y);
  // printf("%d  %d\n", image.cols, points[2].y-points[0].y);  
   img_roi_crop = image(r & bounds); 
  
   return img_roi_crop;
}

Mat Canny_Edge_Detection_r(Mat img)
{
   Mat mat_blur_img, mat_canny_img;
   blur(img, mat_blur_img, Size(3,3));	
   Canny(mat_blur_img,mat_canny_img, 70,150,3); // 70 159 3
	
   return mat_canny_img;	
}

Mat Canny_Edge_Detection_l(Mat img)
{
   Mat mat_blur_img, mat_canny_img;
   blur(img, mat_blur_img, Size(3,3));	
   Canny(mat_blur_img,mat_canny_img, 70,150,3); //
	
   return mat_canny_img;	
}

void Update_lane_detect_ROI_l(int l_center_x)
{
	int m_point;
	
	ROI_lane_left1.top        = m_roi_center - m_roi_height;
	ROI_lane_left1.bottom     = m_roi_center + m_roi_height;	
    
	m_point = ( (l_center_x - m_roi_width) > 0 )  ? (l_center_x - m_roi_width) : 0;
	//printf("m_point %d \n",m_point);
	ROI_lane_left1.left       = m_point;
	ROI_lane_left1.right      = (l_center_x + m_roi_width);	
}

void Update_lane_detect_ROI_r(int r_center_x)
{
	int m_point;
	
	//r_center_x = 517;
    ROI_lane_right1.top       = m_roi_center - m_roi_height;
	ROI_lane_right1.bottom    = m_roi_center + m_roi_height;

    m_point = ( (r_center_x + m_roi_width) > img_width )  ? img_width-1 : r_center_x + m_roi_width; 
	//printf("m_point %d \n",m_point);
	ROI_lane_right1.right     = m_point;
	ROI_lane_right1.left      = (r_center_x - m_roi_width);   
	
}

void Init_lane_detect_ROI_r(void)
{
	ROI_lane_right1.top       = m_roi_center - m_roi_height;
	ROI_lane_right1.bottom    = m_roi_center + m_roi_height;
	ROI_lane_right1.right     = img_width-1;
	ROI_lane_right1.left      = ROI_lane_right1.right  - m_roi_width_large;	
}

void Init_lane_detect_ROI_l(void)
{
	ROI_lane_left1.top        = m_roi_center - m_roi_height;
	ROI_lane_left1.bottom     = m_roi_center + m_roi_height;
	ROI_lane_left1.left       =  0 ;
	ROI_lane_left1.right      = ROI_lane_left1.left + m_roi_width_large;	
}


void Draw_ROI(int line_center_x)
{
	//printf("w : %3d h : %3d \n",img_width,img_height);

    Rect m_ROI_left,m_ROI_right;
    //Update_lane_detect_ROI_r(int r_center_x);
     
    //line_center_x = 100;    
		
    m_ROI_right = Rect( Point(ROI_lane_right1.left,ROI_lane_right1.top), Point(ROI_lane_right1.right,ROI_lane_right1.bottom));
    m_ROI_left  = Rect( Point(ROI_lane_left1.left,ROI_lane_left1.top), Point(ROI_lane_left1.right,ROI_lane_left1.bottom));     
    
	line(mat_image_display_color,Point(0,m_roi_center),Point(img_width,m_roi_center), Scalar(0,255,0), 1, LINE_AA);	
	line(mat_image_display_color,Point(line_center_x+img_width/2,m_roi_center-m_roi_height),Point(line_center_x+img_width/2,m_roi_center+m_roi_height), Scalar(255,255,0), 2, LINE_AA);	   
	
	if(right_line_detect_flag !=0) 
	{
		rectangle(mat_image_display_color,m_ROI_right,Scalar(0,255,0),2,8,0);
    }
    else
    {
        rectangle(mat_image_display_color,m_ROI_right,Scalar(0,0,255),2,8,0);
	}
	
	if(left_line_detect_flag !=0) 
	{
	    rectangle(mat_image_display_color,m_ROI_left,Scalar(0,255,255),2,8,0);
	}
	else
	{
		rectangle(mat_image_display_color,m_ROI_left,Scalar(0,0,255),2,8,0);
	}
	
	line(mat_image_display_color,Point(l_line_center,m_roi_center-m_roi_height),Point(l_line_center,m_roi_center+m_roi_height), Scalar(0,255,255), 2, LINE_AA);	   	
    line(mat_image_display_color,Point(r_line_center,m_roi_center-m_roi_height),Point(r_line_center,m_roi_center+m_roi_height), Scalar(0,255,0), 2, LINE_AA);	   
   
   /*
	for(int i =0 ; i < left_line_detect_flag ;i++)
	{		
	   line(mat_image_display_color, Point(line_data_l[i].x1, line_data_l[i].y1), Point(line_data_l[i].x2, line_data_l[i].y2), Scalar(255,0,255), 1, LINE_AA);	   
	   line(mat_image_display_color, Point(l_line_intersect[i],m_roi_center-m_roi_height),Point(l_line_intersect[i],m_roi_center + m_roi_height), Scalar(0,102,255), 1, LINE_AA);	   
	}
	
	for(int i =0 ; i < right_line_detect_flag ;i++)
	{		
	   line(mat_image_display_color, Point(line_data_r[i].x1, line_data_r[i].y1), Point(line_data_r[i].x2, line_data_r[i].y2), Scalar(255,0,255), 1, LINE_AA);	   
	   line(mat_image_display_color, Point(r_line_intersect[i],m_roi_center-m_roi_height),Point(r_line_intersect[i],m_roi_center + m_roi_height), Scalar(0,102,255), 1, LINE_AA);	   
	}
	*/
	
}

ros::Publisher car_control_pub_cmd;
ros::Publisher car_speed_pub_cmd;

std_msgs::Float32 cmd_steering_msg;  
geometry_msgs::Twist cmd_speed_msg;

void lane_detection(void)
{
     
   float gradient[NO_LINE]  = {0,};
   float intersect[NO_LINE] = {0,};
   float intersect_base[NO_LINE]  = {0,};
   float c_x_sum=0;	     
   
    Point l_points[4];  // ROI(Region of Interest) 
    Point r_points[4];  // ROI(Region of Interest) 
    Point s_points[4];  // ROI(Region of Interest) 
  	 
	l_points[0] = Point(ROI_lane_left1.left,ROI_lane_left1.top);	  
	l_points[1] = Point(ROI_lane_left1.left,ROI_lane_left1.bottom);	  
	l_points[2] = Point(ROI_lane_left1.right,ROI_lane_left1.bottom);	  
	l_points[3] = Point(ROI_lane_left1.right,ROI_lane_left1.top);
	mat_image_roi_left1 = Region_of_Interest_crop(mat_image_org_color,l_points);    // ROI 영역을 추출함
	     
	   //  imshow("Camera Image", mat_image_roi_left1);
	r_points[0] = Point(ROI_lane_right1.left,ROI_lane_right1.top);	  
	r_points[1] = Point(ROI_lane_right1.left,ROI_lane_right1.bottom);	  
	r_points[2] = Point(ROI_lane_right1.right,ROI_lane_right1.bottom);	  
	r_points[3] = Point(ROI_lane_right1.right,ROI_lane_right1.top);	  
	mat_image_roi_right1 = Region_of_Interest_crop(mat_image_org_color,r_points);    // ROI 영역을 추출함   
	mat_image_display_color = mat_image_org_color.clone();
	
	//printf("left  ROI : %3d %3d %3d %3d \n\n",ROI_lane_left1.left, ROI_lane_left1.top, ROI_lane_left1.right , ROI_lane_left1.bottom);
	//printf("right ROI : %3d %3d %3d %3d \n\n",ROI_lane_right1.left, ROI_lane_right1.top, ROI_lane_right1.right , ROI_lane_right1.bottom);

    vision_right_lane_detection();
    vision_left_lane_detection();
     ////////////////////////////////////////// 
    if(l_line_center == r_line_center)
    {
		  Init_lane_detect_ROI_l(); 
		  Init_lane_detect_ROI_r();
	      vision_right_lane_detection();
          vision_left_lane_detection();      
	}	  
	
	if( (left_line_detect_flag != 0) && (right_line_detect_flag != 0))
    {
		  
	  lane_center_x = (l_line_center + r_line_center)/2 -img_width/2; ////pixcel
	  steer_angle_new = (int)( error_lane *Kp + Kd*error_lane_delta/1.42857);
		  
	}
	else if( (left_line_detect_flag != 0) && (right_line_detect_flag == 0))
    {
		  
	  lane_center_x = ((l_line_center + lane_distance/2) -img_width/2)-30;
	  //steer_angle_new = 0.0994*lane_center_x-18.07;
	  // angle = a*lane_center_x + b 
	  steer_angle_new = 0.79*(0.0965*lane_center_x+18.902);
	  printf("HEHE LLLLLLLLLLLLL\n");
		  
	}
	else if( (left_line_detect_flag == 0) && (right_line_detect_flag != 0))
    {
		  
	  lane_center_x = ((-(lane_distance-120)/2 + r_line_center) -img_width/2) ;
	  steer_angle_new = 0.70*(0.0994*lane_center_x-18.07);
	  //steer_angle_new = 0.0965*lane_center_x+18.902;
		printf("HEHE RRRRRRRRRRRRRRRRRRRRRRRRR\n");
	}
	
	else 
	{
	  lane_center_x = lane_center_x_old;
	  steer_angle_new = steer_angle_old;
	} 
	 ////////////////////////////////////////////////////// 
	printf("L:%4.1f R:%4.1f C: %4.3f  \n left_line_detect_flag :%d right_line_detect_flag : %d\n", l_line_center, r_line_center, lane_center_x, left_line_detect_flag,right_line_detect_flag);
	error_lane = lane_center_x;
	sonar_avg_dis = (sonar_l_dis + sonar_r_dis)/2.0;

	//steer_angle_new = (int)( error_lane *Kp + Kd*error_lane_delta/1.42857);
	steer_angle = DEG2RAD(steer_angle_new);
/*	if(steer_angle <0.2 && steer_angle > -0.2){
		steer_angle = 0;
		ROS_INFO("You good!!!");
		
	}*/
	
	printf("steer_angle_new : %d ,steer_angle : %.3f \n", steer_angle_new,steer_angle);
	
	error_lane_delta = error_lane - error_lane_old;
    lane_center_x_old = lane_center_x;        
    
    error_lane_old = error_lane ;
    
    Draw_ROI(lane_center_x);      
    ROS_INFO("%d",steer_angle_new);
    
     
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;
	
	//sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat_image_org_color).toImageMsg();

    img_width = cv_image.size().width ;
    img_height = cv_image.size().height;
   
   // printf("w : %3d h : %3d \n",img_width,img_height);

    mat_image_org_color = cv_image.clone(); 
    
    lane_detection();
    // cv::imshow("view", mat_image_org_color);
    
    imshow("view", mat_image_display_color);
    //imshow("Left Edge Image",mat_image_canny_edge_roi_left1);
    //imshow("Right Edge Image",mat_image_canny_edge_roi_right1);
    //imshow("Right Edge Image", mat_image_roi_gray_right1); 
    //imshow("Left Edge Image", mat_image_roi_gray_left1);
    cv::waitKey(30);
}

void blob_Callback(const std_msgs::Int8 & msg)
{
	no_blob = msg.data;
	//printf("blob data : %d\n", no_blob);
}
void mission_Callback(const std_msgs::Int16 & msg)
{
	mission_flag = msg.data;

}
void count_Callback(const std_msgs::Int16 & msg)
{
	lane_count = msg.data;

}
void sonar_l_callback(const sensor_msgs::Range &msg)
{
   sonar_l_dis = msg.range; 
}
void sonar_r_callback(const sensor_msgs::Range &msg)
{
   sonar_r_dis = msg.range; 
}
int main(int argc, char **argv)
{

   std::string cameraTopicName;
   std::string blob_topic = "/obs_flag" ;

   int cameraQueueSize;
   int control_rate	 = 20;     	 
   
   std::cout << "OpenCV version : " << CV_VERSION << std::endl;   
   
   read_roi_data();  
   
   ros::init(argc, argv, "ros_opencv_line_detect_image_topic");

  /*
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
   ros::NodeHandle nh;
   
   nh.param("subscribers/camera_reading/topic", cameraTopicName, std::string("/usb_cam/image_raw"));
   nh.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);
   
   ros::Subscriber imageSubscriber = nh.subscribe(cameraTopicName, cameraQueueSize, &imageCallback);
   ros::Subscriber sub_blob_no = nh.subscribe(blob_topic, 10, blob_Callback);
   ros::Subscriber sub_mission_num = nh.subscribe("/mission_num", 1, mission_Callback);
   ros::Subscriber sub_count_num = nh.subscribe("/count_num", 1, count_Callback);
   ros::Subscriber left_sonar_sub = nh.subscribe("/Sonar_L",10, &sonar_l_callback);
   ros::Subscriber right_sonar_sub = nh.subscribe("/Sonar_R",10, &sonar_r_callback);

   //car_control_pub_cmd = nh.advertise<std_msgs::Float32>("/yaw_target_topic", 10);
   car_speed_pub_cmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
       
   //geometry_msgs::Twist cmd_speed_msg;      
   //cmd_speed_msg.linear.x  = 0;
   //cmd_speed_msg.angular.z = 0;
   //cmd_steering_msg.data  = 0;

   std::cout << "OpenCV version : " << CV_VERSION << std::endl;
  
   ros::Rate loop_rate(20);
   int count = 0;
   int line_count = 0;
     
   
   
   
   ///////////////  image display window ///////////////////////////
      
   namedWindow("view", WINDOW_NORMAL);
   resizeWindow("Camera Image", IMG_Width/2,IMG_Height/2);
   moveWindow("view", 10, 10);
   
   while (ros::ok())
   { 
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
     
      //cmd_steering_msg.data  = steer_angle_new;
	  
	  //cmd_speed_msg.angular.z = -steer_angle;
	  cmd_speed_msg.angular.z = -steer_angle;
	  cmd_speed_msg.linear.x = 0.23;
/*	if(steer_angle_old !=  steer_angle_new)
	{ 
		//car_control_pub_cmd.publish(cmd_steering_msg);
		car_speed_pub_cmd.publish(cmd_speed_msg);
	}*/
	car_speed_pub_cmd.publish(cmd_speed_msg);
    steer_angle_old =  steer_angle_new ;
   
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  
  destroyWindow("view");
  destroyWindow("Right Edge Image");  
  //destroyWindow("Left Edge Image");
  //destroyWindow("Left Lane Image");
  //destroyWindow("Right Lane Image");
   return 0;
}

 
