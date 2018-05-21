//
//  main.cpp
//  PSOQuatGraphSeg
//
//  Created by Giorgio on 25/10/15.
//  Copyright (c) 2015 Giorgio. All rights reserved.

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <boost/thread.hpp>
#include <assimp/config.h>
#include <assimp/mesh.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "GraphCannySeg.h"
#include "ros/ros.h"
#include "canny_seg_wrapper/CannySegWrapper.h"

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

// Camera parameters for ACCV dataset
#define FX 572.41140
#define FY 573.57043
#define CX 325.26110
#define CY 242.04899

// Camera parameters for Rutgers dataset
//#define FX 575.8157348632812
//#define FY 575.8157348632812
//#define CX 319.5
//#define CY 239.5

// Camera parameters for ICCV challenge dataset
//#define FX 571.9737
//#define FY 571.0073
//#define CX 319.5000
//#define CY 239.5000


#define tomm_ 1000.0f
#define tocast_ 0.5f

//ACCV
double fx=554.9753068503721;
double fy=321.4124782299449;
double cx = 554.6585798566839;
double cy = 213.5722032581315;



float k_vec[9] = {static_cast<float>(fx), 0, static_cast<float>(cx), 0, static_cast<float>(fy), static_cast<float>(cy), 0.f,0.f,1.f};

/*Segmentation Params*/
//params challenge

/* DEFAULT PARAMS
int k=50000;
int kx=1050;
int ky=1500;
int ks=500;
float kdv=4.5f;
float kdc=0.1f;
float min_size=500.0f;
float sigma=0.8f;
float max_ecc = 0.978f;
float max_L1 = 3800.0f;
float max_L2 = 950.0f;


int DTH = 30; //[mm]
int plusD = 30; //7; //for depth boundary
int point3D = 5; //10//for contact boundary
int g_angle = 154; //140;//148;//2.f/3.f*M_PI;
int l_angle = 56; //M_PI/3.f;
int Lcanny = 50;
int Hcanny = 75;
int FarObjZ = 1800; //875;//1800; //[mm]
*/


// SHU FETCH PARAMS -- jayyoung
int k=1000;
int kx=2000;
int ky=30;
int ks=50;
float kdv=4.5f;
float kdc=0.1f;
float min_size=500.0f;
float sigma=0.8f;
float max_ecc = 0.978f;
float max_L1 = 3800.0f;
float max_L2 = 950.0f;


int DTH = 30; //[mm]
int plusD = 30; //7; //for depth boundary
int point3D = 0; //10//for contact boundary
int g_angle = 154; //140;//148;//2.f/3.f*M_PI;
int l_angle = 56; //M_PI/3.f;
int Lcanny = 66;
int Hcanny = 34;
int FarObjZ = 40; //875;//1800; //[mm]

std::string trackBarsWin = "Trackbars";
//Segmentation Results
GraphCanny::GraphCannySeg<GraphCanny::hsv>* gcs=0;
std::vector<GraphCanny::SegResults> vecSegResult;
//used to load rgb and depth images fromt the dataset
cv::Mat kinect_rgb_img;
cv::Mat kinect_depth_img_mm;


std::vector<cv_bridge::CvImage> on_trackbar( int, void* )
{

    if(gcs)
      delete gcs;

    float kfloat = (float)k/10000.f;
    float kxfloat = (float)kx/1000.f;
    float kyfloat = (float)ky/1000.f;
    float ksfloat = (float)ks/1000.f;
    float gafloat = ((float)g_angle)*deg2rad;
    float lafloat = ((float)l_angle)*deg2rad;
    float lcannyf = (float)Lcanny/1000.f;
    float hcannyf = (float)Hcanny/1000.f;
    //GraphCanny::GraphCannySeg<GraphCanny::hsv> gcs(kinect_rgb_img, kinect_depth_img_mm, sigma, kfloat, min_size, kxfloat, kyfloat, ksfloat,k_vec,lcannyf,hcannyf,kdv, kdc,max_ecc,max_L1,max_L2,(uint16_t)DTH,(uint16_t)plusD,(uint16_t)point3D,gafloat,lafloat,(float)FarObjZ);

    ROS_INFO("Setting up seg object");
    gcs = new GraphCanny::GraphCannySeg<GraphCanny::hsv>(kinect_rgb_img, kinect_depth_img_mm, sigma, kfloat, min_size, kxfloat, kyfloat, ksfloat,k_vec,lcannyf,hcannyf,kdv, kdc,max_ecc,max_L1,max_L2,(uint16_t)DTH,(uint16_t)plusD,(uint16_t)point3D,gafloat,lafloat,(float)FarObjZ);
    ROS_INFO("Running..");
    gcs->run();
    ROS_INFO("Complete!");
    vecSegResult = gcs->vecSegResults;

    std::vector<cv_bridge::CvImage> output_segs;

    for(int i=0; i < vecSegResult.size(); i++) {
      ROS_INFO("seg size: %d \n",vecSegResult[i].num_points);
      cv::Mat rgb_img = vecSegResult[i].clusterRGB;
      std::string fname = "cluster-";
      fname += std::to_string(i);
      fname += ".png";
      //Mat gray_image;
      //cvtColor( rgb_image, gray_image, CV_BGR2GRAY );
      //imwrite(fname, rgb_img );

      cv_bridge::CvImage out_msg;
      //out_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
      out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
      out_msg.image    = rgb_img; // Your cv::Mat
      output_segs.push_back(out_msg);
      //saliency_img_pub.publish(out_msg.toImageMsg());
    }

    //text.zeros(480, 640,CV_8UC1);
    cv::Mat text = cv::Mat::zeros(230, 640,CV_8UC1);
    char text_[200]={};
    ROS_INFO(text_, "DTH: %d plusD: %d point3D: %d",DTH,plusD,point3D);
    std::string tstring(text_);
    std::cout<<tstring<<"\n";
    cv::putText(text, tstring, cv::Point(50,50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));

    ROS_INFO(text_, "K: %f Kx: %.2f Ky: %f Ks: %f",kfloat,kxfloat,kyfloat,ksfloat);
    tstring = std::string(text_);
    std::cout<<tstring<<"\n";
    cv::putText(text, tstring, cv::Point(50,100), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));

    ROS_INFO(text_, "G_angle: %d  L_angle: %d  Zmax: %d",g_angle,l_angle, FarObjZ);
    tstring = std::string(text_);
    std::cout<<tstring<<"\n";
    cv::putText(text, tstring, cv::Point(50,150), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));

    ROS_INFO(text_, "Low Canny : %f  High Canny: %f",lcannyf,hcannyf);
    tstring = std::string(text_);
    std::cout<<tstring<<"\n";
    cv::putText(text, tstring, cv::Point(50,200), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));

    return output_segs;

  //  cv::imshow( trackBarsWin, text );
  //  cv::waitKey(5);

}

std::vector<sensor_msgs::Image> initTrackbarsSegmentation(cv::Mat rgb_img, cv::Mat depth_img)
{
    /* Load the RGB and DEPTH */


    //CHALLENGE DATASET-1
    // std::string rgb_name= "img_1164.png";
    // std::string obj_name= "Shampoo";

    // rgb_file_path = "/Volumes/HD-PNTU3/datasets/"+obj_name+"/RGB/"+rgb_name;
    // depth_file_path = "/Volumes/HD-PNTU3/datasets/"+obj_name+"/Depth/"+rgb_name;

    kinect_rgb_img = rgb_img;//,cv::IMREAD_UNCHANGED);
    kinect_depth_img_mm = depth_img;// in mm

  //cv::imshow("kinect_rgb_img",kinect_rgb_img);
    //cv::imshow("kinect_depth_img_mm",kinect_depth_img_mm);
    //cv::waitKey(0);
    //for(int i=250;i<350;i++)
    //    for(int j=200;j<250;j++)
    //        ROS_INFO("%d ",kinect_depth_img_mm.at<uint16_t>(j,i));
    std::vector<cv_bridge::CvImage> results;
    results = on_trackbar( 0, 0 );


    std::vector<sensor_msgs::Image> ros_results;
    for(int i=0; i < results.size(); i++) {
      ROS_INFO("Packing result image");
      sensor_msgs::Image im = *results[i].toImageMsg();
      ros_results.push_back(im);
    }

    /// Wait until user press some key
    //cv::waitKey(0);
    return ros_results;
}

inline float min3(const float &a, const float &b, const float &c)
{ return std::min(a, std::min(b, c)); }

inline float max3(const float &a, const float &b, const float &c)
{ return std::max(a, std::max(b, c)); }

const int imageWidth = 640;
const int imageHeight = 480;

inline
float edgeFunction(
        const float* a, const float* b, const float* c)
{
    return(
        (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]));
}
inline void printVector(const float* w, int size)
{
    ROS_INFO("[ ");
    for (int i=0; i<size; ++i) {
        if (i==size-1) {
            ROS_INFO("%f ",w[i]);
        }
        else
            ROS_INFO("%f, ",w[i]);
    }
    ROS_INFO("]\n");
}

bool ros_segment(canny_seg_wrapper::CannySegWrapper::Request  &req,
         canny_seg_wrapper::CannySegWrapper::Response &res )
{
  ROS_INFO("Canny seg wrapper received request");
  //cv_bridge::CvImagePtr rgb;
  //cv_bridge::CvImagePtr depth;
  sensor_msgs::ImageConstPtr rgb_const( new sensor_msgs::Image( req.rgb ) );
  sensor_msgs::ImageConstPtr depth_const( new sensor_msgs::Image( req.depth ) );
  ROS_INFO("Preparing images");
  cv_bridge::CvImagePtr rgb_ptr;
  rgb_ptr = cv_bridge::toCvCopy(rgb_const);
  cv_bridge::CvImagePtr depth_ptr;
  depth_ptr = cv_bridge::toCvCopy(depth_const); // container of typ
  ROS_INFO("Performing seg");

  res.output = initTrackbarsSegmentation(rgb_ptr->image, depth_ptr->image);

  ROS_INFO("Done!");

  //delete gcs;

  return true;
}


int main( int argc, char *argv[] )
{
    ros::init(argc, argv, "canny_seg_wrapper_node");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("/canny_seg_wrapper/segment", ros_segment);
    ROS_INFO("Ready to segment");
    ros::spin();

    delete gcs;

//  cv::Mat r = cv::imread("./rgb.png");//,cv::IMREAD_UNCHANGED);
//  cv::Mat d = cv::imread("./depth.png",cv::IMREAD_UNCHANGED);// in mm

//  initTrackbarsSegmentation(r, d);


    return 0;
}
