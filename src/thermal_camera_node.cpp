////////////////////////////////////////////////////////////////////////////////////
///
///  \file thermal_camera_node.h
///
///  <br>Author(s): Germán Moreno
///  <br>Created: 17 October 2016
///  <br>Copyright (c) 2016
///  <br>Eurecat
///  <br>All rights reserved.
///  <br>Email: german.moreno@eurecat.org
///  <br>Web:  http://www.eurecat.org
///
////////////////////////////////////////////////////////////////////////////////////

#include "ros/ros.h"
#include <image_transport/image_transport.h>

#include "thermalgrabber.h"

#include "stdlib.h"
#include <iostream>
#include <sstream>

ThermalGrabber* thermalGrabber;
image_transport::CameraPublisher cameraPub;
sensor_msgs::CameraInfo cameraInfo;
boost::shared_ptr<TauRGBBitmap> tauRGBBitmap;
sensor_msgs::Image rosImage;
unsigned int mWidth;
unsigned int mHeight;

class Thermal
{
public:
    void thermal();
};

void callbackTauImage(TauRawBitmap& tauRawBitmap, void* caller)
{
  if ( !tauRGBBitmap || tauRGBBitmap->width != tauRawBitmap.width || tauRGBBitmap->height != tauRawBitmap.height )
	  tauRGBBitmap = boost::make_shared<TauRGBBitmap>( tauRawBitmap.width, tauRawBitmap.height );

  thermalGrabber->convertTauRawBitmapToTauRGBBitmap (tauRawBitmap, *tauRGBBitmap);

  rosImage.width = tauRGBBitmap->width;
  rosImage.height = tauRGBBitmap->height;
  rosImage.step = tauRGBBitmap->width * 3;

  if ( tauRGBBitmap->data != NULL && tauRGBBitmap->width == mWidth && tauRGBBitmap->height == mHeight)
  {
      rosImage.data.resize(tauRGBBitmap->height*tauRGBBitmap->width*3);
      memcpy(rosImage.data.data(), tauRGBBitmap->data, tauRGBBitmap->height*tauRGBBitmap->width*3);
  }

  cameraPub.publish(rosImage,cameraInfo);
}

void Thermal::thermal()
{
    thermalGrabber = new ThermalGrabber(callbackTauImage, this);
    mWidth = thermalGrabber->getResolutionWidth();
    mHeight = thermalGrabber->getResolutionHeight();
    
    cameraInfo.width = thermalGrabber->getResolutionWidth();
    cameraInfo.height = thermalGrabber->getResolutionHeight();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME );

  ros::NodeHandle nh("~");
  
  int queue_size;
  double rate_hz;
  std::string frame_id;
  
  nh.param<int>( "queue_size", queue_size, 1 );
  nh.param<double>( "rate_hz", rate_hz, 30 );
  nh.param<std::string>( "frame_id", rosImage.header.frame_id, "thermalCamera" );
  nh.param<std::string>( "frame_id", rosImage.encoding, "rgb8" );
  
  image_transport::ImageTransport it(nh);
  cameraPub = it.advertiseCamera( "thermal_camera", queue_size );
  
  Thermal thermal;
  thermal.thermal();

  ros::Rate loop_rate(rate_hz);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
