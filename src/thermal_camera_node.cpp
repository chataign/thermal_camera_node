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
sensor_msgs::CameraInfo cameraInfo;
image_transport::CameraPublisher cameraPub;
sensor_msgs::Image rosImage;
unsigned int mWidth;
unsigned int mHeight;
boost::shared_ptr<TauRGBBitmap> tauRGBBitmap;

void callbackTauImage(TauRawBitmap& tauRawBitmap, void* caller)
{
  if ( !tauRGBBitmap || tauRGBBitmap->width != tauRawBitmap.width || tauRGBBitmap->height != tauRawBitmap.height )
    tauRGBBitmap = boost::make_shared<TauRGBBitmap>(tauRawBitmap.width,tauRawBitmap.height);

  thermalGrabber->convertTauRawBitmapToTauRGBBitmap (tauRawBitmap, *tauRGBBitmap);

  rosImage.header.stamp = ros::Time::now();
  rosImage.width = tauRGBBitmap->width;
  rosImage.height = tauRGBBitmap->height;
  rosImage.step = tauRGBBitmap->width * 3;

  if ( tauRGBBitmap->data != NULL 
	&& tauRGBBitmap->width == thermalGrabber->getResolutionWidth() 
	&& tauRGBBitmap->height == thermalGrabber->getResolutionHeight() )
  {
      rosImage.data.resize(tauRGBBitmap->height*tauRGBBitmap->width*3);
      memcpy(rosImage.data.data(), tauRGBBitmap->data, tauRGBBitmap->height*tauRGBBitmap->width*3);
  }

  cameraPub.publish(rosImage,cameraInfo);
}

struct ThermalWrapper
{
    ThermalWrapper() { thermalGrabber = new ThermalGrabber(callbackTauImage, this); }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME );
  ros::NodeHandle nh("~");

  ThermalWrapper thermal_wrapper;

  int queue_size;
  double rate_hz;

  nh.param<int>( "queue_size", queue_size, 1 );
  nh.param<double>( "rate_hz", rate_hz, 30 );
  nh.param<std::string>( "encoding", rosImage.header.frame_id, "thermalCamera" );
  nh.param<std::string>( "encoding", rosImage.encoding, "rgb8" );

  cameraInfo.width = thermalGrabber->getResolutionWidth();
  cameraInfo.height = thermalGrabber->getResolutionHeight();

  image_transport::ImageTransport it(nh);
  cameraPub = it.advertiseCamera("image", queue_size);

  ros::Rate loop_rate(rate_hz);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
