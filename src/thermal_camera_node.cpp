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
TauRGBBitmap* tauRGBBitmap = NULL;
image_transport::Publisher rosMsgPub;
unsigned int mWidth;
unsigned int mHeight;

class Thermal
{
public:
    void thermal();
};

void callbackTauImage(TauRawBitmap& tauRawBitmap, void* caller)
{
  sensor_msgs::ImagePtr rosImage = boost::make_shared<sensor_msgs::Image>();

  tauRGBBitmap = new TauRGBBitmap(tauRawBitmap.width,tauRawBitmap.height);

  thermalGrabber->convertTauRawBitmapToTauRGBBitmap (tauRawBitmap, *tauRGBBitmap);

  rosImage->header.frame_id = "thermalCamera"; 
  rosImage->encoding = "rgb8";
  rosImage->width = tauRGBBitmap->width;
  rosImage->height = tauRGBBitmap->height;
  rosImage->step = tauRGBBitmap->width * 3;

  if (tauRGBBitmap != NULL && tauRGBBitmap->data != NULL && tauRGBBitmap->width == mWidth && tauRGBBitmap->height == mHeight)
  {
      rosImage->data.resize(tauRGBBitmap->height*tauRGBBitmap->width*3);
      memcpy(rosImage->data.data(), tauRGBBitmap->data, tauRGBBitmap->height*tauRGBBitmap->width*3);
  }

  delete tauRGBBitmap;

  rosMsgPub.publish(rosImage);
}

void Thermal::thermal()
{
    thermalGrabber = new ThermalGrabber(callbackTauImage, this);
    mWidth = thermalGrabber->getResolutionWidth();
    mHeight = thermalGrabber->getResolutionHeight();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thermal_camera_node");

  ros::NodeHandle n("~");
  image_transport::ImageTransport it(n);
  rosMsgPub = it.advertise("/thermal_camera/Image", 1);

  Thermal* thermal = new Thermal();
  thermal->thermal();

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }

  delete thermal;

  return 0;
}
