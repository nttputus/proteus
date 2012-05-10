#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Point.h>
#include <ocl/Component.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <numeric>
#include <vector>
#include <cstring>
#include <string>

using namespace std;
using namespace RTT;

class RMaxManager : public RTT::TaskContext {
private:  
  InputPort<sensor_msgs::Image> inport_image;
  InputPort<sensor_msgs::CameraInfo> inport_info;
  InputPort<std_msgs::String> inport_gyro;
  InputPort<std_msgs::String> inport_gps;
  OutputPort<geometry_msgs::Point> outport;
  vector<geometry_msgs::Point> route;
  bool wp_reached;
  int curent_wp;
  double minvar;
  double x, y, z;
  double argminx, argminy;
  bool landing_site_found;

public:
  RMaxManager(const std::string& name):
    TaskContext(name),
    inport_image("image_in"),
    inport_info("camera_info_in"),
    inport_gyro("gyroscope_in"),
    inport_gps("gps_in"),
    outport("point_out")
  {
    ports()->addPort(inport_image);
    ports()->addPort(inport_info);
    ports()->addPort(inport_gyro);
    ports()->addPort(inport_gps);
    ports()->addPort(outport);
    geometry_msgs::Point* pt = new geometry_msgs::Point();
    pt->x = 20.0; pt->y = 20.0; pt->z = 10.0;
    route.push_back(*pt);
    pt->x = -20.0; pt->y = 20.0; pt->z = 10.0;
    route.push_back(*pt);
    pt->x = -20.0; pt->y = -20.0; pt->z = 10.0;
    route.push_back(*pt);
    pt->x = 20.0; pt->y = -20.0; pt->z = 10.0;   
    route.push_back(*pt);
    delete pt;
    wp_reached = true;
    curent_wp = -1;
    minvar = 10000000000000000.0;
    landing_site_found = false;
  }
  ~RMaxManager()
 {
    route.clear();
 }
private:
  void updateHook() {
    log(Info)<<"updateHook"<<endlog();
    sensor_msgs::CameraInfo msg_camera_info;
    if (NewData == inport_info.read(msg_camera_info)) {
      log(Info)<<"CameraInfo: "<<endlog(); }
    std_msgs::String msg_gyro;
    if (NewData == inport_gyro.read(msg_gyro)) {
      log(Info)<<"Gyro: "<<endlog(); }
    std_msgs::String msg_gps;
    if (NewData == inport_gps.read(msg_gps)) {
      log(Info)<<"GPS: "<<msg_gps.data<<endlog();
      char *cstr = new char [msg_gps.data.size()+1];
      strcpy(cstr,msg_gps.data.c_str());
      sscanf(cstr,"%lf, %lf, %lf",&x,&y,&z);
      if(wp_reached == false) {
        double dist2 = (x-route[curent_wp].x)*(x-route[curent_wp].x) + (y-route[curent_wp].y)*(y-route[curent_wp].y) + (z-route[curent_wp].z)*(z-route[curent_wp].z);
        log(Info)<<"x = "<<x<<" y = "<<y<<" z = "<<z<<"dist2 = "<<dist2<<endlog();
        if(dist2 < 0.5) wp_reached = true;
      }
    }
    sensor_msgs::Image msg_image;
    if (NewData == inport_image.read(msg_image)) {
      //geometry_msgs::Point cmd;
      log(Info)<<"Image height:"<<msg_image.height<<", width:"<<msg_image.width<<endlog();
      double sumr = 0.0;
      double sumg = 0.0;
      double sumb = 0.0;
      double sumr2 = 0.0;
      double sumg2 = 0.0;
      double sumb2 = 0.0;
      for(int i = 0; i < msg_image.height; i++) {
        for(int j = 0; j < msg_image.width; j++) {
          int k = i*msg_image.height + j;
          int r = (int)msg_image.data[4*k];
          int g = (int)msg_image.data[4*k+1];
          int b = (int)msg_image.data[4*k+2];
          //int a = (int)msg_image.data[4*k+3];
          sumr += (double)r; sumr2 += (double)r*(double)r;
          sumg += (double)g; sumg2 += (double)g*(double)g;
          sumb += (double)b; sumb2 += (double)b*(double)b;
          //log(Info)<<"["<<i<<","<<j<<"] R"<<r<<" G "<<g<<" B "<<b<<" A "<<a<<endlog();
        }
      }
      double meanr = sumr / (msg_image.height*msg_image.width);
      double meang = sumg / (msg_image.height*msg_image.width);
      double meanb = sumb / (msg_image.height*msg_image.width);
      double varr = sumr2 / (msg_image.height*msg_image.width) - meanr*meanr;
      double varg = sumg2 / (msg_image.height*msg_image.width) - meang*meang;
      double varb = sumb2 / (msg_image.height*msg_image.width) - meanb*meanb;
      double vartot = varr+varb+varg;
      if(vartot < minvar) {
        minvar = vartot;
        argminx = x;
        argminy = y;
      }
      log(Info)<<"Image total variance "<<vartot<<" optimum "<<minvar<<" for "<<argminx<<" "<<argminy<<endlog();
      if (wp_reached) {
        wp_reached = false;
        curent_wp++;
        if(curent_wp == route.size()) {
          if(landing_site_found == false) {
            landing_site_found = true;
            geometry_msgs::Point* pt = new geometry_msgs::Point();
            pt->x = argminx; pt->y = argminy; pt->z = 10.0;
            route.push_back(*pt);
            pt->x = argminx; pt->y = argminy; pt->z = 0.0;
            route.push_back(*pt);
            delete pt;
            outport.write(route[curent_wp]);
          }
        }
        else outport.write(route[curent_wp]);   
      }
    }
  }
};

ORO_CREATE_COMPONENT(RMaxManager)
