#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Twist.h>
#include <ocl/Component.hpp>
#include <sensor_msgs/LaserScan.h>
#include <numeric>

using namespace std;
using namespace RTT;

class AvoidObstacleLaser : public RTT::TaskContext {
private:  
  InputPort<sensor_msgs::LaserScan> inport;
  OutputPort<geometry_msgs::Twist> outport;

public:
  AvoidObstacleLaser(const std::string& name):
    TaskContext(name),
    inport("laser_in"),
    outport("twist_out")
  {
    ports()->addPort(inport);
    ports()->addPort(outport);
  }
  ~AvoidObstacleLaser() {}
private:
  void updateHook() {
    sensor_msgs::LaserScan msg;
    if (NewData == inport.read(msg)) {
      geometry_msgs::Twist cmd;

      bool halt = false;
      for (int i = 75; i <= 105; i++) {
        if (msg.ranges[i] < 2.0) {
          halt = true;
          break;
        }
      }
      if (halt) {
        double midA, midB;
        // we go to the highest-range side scanned
        midA = std::accumulate(msg.ranges.begin(), msg.ranges.end()-90, 0);
        midB = std::accumulate(msg.ranges.begin()+90, msg.ranges.end(), 0);
        log(Info)<<"A:"<<midA<<", B:"<<midB<<endlog();
        if (midA > midB) {
          cmd.angular.z = -1;
        } else {
          cmd.angular.z = 1;
        }
      } else {
        cmd.linear.x = 1;
      }
      outport.write(cmd);
    }
  }
};

ORO_CREATE_COMPONENT(AvoidObstacleLaser)

