#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

// rosrun proteus_demo lasertwist cmd:=/ATRV/Motion_Controller laser:=/ATRV/Sick

// std::accumulate(msg.ranges.begin(), msg.ranges.end()-mid, 0);
// std::accumulate(msg.ranges.begin()+mid, msg.ranges.end(), 0);

double 
accumulate(const float * ranges, int start, int end) 
{
    double sum = 0.0;
    int angle;
    for (angle = start; angle < end; angle++) {
        sum += ranges[angle];
    }
    return sum;
}

float * 
rawlasertwist(const float * ranges, int len)
{
    float * velocity = (float *) calloc(2, sizeof(float)); // v,omega

    if (len >= 30) {
        int halt = 0;
        int mid = len / 2;
        int angle;
        // halt if an object is less than 2m in a 30deg angle
        for (angle = mid - 15; angle < mid + 15; angle++) {
            if (ranges[angle] < 2) {
                halt = 1;
                break;
            }
        }
        if (halt != 0) {
            double midL, midR;
            midL = accumulate(ranges, 0, mid);
            midR = accumulate(ranges, mid, len);
            // we go to the highest-range side scanned
            if (midL > midR) {
                velocity[1] = -1.0;
            } else {
                velocity[1] = 1.0;
            }
        } else {
            velocity[0] = 1.0;
        }
    }
    return velocity;
}

ros::Publisher topic;

void handle_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    geometry_msgs::Twist cmd;
    float * velocity = rawlasertwist(&(msg->ranges[0]), msg->ranges.size());
    cmd.linear.x = velocity[0];
    cmd.angular.z = velocity[1];
    topic.publish(cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lasertwist");
    ros::NodeHandle n;
    topic = n.advertise<geometry_msgs::Twist>("cmd", 1000);
    ros::Subscriber sub = n.subscribe("laser", 1000, handle_laser);
    ros::spin();
    return 0;
}

