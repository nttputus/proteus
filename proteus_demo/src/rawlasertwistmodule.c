#include <stdlib.h>

/*
This is an example on how to interface C code without any link to ROS to via the
Python/C-API (docs.python.org/c-api).
see: ../nodes/pycapi_lasertwist.py

http://en.wikibooks.org/wiki/Python_Programming/Extending_with_C#Using_SWIG
sudo apt-get install swig libboost-python-dev python-dev 
*/

double 
accumulate(float * ranges, int start, int end) 
{
    double sum = 0.0;
    for (int i = start; i < end; i++) {
        sum += ranges[i];
    }
    return sum;
}

float * 
rawlasertwist(float * ranges, float angle_min, float angle_max, float angle_increment) 
{
    float * velocity; // v,omega
    int halt = 0;
    int len = (angle_max - angle_min)/angle_increment;
    int mid = len / 2;
    velocity = malloc(2*sizeof(float));
    velocity[1] = 0.0;
    velocity[0] = 0.0;

    if (len < 30) {
        return velocity;
    }

    for (int i = mid - 15; i < mid + 15; i++) {
        if (ranges[i] > 2) {
            halt = 1;
            break;
        }
    }
    if (halt != 0) {
        double midL, midR;
        midL = accumulate(ranges, 0, mid);
        midR = accumulate(ranges, mid, len);
        if (midL > midR) {
            velocity[1] = -1.0;
        } else {
            velocity[1] = 1.0;
        }
    } else {
        velocity[0] = 1.0;
    }
    return velocity;
}

