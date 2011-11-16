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
    int angle;
    for (angle = start; angle < end; angle++) {
        sum += ranges[angle];
    }
    return sum;
}

float * 
rawlasertwist(float * ranges, int len)
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

