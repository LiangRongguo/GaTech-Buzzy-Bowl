/*
Author: Rongguo Liang
Class: ECE6122
Last Date Modified: 12.3
Description:
    UAV CLASS head file
*/

#pragma once
#include <iostream>
#include <stdio.h>
#include <math.h>

using namespace std;

// class for UAV
class UAV
{
private:
    // private variables
    double m;
    double x, y, z;
    double Fx, Fy, Fz, Fg;
    double Vx, Vy, Vz;
    double Fmax;
    
    // parameter for different orbits
    double nx, ny, nz; // normalized vector of the orbit
    double ax, ay, az;
    double bx, by, bz; // two vectors
    double theta; // parameter for the location of the point on the orbit
    
    // target location for the uav
    double tx, ty, tz;
    
    // change color
    double color = 255.0; // init color
    bool ascending = false; // boolean flag for changing color
    
public:
    // constructor
    UAV();
    
    // init location by given location
    void initLocation(double x_, double y_, double z_);
    
    // use the uav location information to draw this uav
    void draw();
    
    // update the uav location
    void updateLocation();
    
    // load data into sendbuffer for further gather_all
    void loadSendBuffer(double *sendBuffer);
    // after gather_all, read data from receive buffer
    void readRcvBuffer(double *rcvBuffer, int rank);
        
    // some public functions to get private variables
    double getX()
    {
        return x;
    };
    
    double getY()
    {
        return y;
    };
    
    double getZ()
    {
        return z;
    };
    
    double getVx()
    {
        return Vx;
    };
    
    double getVy()
    {
        return Vy;
    };
    
    double getVz()
    {
        return Vz;
    };
    
    // function to adjust location when Elastic Collision happens
    void setElasticCollision(double Vx_, double Vy_, double Vz_, double x, double y, double z);
    
    // function to update color
    void updateColor();
    
};

// function to check Elastic Collision with all other uavs
void elasticCollision(UAV *uavs, int rank);
