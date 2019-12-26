//
//  UAV.hpp
//  FinalProj
//
//  Created by Rongguo Liang on 11/25/19.
//  Copyright © 2019 Rongguo Liang. All rights reserved.
//

#pragma once
#include <iostream>
#include <stdio.h>
#include <math.h>

#define pi 3.1415926

using namespace std;

class UAV
{
private:
    double m;
    double x, y, z;
    double Fx, Fy, Fz, Fg;
    double Vx, Vy, Vz;
    double Fmax;

    double u;
    double virtualPointX;
    double virtualPointY;
    double virtualPointZ;
    
    double tx = 0.0, ty = 0.0, tz = 0.0;
    
    double degree;
    
public:
    UAV();
    
    void initLocation(double x_, double y_, double z_);
    void setDegree(double degree_)
    {
        degree = degree_;
    };
    
    void setVirtualPoint(double u_)
    {
        this->u = u_;
        this->virtualPointX = - 10.0 * sin(2 * pi * u);
        this->virtualPointY = 10.0 * cos(2 * pi * u);
        this->virtualPointZ = 50;
    }
    
    double getU()
    {
        return u;
    }
    
    void draw();
    void updateLocation();
    void updateLocation1();
    void loadSendBuffer(double *sendBuffer);
    void readRcvBuffer(double *rcvBuffer, int rank);
    double getVelocity();
    
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
    
    void setElasticCollision(double Vx_, double Vy_, double Vz_);
    
    void updateVirtualPoint()
    {
        setVirtualPoint(u + 1.0 / 60.0);
    };
    
    double getVPX()
    {
        return virtualPointX;
    };
    
    double getVPY()
    {
        return virtualPointY;
    };
    
    double getVPZ()
    {
        return virtualPointZ;
    };
    
    double getDegree()
    {
        return degree;
    }
};

void elasticCollision(UAV *uavs, int rank);
