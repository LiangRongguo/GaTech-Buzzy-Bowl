//
//  UAV.cpp
//  FinalProj
//
//  Created by Rongguo Liang on 11/25/19.
//  Copyright © 2019 Rongguo Liang. All rights reserved.
//

#include "UAV.h"
#include <iostream>
#include <GLUT/GLUT.h>
#include <math.h>
#include "mpi.h"

using namespace std;

double distanceOf(double x1, double y1, double z1, double x2, double y2, double z2)
{
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

double direcOf(double x1, double y1, double z1, double x2, double y2, double z2)
{
    return 0;
}

UAV::UAV()
{
    m = 1;
    Fmax = 20;
    Fx = 0.0;
    Fy = 0.0;
    Fz = 0.0;
    Fg = m * 10.0;
    Vx = 0;
    Vy = 0;
    Vz = 0;
    degree = 0;
}

void UAV::initLocation(double x_, double y_, double z_)
{
    x = x_;
    y = y_;
    z = z_;
}

void UAV::draw()
{
    glPushMatrix();
        glColor3f(1.0, 0.0, 0.0);
        glTranslatef(x, y, z);
        glutSolidIcosahedron();
    glPopMatrix();
    
    glPushMatrix();
        glColor3f(1.0, 1.0, 0.0);
        glTranslatef(virtualPointX, virtualPointY, virtualPointZ);
        glutSolidIcosahedron();
    glPopMatrix();
    
    glPushMatrix();
        glColor3f(0.0, 1.0, 1.0);
        glTranslatef(tx, ty, tz);
        glutSolidIcosahedron();
    glPopMatrix();
}

void UAV::updateLocation()
{
    double t = 0.1;
    double targetX = virtualPointX;
    double targetY = virtualPointY * cos(degree) - virtualPointZ * sin(degree);
    double targetZ = virtualPointY * sin(degree) + virtualPointZ * cos(degree) + 50.0;
    
    double dist = distanceOf(0, 0, 50, targetX, targetY, targetZ);
    targetX = targetX * 10.0 / dist;
    targetY = targetY * 10.0 / dist;
    targetZ = 10 * (targetZ - 50) / dist + 50;
    
    tx = targetX;
    ty = targetY;
    tz = targetZ;
    
    double dx = targetX - x;
    double dy = targetY - y;
    double dz = targetZ - z;
    double d  = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
    
    Fx = dx / d * (Fmax - Fg);
    Fy = dy / d * (Fmax - Fg);
    Fz = dz / d * (Fmax - Fg);

    double ax = Fx / m;
    double ay = Fy / m;
    double az = Fz / m;
    
    double Vx_ = Vx + ax * t;
    double Vy_ = Vy + ay * t;
    double Vz_ = Vz + az * t;
    double V_  = sqrt(pow(Vx_, 2) + pow(Vy_, 2) + pow(Vz_, 2));
    
    // double distToCenter = distanceOf(x, y, z, 0, 0, 50);
    
    if (V_ > 2)
    {
        Vx_ /= V_ * 2;
        Vy_ /= V_ * 2;
        Vz_ /= V_ * 2;
    }
    /*
    if (distToCenter >= 11)
    {
        if (V_ > 2)
        {
            Vx_ /= V_ * 2;
            Vy_ /= V_ * 2;
            Vz_ /= V_ * 2;
        }
    }
    else
    {
        if (V_ > 10)
        {
            Vx_ /= V_ * 10;
            Vy_ /= V_ * 10;
            Vz_ /= V_ * 10;
        }
    }
     */
    
    x += 0.5 * (Vx + Vx_) * t;
    y += 0.5 * (Vy + Vy_) * t;
    z += 0.5 * (Vz + Vz_) * t;
    
    Vx = Vx_;
    Vy = Vy_;
    Vz = Vz_;
}

void UAV::loadSendBuffer(double *sendBuffer)
{
    sendBuffer[0] = x;
    sendBuffer[1] = y;
    sendBuffer[2] = z;
    sendBuffer[3] = Vx;
    sendBuffer[4] = Vy;
    sendBuffer[5] = Vz;
    sendBuffer[6] = Fx;
    sendBuffer[7] = Fy;
    sendBuffer[8] = Fz;
    sendBuffer[9] = virtualPointX;
    sendBuffer[10] = virtualPointY;
    sendBuffer[11] = virtualPointZ;
    sendBuffer[12] = tx;
    sendBuffer[13] = ty;
    sendBuffer[14] = tz;
}

void UAV::readRcvBuffer(double *rcvBuffer, int index)
{
    x = rcvBuffer[index + 0];
    y = rcvBuffer[index + 1];
    z = rcvBuffer[index + 2];
    Vx = rcvBuffer[index + 3];
    Vy = rcvBuffer[index + 4];
    Vz = rcvBuffer[index + 5];
    Fx = rcvBuffer[index + 6];
    Fy = rcvBuffer[index + 7];
    Fz = rcvBuffer[index + 8];
    virtualPointX = rcvBuffer[index + 9];
    virtualPointY = rcvBuffer[index + 10];
    virtualPointZ = rcvBuffer[index + 11];
    tx = rcvBuffer[index + 12];
    ty = rcvBuffer[index + 13];
    tz = rcvBuffer[index + 14];
}

double UAV::getVelocity()
{
    return sqrt(pow(Vx, 2) + pow(Vy, 2) + pow(Vz, 2));
}

void UAV::setElasticCollision(double Vx_, double Vy_, double Vz_)
{
    Vx = Vx_;
    Vy = Vy_;
    Vz = Vz_;
}

void elasticCollision(UAV *uavs, int rank)
{
    double x = uavs[rank].getX(), y = uavs[rank].getY(), z = uavs[rank].getZ();
    for (int i = 0; i < 15; i++)
    {
        if (i == rank)
        {
            continue;
        }
        
        double distance = distanceOf(x, y, z, uavs[i].getX(), uavs[i].getY(), uavs[i].getZ());
        if (distance <= 2)
        {
            uavs[rank].setElasticCollision(uavs[i].getVx(), uavs[i].getVy(), uavs[i].getVz());
            uavs[i].setElasticCollision(uavs[rank].getVx(), uavs[rank].getVy(), uavs[rank].getVz());
            return;
        }
    }
}
