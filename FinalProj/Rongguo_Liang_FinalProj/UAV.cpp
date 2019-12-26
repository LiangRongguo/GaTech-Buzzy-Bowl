/*
Author: Rongguo Liang
Class: ECE6122
Last Date Modified: 12.3
Description:
    UAV CLASS file
*/

// define PI
#define pi 3.1415926

#include "UAV.h"
#include <iostream>
#include <GL/glut.h>

#include <chrono>
#include <math.h>
#include <locale>
#include "mpi.h"

using namespace std;


// helper function to calculate the distance between 2 points
double distanceOf(double x1, double y1, double z1, double x2, double y2, double z2)
{
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

// constructor
UAV::UAV()
{
    // initializer for default variables
    m = 1;
    Fmax = 20;
    Fx = 0.0;
    Fy = 0.0;
    Fz = 0.0;
    Fg = m * 10.0;
    Vx = 0;
    Vy = 0;
    Vz = 0;
    
    // for each uav, we need a random normalized vector n to define its orbit
    nx = rand() % 11 - 5.0;
    ny = rand() % 11 - 5.0;
    nz = rand() % 11 - 5.0;
    // init location for a point on its orbit
    theta = 0.0;
    
    // Find two vectors "a" and "b" that are perpendicular to each other and to n for further calculating the orbit

    // corss product
    double temp_x = 1.0;
    double temp_y = 0.0;
    double temp_z = 0.0;
    
    ax = ny * temp_z - nz * temp_y;
    ay = nz * temp_x - nx * temp_z;
    az = nx * temp_y - ny * temp_x;
    if (ax == 0.0 && ay == 0.0 && az == 0.0)
    {
        temp_x = 0.0;
        temp_y = 1.0;
        temp_z = 0.0;
        ax = ny * temp_z - nz * temp_y;
        ay = nz * temp_x - nx * temp_z;
        az = nx * temp_y - ny * temp_x;
    }
    
    bx = ny * az - nz * ay;
    by = nz * ax - nx * az;
    bz = nx * ay - ny * ax;
    
    // normalize a and b vectors
    double a = distanceOf(ax, ay, az, 0, 0, 0);
    ax /= a;
    ay /= a;
    az /= a;
    double b = distanceOf(bx, by, bz, 0, 0, 0);
    bx /= b;
    by /= b;
    bz /= b;
    
    // using the Parametric equation of a circle in 3D space to calculate the target location on that orbit
    tx = 10 * cos(2 * pi * theta) * ax + 10 * sin(2 * pi * theta) * bx;
    ty = 10 * cos(2 * pi * theta) * ay + 10 * sin(2 * pi * theta) * by;
    tz = 10 * cos(2 * pi * theta) * az + 10 * sin(2 * pi * theta) * bz + 50.0;
}

void UAV::initLocation(double x_, double y_, double z_)
{
    // init the uav location using given location
    x = x_;
    y = y_;
    z = z_;
}

void UAV::draw()
{
    // draw the uav at current location(x,y,z)
    glPushMatrix();
        // use dynamic color
        glColor3f(this->color / 255.0, 0.0, 0.0);
        glTranslatef(x, y, z);
        // according to my name, i use Icosahedron to represent a uav
        glutSolidIcosahedron();
    glPopMatrix();
}

void UAV::updateLocation()
{
    // first we calculate the distance of current location to its target location
    double distanceToTarget = distanceOf(x, y, z, tx, ty, tz);

    // the time for each time step is 0.1s
    double t = 0.1;
    
    // get current velocity
    double v = sqrt(pow(Vx, 2) + pow(Vy, 2) + pow(Vz, 2));
    
    // adjust the target point velocity according to the location of uav and its speed
    // note that theta is an angular velocity
    // we use formula "theta = v / R"
    if (distanceToTarget >= 3.0)
    {
        // if they are too far,
        // make the target point move slowly
        // so that the uav can catch it along the surface
        theta += v * 0.1 / 100.0;
    }
    else
    {
        // when they are close enough,
        // means that they have been together on the surface,
        // then we can set the speed of target point as fast as the uav
        theta += v * 1.0 / 100.0;
    }
    
    // use the parametric equation of a circle in 3D space to update the target point's location
    double targetX = 10 * cos(2 * pi * theta) * ax + 10 * sin(2 * pi * theta) * bx;
    double targetY = 10 * cos(2 * pi * theta) * ay + 10 * sin(2 * pi * theta) * by;
    double targetZ = 10 * cos(2 * pi * theta) * az + 10 * sin(2 * pi * theta) * bz + 50.0;
    
    // calculate the direction vector from uav to its target
    double dx = targetX - x;
    double dy = targetY - y;
    double dz = targetZ - z;
    double d  = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
    
    // get the relevant force vector
    Fx = dx / d * (Fmax - Fg);
    Fy = dy / d * (Fmax - Fg);
    Fz = dz / d * (Fmax - Fg);
    
    // get the relevant acceleration
    double ax = Fx / m;
    double ay = Fy / m;
    double az = Fz / m;
    
    // get the relevant velocity under that acceleration
    double Vx_ = Vx + ax * t;
    double Vy_ = Vy + ay * t;
    double Vz_ = Vz + az * t;
    double V_  = sqrt(pow(Vx_, 2) + pow(Vy_, 2) + pow(Vz_, 2));
    
    // get the distance from uav to the point(0,0,50)
    // because when it is approaching the point, its speed limit is 2m/s
    // when it is on the surfac of the sphere, its speed limit is 2-10m/s
    double distance = distanceOf(x, y, z, 0, 0, 50);
    
    // if it exceeds the speed limit, curve the speed
    // I set the threshold as 11
    if (distance >= 11.0)
    {
        if (V_ > 2.0)
        {
            // when it is approaching the point, its speed limit is 2m/s
            Vx_ /= V_ * 2.0;
            Vy_ /= V_ * 2.0;
            Vz_ /= V_ * 2.0;
        }
    }
    else
    {
        if (V_ > 10.0)
        {
            // when it is on the surfac of the sphere, its speed limit is 2-10m/s
            Vx_ /= V_ * 10.0;
            Vy_ /= V_ * 10.0;
            Vz_ /= V_ * 10.0;
        }
    }
    
    // update the location according to curved speed
    x += 0.5 * (Vx + Vx_) * t;
    y += 0.5 * (Vy + Vy_) * t;
    z += 0.5 * (Vz + Vz_) * t;
    
    // set velocity into uav class
    Vx = Vx_;
    Vy = Vy_;
    Vz = Vz_;
    
    // set target location into uav class
    tx = targetX;
    ty = targetY;
    tz = targetZ;
}

void UAV::loadSendBuffer(double *sendBuffer)
{
    // load needed variables into send buffer
    sendBuffer[0] = x;
    sendBuffer[1] = y;
    sendBuffer[2] = z;
    sendBuffer[3] = Vx;
    sendBuffer[4] = Vy;
    sendBuffer[5] = Vz;
    sendBuffer[6] = Fx;
    sendBuffer[7] = Fy;
    sendBuffer[8] = Fz;
    sendBuffer[9] = tx;
    sendBuffer[10] = ty;
    sendBuffer[11] = tz;
    sendBuffer[12] = color;
}

void UAV::readRcvBuffer(double *rcvBuffer, int index)
{
    // read needed variables from the relevant part of the receive buffer
    x = rcvBuffer[index + 0];
    y = rcvBuffer[index + 1];
    z = rcvBuffer[index + 2];
    Vx = rcvBuffer[index + 3];
    Vy = rcvBuffer[index + 4];
    Vz = rcvBuffer[index + 5];
    Fx = rcvBuffer[index + 6];
    Fy = rcvBuffer[index + 7];
    Fz = rcvBuffer[index + 8];
    tx = rcvBuffer[index + 9];
    ty = rcvBuffer[index + 10];
    tz = rcvBuffer[index + 11];
    color = rcvBuffer[index + 12];
}

void UAV::setElasticCollision(double Vx_, double Vy_, double Vz_, double x, double y, double z)
{
    // when ElasticCollision happens, use this function to update the location of 2 uavs
    
    // first exchange velocity
    Vx = Vx_;
    Vy = Vy_;
    Vz = Vz_;
    
    // then we force those 2 uavs to get away from each other by a certain distance
    // otherwist, it they only exchange velocity, then their distance is still below the ElasticCollision limit
    // it would be stuck in a dead end that they can never get rid of each other and stick to other forever
    double midX = (this->x + x) / 2;
    double midY = (this->y + y) / 2;
    double midZ = (this->z + z) / 2;
    
    // So I force it to get away from each other by 2m
    double midDist = distanceOf(this->x, this->y, this->z, midX, midY, midZ);
    this->x = midX + (this->x - midX) / midDist * 1.0;
    this->y = midY + (this->y - midY) / midDist * 1.0;
    this->z = midZ + (this->z - midZ) / midDist * 1.0;
}

void elasticCollision(UAV *uavs, int rank)
{
    // find the cloeset uav to uav #rank
    // if they are closer than the threshold, then update their location
    
    // get the current location of this rank' uav
    double x = uavs[rank].getX(), y = uavs[rank].getY(), z = uavs[rank].getZ();
    
    // then we find the cloeset uav, default is -1, meaning that closest uav is farther than threshold
    int closestUav = -1;
    // set default dist to infinity
    double dist = numeric_limits<double>::max();
    
    // use a for loop to traverse all uavs
    for (int i = 0; i < 15; i++)
    {
        if (i == rank)
        {
            // skip itself
            continue;
        }
        
        // calculate distance
        double tmpDist = distanceOf(x, y, z, uavs[i].getX(), uavs[i].getY(), uavs[i].getZ());
        if (tmpDist <= dist && tmpDist <= 1.01)
        {
            // if closer than the distance ever found so far and closer than the threshold
            // update the closest uav
            closestUav = i;
            dist = tmpDist;
        }
    }
    
    if (closestUav == -1)
    {
        // if not found, simply break
        return;
    }
    else
    {
        // call function to update status of two
        uavs[rank].setElasticCollision(uavs[closestUav].getVx(), uavs[closestUav].getVy(), uavs[closestUav].getVz(),
                                       uavs[closestUav].getX(),  uavs[closestUav].getY(),  uavs[closestUav].getZ());
    }
}


void UAV::updateColor()
{
    // update the color from 128 to 255
    if (ascending)
    {
        // if ascending, check whether it reached the upper bound
        if (color == 255.0)
        {
            // if yes, change into descending
            color--;
            ascending = not ascending;
        }
        else
        {
            // if no, keep ascending
            color++;
        }
    }
    else
    {
        // if decending, check whether it reached the lower bound
        if (color == 128.0)
        {
            // if yes, change into ascending
            color++;
            ascending = not ascending;
        }
        else
        {
            // if no, keep descending
            color--;
        }
    }
}
