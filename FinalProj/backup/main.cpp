#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "mpi.h"
#include "iomanip"
#include <cmath>
#include <math.h>
#include <cstdlib>
//#include <GL/glut.h>
//#include <chrono>
#include <GLUT/GLUT.H>
#include <thread>
#include "UAV.h"

#define ESC 27

GLfloat rotateDegree = 0.0;

using namespace std;

// Send location and velocity vector in each direction
const int numElements = 15; // x, y, z, vx, vy, vz, VPX, VPY, VPZ, tx, ty, tz

const int rcvSize = 16 * numElements; // (Main task + 15 UAVs) * numElements

double* rcvbuffer = new double[rcvSize];

double sendBuffer[numElements];

UAV *uavs = new UAV[15];

//----------------------------------------------------------------------
// Reshape callback
//
// Window size has been set/changed to w by h pixels. Set the camera
// perspective to 45 degree vertical field of view, a window aspect
// ratio of w/h, a near clipping plane at depth 1, and a far clipping
// plane at depth 100. The viewport is the entire window.
//
//----------------------------------------------------------------------
void changeSize(int w, int h)
{
    float ratio = ((float)w) / ((float)h); // window aspect ratio
    glMatrixMode(GL_PROJECTION); // projection matrix is active
    glLoadIdentity(); // reset the projection
    gluPerspective(60.0, ratio, 0.1, 1000.0); // perspective transformation
    glMatrixMode(GL_MODELVIEW); // return to modelview mode
    glViewport(0, 0, w, h); // set viewport (drawing area) to entire window
}

void displayFootballField()
{
    glPushMatrix();
        glColor3f(0.0, 1.0, 0.0);
        glBegin(GL_QUADS);
        glNormal3f(0,0,1);
        glVertex3f( 24.4,  55.0, 0.0);
        glVertex3f(-24.4,  55.0, 0.0);
        glVertex3f(-24.4, -55.0, 0.0);
        glVertex3f( 24.4, -55.0, 0.0);
        glEnd();
    glPopMatrix();
    
    glPushMatrix();
        glTranslatef(0.0, 0.0, 50.0);
        glColor3f(0.2, 0.8, 1.0);
        glutSolidSphere(1.0, 100, 100);
        glColor3f(1.0, 1.0, 1.0);
        glutWireSphere(10.0, 20, 20);
    glPopMatrix();
    
    
}

void drawUAVs()
{
    for (int i = 0; i < 15; i++)
    {
        uavs[i].draw();
    }
}

//----------------------------------------------------------------------
// Draw the entire scene
//
// We first update the camera location based on its distance from the
// origin and its direction.
//----------------------------------------------------------------------
void renderScene()
{
    // Clear color and depth buffers
    glClearColor(0.2, 0.2, 0.2, 1.0); // background color to green??
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Reset transformations
    glLoadIdentity();

    gluLookAt(-200, 0.0, 80.0,
               0.0, 0.0, 50.0,
               0.0, 0.0,  1.0);
    
    // rotate the chess board according to rotateDegree by key "r/R"
    glTranslatef(4.0, 4.0, 0.0);
    glRotatef(rotateDegree, 0.0, 0.0, 1.0);
    glTranslatef(-4.0, -4.0, 0.0);

    glMatrixMode(GL_MODELVIEW);
    
    displayFootballField();

    drawUAVs();

    glutSwapBuffers(); // Make it all visible
    
    for (int i = 0; i < numElements; i++)
    {
        sendBuffer[i] = 0;
    }

    MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);
    
    // load data into class array
    for (int i = 0; i < 15; i++)
    {
        uavs[i].readRcvBuffer(rcvbuffer, (i + 1) * numElements);
    }
}

//----------------------------------------------------------------------
// timerFunction  - called whenever the timer fires
//----------------------------------------------------------------------
void timerFunction(int id)
{
    glutPostRedisplay();
    glutTimerFunc(100, timerFunction, 0);
}

//----------------------------------------------------------------------
// Keyboard callback
//
// when target key is pressed down, update relevant variables and update and redraw
// the view
//
//----------------------------------------------------------------------
void keyboard(unsigned char key, int x, int y)
{
    switch (key) {
    case ESC: // press ESC to quit
        exit(0);
        break;
    case 'r': // press r to rotate the chess board
        rotateDegree += 10.0;
        glutPostRedisplay();
        break;
    case 'R': // same as R
        rotateDegree += 10.0;
        glutPostRedisplay();
    default:
        break;
    }
}

//----------------------------------------------------------------------
// mainOpenGL  - standard GLUT initializations and callbacks
//----------------------------------------------------------------------
void mainOpenGL(int argc, char**argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(400, 400);

    glutCreateWindow(argv[0]);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_SMOOTH);
    
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    // Setup lights as needed
    // ...

    glutReshapeFunc(changeSize);
    glutDisplayFunc(renderScene);
    glutKeyboardFunc(keyboard);
    glutTimerFunc(100, timerFunction, 0);
    glutMainLoop();
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
// Main entry point determines rank of the process and follows the
// correct program path
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
int main(int argc, char**argv)
{
    int numTasks, rank;

    int rc = MPI_Init(&argc, &argv);

    if (rc != MPI_SUCCESS)
    {
        printf("Error starting MPI program. Terminating.\n");
        MPI_Abort(MPI_COMM_WORLD, rc);
    }

    MPI_Comm_size(MPI_COMM_WORLD, &numTasks);

    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
        
    for (int i = 0; i < 15; i++)
    {
        double x = i % 3 * 24.4 - 24.4;
        double y = i / 3 * 27.5 - 55.0;
        double z = 0;
        uavs[i].initLocation(x, y, z);
        uavs[i].setVirtualPoint(0.0);
        uavs[i].setDegree(i * 12.0);
    }
    
    if (rank == 0)
    {
        mainOpenGL(argc, argv);
    }
    else
    {
        // Sleep for 5 seconds
        uavs[rank-1].loadSendBuffer(sendBuffer);
        MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        for (int ii = 0; ii < 6000 ; ii++)
        {
            // first check "elastic collision"
            elasticCollision(uavs, rank-1);
            
            uavs[rank-1].updateVirtualPoint();
            
            uavs[rank-1].updateLocation();
            
            uavs[rank-1].loadSendBuffer(sendBuffer);
            
            MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);
            
            // load data into class array
            for (int i = 0; i < 15; i++)
            {
                uavs[i].readRcvBuffer(rcvbuffer, (i + 1) * numElements);
            }
        }
    }
    
    return 0;
}
