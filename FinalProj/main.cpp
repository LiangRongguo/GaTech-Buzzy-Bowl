/*
Author: Rongguo Liang
Class: ECE6122
Last Date Modified: 12.3
Description:
    Final project main file, using ECE_Bitmap.h, UAV.h and UAV.cpp
*/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "mpi.h"
#include "iomanip"
#include <cmath>
#include <math.h>
#include <cstdlib>
#include <GLUT/GLUT.h>
#include <chrono>
#include <thread>
#include "UAV.h"
#include "ECE_Bitmap.h"

using namespace std;

// define the key functions
// press "ESC" to exit program
// press R tp rotate the view
#define ESC 27
double rotateDegree = 0.0;
// press w to move the camera forward
// press s to move the camera backward
double forwardDist = 0.0;

// texture image struct
GLuint texture;
struct Image
{
    unsigned long sizeX;
    unsigned long sizeY;
    char* data;
};
typedef struct Image Image;

// class for bitmap
// using the class file "ECE_Bitmap.h" given by Prof. in class
BMP inBitmap;


// Send location and velocity vector in each direction
const int numElements = 13; // x, y, z, vx, vy, vz, tx, ty, tz, color  (tx stands for target_x, the target location each uav is pointing at)

// receive buffer size is (Main task + 15 UAVs) * numElements
const int rcvSize = 16 * numElements;
// receive buffer for all_gather
double* rcvbuffer = new double[rcvSize];
// send buffer for all_gather
double sendBuffer[numElements];

// declare the uav class array to store the information of all uavs
UAV *uavs = new UAV[15];

// light0 attribute
GLfloat light0_ambient[] = { 0.2, 0.2, 0.2, 1.0 };

// light1 attribute
GLfloat light1_diffuse[] = { 0.5, 0.5, 0.5, 1.0 };
GLfloat light1_position[] = { 5.0, 5.0, 8.0};
GLfloat light1_specular[] = { 0.3, 0.3, 0.3, 1.0 };

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
    // display field
    glPushMatrix();
        // enable texture
        glEnable(GL_TEXTURE_2D);
        // bind texture
        glBindTexture(GL_TEXTURE_2D, texture);
        // set color as green
        // actually this is unnecessary since we use texture
        glColor3f(0.0, 1.0, 0.0);
        // draw a quad
        glBegin(GL_QUADS);
            // set normal
            glNormal3f(0,0,1);
            // bind it with texture(0,1)
            glTexCoord2f(0, 1);
            // manual paramaters to scale the field to make it fit the texture
            double scaleX = 1.13;
            double scaleY = 1.27;
            glVertex3f( 24.4 * scaleX,  55.0 * scaleY, 0.0);
            // bind it with texture(0,0)
            glTexCoord2f(0, 0);
            glVertex3f(-24.4 * scaleX,  55.0 * scaleY, 0.0);
            // bind it with texture(1,0)
            glTexCoord2f(1, 0);
            glVertex3f(-24.4 * scaleX, -55.0 * scaleY, 0.0);
            // bind it with texture(1,1)
            glTexCoord2f(1, 1);
            glVertex3f( 24.4 * scaleX, -55.0 * scaleY, 0.0);
        glEnd();
        glDisable(GL_TEXTURE_2D);
    glPopMatrix();
        
    glPushMatrix();
        glTranslatef(0.0, 0.0, 50.0);
        glColor3f(0.8, 0.8, 0.0);
        // draw the point at (0,0,50)
        glutSolidSphere(1.0, 100, 100);
        glColor3f(1.0, 1.0, 1.0);
        // draw the virtual sphere with center at (0,0,50) and a radius of 10
        glutWireSphere(10.0, 20, 20);
    glPopMatrix();
}

// function to draw uavs
void drawUAVs()
{
    for (int i = 0; i < 15; i++)
    {
        // traverse the uavs class array
        // call the class's member public function to draw the uav
        // using the information stored in that uav class
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

    // set the look-at
    // press key w/s to adjust the camera location to get a better view
    gluLookAt(-200 + forwardDist, 0.0, 80.0,
               0.0, 0.0, 50.0,
               0.0, 0.0,  1.0);
    
    // rotate the field according to rotateDegree by key "r/R"
    glTranslatef(4.0, 4.0, 0.0);
    glRotatef(rotateDegree, 0.0, 0.0, 1.0);
    glTranslatef(-4.0, -4.0, 0.0);

    // set model view
    glMatrixMode(GL_MODELVIEW);
    
    // display function1:
    // draw footballfield
    displayFootballField();

    // display function2:
    // draw uavs
    drawUAVs();

    glutSwapBuffers(); // Make it all visible
    
    // dumb steps to set values into send buffer
    // but no one need the data in rank:0
    // so this step is just to make sure the send buffer is all 0 instead of something werid
    for (int i = 0; i < numElements; i++)
    {
        sendBuffer[i] = 0;
    }

    // all_gather function: to broadcast and get the information of others
    MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);
    
    // load data into class array to store information of all uavs
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
    // frequency: 100ms
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
    switch (key)
    {
        case ESC: // press ESC to quit
            exit(0);
            break;
        case 'r': // press r to rotate the field
            rotateDegree += 10.0;
            glutPostRedisplay();
            break;
        case 'R': // same as R
            rotateDegree += 10.0;
            glutPostRedisplay();
        case 'w': // press w to move the camera forward
            forwardDist += 10.0;
            glutPostRedisplay();
            break;
        case 'W': // same as w
            forwardDist += 10.0;
            glutPostRedisplay();
        case 's': // press s to move the camera backward
            forwardDist -= 10.0;
            glutPostRedisplay();
            break;
        case 'S': // same as s
            forwardDist -= 10.0;
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
    cout << "For TA's viewing and grading convenience, I set up several keys to make it convenient for you." << endl;
    cout << "\tYou can press key ESC to exit the program." << endl;
    cout << "\tYou can press Key r/R to rotate the field." << endl;
    cout << "\tYou can press Key w/W to move the camera forward." << endl;
    cout << "\tYou can press Key s/S to move the camera backward." << endl;
    cout << "Thank you for your work for this whole semester :)\n\n" << endl;
    
    // init
    glutInit(&argc, argv);
    // set mode
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    // set window size and location
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(400, 400);

    // create window, set background color and title
    glutCreateWindow(argv[0]);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    
    // set and enable modes
    glShadeModel(GL_SMOOTH);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    
    // set light
    glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light1_specular);
    glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
    // enable light
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    
    // set up texture part
    inBitmap.read("AmFBfield.bmp");
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); //scale linearly when image bigger than texture
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); //scale linearly when image smalled than texture
    glTexImage2D(GL_TEXTURE_2D, 0, 3, inBitmap.bmp_info_header.width, inBitmap.bmp_info_header.height, 0,
            GL_BGR_EXT, GL_UNSIGNED_BYTE, &inBitmap.data[0]);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
    
    // set recall functions
    glutReshapeFunc(changeSize);
    glutDisplayFunc(renderScene);
    glutKeyboardFunc(keyboard);
    glutTimerFunc(100, timerFunction, 0);
    
    // enter the opengl main loop
    // never return
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
    // variables for number of ranks and the rank of current thread
    int numTasks, rank;
    
    // init MPI
    int rc = MPI_Init(&argc, &argv);

    if (rc != MPI_SUCCESS)
    {
        // defensive coding
        // if initialize not done, quit
        printf("Error starting MPI program. Terminating.\n");
        MPI_Abort(MPI_COMM_WORLD, rc);
    }

    // get the number of ranks and the rank of current thread
    MPI_Comm_size(MPI_COMM_WORLD, &numTasks);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    
    // init the UAV class array
    // including initializing the uav's location
    for (int i = 0; i < 15; i++)
    {
        double x = i % 3 * 24.4 - 24.4;
        double y = i / 3 * 27.5 - 55.0;
        double z = 0;
        uavs[i].initLocation(x, y, z);
    }
    
    if (rank == 0)
    {
        // rank 0: enter opengl main loop
        mainOpenGL(argc, argv);
    }
    else
    {
        // rank other than 0
        
        // call gather_all to synchronize information
        uavs[rank-1].loadSendBuffer(sendBuffer);
        MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);
        
        // Sleep for 5 seconds
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        for (int ii = 0; ii < 1200 ; ii++)
        {
            // use a for loop to simulate the time in real world
            // According to the test, 1200 rounds takes about 60 seconds
            
            // first check "elastic collision"
            elasticCollision(uavs, rank-1);
            
            // update the color of uavs at each time step
            uavs[rank-1].updateColor();
            
            // update the location of uavs
            uavs[rank-1].updateLocation();
            
            // call gather_all to synchronize information
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
