//-----------------------------------------------
// -- EXTERNAL LIBRARIES
//-----------------------------------------------

#include <GL/freeglut.h>
#include <cstdlib>
#include <iostream>

//-----------------------------------------------

#include "billiard_geom.h"

//-----------------------------------------------
// -- AUXILIARY METHODS
//-----------------------------------------------

void initContext();
void initGraphicContext(int, char**);
void cleanContext();
double toSecondsDelta(long);

//-----------------------------------------------
// -- DISPLAY METHODS
//-----------------------------------------------

void display();
void keyboard(uchar, int, int);
void idle();

//-----------------------------------------------
// -- VARIABLES
//-----------------------------------------------

Ball *ball;

Point3 *ballPosition;
Vector3 *ballVelocity;
RGBColor *ballColor, *tableColor;

BilliardsTable *table;

bool bounce = false;

float worldx = 2.0, worldy = 1.0;

double speedx, speedy;

long elapsedTime, deltaTime, lastElapsed;
double deltaSeconds;

//-----------------------------------------------
// -- MAIN PROCEDURE
//-----------------------------------------------

int main(int argc,char *argv[]) {

    initContext();

    initGraphicContext(argc, argv);

    cleanContext();
    return 0;
    
}

//-----------------------------------------------

void initContext() {
    ballPosition = new Point3(0.5, 0.5, 0.0);
    ballVelocity = new Vector3(0.0, 0.0, 0.0);
    ballColor = new RGBColor(255, 255, 255);
    tableColor = new RGBColor(0, 102, 0);
    ball = new Ball(0.156, *ballPosition, *ballVelocity, 0.1, *ballColor);
    Plane north = Plane::createPlane(Vector3(0.0, -1.0, 0.0), Point3(0.0, 1.0, 0.0));
    Plane south = Plane::createPlane(Vector3(0.0, 1.0, 0.0), Point3(2.0, 0.0, 0.0));
    Plane east = Plane::createPlane(Vector3(-1.0, 0.0, 0.0), Point3(2.0, 1.0, 0.0));
    Plane west = Plane::createPlane(Vector3(1.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
    table = new BilliardsTable(north, south, east, west, *ball);
}

void initGraphicContext(int argc, char **argv) {
    glutInit(&argc, argv); // Utility Toolkit

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

    glutInitWindowPosition(300, 200);
    glutInitWindowSize(640, 640 * (worldy / worldx));

    glutCreateWindow("Billiards Game");

    glMatrixMode(GL_PROJECTION);
    gluOrtho2D(0, worldx, 0, worldy);

    glClearColor(tableColor->getRed() / 255.0 ,
                 tableColor->getGreen() / 255.0,
                 tableColor->getBlue() / 255.0,
                 0.0); // Clear screen color

    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutIdleFunc(idle);
    glutCloseFunc(cleanContext);

    glutMainLoop();
}

void cleanContext() {
    delete ballPosition;
    delete ballVelocity;
    delete ballColor;
    delete tableColor;
    delete ball;
}

double toSecondsDelta(long deltamilis) {
    return (((double) deltamilis) / 1000.0);
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT); // Clear
    table->draw();

    glutSwapBuffers();
}

void keyboard(uchar c, int x, int y) {
    char key = c;
    switch(key) {
        case ' ':
            speedx = (double) (rand() % 23) / 10.0;
            speedy = (double) (rand() % 31) / 10.0;
            if(rand() % 2) {
                speedx *= -1.0;
            }
            if(rand() % 2) {
                speedy *= -1.0;
            }
            table->hitBall(Vector3(speedx, speedy, 0.0));
            break;
        default:
            break;
    }
    glutPostRedisplay();
}

void idle() {
    elapsedTime = glutGet(GLUT_ELAPSED_TIME);
    deltaTime = elapsedTime - lastElapsed;
    deltaSeconds = toSecondsDelta(deltaTime);

    table->integrate(deltaSeconds);

    lastElapsed = elapsedTime;

    glutPostRedisplay();
}
