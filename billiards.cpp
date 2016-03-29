//-----------------------------------------------
// -- EXTERNAL LIBRARIES
//-----------------------------------------------

#include <GL/freeglut.h>
#include <cstdlib>
#include <iostream>

//-----------------------------------------------

#include "billiard_geom.h"

//-----------------------------------------------
// -- MACRO DEFINITION
//-----------------------------------------------

#define BALL_MASS 0.130
#define BALL_RADIUS 0.056

#define HOLE_RADIUS 0.112

#define SCREEN_BASE 640

//-----------------------------------------------
// -- AUXILIARY METHODS
//-----------------------------------------------

void initContext();
void initGraphicContext(int, char**);
void cleanContext();
double toSecondsDelta(long);
extern void drawCircle(float, float, float, int, RGBColor);

//-----------------------------------------------
// -- DISPLAY METHODS
//-----------------------------------------------

void display();
void keyboard(uchar, int, int);
void mouse(int, int, int, int);
void passiveMouse(int, int);
void idle();

//-----------------------------------------------
// -- VARIABLES
//-----------------------------------------------

extern bool ballInHole;
extern bool ballMoving;

Ball ball;

RGBColor *ballColor, *tableColor;
BallGenerator *generator;
HoleGenerator *holeGenerator;
BilliardsTable *table;

float worldx = 2.0, worldy = 1.0;

double mousepx, mousepy;

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
    generator = new BallGenerator(BALL_MASS, BALL_RADIUS, 
        RGBColor(255, 255, 255));

    holeGenerator = new HoleGenerator(HOLE_RADIUS);
    ball = generator->generate(Point3(0.3, 0.5, 0.0));

    tableColor = new RGBColor(0, 102, 0);

    generator->setColor(RGBColor(230, 0, 0));

    Plane north = Plane::createPlane(Vector3(0.0, -1.0, 0.0), 
        Point3(0.0, 1.0, 0.0));
    Plane south = Plane::createPlane(Vector3(0.0, 1.0, 0.0), 
        Point3(2.0, 0.0, 0.0));
    Plane east = Plane::createPlane(Vector3(-1.0, 0.0, 0.0), 
        Point3(2.0, 1.0, 0.0));
    Plane west = Plane::createPlane(Vector3(1.0, 0.0, 0.0), 
        Point3(0.0, 0.0, 0.0));
    table = new BilliardsTable(north, south, east, west, ball);

    table->addBall(generator->generate(Point3(1.2, 0.5, 0.0)));

    table->addBall(generator->generate(Point3(1.35, 0.57, 0.0)));
    table->addBall(generator->generate(Point3(1.35, 0.43, 0.0)));

    table->addBall(generator->generate(Point3(1.5, 0.64, 0.0)));
    table->addBall(generator->generate(Point3(1.5, 0.5, 0.0)));
    table->addBall(generator->generate(Point3(1.5, 0.36, 0.0)));

    table->addBall(generator->generate(Point3(1.65, 0.71, 0.0)));
    table->addBall(generator->generate(Point3(1.65, 0.57, 0.0)));
    table->addBall(generator->generate(Point3(1.65, 0.43, 0.0)));
    table->addBall(generator->generate(Point3(1.65, 0.29, 0.0)));

    table->addBall(generator->generate(Point3(1.80, 0.78, 0.0)));
    table->addBall(generator->generate(Point3(1.80, 0.64, 0.0)));
    table->addBall(generator->generate(Point3(1.80, 0.5, 0.0)));
    table->addBall(generator->generate(Point3(1.80, 0.36, 0.0)));
    table->addBall(generator->generate(Point3(1.80, 0.22, 0.0)));

    table->addHole(holeGenerator->generate(Point3(0.04,0.02,0.0)));
    table->addHole(holeGenerator->generate(Point3(worldx/2,0.02,0.0)));
    table->addHole(holeGenerator->generate(Point3(worldx - 0.02,0.02,0.0)));
    table->addHole(holeGenerator->generate(Point3(0.04,
        worldy - 0.02,0.0)));
    table->addHole(holeGenerator->generate(Point3(worldx/2,
        worldy - 0.02,0.0)));
    table->addHole(holeGenerator->generate(Point3(worldx - 0.02,
        worldy - 0.02,0.0)));
}

void initGraphicContext(int argc, char **argv) {
    glutInit(&argc, argv); // Utility Toolkit

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

    glutInitWindowPosition(300, 200);
    glutInitWindowSize(SCREEN_BASE, SCREEN_BASE * (worldy / worldx));

    glutCreateWindow("Billiards Game");

    glMatrixMode(GL_PROJECTION);
    gluOrtho2D(0, worldx, 0, worldy);

    glClearColor(tableColor->getRed() / 255.0 ,
                 tableColor->getGreen() / 255.0,
                 tableColor->getBlue() / 255.0,
                 0.0); // Clear screen color

    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutPassiveMotionFunc(passiveMouse);
    glutIdleFunc(idle);
    glutCloseFunc(cleanContext);

    glutMainLoop();
}

void cleanContext() {
    delete ballColor;
    delete tableColor;
    delete table;
    delete generator;
    delete holeGenerator;
}

double toSecondsDelta(long deltamilis) {
    return (((double) deltamilis) / 1000.0);
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT); // Clear
    table->draw();

    drawCircle(mousepx, mousepy, 0.01, 20, RGBColor(255, 255, 0));

    glutSwapBuffers();
}

void keyboard(uchar c, int x, int y) {
    // PASS
}

void mouse(int button, int state, int x, int y) {

    if(!ballInHole && !ballMoving) {
        double px = (double) x / 320.0;
        double py = 1 - ((double) y / 320.0);

        Point3 p = Point3(px, py, 0.0);
        table->hitBall(p);
        ballMoving = true;
    }

    glutPostRedisplay();
}

void passiveMouse(int x, int y) {
    mousepx = (double) x / 320.0;
    mousepy = 1 - ((double) y / 320.0);
}

void idle() {
    elapsedTime = glutGet(GLUT_ELAPSED_TIME);
    deltaTime = elapsedTime - lastElapsed;
    deltaSeconds = toSecondsDelta(deltaTime);

    if(table->integrate(deltaSeconds) == NULL) {
        if(table->resetReady()) {
            if(ballInHole) {
                table->resetBall();
            } else if(!table->mainBallMoving()) {
                ballMoving = false;
            }
        }
    }

    lastElapsed = elapsedTime;

    glutPostRedisplay();
}
