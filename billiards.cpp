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

#define SCREEN_BASE 640

#define TABLE_COLOR RGBColor(0, 102, 0)

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

Game *game;

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
    //table = new BilliardsTable(worldx, worldy);
    game = new Game(worldx, worldy, new HumanPlayer("P1", SOLID), 
        new HumanPlayer("P2", STRIPED));
    game->startGame(0);
}

void initGraphicContext(int argc, char **argv) {
    glutInit(&argc, argv); // Utility Toolkit

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

    glutInitWindowPosition(300, 200);
    glutInitWindowSize(SCREEN_BASE, SCREEN_BASE * (worldy / worldx));

    glutCreateWindow("Billiards Game");

    glMatrixMode(GL_PROJECTION);
    gluOrtho2D(0, worldx, 0, worldy);

    glClearColor(TABLE_COLOR.getRed() / 255.0 ,
                 TABLE_COLOR.getGreen() / 255.0,
                 TABLE_COLOR.getBlue() / 255.0,
                 0.0); // Clear screen color

    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    //glutMouseFunc(mouse);
    glutPassiveMotionFunc(passiveMouse);
    glutIdleFunc(idle);
    glutCloseFunc(cleanContext);

    glutMainLoop();
}

void cleanContext() {
    delete game;
}

double toSecondsDelta(long deltamilis) {
    return (((double) deltamilis) / 1000.0);
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT); // Clear
    game->draw();

    drawCircle(mousepx, mousepy, 0.01, 20, RGBColor(255, 255, 0));

    glutSwapBuffers();
}

void keyboard(uchar c, int x, int y) {
    // PASS
}

void mouse(int button, int state, int x, int y) {
        std::cout << "Called" << std::endl;
    double px = (double) x / 320.0;
    double py = 1 - ((double) y / 320.0);

    Point3 p = Point3(px, py, 0.0);
    //table->hitBall(p);

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

    game->integrate(deltaSeconds);

    lastElapsed = elapsedTime;

    glutPostRedisplay();
}
