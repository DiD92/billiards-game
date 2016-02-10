//-----------------------------------------------
// -- EXTERNAL LIBRARIES
//-----------------------------------------------

#include <GL/freeglut.h>

//-----------------------------------------------

#include "billiard_geom.h"

//-----------------------------------------------
// -- AUXILIARY METHODS
//-----------------------------------------------

void initContext();
void initGraphicContext(int, char**);
void cleanContext();

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
RGBColor *color;

//-----------------------------------------------
// -- MAIN PROCEDURE
//-----------------------------------------------

int main(int argc,char *argv[]) {

    initContext();

    initGraphicContext(argc, argv);

    cleanContext();
    return 0;
    
}

void initContext() {
    ballPosition = new Point3(0.5, 0.5, 0.0);
    ballVelocity = new Vector3();
    color = new RGBColor(123, 123, 123);
    ball = new Ball(1.0, *ballPosition, *ballVelocity, 0.1, *color);

}

void initGraphicContext(int argc, char **argv) {
    glutInit(&argc, argv); // Utility Toolkit

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

    glutInitWindowPosition(300, 200);
    glutInitWindowSize(640, 480);

    glutCreateWindow("Billiards Game");

    glMatrixMode(GL_PROJECTION);
    gluOrtho2D(0, 2, 0, 1);

    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutIdleFunc(idle);
    glutCloseFunc(cleanContext);

    glutMainLoop();
}

void cleanContext() {
    delete ballPosition;
    delete ballVelocity;
    delete color;
    delete ball;

}

void display() {
    ball->draw();
    glutSwapBuffers();
}

void keyboard(uchar c, int x, int y) {

}

void idle() {

}
