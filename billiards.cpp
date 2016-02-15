//-----------------------------------------------
// -- EXTERNAL LIBRARIES
//-----------------------------------------------

#include <GL/freeglut.h>
#include <cstdlib>

//-----------------------------------------------

#include "billiard_geom.h"

//-----------------------------------------------
// -- AUXILIARY METHODS
//-----------------------------------------------

void initContext();
void initGraphicContext(int, char**);
void cleanContext();

void testBounceBall(long);

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

bool bounce = false;

float worldx = 2.0, worldy = 1.0;

double speedx, speedy;

long elapsedTime, deltaTime, lastElapsed;

//-----------------------------------------------
// -- MAIN PROCEDURE
//-----------------------------------------------

int main(int argc,char *argv[]) {

    speedx = (double) (rand() % 20) / 10.0;
    speedy = (double) (rand() % 20) / 10.0;

    initContext();

    initGraphicContext(argc, argv);

    cleanContext();
    return 0;
    
}

//-----------------------------------------------

void initContext() {
    ballPosition = new Point3(0.5, 0.5, 0.0);
    ballVelocity = new Vector3(speedx, speedy, 0.0);
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
    gluOrtho2D(0, worldx, 0, worldy);

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
    glClearColor(1.0,1.0,1.0,0.0); // Clear screen color
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT); // Clear
    
    ball->draw();
    glutSwapBuffers();
}

void keyboard(uchar c, int x, int y) {
    char key = c;
    switch(key) {
        case ' ':
            bounce = !bounce;
            break;
        default:
            break;
    }
    glutPostRedisplay();
}

void idle() {
    elapsedTime = glutGet(GLUT_ELAPSED_TIME);
    deltaTime = elapsedTime - lastElapsed;

    if(bounce) {
        testBounceBall(deltaTime);
    }

    lastElapsed = elapsedTime;

    glutPostRedisplay();
}

void testBounceBall(long delta) {
    double deltaSec = ((double) delta) / 1000.0;
    double x, y;
    bool changed = false;
    Vector3 vector;

    Point3 currentPos = ball->integrate(deltaSec);
    vector = ball->getVelocity();
    x = currentPos.getX();
    y = currentPos.getY();

    if(x <= 0.0 || x > worldx) {
        vector.setX(vector.getX() * -1.0);
        changed = true;
    }

    if(y <= 0.0 || y > worldy) {
        vector.setY(vector.getY() * -1.0);
        changed = true;
    }
    if(changed) {
        ball->setVelocity(vector);
    }
}
