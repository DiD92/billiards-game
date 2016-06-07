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

int p1type, p2type;

//-----------------------------------------------
// -- MAIN PROCEDURE
//-----------------------------------------------

int main(int argc,char *argv[]) {

    if(argc != 3) {
        std::cout << "Usage: " << argv[0] << " <player 1> <player 2>" << std::endl;
        std::cout << "Player types are: " << std::endl;
        std::cout << "0 - Human player, 1 - Base AI, 2 - DirectShot AI" << std::endl;

        return -1;
    } else {
        p1type = atoi(argv[1]);
        p2type = atoi(argv[2]);

        if(p1type < 0 || p1type > 2) {
            std::cout << "Error, player 1 valid type range: 0 - 2" << std::endl;
            return -1;
        }

        if(p2type < 0 || p2type > 2) {
            std::cout << "Error, player 2 valid type range: 0 - 2" << std::endl;
            return -1;
        }
    }

    initContext();

    initGraphicContext(argc, argv);

    cleanContext();

    return 0;  
}

//-----------------------------------------------

void initContext() {

    Player *p1 = NULL, *p2 = NULL;

    switch(p1type) {
        case 0:
            p1 = new HumanPlayer("P1", CUE);
            break;
        case 1:
            p1 = new BotPlayer("P1", CUE, new AI());
            break;
        default:
            p1 = new BotPlayer("P1", CUE, new DirectShotAI());
            break;
    }

    switch(p2type) {
        case 0:
            p2 = new HumanPlayer("P2", CUE);
            break;
        case 1:
            p2 = new BotPlayer("P2", CUE, new AI());
            break;
        default:
            p2 = new BotPlayer("P2", CUE, new DirectShotAI());
            break;
    }

    AI *ai = new DirectShotAI();
    game = new Game(worldx, worldy, p1, p2);
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
    glutMouseFunc(mouse);
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
    if(game != NULL) {
        game->processMouse(x, y);
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

    if(!game->gameEnded()) {
        game->nextShot();
    }    

    game->integrate(deltaSeconds);

    lastElapsed = elapsedTime;

    glutPostRedisplay();
}
