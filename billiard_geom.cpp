//-----------------------------------------------
// -- EXTERNAL LIBRARIES
//-----------------------------------------------

#include <GL/glut.h>
#include <cmath>
#include <iostream>
#include <limits>

//-----------------------------------------------

#include "billiard_geom.h"

//-----------------------------------------------
// -- MACRO DEFINITION
//-----------------------------------------------

#define NUM_SEGMENTS 20
#define VEL_THRESHOLD 0.008
#define TWICE_PI 2 * M_PI
#define REBOUND_FACTOR 1.9 // The closer to 1.0 the more energy absorbed
#define IMPULSE_FACTOR 2.0
#define LINE_SIZE 0.25
#define DOUBLE_INF std::numeric_limits<double>::infinity()

#define BALL_MASS 0.130
#define BALL_RADIUS 0.056

#define HOLE_RADIUS 0.112

#define RGB_BLACK RGBColor()
#define RGB_GREY RGBColor(128, 128, 128)
#define RGB_WHITE RGBColor(255, 255, 255)
#define RGB_RED RGBColor(204, 0, 0)
#define RGB_BLUE RGBColor(51, 51, 255)

//-----------------------------------------------
// -- AUXILIARY METHODS
//-----------------------------------------------

void drawCircle(float, float, float, int, RGBColor);
void drawDirectionLine(Ball*, float);

//-----------------------------------------------
// -- AUXILIARY VARIABLES
//-----------------------------------------------

RGBColor lineColor = RGBColor(204, 102, 0);

bool ballInHole = false;
bool ballMoving = false;

//-----------------------------------------------
// -- LIBRARY IMPLEMENTATION
//-----------------------------------------------

RGBColor::RGBColor() : red(0), green(0), blue(0) {}

RGBColor::RGBColor(uchar red, uchar green, uchar blue) :
    red(red), green(green), blue(blue) {}

uchar RGBColor::getRed() { return this->red; }

uchar RGBColor::getGreen() { return this->green; }

uchar RGBColor::getBlue() { return this->blue; }

//-----------------------------------------------

Point3::Point3() : x(0.0), y(0.0), z(0.0) {}

Point3::Point3(double x, double y, double z) : 
    x(x), y(y), z(z) {}

Point3::Point3(const Point3 &point) : 
    x(point.x), y(point.y), z(point.z) {}

double Point3::getX() const { return this->x; }

void Point3::setX(double x) { this->x = x; }

double Point3::getY() const { return this->y; }

void Point3::setY(double y) { this->y = y; }

double Point3::getZ() const { return this->z; }

void Point3::setZ(double z) { this->z = z; }

Point3 Point3::operator + (const Point3 &point) {
    return Point3(getX() + point.getX(), 
        getY() + point.getY(),
        getZ() + point.getZ());
}

Point3 Point3::operator - (const Point3 &point) {
    return Point3(getX() - point.getX(), 
        getY() - point.getY(),
        getZ() - point.getZ());
}

Point3 Point3::operator ^ (const Point3 &point) {
    double newX = (this->x + point.x) / 2.0;
    double newY = (this->y + point.y) / 2.0;
    double newZ = (this->z + point.z) / 2.0;

    return Point3(newX, newY, newZ);
}

//-----------------------------------------------

Vector3::Vector3() : Point3() {}

Vector3::Vector3(double x, double y, double z) : 
    Point3(x, y, z) {}

Vector3::Vector3(const Point3 &point) : 
    Point3(point) {}

double Vector3::modulus() {
    return sqrt(
        pow(getX(), 2) + 
        pow(getY(), 2) + 
        pow(getZ(), 2));
}

Vector3 Vector3::normalized() {
    double mod = modulus();
    if(mod != 0) {
        double newX = getX() / mod;
        double newY = getY() / mod;
        double newZ = getZ() / mod;
        return Vector3(newX, newY, newZ);
    } else {
        return Vector3();
    }
    
}

Vector3 Vector3::operator * (double scalar) {
    return Vector3(getX() * scalar, 
        getY() * scalar,
        getZ() * scalar);
}

double Vector3::operator * (const Vector3 &vector) {
    return (getX() * vector.getX()) + 
           (getY() * vector.getY()) + 
           (getZ() * vector.getZ());
}

//-----------------------------------------------

Particle::Particle() : mass(0.0f), position(Point3()), velocity(Vector3()),
    acceleration(Vector3()), forceAcumulator(Vector3()) {}

Particle::Particle(double mass, Point3 position, Vector3 velocity) :
    mass(mass), position(position), velocity(velocity), 
    acceleration(Vector3()), forceAcumulator(Vector3()) {}

Particle::~Particle() {
    //CODE HERE
}

double Particle::getMass() { return this->mass; }

void Particle::setMass(double mass) { this->mass = mass; }

Point3 Particle::getPosition() { return Point3(this->position); }

void Particle::setPosition(const Point3 position) { 
    this->position = Point3(position);
}

Vector3 Particle::getVelocity() { return Vector3(this->velocity); }

void Particle::setVelocity(const Vector3 velocity) {
    this->velocity = Vector3(velocity);
}

Point3 Particle::integrate(double ftime) {
    this->setPosition(getPosition() + (getVelocity() * ftime));
    this->setVelocity(getVelocity() + (this->acceleration * ftime));

    if(this->getVelocity().modulus() <= VEL_THRESHOLD) {
        this->setVelocity(Vector3());
    }

    return getPosition();
}

void Particle::addForce(const Vector3 force) {
    this->forceAcumulator = this->forceAcumulator + force;
    this->acceleration = this->forceAcumulator * (1.0 / this->mass);
}

void Particle::clearForceAcumulator() {
    this->forceAcumulator = Vector3();
}

//-----------------------------------------------

Ball::Ball() : Particle(), number(0), radius(0.0) {
    setBallType();
}

Ball::Ball(int number, double mass, Point3 position, Vector3 velocity, 
    double radius) : Particle(mass, position, velocity), 
    number(number), radius(radius) {
        setBallType();
    }

Ball::~Ball() {
    //CODE HERE
}

double Ball::getRadius() { return this->radius; }

void Ball::setRadius(double radius) { this->radius = radius; }

void Ball::setOnTable(bool onTable) {
    this->onTable = onTable;
}

bool Ball::isOnTable() {
    return (this->onTable);
}

void Ball::draw() {
    Point3 p = getPosition();
    drawCircle(p.getX(), p.getY(), getRadius(), NUM_SEGMENTS, color);
}

void Ball::setBallType() {
    int num = this->number;
    if(num > 0 && num < 8) {
        this->type = SOLID;
    } else if (num > 8 && num < 16) {
        this->type = STRIPED;
    } else if (num == 8) {
        this->type = EIGHT;
    } else if (num == 0) {
        this->type = CUE;
    } else {
        this->type = UNASSIGNED;
    }
    setBallColor();
}

void Ball::setBallColor() {
    int num = this->number;
    if(num > 0 && num < 8) {
        this->color = RGB_RED;
    } else if (num > 8 && num < 16) {
        this->color = RGB_BLUE;
    } else if (num == 8) {
        this->color = RGB_BLACK;
    } else if (num == 0) {
        this->color = RGB_WHITE;
    } else {
        this->color = RGB_GREY;
    }
}

//-----------------------------------------------

Hole::Hole(Point3 position, double radius) : Ball(8, DOUBLE_INF, position, 
    Vector3(), radius) {}

Hole::~Hole() {
    //CODE HERE
}

//-----------------------------------------------

DragForceGenerator::DragForceGenerator() : k1(0.27), k2(0.32) {}

DragForceGenerator::DragForceGenerator(double k1, double k2) : k1(k1), 
    k2(k2) {}

double DragForceGenerator::getK1() {
    return this->k1;
}

void DragForceGenerator::setK1(double k1) {
    this->k1 = k1;
}

double DragForceGenerator::getK2() {
    return this->k2;
}

void DragForceGenerator::setK2(double k2) {
    this->k2 = k2;
}

void DragForceGenerator::updateForce(Particle *p) {
    Vector3 velocity = p->getVelocity();
    double speed = velocity.modulus();

    Vector3 frictionForce = velocity.normalized() * -1.0; 

    frictionForce = frictionForce * ((k1 * speed) + (k2 * pow(speed,2)));

    p->addForce(frictionForce);
}

//-----------------------------------------------

Plane* Plane::createPlane(Vector3 vector, Point3 point) {
    Plane *p;
    Vector3 norm = vector.normalized();

    double ax = vector.getX() * point.getX();
    double by = vector.getY() * point.getY();
    double cz = vector.getZ() * point.getZ();
    double d = - (ax + by + cz);

    p = new Plane(norm, d);

    return p;
}

double Plane::distanceToPoint(Point3 point) {
    double ax = this->a * point.getX();
    double by = this->b * point.getY();
    double cz = this->c * point.getZ();

    return fabs(ax + by + cz + d);
}

Vector3 Plane::getNormal() {
    return this->normal;
}

Plane::Plane(Vector3 normal, double d) : a(normal.getX()), b(normal.getY()),
    c(normal.getZ()), d(d), normal(normal) {}

//-----------------------------------------------

ParticleContact::ParticleContact(Particle *p1, Particle *p2, 
    Vector3 contactNormal, double interpenetration) : p1(p1), p2(p2),
    contactNormal(contactNormal), interpenetration(interpenetration) {}

Particle* ParticleContact::getP1() {
    return this->p1;
}

Particle* ParticleContact::getP2() {
    return this->p2;
}

Vector3 ParticleContact::getContactNormal() {
    return this->contactNormal;
}

double ParticleContact::getInterpenetration() {
    return this->interpenetration;
}

void ParticleContact::resolve() {
    if(this->p2 == NULL) {
        //PLANE COLLSION CODE
        if(interpenetration < 0.0) {
            Vector3 ballPos = this->p1->getPosition();
            ballPos = ballPos + (this->contactNormal * interpenetration);
            this->p1->setPosition(ballPos);
        }
        Vector3 ballVeloc = this->p1->getVelocity();
        Vector3 closingVeloc = this->contactNormal * 
            (this->contactNormal * ballVeloc);
        ballVeloc = ballVeloc - (closingVeloc * REBOUND_FACTOR);
        this->p1->setVelocity(ballVeloc);
    } else {
        //BALL COLLISION CODE
        double d1, d2, m1, m2, im1, im2;

        m1 = p1->getMass();
        m2 = p2->getMass();

        if(interpenetration < 0.0) {
            

            Vector3 ballPos;

            d1 = (m2 * interpenetration) / (m1 + m2);
            d2 = (m1 * interpenetration) / (m1 + m2);

            ballPos = this->p1->getPosition();
            ballPos = ballPos - (this->contactNormal * d1);
            this->p1->setPosition(ballPos);

            ballPos = this->p2->getPosition();
            ballPos = ballPos + (this->contactNormal * d2);
            this->p2->setPosition(ballPos);
        }
        Vector3 ballVeloc1, ballVeloc2;
        Vector3 closingVeloc1, closingVeloc2, closingVeloc;

        ballVeloc1 = this->p1->getVelocity();
        ballVeloc2 = this->p2->getVelocity();

        closingVeloc1 = this->contactNormal * 
            (this->contactNormal * ballVeloc1);

        closingVeloc2 = this->contactNormal * 
            (this->contactNormal * ballVeloc2);

        closingVeloc = closingVeloc1 - closingVeloc2;

        im1 = 1.0 / m1;
        im2 = 1.0 / m2;

        ballVeloc1 = ballVeloc1 - (closingVeloc * (im1 / (im1 + im2))) * 2;
        ballVeloc2 = ballVeloc2 + (closingVeloc * (im2 / (im1 + im2))) * 2;

        this->p1->setVelocity(ballVeloc1);
        this->p2->setVelocity(ballVeloc2);
    }
}

//-----------------------------------------------

ParticleContact* BallPlaneColDetect::checkCollision(Ball *b, Plane *p) {

    double distance = p->distanceToPoint(b->getPosition()) - b->getRadius();

    if(distance <= 0.0) {
        return new ParticleContact(b, NULL, p->getNormal() * -1.0, distance);
    } else {
        return NULL;
    }
}

//-----------------------------------------------

ParticleContact* BallBallColDetect::checkCollision(Ball *b1, Ball *b2) {

    Vector3 distanceVector = (b1->getPosition() - b2->getPosition());

    double distance = distanceVector.modulus() - 
        (b1->getRadius() + b2->getRadius());

    if(distance <= 0.0) {
        return new ParticleContact(b1, b2, 
            distanceVector.normalized(), distance);
    } else {
        return NULL;
    }
}

//-----------------------------------------------

bool BallHoleColDetect::checkCollision(Ball *b, Hole *h) {

    Vector3 distanceVector = (b->getPosition() - h->getPosition());

    double distance = distanceVector.modulus();

    if(distance <= h->getRadius()) {
        Vector3 v1 = b->getVelocity().normalized();
        Vector3 v2 = distanceVector.normalized();

        double n1 = v1 * v2;
        double n2 = v1.modulus() * v2.modulus();
        double alpha = (1 - fabs(n1 / n2));

        if(distance <= (b->getRadius() * (1 - alpha)) + 
            (h->getRadius() * alpha)) {
            return true;
        }

        return false;
    }

    return false;
}

//-----------------------------------------------

BilliardsTable::BilliardsTable(double w, double h) : width(w), height(h) {
        this->bgen = new BallGenerator(BALL_MASS, BALL_RADIUS);
        this->hgen = new HoleGenerator(HOLE_RADIUS);
        this->planes[0] = Plane::createPlane(Vector3(0.0, -1.0, 0.0), 
        Point3(0.0, 1.0, 0.0));
        this->planes[1] = Plane::createPlane(Vector3(0.0, 1.0, 0.0), 
        Point3(2.0, 0.0, 0.0));
        this->planes[2] = Plane::createPlane(Vector3(-1.0, 0.0, 0.0), 
        Point3(2.0, 1.0, 0.0));
        this->planes[3] = Plane::createPlane(Vector3(1.0, 0.0, 0.0), 
        Point3(0.0, 0.0, 0.0));
        this->drag = new DragForceGenerator(0.084, 0.05);
        initTable();
    }

void BilliardsTable::draw() {
    for(int i = 0; i < 6; i++) {
        holes[i]->draw();
    }


    for(int i = 0; i < 16; i++) {
        if(balls[i] != NULL && balls[i]->isOnTable()) {
            balls[i]->draw();
        }
    }

    if(balls[0] != NULL) {
        drawDirectionLine(balls[0], LINE_SIZE);
    }
}


Point3* BilliardsTable::integrate(double ftime) {
    ParticleContact *cP = NULL, *cB = NULL;

    // CLEARING FORCE AND UPDATING DRAG
    for(unsigned int i = 0; i < 16; i++) {
        balls[i]->clearForceAcumulator();
        drag->updateForce(balls[i]);
    }

    //HOLE COLLISION CHECK
    for(unsigned int i = 0; i < 6; i++) {
        for(unsigned int j = 0; j < 16; j++) {
            if(BallHoleColDetect::checkCollision(balls[j], holes[i])) {
                balls[j]->setOnTable(false);
            }
        }
    }

    //WALL COLLSION CHECK
    for (unsigned int i = 0; i < 16; i++) {
        for(unsigned int j = 0; j < 4; j++) {
            if(balls[i]->isOnTable()) {
                cP = BallPlaneColDetect::checkCollision(balls[i], planes[j]);
                if(cP != NULL) {
                    cP->resolve();
                }
                cP = NULL;
            }
        }
    }

    //BALL COLLISION CHECK
    for(unsigned int i = 0; i < 15; i++) {
        for(unsigned int j = i + 1; j < 16; j++) {
            if(balls[i]->isOnTable() && balls[j]->isOnTable()) {
                cB = BallBallColDetect::checkCollision(balls[i], balls[j]);
                if(cB != NULL) {
                    cB->resolve();
                }
                cB = NULL;
            }
        }
    }

    for(unsigned int i = 1; i < 16; i++) {
        balls[i]->integrate(ftime);
    }

    return new Point3(balls[0]->integrate(ftime));
}

void BilliardsTable::hitBall(Point3 position) {

    Ball *b = this->balls[0];

    b->setVelocity(Vector3(position - b->getPosition()) * IMPULSE_FACTOR);
}

void BilliardsTable::placeObjectBalls() {
    double w = this->width;
    double h = this->height;
    double bx = 2*(w/3.0), by = h/2.0, sby = h/2.0;
    double sep = 0.01, sep2 = sep / 2.0;
    int k = 1, c = 0;

    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < (i+1); j++, k++) {
            if(k==5) {
                balls[k] = this->bgen->generate(8, Point3(bx, by, 0.0));
            } else {
                if(k%2 == 0) { c = 1;} else {c = 9;}
                balls[k] = this->bgen->generate(c, Point3(bx, by, 0.0));
            }
            balls[k]->setOnTable(true);
            by -= (2*BALL_RADIUS)+sep;
        }
        by = sby + ((BALL_RADIUS)*(i+1)) + (sep2*(i+1));
        bx += BALL_RADIUS*2;
    }
}

void BilliardsTable::placeCueBall() {
    balls[0] = this->bgen->generate(0, 
        Point3(this->width/4.0, this->height/2.0, 0.0));
    balls[0]->setOnTable(true);
}

void BilliardsTable::initTable() {
    double w = this->width, iw = w / 2.0, pw = 0.0;
    double h = this->height, ph = h;

    w = w + 1.0;

    for(int i = 0; i < 2; i++, ph = (ph - h)) {
        for(int j = 0; j < 3; j++, pw = fmod(pw + iw, w)) {
            holes[(3 * i) + j] = this->hgen->generate(Point3(pw, ph, 0.0));
        }
    }
}

//-----------------------------------------------

BallGenerator::BallGenerator(double mass, double radius) : 
    mass(mass), radius(radius) {}

void BallGenerator::setMass(double mass) {
    this->mass = mass;
}

void BallGenerator::setRadius(double radius) {
    this->radius = radius;
}

Ball* BallGenerator::generate(int num) {
    return generate(num, Vector3());
}

Ball* BallGenerator::generate(int num, Point3 position) {
    Ball *b = NULL;
    b = new Ball(num, this->mass, position, Vector3(), 
        this->radius);
    return b;
}

//-----------------------------------------------

HoleGenerator::HoleGenerator(double radius) : BallGenerator(DOUBLE_INF, 
    radius) {}

void HoleGenerator::setRadius(double radius) {
    this->radius = radius;
}

Hole* HoleGenerator::generate(Point3 position) {
    return new Hole(position, this->radius);
}

//-----------------------------------------------

Player::Player(std::string name, BallType type) : name(name), 
assigType(type) {}

std::string Player::getName() {
    return this->name;
}

void Player::setName(std::string name) {
    this->name = name;
}

BallType Player::getBallType() {
    return this->assigType;
}

void Player::setBallType(BallType type) {
    this->assigType = type;
}

//-----------------------------------------------

HumanPlayer::HumanPlayer(std::string name, BallType type) : 
    Player(name, type) {}

Vector3 HumanPlayer::getShot() {
    while(!this->listener.shotRegistered());

    return Vector3(*this->listener.getRegisteredShot());
}

void HumanPlayer::changeFromPrevious() {
    // FICAR FORA DE CLASSE
    glutMouseFunc((void(*)(void*,int,int,int,int))&this->listener.shotRegisterer);
}

//-----------------------------------------------

MouseShot::MouseShot() : sReg(false), shot(NULL) {}

bool MouseShot::shotRegistered() {
    return sReg;
}

void MouseShot::shotRegisterer(int button, int state, int x, int y) {
    std::cout << "Called" << std::endl;
    double px = (double) x / 320.0;
    double py = 1 - ((double) y / 320.0);

    shot = new Point3(px, py, 0.0);

    sReg = true;
}

Point3* MouseShot::getRegisteredShot() {
    return this->shot;
}

//-----------------------------------------------

Game::Game(double w, double h, Player *p0, Player *p1) {
    this->table = new BilliardsTable(w, h);
    this->players[0] = p0;
    this->players[1] = p1;
    this->turn = -1;
    this->playing = false;
}

void Game::draw() {
    table->draw();
}

void Game::startGame(int startPlayer) {
    this->turn = startPlayer;
    table->placeObjectBalls();
    table->placeCueBall();
}

int Game::winner() {
    return -1; // TODO
}

void Game::nextShot() {
    Vector3 shot = players[turn]->getShot();
}

void Game::integrate(double ftime) {
    if(this->playing) {
        table->integrate(ftime);
    }
}

//-----------------------------------------------

void drawCircle(float cx, float cy, float r, 
    int numSegments, RGBColor color) {

    int i;
    glColor3ub(color.getRed(), color.getGreen(), color.getBlue());
    glBegin(GL_TRIANGLE_FAN);
        glVertex2f(cx, cy);
        for(i = 0; i <= numSegments; i++) { 
            glVertex2f(
                    cx + (r * cos(i *  TWICE_PI / numSegments)), 
                cy + (r * sin(i * TWICE_PI / numSegments))
            );
        }
    glEnd();
}

void drawDirectionLine(Ball *b, float size) {
    Vector3 ballPos = b->getPosition(), nBallPos;
    Vector3 direction = b->getVelocity();

    direction = direction.normalized() * size;
    nBallPos = ballPos + direction;

    glColor3ub(lineColor.getRed(), 
        lineColor.getGreen(), lineColor.getBlue());
    glPointSize(3.0);

    glBegin(GL_LINES);
    glVertex2f(ballPos.getX(), ballPos.getY());
    glVertex2f(nBallPos.getX(), nBallPos.getY());
    glEnd();
}

//-----------------------------------------------

#ifndef NO_TEST

//-----------------------------------------------
// -- TEST MAIN
//-----------------------------------------------

#include <cassert>

//-----------------------------------------------

int main() {

    RGBColor *color;
    Point3 *point1, *point2, point3, *point4, point5, point6;
    Vector3 *vector1, *vector2, vector3, vector4, vector5, vector6;
    Particle *particle;
    Ball *ball; 

    double sqrt5 = sqrt(5.0);

    color = new RGBColor(123, 23, 45);

    point1 = new Point3(0.2, 1.1, -0.2);
    point2 = new Point3(1.1, -0.2, 2.2);

    point3 = *point1 ^ *point2;

    point4 = new Point3(
        (point1->getX() + point2->getX()) / 2.0, 
        (point1->getY() + point2->getY()) / 2.0, 
        (point1->getZ() + point2->getZ()) / 2.0);

    vector1 = new Vector3(2.0, 1.0, 0.0);
    vector2 = new Vector3(
        2.0 / sqrt5, 
        1.0 / sqrt5, 
        0.0 / sqrt5);

    vector3 = vector1->normalized();

    vector4 = Vector3(*vector1 + *vector1);
    vector5 = Vector3(*vector1 - vector4);
    vector6 = *vector1 * 4.0;

    particle = new Particle(2.0, *point1, *vector1);

    point5 = particle->integrate(1.0);
    point6 = particle->getPosition();

    ball = new Ball(1.1, *point2, *vector2, 0.1, *color);

    assert(color->getRed() == 123);
    assert(color->getGreen() == 23);
    assert(color->getBlue() == 45);

    assert(point1->getX() == 0.2);
    assert(point1->getY() == 1.1);
    assert(point1->getZ() == -0.2);

    assert(point2->getX() == 1.1);
    assert(point2->getY() == -0.2);
    assert(point2->getZ() == 2.2);

    assert(point3.getX() == point4->getX());
    assert(point3.getY() == point4->getY()); 
    assert(point3.getZ() == point4->getZ());

    assert(vector1->modulus() == sqrt5);
    assert(vector3.getX() == vector2->getX());
    assert(vector3.getY() == vector2->getY());
    assert(vector3.getZ() == vector2->getZ());

    assert(vector4.getX() == 4.0);
    assert(vector4.getY() == 2.0);
    assert(vector4.getZ() == 0.0);

    assert(vector5.getX() == -2.0);
    assert(vector5.getY() == -1.0);
    assert(vector5.getZ() == 0.0);

    assert(vector6.getX() == 8.0);
    assert(vector6.getY() == 4.0);
    assert(vector6.getZ() == 0.0);

    assert(point5.getX() == 2.2);
    assert(point5.getY() == 2.1);
    assert(point5.getZ() == -0.2);

    assert(point5.getX() == point6.getX());
    assert(point5.getY() == point6.getY());
    assert(point5.getZ() == point6.getZ());

    delete color;
    delete point1; delete point2; delete point4;
    delete vector1; delete vector2; 
    delete particle;
    delete ball;

    std::cout << "All tests succesful!" << std::endl;

    return 0;
}

//-----------------------------------------------

#endif
