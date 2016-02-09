//-----------------------------------------------
// -- EXTERNAL LIBRARIES
//-----------------------------------------------

#include <GL/glut.h>
#include <cmath>
#include <iostream>

//-----------------------------------------------

#include "billiard_geom.h"

//-----------------------------------------------
// -- MACRO DEFINITION
//-----------------------------------------------

#define NUM_SEGMENTS 20

//-----------------------------------------------
// -- AUXILIARY METHODS
//-----------------------------------------------

void drawCircle(float cx, float cy, float r, int numSegments);

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
    double newX = getX() / mod;
    double newY = getY() / mod;
    double newZ = getZ() / mod;

    return Vector3(newX, newY, newZ);
}

Vector3 Vector3::operator * (double scalar) {
    return Vector3(getX() * scalar, 
        getY() * scalar,
        getZ() * scalar);
}

//-----------------------------------------------

Particle::Particle() : mass(0.0f), position(Point3()), velocity(Vector3()) {}

Particle::Particle(double mass, Point3 position, Vector3 velocity) :
    mass(mass), position(position), velocity(velocity) {}

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

    return getPosition();
}

//-----------------------------------------------

Ball::Ball() : Particle(), radius(0.0), color(RGBColor()) {}

Ball::Ball(double mass, Point3 position, Vector3 velocity, double radius, 
            RGBColor color) : Particle(mass, position, velocity), 
            radius(radius), color(color) {}

Ball::~Ball() {
    //CODE HERE
}

double Ball::getRadius() { return this->radius; }

void Ball::setRadius(double radius) { this->radius = radius; }

void Ball::setColor(RGBColor color) { this->color = color; }

void Ball::draw() {
    //CODE HERE
    Point3 p = getPosition();
    drawCircle(p.getX(), p.getY(), getRadius(), NUM_SEGMENTS);
}

void drawCircle(float cx, float cy, float r, int num_segments) {

    glBegin(GL_LINE_LOOP);
    for(int ii = 0; ii < num_segments; ii++) {
        float theta = 2.0f * M_PI * float(ii) / float(num_segments);

        float x = r * cosf(theta);//calculate the x component
        float y = r * sinf(theta);//calculate the y component

        glVertex2f(x + cx, y + cy);//output vertex

    }
    glEnd();
}

//-----------------------------------------------

#ifndef _BILLIARD_MAIN_
#define _BILLIARD_MAIN_

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

    return 0;
}

//-----------------------------------------------

#endif
