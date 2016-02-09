#include <cmath>

#include "billiard_geom.h"

RGBColor::RGBColor() : red(0), green(0), blue(0) {}

RGBColor::RGBColor(uchar red, uchar green, uchar blue) :
	red(red), green(green), blue(blue) {}

uchar RGBColor::getRed() { return this->red; }

uchar RGBColor::getGreen() { return this->green; }

uchar RGBColor::getBlue() { return this->blue; }

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

Point3 Point3::operator ^ (const Point3 &point) {
	double newX = (this->x + point.x) / 2.0;
	double newY = (this->y + point.y) / 2.0;
	double newZ = (this->z + point.z) / 2.0;

	return Point3(newX, newY, newZ);
}

Vector3::Vector3() : Point3() {}

Vector3::Vector3(double x, double y, double z) : 
	Point3(x, y, z) {}

Vector3::Vector3(const Vector3 &vector) : 
	Point3(vector) {}

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

Vector3 Vector3::operator + (const Vector3 &vector) {
	return Vector3(getX() + vector.getX(), 
		getY() + vector.getY(),
		getZ() + vector.getZ());
}

Vector3 Vector3::operator - (const Vector3 &vector) {
	return Vector3(getX() - vector.getX(), 
		getY() - vector.getY(),
		getZ() - vector.getZ());
}

Vector3 Vector3::operator * (double scalar) {
	return Vector3(getX() * scalar, 
		getY() * scalar,
		getZ() * scalar);
}

int main() {
	return 0;
}