//-----------------------------------------------
// -- PREPROCESSOR DIRECTIVES
//-----------------------------------------------

#ifndef _BILLIARDS_GEOM_H_
#define _BILLIARDS_GEOM_H_

//-----------------------------------------------
// -- TYPE DEFINITIONS
//-----------------------------------------------

typedef unsigned char uchar;

//-----------------------------------------------
// -- LIBRARY DEFINITIONS
//-----------------------------------------------

class IDrawable {

    public:    
        virtual ~IDrawable() {}
        virtual void draw() = 0;
};

class RGBColor {

    public:
        RGBColor();
        RGBColor(uchar red, uchar green, uchar blue);

        uchar getRed();
        uchar getGreen();
        uchar getBlue();

    private:
        uchar red;
        uchar green;
        uchar blue;

};

class Point3 {

    public:
        Point3();
        Point3(double x, double y, double z);
        Point3(const Point3 &point);

        double getX() const;
        void setX(double x);
        double getY() const;
        void setY(double y);
        double getZ() const;
        void setZ(double z);

        Point3 operator + (const Point3 &point);
        Point3 operator - (const Point3 &point);
        Point3 operator ^ (const Point3 &point);

    private:
        double x, y, z;
};

class Vector3 : public Point3 {

    public:
        Vector3();
        Vector3(double x, double y, double z);
        Vector3(const Point3 &point);

        double modulus();
        Vector3 normalized();

        Vector3 operator * (double scalar);
};

class Particle {

    public:
        Particle();
        Particle(double mass, Point3 position, Vector3 velocity);
        ~Particle();

        double getMass();
        void setMass(double mass);
        Point3 getPosition();
        void setPosition(Point3 position);
        Vector3 getVelocity();
        void setVelocity(Vector3 velocity);

        Point3 integrate(double time);
        void addForce(Vector3 force);
        void clearForceAcumulator();

    private:
        double mass;
        Point3 position;
        Vector3 velocity;
        Vector3 acceleration;
        Vector3 forceAcumulator;

};

class Ball : public Particle, public IDrawable {

    public:
        Ball();
        Ball(double mass, Point3 position, Vector3 velocity, double radius, 
            RGBColor color);
        ~Ball();

        double getRadius();
        void setRadius(double radius);
        void setColor(RGBColor color);

        void draw();

    private:
        double radius;
        RGBColor color;

};

class IForceGenerator {

    public:
        virtual ~IForceGenerator() {}
        virtual void updateForce(Particle *p, double ftime) = 0;
};

class DragForceGenerator : IForceGenerator {

    public:
        DragForceGenerator();
        DragForceGenerator(double k1, double k2);
        double getK1();
        void setK1(double k1);
        double getK2();
        void setK2(double k2);

        void updateForce(Particle *p, double ftime);

    private:
        double k1, k2;
};

#endif
