//-----------------------------------------------
// -- PREPROCESSOR DIRECTIVES
//-----------------------------------------------

#ifndef _BILLIARDS_GEOM_H_
#define _BILLIARDS_GEOM_H_

//-----------------------------------------------
// -- EXTERNAL LIBRARIES
//-----------------------------------------------

#include <string>

//-----------------------------------------------
// -- TYPE DEFINITIONS
//-----------------------------------------------

typedef unsigned char uchar;

//-----------------------------------------------
// -- LIBRARY DEFINITIONS
//-----------------------------------------------

enum BallType { CUE, SOLID, EIGHT, STRIPED, UNASSIGNED };

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
        double operator * (const Vector3 &vector);
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
        Ball(int number, double mass, Point3 position, 
            Vector3 velocity, double radius);
        ~Ball();

        double getRadius();
        void setRadius(double radius);
        void setOnTable(bool onTable);
        bool isOnTable();

        void draw();

    private:
        void setBallType();
        void setBallColor();

        int number;
        double radius;
        RGBColor color;
        BallType type;
        bool onTable;

};

class Hole : public Ball {

    public:
        Hole(Point3 position, double radius);
        ~Hole();

};

class IForceGenerator {

    public:
        virtual ~IForceGenerator() {}
        virtual void updateForce(Particle *p) = 0;
};

class DragForceGenerator : IForceGenerator {

    public:
        DragForceGenerator();
        DragForceGenerator(double k1, double k2);
        double getK1();
        void setK1(double k1);
        double getK2();
        void setK2(double k2);

        void updateForce(Particle *p);

    private:
        double k1, k2;
};

class Plane {

    public:
        static Plane* createPlane(Vector3 vector, Point3 point);
        double distanceToPoint(Point3 point);
        Vector3 getNormal();

    private:
        Plane(Vector3 normal, double d);

        double a, b, c, d;
        Vector3 normal;
};

class ParticleContact {

    public:
        ParticleContact(Particle *p1, Particle *p2, 
            Vector3 contactNormal, double interpenetration);
        Particle* getP1();
        Particle* getP2();
        Vector3 getContactNormal();
        double getInterpenetration();
        void resolve();

    private:
        Particle *p1, *p2;
        Vector3 contactNormal;
        double interpenetration;
};

class BallPlaneColDetect {

    public:
        static ParticleContact* checkCollision(Ball *b, Plane *p);  

};

class BallBallColDetect {

    public:
        static ParticleContact* checkCollision(Ball *b1, Ball *b2);
};

class BallHoleColDetect {

    public:
        static bool checkCollision(Ball *b, Hole *h);
};

class BallGenerator {

    public:
        BallGenerator(double mass, double radius);
        void setMass(double mass);
        void setRadius(double radius);
        Ball* generate(int num);
        Ball* generate(int num, Point3 position);
    protected:
        double mass, radius;
};

class HoleGenerator : BallGenerator {

    public:
        HoleGenerator(double radius);
        void setRadius(double radius);
        Hole* generate(Point3 position);
};

class BilliardsTable {

    public: 
        BilliardsTable(double w, double h);

        void draw();
        Point3* integrate(double ftime);
        void hitBall(Point3 vector);
        void placeObjectBalls();
        void placeCueBall();

    private:
        void initTable();

        double width, height;
        Plane *planes[4];
        Hole *holes[6];
        Ball *balls[16];

        DragForceGenerator *drag;
        BallGenerator *bgen;
        HoleGenerator *hgen;

};

class Player {

    public:
        Player(std::string name, BallType type);
        std::string getName();
        void setName(std::string name);
        BallType getBallType();
        void setBallType(BallType t);

        virtual Vector3 getShot() = 0;
        virtual void changeFromPrevious() = 0;
        
    private:
        std::string name;
        BallType assigType;

};

class MouseShot {

    public:
        MouseShot();
        bool shotRegistered();
        void shotRegisterer(int button, int state, int x, int y);
        Point3* getRegisteredShot();

    private:
        bool sReg;
        Point3 *shot;
};

class HumanPlayer : public Player {

    public:
        HumanPlayer(std::string name, BallType type);
        Vector3 getShot();
        void changeFromPrevious();
    private: 
        MouseShot listener;
};

class Game : IDrawable {

    public:
        Game(double w, double h, Player *p0, Player *p1);
        void draw();
        void startGame(int startPlayer);
        int winner();
        void nextShot();

        void integrate(double ftime);

    private:
        BilliardsTable *table;
        Player *players[2];
        int turn;
        bool playing;
};

#endif
