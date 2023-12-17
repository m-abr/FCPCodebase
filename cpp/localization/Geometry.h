#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <cmath>
#include <algorithm>

using namespace std;

#define EPSILON     1e-10

/**
 * Useful functions to operate with angles in degrees
 */
float Cos(float x);
float Sin(float x);
float ATan2(float x, float y);


/**
 * @class Vector
 *
 * @brief This class represents a position in the 2d space
 *
 * A position is represented by a x-axis coordinate and a
 * y-axis coordinate or in polar coordinates (r, phi)
 *
 * @author Hugo Picado (hugopicado@ua.pt)
 * @author Nuno Almeida (nuno.alm@ua.pt)
 * Adapted - Miguel Abreu
 */
class Vector {
public:
    Vector(float vx = 0, float vy = 0);

    // overloaded arithmetic operators
    Vector operator-() const;
    Vector operator+(const float &d) const;
    Vector operator+(const Vector &p) const;
    Vector operator-(const float &d) const;
    Vector operator-(const Vector &p) const;
    Vector operator*(const float &d) const;
    Vector operator*(const Vector &p) const;
    Vector operator/(const float &d) const;
    Vector operator/(const Vector &p) const;
    void operator=(const float &d);
    void operator+=(const Vector &p);
    void operator+=(const float &d);
    void operator-=(const Vector &p);
    void operator-=(const float &d);
    void operator*=(const Vector &p);
    void operator*=(const float &d);
    void operator/=(const Vector &p);
    void operator/=(const float &d);
    bool operator!=(const Vector &p);
    bool operator!=(const float &d);
    bool operator==(const Vector &p);
    bool operator==(const float &d);

    float getDistanceTo(const Vector p);
    float crossProduct(const Vector p);
    float length() const;
    float innerProduct(const Vector &p) const;

public:
    float x;
    float y;
};



#endif // GEOMETRY_H
