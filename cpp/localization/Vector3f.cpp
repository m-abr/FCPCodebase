#include "Vector3f.h"


using namespace std;

Vector3f::Vector3f() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
}

Vector3f::Vector3f(float x, float y, float z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

Vector3f::Vector3f(const Vector3f& other) {
    x = other.x;
    y = other.y;
    z = other.z;
}

Vector3f::Vector3f(const Vector& other) {
    x = other.x;
    y = other.y;
    z = 0.0;
}

Vector3f::~Vector3f() {
}

float Vector3f::getX() const {
    return x;
}

void Vector3f::setX(float x) {
    this->x = x;
}

float Vector3f::getY() const {
    return y;
}

void Vector3f::setY(float y) {
    this->y = y;
}

float Vector3f::getZ() const {
    return z;
}

void Vector3f::setZ(float z) {
    this->z = z;
}

float Vector3f::operator[](const int index) const {
    float val=0.0;
    switch (index) {
        case 0: val = x;
                break;
        case 1: val = y;
                break;
        case 2: val = z;
                break;
    }
    return val;
}

Vector3f Vector3f::operator+(const Vector3f& other) const {
    return Vector3f(x + other.x, y + other.y, z + other.z);
}

Vector3f Vector3f::operator-(const Vector3f& other) const {
    return Vector3f(x - other.x, y - other.y, z - other.z);
}

Vector3f Vector3f::operator-() const {
    return Vector3f() - * this;
}

Vector3f Vector3f::operator*(const Vector3f& other) const {
    return Vector3f(x * other.x, y * other.y, z * other.z);
}

Vector3f Vector3f::operator/(const Vector3f& other) const {
    return Vector3f(x / other.x, y / other.y, z / other.z);
}

bool Vector3f::operator==(const Vector3f& other) const {
    return x == other.x && y == other.y && z == other.z;
}

Vector3f Vector3f::operator/(float factor) const {
    return Vector3f(x / factor, y / factor, z / factor);
}

Vector3f Vector3f::operator+(float factor) const {
    return Vector3f(x + factor, y + factor, z + factor);
}

Vector3f Vector3f::operator%(float factor) const {
    return Vector3f(fmod(x, factor), fmod(y, factor), fmod(z, factor));
}

Vector3f Vector3f::operator*(float factor) const {
    return Vector3f(x * factor, y * factor, z * factor);
}

Vector3f Vector3f::operator+=(const Vector3f& other) {
    x += other.x;
    y += other.y;
    z += other.z;

    return *this;
}

Vector3f Vector3f::operator+=(float factor) {
    x += factor;
    y += factor;
    z += factor;

    return *this;
}

Vector3f Vector3f::operator-=(const Vector3f& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;

    return *this;
}

Vector3f Vector3f::operator-=(float factor) {
    x -= factor;
    y -= factor;
    z -= factor;

    return *this;
}

Vector3f Vector3f::operator/=(float factor) {
    x /= factor;
    y /= factor;
    z /= factor;

    return *this;
}


float Vector3f::innerProduct(const Vector3f& other) const {
    return x * other.x + y * other.y + z * other.z;
}

Vector3f Vector3f::crossProduct(const Vector3f& other) const {
    Vector3f aux;

    aux.x = this->y * other.z - this->z * other.y;
    aux.y = this->z * other.x - this->x * other.z;
    aux.z = this->x * other.y - this->y * other.x;

    return aux;
}

float Vector3f::length() const {
    return sqrt(x * x + y * y + z * z);
}


Vector3f Vector3f::normalize(float len) const {
    return (*this) * (len / this->length());
}

Vector3f Vector3f::toCartesian() const {
    // x = distance
    // y = theta
    // z = phi
    return Vector3f(x * Cos(z) * Cos(y), x * Cos(z) * Sin(y), x * Sin(z));
}

Vector3f Vector3f::toPolar() const {
    return Vector3f(this->length(), // distance
            ATan2(y, x), // theta
            ATan2(z, sqrt(x * x + y * y))); // phi
}

float Vector3f::dist(const Vector3f &other) const {
    return (*this -other).length();
}

Vector Vector3f::to2d() const {
    return Vector(x, y);
}

Vector3f Vector3f::determineMidpoint(Vector3f a, Vector3f b) {
    return (a+b)/2; /* m.abreu@2020 */
}
