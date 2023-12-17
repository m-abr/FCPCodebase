#include "Geometry.h"


/**
 * This function returns the cosine of a given angle in degrees using the
 * built-in cosine function that works with angles in radians.
 *
 * @param x an angle in degrees
 * @return the cosine of the given angle
 */
float Cos(float x) {
    return ( cos(x * M_PI / 180));
}

/**
 * This function returns the sine of a given angle in degrees using the
 * built-in sine function that works with angles in radians.
 *
 * @param x an angle in degrees
 * @return the sine of the given angle
 */
float Sin(float x) {
    return ( sin(x * M_PI / 180));
}

/**
 * This function returns the principal value of the arc tangent of y/x in
 * degrees using the signs of both arguments to determine the quadrant of the
 * return value. For this the built-in 'atan2' function is used which returns
 * this value in radians.
 *
 * @param x a float value
 * @param y a float value
 * @return the arc tangent of y/x in degrees taking the signs of x and y into
 * account
 */
float ATan2(float x, float y) {
    if (fabs(x) < EPSILON && fabs(y) < EPSILON)
        return ( 0.0);

    return ( atan2(x, y) * 180 / M_PI );
}




/************************************************************************/
/*******************   CLASS VECTOR   ***********************************/
/************************************************************************/

/*! Constructor for the Vector class. Arguments x and y
    denote the x- and y-coordinates of the new position. 
    \param x the x-coordinate of the new position 
    \param y the y-coordinate of the new position
    \return the Vector corresponding to the given arguments */
Vector::Vector(float vx, float vy) : x(vx), y(vy) {}

/*! Overloaded version of unary minus operator for Vectors. It returns the
    negative Vector, i.e. both the x- and y-coordinates are multiplied by
    -1. The current Vector itself is left unchanged.
    \return a negated version of the current Vector */
Vector Vector::operator-() const{
    return ( Vector(-x, -y));
}

/*! Overloaded version of the binary plus operator for adding a given float
    value to a Vector. The float value is added to both the x- and
    y-coordinates of the current Vector. The current Vector itself is
    left unchanged.
    \param d a float value which has to be added to both the x- and
    y-coordinates of the current Vector
    \return the result of adding the given float value to the current
    Vector */
Vector Vector::operator+(const float &d) const{
    return ( Vector(x + d, y + d));
}

/*! Overloaded version of the binary plus operator for Vectors. It returns
    the sum of the current Vector and the given Vector by adding their
    x- and y-coordinates. The Vectors themselves are left unchanged.
    \param p a Vector
    \return the sum of the current Vector and the given Vector */
Vector Vector::operator+(const Vector &p) const{
    return ( Vector(x + p.x, y + p.y));
}

/*! Overloaded version of the binary minus operator for subtracting a
    given float value from a Vector. The float value is
    subtracted from both the x- and y-coordinates of the current
    Vector. The current Vector itself is left unchanged.
    \param d a float value which has to be subtracted from both the x- and
    y-coordinates of the current Vector
    \return the result of subtracting the given float value from the current
    Vector */
Vector Vector::operator-(const float &d) const{
    return ( Vector(x - d, y - d));
}

/*! Overloaded version of the binary minus operator for
    Vectors. It returns the difference between the current
    Vector and the given Vector by subtracting their x- and
    y-coordinates. The Vectors themselves are left unchanged.

    \param p a Vector
    \return the difference between the current Vector and the given
    Vector */
Vector Vector::operator-(const Vector &p) const {
    return ( Vector(x - p.x, y - p.y));
}

/*! Overloaded version of the multiplication operator for multiplying a
    Vector by a given float value. Both the x- and y-coordinates of the
    current Vector are multiplied by this value. The current Vector
    itself is left unchanged.
    \param d the multiplication factor
    \return the result of multiplying the current Vector by the given
    float value */
Vector Vector::operator*(const float &d) const{
    return ( Vector(x * d, y * d));
}

/*! Overloaded version of the multiplication operator for
    Vectors. It returns the product of the current Vector
    and the given Vector by multiplying their x- and
    y-coordinates. The Vectors themselves are left unchanged.

    \param p a Vector
    \return the product of the current Vector and the given Vector */
Vector Vector::operator*(const Vector &p) const{
    return ( Vector(x * p.x, y * p.y));
}

/*! Overloaded version of the division operator for dividing a
    Vector by a given float value. Both the x- and y-coordinates
    of the current Vector are divided by this value. The current
    Vector itself is left unchanged.

    \param d the division factor
    \return the result of dividing the current Vector by the given float
    value */
Vector Vector::operator/(const float &d) const{
    return ( Vector(x / d, y / d));
}

/*! Overloaded version of the division operator for Vectors. It
    returns the quotient of the current Vector and the given
    Vector by dividing their x- and y-coordinates. The
    Vectors themselves are left unchanged.

    \param p a Vector
    \return the quotient of the current Vector and the given one */
Vector Vector::operator/(const Vector &p) const{
    return ( Vector(x / p.x, y / p.y));
}

/*! Overloaded version of the assignment operator for assigning a given float
    value to both the x- and y-coordinates of the current Vector. This
    changes the current Vector itself.
    \param d a float value which has to be assigned to both the x- and
    y-coordinates of the current Vector */
void Vector::operator=(const float &d) {
    x = d;
    y = d;
}

/*! Overloaded version of the sum-assignment operator for Vectors. It
    returns the sum of the current Vector and the given Vector by
    adding their x- and y-coordinates. This changes the current Vector
    itself.
    \param p a Vector which has to be added to the current Vector */
void Vector::operator+=(const Vector &p) {
    x += p.x;
    y += p.y;
}

/*! Overloaded version of the sum-assignment operator for adding a given float
    value to a Vector. The float value is added to both the x- and
    y-coordinates of the current Vector. This changes the current
    Vector itself.
    \param d a float value which has to be added to both the x- and
    y-coordinates of the current Vector */
void Vector::operator+=(const float &d) {
    x += d;
    y += d;
}

/*! Overloaded version of the difference-assignment operator for
    Vectors.  It returns the difference between the current
    Vector and the given Vector by subtracting their x- and
    y-coordinates. This changes the current Vector itself.

    \param p a Vector which has to be subtracted from the current
    Vector */
void Vector::operator-=(const Vector &p) {
    x -= p.x;
    y -= p.y;
}

/*! Overloaded version of the difference-assignment operator for
    subtracting a given float value from a Vector. The float
    value is subtracted from both the x- and y-coordinates of the
    current Vector. This changes the current Vector itself.

    \param d a float value which has to be subtracted from both the x- and
    y-coordinates of the current Vector */
void Vector::operator-=(const float &d) {
    x -= d;
    y -= d;
}

/*! Overloaded version of the multiplication-assignment operator for
    Vectors. It returns the product of the current Vector
    and the given Vector by multiplying their x- and
    y-coordinates. This changes the current Vector itself.

    \param p a Vector by which the current Vector has to be
    multiplied */
void Vector::operator*=(const Vector &p) {
    x *= p.x;
    y *= p.y;
}

/*! Overloaded version of the multiplication-assignment operator for
    multiplying a Vector by a given float value. Both the x- and
    y-coordinates of the current Vector are multiplied by this
    value. This changes the current Vector itself.

    \param d a float value by which both the x- and y-coordinates of the
    current Vector have to be multiplied */
void Vector::operator*=(const float &d) {
    x *= d;
    y *= d;
}

/*! Overloaded version of the division-assignment operator for
    Vectors. It returns the quotient of the current Vector
    and the given Vector by dividing their x- and
    y-coordinates. This changes the current Vector itself.

    \param p a Vector by which the current Vector is divided */
void Vector::operator/=(const Vector &p) {
    x /= p.x;
    y /= p.y;
}

/*! Overloaded version of the division-assignment operator for
    dividing a Vector by a given float value. Both the x- and
    y-coordinates of the current Vector are divided by this
    value. This changes the current Vector itself.

    \param d a float value by which both the x- and y-coordinates of the
    current Vector have to be divided */
void Vector::operator/=(const float &d) {
    x /= d;
    y /= d;
}

/*! Overloaded version of the inequality operator for Vectors. It
    determines whether the current Vector is unequal to the given
    Vector by comparing their x- and y-coordinates.

    \param p a Vector
    \return true when either the x- or y-coordinates of the given Vector
    and the current Vector are different; false otherwise */
bool Vector::operator!=(const Vector &p) {
    return ( (x != p.x) || (y != p.y));
}

/*! Overloaded version of the inequality operator for comparing a
    Vector to a float value. It determines whether either the x-
    or y-coordinate of the current Vector is unequal to the given
    float value.

    \param d a float value with which both the x- and y-coordinates of the
    current Vector have to be compared.
    \return true when either the x- or y-coordinate of the current Vector
    is unequal to the given float value; false otherwise */
bool Vector::operator!=(const float &d) {
    return ( (x != d) || (y != d));
}

/*! Overloaded version of the equality operator for Vectors. It
    determines whether the current Vector is equal to the given
    Vector by comparing their x- and y-coordinates.

    \param p a Vector
    \return true when both the x- and y-coordinates of the given
    Vector and the current Vector are equal; false
    otherwise */
bool Vector::operator==(const Vector &p) {
    return ( (x == p.x) && (y == p.y));
}

/*! Overloaded version of the equality operator for comparing a
    Vector to a float value. It determines whether both the x-
    and y-coordinates of the current Vector are equal to the
    given float value.

    \param d a float value with which both the x- and y-coordinates of the
    current Vector have to be compared.
    \return true when both the x- and y-coordinates of the current Vector
    are equal to the given float value; false otherwise */
bool Vector::operator==(const float &d) {
    return ( (x == d) && (y == d));
}


/*! This method determines the distance between the current
    Vector and a given Vector. This is equal to the
    magnitude (length) of the vector connecting the two positions
    which is the difference vector between them.

    \param p a Vecposition
    \return the distance between the current Vector and the given
    Vector */
float Vector::getDistanceTo(const Vector p) {
    return ( (*this -p).length());
}


/*! This method determines the magnitude (length) of the vector
    corresponding with the current Vector using the formula of
    Pythagoras.

    \return the length of the vector corresponding with the current
    Vector */
float Vector::length() const {
    return ( sqrt(x * x + y * y));
}

float Vector::crossProduct(const Vector p) {
    return this->x*p.y - this->y*p.x;
}


/**
 * This methods returns the inner product of this vector with another
 *
 * @param other the other vector
 * @return inner product
 */
float Vector::innerProduct(const Vector& other) const {
    return x * other.x + y * other.y;
}