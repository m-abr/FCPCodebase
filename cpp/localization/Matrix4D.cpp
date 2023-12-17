#include "Matrix4D.h"



Matrix4D::Matrix4D() {
    // identity matrix
    const float tmp[M_LENGTH] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };

    for (int i = 0; i < M_LENGTH; i++)
        content[i] = tmp[i];
}

Matrix4D::Matrix4D(const float a[M_LENGTH]) {
    // creates a matrix using a vector of floats
    for (int i = 0; i < M_LENGTH; i++)
        content[i] = a[i];
}

Matrix4D::Matrix4D(const Matrix4D& other) {
    // creates a matrix using another matrix
    for (int i = 0; i < M_LENGTH; i++)
        content[i] = other.content[i];
}

Matrix4D::Matrix4D(const Vector3f& v) {
    float x = v.getX();
    float y = v.getY();
    float z = v.getZ();

    // gets a translation matrix from xyz coordinates
    const float tmp[M_LENGTH] = {
        1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1
    };

    for (int i = 0; i < M_LENGTH; i++)
        content[i] = tmp[i];
}

Matrix4D::~Matrix4D() {
    // nothing to do
}

void Matrix4D::set(unsigned i, float value) {
    content[i] = value;
}

void Matrix4D::set(unsigned i, unsigned j, float value) {
    content[M_COLS * i + j] = value;
}

float Matrix4D::get(unsigned i) const{
    return content[i];
}

float Matrix4D::get(unsigned i, unsigned j) const{
    return content[M_COLS * i + j];
}

Matrix4D Matrix4D::operator+(const Matrix4D& other) const {
    // sums two matrices
    float tmp[M_LENGTH];

    for (int i = 0; i < M_LENGTH; i++)
        tmp[i] = this->content[i] + other.content[i];

    return Matrix4D(tmp);
}

Matrix4D Matrix4D::operator-(const Matrix4D& other) const {
    // subtracts a matrix from another
    float tmp[M_LENGTH];

    for (int i = 0; i < M_LENGTH; i++)
        tmp[i] = this->content[i] - other.content[i];

    return Matrix4D(tmp);
}

Matrix4D Matrix4D::operator*(const Matrix4D& other) const {

    // multiplies two matrices
    float tmp[M_LENGTH];

    for (int i = 0; i < M_ROWS; i++) {
        for (int j = 0; j < M_COLS; j++) {
            tmp[M_COLS * i + j] = 0;

            for (int k = 0; k < M_COLS; k++)
                tmp[M_COLS * i + j] += this->content[M_COLS * i + k] * other.content[M_COLS * k + j];
        }
    }

    return Matrix4D(tmp);
}

Vector3f Matrix4D::operator*(const Vector3f& vec) const {
    // multiplies this matrix by a vector of four floats
    // the first three are from vec and the remaining float is 1.0
    float x = 0;
    float y = 0;
    float z = 0;

    x = this->content[0] * vec.getX();
    x += this->content[1] * vec.getY();
    x += this->content[2] * vec.getZ();
    x += this->content[3];

    y = this->content[4] * vec.getX();
    y += this->content[5] * vec.getY();
    y += this->content[6] * vec.getZ();
    y += this->content[7];

    z = this->content[8] * vec.getX();
    z += this->content[9] * vec.getY();
    z += this->content[10] * vec.getZ();
    z += this->content[11];

    return Vector3f(x, y, z);
}

void Matrix4D::operator=(const Matrix4D& other) {
    // assigns another matrix to this one
    for (int i = 0; i < M_LENGTH; i++)
        content[i] = other.content[i];
}

bool Matrix4D::operator==(const Matrix4D& other) const {
    // checks whether this matrix is equal to another
    for (int i = 0; i < M_LENGTH; i++)
        if (content[i] != other.content[i])
            return false;

    return true;
}

void Matrix4D::operator+=(const Matrix4D& other) {
    // sums this matrix to another and returns the result
    for (int i = 0; i < M_LENGTH; i++)
        content[i] += other.content[i];
}

void Matrix4D::operator-=(const Matrix4D& other) {
    // subtracts this matrix from another and returns the result
    for (int i = 0; i < M_LENGTH; i++)
        content[i] -= other.content[i];
}

float& Matrix4D::operator[](const unsigned pos) {
    // gets a value from position
    return content[pos];
}

Vector3f Matrix4D::toVector3f() const {
    // gets the translation vector from the matrix
    float x = get(0, M_COLS - 1);
    float y = get(1, M_COLS - 1);
    float z = get(2, M_COLS - 1);

    return Vector3f(x, y, z);
}

Matrix4D Matrix4D::transpose() {
    // returns the transpose of this matrix
    Matrix4D result;

    for (int i = 0; i < M_ROWS; i++)
        for (int j = 0; j < M_COLS; j++)
            result.set(j, i, get(i, j));

    return result;
}

Matrix4D Matrix4D::inverse_tranformation_matrix() const {

    Matrix4D inv; //Initialized as identity matrix
    inverse_tranformation_matrix(inv);
    return inv;
}

void Matrix4D::inverse_tranformation_matrix(Matrix4D& inv) const {

    //Rotation
    inv[0] = content[0];    inv[1] = content[4];    inv[2] = content[8];
    inv[4] = content[1];    inv[5] = content[5];    inv[6] = content[9];
    inv[8] = content[2];    inv[9] = content[6];    inv[10] = content[10];

    //Translation
    inv[3] = -content[0]*content[3] - content[4]*content[7] - content[8]*content[11];
    inv[7] = -content[1]*content[3] - content[5]*content[7] - content[9]*content[11];
    inv[11] = -content[2]*content[3] - content[6]*content[7] - content[10]*content[11];

}

bool Matrix4D::inverse(Matrix4D& inverse_out) const{
    // returns the inverse of this matrix

    float inv[16], det;
    const float* m = content;
    int i;

    inv[0] =   m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
    inv[4] =  -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
    inv[8] =   m[4] * m[9]  * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
    inv[12] = -m[4] * m[9]  * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
    inv[1] =  -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
    inv[5] =   m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
    inv[9] =  -m[0] * m[9]  * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
    inv[13] =  m[0] * m[9]  * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
    inv[2] =   m[1] * m[6]  * m[15] - m[1] * m[7]  * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7]  - m[13] * m[3] * m[6];
    inv[6] =  -m[0] * m[6]  * m[15] + m[0] * m[7]  * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7]  + m[12] * m[3] * m[6];
    inv[10] =  m[0] * m[5]  * m[15] - m[0] * m[7]  * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7]  - m[12] * m[3] * m[5];
    inv[14] = -m[0] * m[5]  * m[14] + m[0] * m[6]  * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6]  + m[12] * m[2] * m[5];
    inv[3] =  -m[1] * m[6]  * m[11] + m[1] * m[7]  * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[9]  * m[2] * m[7]  + m[9]  * m[3] * m[6];
    inv[7] =   m[0] * m[6]  * m[11] - m[0] * m[7]  * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8]  * m[2] * m[7]  - m[8]  * m[3] * m[6];
    inv[11] = -m[0] * m[5]  * m[11] + m[0] * m[7]  * m[9]  + m[4] * m[1] * m[11] - m[4] * m[3] * m[9]  - m[8]  * m[1] * m[7]  + m[8]  * m[3] * m[5];
    inv[15] =  m[0] * m[5]  * m[10] - m[0] * m[6]  * m[9]  - m[4] * m[1] * m[10] + m[4] * m[2] * m[9]  + m[8]  * m[1] * m[6]  - m[8]  * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0)
        return false;

    det = 1.0 / det;

    for (i = 0; i < 16; i++)
        inverse_out.set(i, inv[i] * det);

    return true;
}


Matrix4D Matrix4D::rotationX(float angle) {
    // rotates this matrix around the x-axis
    const float tmp[M_LENGTH] = {
        1, 0, 0, 0,
        0, Cos(angle), -Sin(angle), 0,
        0, Sin(angle), Cos(angle), 0,
        0, 0, 0, 1
    };

    return Matrix4D(tmp);
}

Matrix4D Matrix4D::rotationY(float angle) {
    // rotates this matrix around the y-axis
    const float tmp[M_LENGTH] = {
        Cos(angle), 0, Sin(angle), 0,
        0, 1, 0, 0,
        -Sin(angle), 0, Cos(angle), 0,
        0, 0, 0, 1
    };

    return Matrix4D(tmp);
}

Matrix4D Matrix4D::rotationZ(float angle) {
    // rotates this matrix around the z axis
    const float tmp[M_LENGTH] = {
        Cos(angle), -Sin(angle), 0, 0,
        Sin(angle), Cos(angle), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };

    return Matrix4D(tmp);
}

Matrix4D Matrix4D::rotation(Vector3f axis, float angle) {
    // assuming that axis is a unit vector
    float x = axis.getX();
    float y = axis.getY();
    float z = axis.getZ();

    const float tmp[M_LENGTH] = {
        (x * x * (1 - Cos(angle)) + Cos(angle)), (x * y * (1 - Cos(angle)) - z * Sin(angle)), (x * z * (1 - Cos(angle)) + y * Sin(angle)), 0,
        (x * y * (1 - Cos(angle)) + z * Sin(angle)), (y * y * (1 - Cos(angle)) + Cos(angle)), (y * z * (1 - Cos(angle)) - x * Sin(angle)), 0,
        (x * z * (1 - Cos(angle)) - y * Sin(angle)), (y * z * (1 - Cos(angle)) + x * Sin(angle)), (z * z * (1 - Cos(angle)) + Cos(angle)), 0,
        0, 0, 0, 1
    };

    return Matrix4D(tmp);
}

Matrix4D Matrix4D::translation(float x, float y, float z) {
    // gets a translation matrix from xyz coordinates
    const float tmp[M_LENGTH] = {
        1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1
    };

    return Matrix4D(tmp);
}

