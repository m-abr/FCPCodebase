#include <iostream>
#include "RobovizLogger.h"
#include "robovizdraw.h"

#define ROBOVIZ_HOST "localhost"
#define ROBOVIZ_PORT "32769"


RobovizLogger* RobovizLogger::Instance() {
    static RobovizLogger instance;
    return &instance;
}

RobovizLogger::RobovizLogger() {}

RobovizLogger::~RobovizLogger() {
    destroy();
}

int RobovizLogger::init() {
    if (is_initialized) return 0;
    struct addrinfo hints;
    int rv;
    int numbytes;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;

    if ((rv = getaddrinfo(ROBOVIZ_HOST, ROBOVIZ_PORT, &hints, &servinfo)) != 0) {
        //fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }

    // loop through all the results and make a socket
    for (p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1) {
            perror("socket");
            continue;
        }
        break;
    }

    if (p == NULL) {
        return 2;
    }

    is_initialized = true;
    return 0;
}

void RobovizLogger::destroy() {
    freeaddrinfo(servinfo);
    servinfo=NULL;
    close(sockfd);
}

void RobovizLogger::swapBuffers(const string* setName) {
    int bufSize = -1;
    unsigned char* buf = newBufferSwap(setName, &bufSize);
    sendto(sockfd, buf, bufSize, 0, p->ai_addr, p->ai_addrlen);
    delete[] buf;
}

void RobovizLogger::drawLine(float x1, float y1, float z1, float x2, float y2, float z2,
        float thickness, float r, float g, float b, const string* setName) {
    float pa[3] = {x1, y1, z1};
    float pb[3] = {x2, y2, z2};
    float color[3] = {r, g, b};

    int bufSize = -1;
    unsigned char* buf = newLine(pa, pb, thickness, color, setName, &bufSize);
    sendto(sockfd, buf, bufSize, 0, p->ai_addr, p->ai_addrlen);
    delete[] buf;
}

void RobovizLogger::drawCircle(float x, float y, float radius, float thickness,
        float r, float g, float b, const string* setName) {
    float center[2] = {x, y};
    float color[3] = {r, g, b};

    int bufSize = -1;
    unsigned char* buf = newCircle(center, radius, thickness, color, setName, &bufSize);
    sendto(sockfd, buf, bufSize, 0, p->ai_addr, p->ai_addrlen);
    delete[] buf;
}

void RobovizLogger::drawSphere(float x, float y, float z, float radius,
        float r, float g, float b, const string* setName) {
    float center[3] = {x, y, z};
    float color[3] = {r, g, b};

    int bufSize = -1;
    unsigned char* buf = newSphere(center, radius, color, setName, &bufSize);
    sendto(sockfd, buf, bufSize, 0, p->ai_addr, p->ai_addrlen);
    delete[] buf;
}

void RobovizLogger::drawPoint(float x, float y, float z, float size,
        float r, float g, float b, const string* setName) {
    float center[3] = {x, y, z};
    float color[3] = {r, g, b};

    int bufSize = -1;
    unsigned char* buf = newPoint(center, size, color, setName, &bufSize);
    sendto(sockfd, buf, bufSize, 0, p->ai_addr, p->ai_addrlen);
    delete[] buf;
}

void RobovizLogger::drawPolygon(const float* v, int numVerts, float r, float g, float b,
        float a, const string* setName) {
    float color[4] = {r, g, b, a};

    int bufSize = -1;
    unsigned char* buf = newPolygon(v, numVerts, color, setName, &bufSize);
    sendto(sockfd, buf, bufSize, 0, p->ai_addr, p->ai_addrlen);
    delete[] buf;
}

void RobovizLogger::drawAnnotation(const string* text, float x, float y, float z, float r,
        float g, float b, const string* setName) {
    float color[3] = {r, g, b};
    float pos[3] = {x, y, z};

    int bufSize = -1;
    unsigned char* buf = newAnnotation(text, pos, color, setName, &bufSize);
    sendto(sockfd, buf, bufSize, 0, p->ai_addr, p->ai_addrlen);
    delete[] buf;
}

