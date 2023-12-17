/*
 * RobovizLogger.h
 *
 * Created on: 2013/06/24
 * Author: Rui Ferreira
 */

#ifndef _ROBOVIZLOGGER_H_
#define _ROBOVIZLOGGER_H_

#define RobovizLoggerInstance RobovizLogger::Instance()

#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <math.h>

class RobovizLogger {
private:
    RobovizLogger();
    RobovizLogger(const RobovizLogger&);
    RobovizLogger& operator=(const RobovizLogger&);
    virtual ~RobovizLogger();
    bool is_initialized = false;

public:
    static RobovizLogger* Instance();

    int init();
    void destroy();
    void swapBuffers(const std::string* setName);

    void drawLine(float x1, float y1, float z1, float x2, float y2, float z2,
            float thickness, float r, float g, float b, const std::string* setName);
    void drawCircle(float x, float y, float radius, float thickness,
            float r, float g, float b, const std::string* setName);
    void drawSphere(float x, float y, float z, float radius,
            float r, float g, float b, const std::string* setName);
    void drawPoint(float x, float y, float z, float size,
            float r, float g, float b, const std::string* setName);
    void drawPolygon(const float* v, int numVerts, float r, float g, float b,
            float a, const std::string* setName);
    void drawAnnotation(const std::string* text, float x, float y, float z, float r,
            float g, float b, const std::string* setName);

private:
    int sockfd;
    struct addrinfo* p;
    struct addrinfo* servinfo;
};

#endif // _ROBOVIZLOGGER_H_
