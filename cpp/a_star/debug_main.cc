#include "a_star.h"
#include <chrono>
#include <iostream>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

std::chrono::_V2::system_clock::time_point t1,t2;

float params[] = {
    15.78,-0.07, //start
    1,1, //out of bounds? go to goal?
    0,0, //target (if not go to goal)
    500000, // timeout
    -10,0,1,5,5,
    -10,1,1,7,10,
    -10,-7,0,5,1
};
int params_size = sizeof(params)/sizeof(params[0]);


int main(){

    t1 = high_resolution_clock::now();
    astar(params, params_size);   
    t2 = high_resolution_clock::now();

    std::cout << duration_cast<microseconds>(t2 - t1).count() << "us (includes initialization)\n";

    t1 = high_resolution_clock::now();
    astar(params, params_size);
    t2 = high_resolution_clock::now();

    std::cout << duration_cast<microseconds>(t2 - t1).count() << "us\n";

}