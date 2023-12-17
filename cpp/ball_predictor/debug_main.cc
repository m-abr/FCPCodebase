#include "ball_predictor.h"
#include <chrono>
#include <iostream>
#include <iomanip>

using std::cout;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

std::chrono::_V2::system_clock::time_point t1,t2;

int main(){

    // ================================================= 1. Generate data

    float px = 3;
    float py = 4;
    float vx = -5;
    float vy = -1;

    // ================================================= 2. Compute prediction

    t1 = high_resolution_clock::now();
    predict_rolling_ball_pos_vel_spd(px, py, vx, vy);
    t2 = high_resolution_clock::now();

    cout << std::fixed << std::setprecision(8);

    for(int i=0; i<pos_pred_len; i+=2){
        cout << i/2 << " pos:" << ball_pos_pred[i] << "," << ball_pos_pred[i+1] <<
                       " vel:" << ball_vel_pred[i] << "," << ball_vel_pred[i+1] <<
                       " spd:" << ball_spd_pred[i/2] << "\n";
    }    

    cout << "\n\n" << duration_cast<microseconds>(t2 - t1).count() << "us for prediction\n";

    // ================================================= 3. Generate data

    float robot_x = -1;
    float robot_y = 1;
    float max_speed_per_step = 0.7*0.02;
    float ret_x, ret_y, ret_d;

    // ================================================= 4. Compute intersection

    t1 = high_resolution_clock::now();
    get_intersection_with_ball(robot_x, robot_y, max_speed_per_step, ball_pos_pred, pos_pred_len, ret_x, ret_y, ret_d);
    t2 = high_resolution_clock::now();

    cout << duration_cast<microseconds>(t2 - t1).count() << "us for intersection\n\n";
    cout << "Intersection: " << ret_x << "," << ret_y << " dist: " << ret_d << "\n\n";

}
