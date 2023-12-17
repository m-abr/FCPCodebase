#include <cmath>
#include "ball_predictor.h"


float ball_pos_pred[600]; // ball position   (x,y) prediction for 300*0.02s = 6s 
float ball_vel_pred[600]; // ball velocity   (x,y) prediction for 300*0.02s = 6s 
float ball_spd_pred[300]; // ball linear speed (s) prediction for 300*0.02s = 6s 
int pos_pred_len=0;

/**
 * @brief Get intersection with moving ball (intersection point and distance)
 * @param x robot position (x)
 * @param y robot position (y)
 * @param max_robot_sp_per_step maximum speed per step
 * @param ball_pos imported ball positions (possibly modified version of ball_pos_pred)
 * @param ball_pos_len length of ball_pos
 * @param ret_x returned position (x) of intersection point
 * @param ret_y returned position (y) of intersection point
 * @param ret_d returned distance between robot and intersection point
 */
void get_intersection_with_ball(float x, float y, float max_robot_sp_per_step, float ball_pos[], float ball_pos_len,
                                float &ret_x, float &ret_y, float &ret_d){

    float robot_max_displacement = 0.2; // robot has an immediate reach radius of 0.2m
    int j=0;
    
    while(1){
        float vec_x = ball_pos[j++] - x;
        float vec_y = ball_pos[j++] - y;
        float b_dist_sq = vec_x*vec_x + vec_y*vec_y; // squared ball distance
        
        // If robot has reached the ball, or the ball has stopped but the robot is still not there
        if (b_dist_sq <= robot_max_displacement*robot_max_displacement or j>=ball_pos_len){
            float d = sqrtf(b_dist_sq);
            ret_d = d; 
            ret_x = ball_pos[j-2];
            ret_y = ball_pos[j-1];
            break;
        }
        robot_max_displacement += max_robot_sp_per_step;
    }
}


/**
 * @brief Predict ball position/velocity until the ball stops or gets out of bounds (up to 6s)
 * Adequate when the ball is rolling on the ground
 * @param bx ball position (x)
 * @param by ball position (y)
 * @param vx ball velocity (x)
 * @param vy ball velocity (y)
 */
void predict_rolling_ball_pos_vel_spd(double bx, double by, double vx, double vy){

    // acceleration = Rolling Drag Force * mass (constant = 0.026 kg)
    // acceleration = k1 * velocity^2 + k2 * velocity
    const double k1 = -0.01;
    const double k2 = -1;

    const double k1_x = (vx < 0) ? -k1 : k1; // invert k1 if vx is negative, because vx^2 absorbs the sign
    const double k1_y = (vy < 0) ? -k1 : k1; // invert k1 if vy is negative, because vy^2 absorbs the sign
    
    ball_pos_pred[0] = bx; // current ball position
    ball_pos_pred[1] = by;
    ball_vel_pred[0] = vx; // current ball velocity
    ball_vel_pred[1] = vy;
    ball_spd_pred[0] = sqrt(vx*vx+vy*vy);

    int counter = 2;

    while(counter < 600){

        // acceleration
        double acc_x = vx*vx*k1_x + vx*k2;
        double acc_y = vy*vy*k1_y + vy*k2;

        // second equation of motion: displacement = v0*t + 0.5*a*t^2
        double dx = vx*0.02 + acc_x*0.0002; // 0.5*0.02^2 = 0.0002
        double dy = vy*0.02 + acc_y*0.0002; // 0.5*0.02^2 = 0.0002

        // position
        bx += dx;
        by += dy;

        // abort when displacement is low or ball is out of bounds
        if ((fabs(dx) < 0.0005 and fabs(dy) < 0.0005) or fabs(bx) > 15 or fabs(by) > 10){
            break;
        }

        // velocity
        vx += acc_x*0.02;
        vy += acc_y*0.02;

        // store as 32b
        ball_spd_pred[counter/2] = sqrt(vx*vx+vy*vy);
        ball_vel_pred[counter] = vx;
        ball_pos_pred[counter++] = bx;
        ball_vel_pred[counter] = vy;
        ball_pos_pred[counter++] = by;
        
    }

    pos_pred_len = counter;
}