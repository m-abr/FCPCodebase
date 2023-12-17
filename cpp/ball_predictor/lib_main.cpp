#include "ball_predictor.h"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;
using namespace std;


/**
 * @brief Predict rolling ball position, velocity, linear speed
 * 
 * @param parameters 
 *        ball_x, ball_y, ball_vel_x, ball_vel_y
 * @return ball_pos_pred, ball_vel_pred, ball_spd_pred
 */
py::array_t<float> predict_rolling_ball( py::array_t<float> parameters ){

    // ================================================= 1. Parse data
    
    py::buffer_info parameters_buf = parameters.request();
    float* parameters_ptr = (float*)parameters_buf.ptr;

    float px = parameters_ptr[0];
    float py = parameters_ptr[1];
    float vx = parameters_ptr[2];
    float vy = parameters_ptr[3];

    // ================================================= 2. Compute path

    predict_rolling_ball_pos_vel_spd(px, py, vx, vy);
    
    // ================================================= 3. Prepare data to return
    
    py::array_t<float> retval = py::array_t<float>(pos_pred_len+pos_pred_len+pos_pred_len/2); //allocate
    py::buffer_info buff = retval.request();
    float *ptr = (float *) buff.ptr;

    for(int i=0; i<pos_pred_len; i++){
        ptr[i] = ball_pos_pred[i];
    }
    ptr+=pos_pred_len;
    for(int i=0; i<pos_pred_len; i++){
        ptr[i] = ball_vel_pred[i];
    }
    ptr+=pos_pred_len;
    for(int i=0; i<pos_pred_len/2; i++){
        ptr[i] = ball_spd_pred[i];
    }
    return retval;
}


/**
 * @brief Get point of intersection with moving ball
 * 
 * @param parameters 
 *        robot_x, robot_y, robot_max_speed_per_step
 * @return intersection_x, intersection_y, intersection_distance
 */
py::array_t<float> get_intersection( py::array_t<float> parameters ){

    // ================================================= 1. Parse data
    
    py::buffer_info parameters_buf = parameters.request();
    float* parameters_ptr = (float*)parameters_buf.ptr;
    int params_len = parameters_buf.shape[0];

    float x = parameters_ptr[0];
    float y = parameters_ptr[1];
    float max_sp = parameters_ptr[2];
    float* ball_pos = parameters_ptr + 3;
    float ret_x, ret_y, ret_d;

    // ================================================= 2. Compute path

    get_intersection_with_ball(x, y, max_sp, ball_pos, params_len-3, ret_x, ret_y, ret_d);
    
    // ================================================= 3. Prepare data to return
    
    py::array_t<float> retval = py::array_t<float>(3); //allocate
    py::buffer_info buff = retval.request();
    float *ptr = (float *) buff.ptr;

    ptr[0] = ret_x;
    ptr[1] = ret_y;
    ptr[2] = ret_d;
    return retval;
}


using namespace pybind11::literals; // to add informative argument names as -> "argname"_a

PYBIND11_MODULE(ball_predictor, m) {  // the python module name, m is the interface to create bindings
    m.doc() = "Ball predictor"; // optional module docstring

    // optional arguments names
    m.def("predict_rolling_ball", &predict_rolling_ball, "Predict rolling ball", "parameters"_a); 
    m.def("get_intersection", &get_intersection, "Get point of intersection with moving ball", "parameters"_a); 
}

