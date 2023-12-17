#include "a_star.h"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;
using namespace std;


py::array_t<float> compute( py::array_t<float> parameters ){

    // ================================================= 1. Parse data
    
    py::buffer_info parameters_buf = parameters.request();
    int params_len = parameters_buf.shape[0];

    // ================================================= 2. Compute path

    astar( (float*)parameters_buf.ptr, params_len );
    
    // ================================================= 3. Prepare data to return
    
    py::array_t<float> retval = py::array_t<float>(final_path_size); //allocate
    py::buffer_info buff = retval.request();
    float *ptr = (float *) buff.ptr;

    for(int i=0; i<final_path_size; i++){
        ptr[i] = final_path[i];
    }


    return retval;
}



using namespace pybind11::literals; // to add informative argument names as -> "argname"_a

PYBIND11_MODULE(a_star, m) {  // the python module name, m is the interface to create bindings
    m.doc() = "Custom A-star implementation"; // optional module docstring

    // optional arguments names
    m.def("compute", &compute, "Compute the best path", "parameters"_a); 
}