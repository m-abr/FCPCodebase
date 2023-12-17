#include "FieldNoise.h"

double FieldNoise::log_prob_r(double d, double r){
    double c1 = 100.0 * ((r-0.005)/d - 1);
    double c2 = 100.0 * ((r+0.005)/d - 1);
    return log_prob_normal_distribution(0, 0.0965, c1, c2);
}

double FieldNoise::log_prob_h(double h, double phi){
    double c1 = phi - 0.005 - h;
    double c2 = phi + 0.005 - h;
    return log_prob_normal_distribution(0, 0.1225, c1, c2);
}

double FieldNoise::log_prob_v(double v, double theta){
    double c1 = theta - 0.005 - v;
    double c2 = theta + 0.005 - v;
    return log_prob_normal_distribution(0, 0.1480, c1, c2);
}



double FieldNoise::log_prob_normal_distribution(double mean, double std, double interval1, double interval2){
    const double std2 = std * sqrt(2);
    double erf1_x = (mean - interval1)/std2; //lowest interval, highest expression
    double erf2_x = (mean - interval2)/std2; //highest interval, lowest expression

    /**
     * Computing erf(erf_x1) - erf(erf_x2) is the same as erf(erf_x1) + erf(-erf_x2).
     * Intuitively, the former seems more natural.
     * So, computationally, the former expression is accurate in the following situations:
     * - erf_x1 * erf_x2 <= 0
     * - |erf_x1| < 3   _or_   |erf_x2| < 3  ('3' is just a ballpark figure, not really relevant)
     * 
     * Known issues: erf(6.5)-erf(6.0) = 1-1 = 0  (actual result: 2.148e-17)
     *               Using 128b functions only mitigates the issue, which is quite common actually.
     * 
     * For these cases, erf_aux(x) is used, although it is not precise for |x|<1.
     * - erf_aux(x) allows the computation of erf(6.5)-erf(6.0) with 7 digits of precision
     * - erf_aux(x) allows the computation of erf(8.5)-erf(8.0) with 3 digits of precision
     * - erf(12.5)-erf(12) = 0 (which is not good for probability comparison)
     * - erf_aux(x) allows the computation of log(erf(6.5)-erf(6.0)) with 8 digits of precision
     * - erf_aux(x) allows the computation of log(erf(8.5)-erf(8.0)) with 5 digits of precision
     * - log(erf(12.5)-erf(12)) = -4647 (real: -147) (not accurate but good for comparisons)
     * 
     * The complete algorithm below that uses erf_aux(x) is almost as fast as the one which uses erf() from math.h (+30% runtime)
     */

    const double log05 = log(0.5);

    //If they have different sign or |erf1_x|<1 || |erf2_x|<1
    if( fabs(erf1_x) < 2 || fabs(erf2_x) < 2 || ((erf1_x > 0) ^ (erf2_x > 0))){
        return log( erf(erf1_x) - erf(erf2_x) ) + log05; // same but faster than log( 0.5 * (erf(erf1_x) - erf(erf2_x)) )
    }

    //Otherwise use erf_aux(x)
    //At this point, erf1_x and erf2_x have the same sign and are both distant from 0

    double erf1 = erf_aux(erf1_x);
    double erf2 = erf_aux(erf2_x);

    //These operations are described in the documentation of erf_aux()
    if(erf1_x > 0){ //both are positive
        return log( 1.0 - exp(erf1-erf2) ) + erf2 + log05;
    }else{ //both are negative
        return log( 1.0 - exp(erf2-erf1) ) + erf1 + log05;
    }

}



double FieldNoise::erf_aux(double a){
    double r, s, t, u;

    t = fabs (a);
    s = a * a;

    r = fma (-5.6271698458222802e-018, t, 4.8565951833159269e-016);
    u = fma (-1.9912968279795284e-014, t, 5.1614612430130285e-013);
    r = fma (r, s, u);
    r = fma (r, t, -9.4934693735334407e-012);  
    r = fma (r, t,  1.3183034417266867e-010);  
    r = fma (r, t, -1.4354030030124722e-009);  
    r = fma (r, t,  1.2558925114367386e-008);  
    r = fma (r, t, -8.9719702096026844e-008);  
    r = fma (r, t,  5.2832013824236141e-007);  
    r = fma (r, t, -2.5730580226095829e-006);  
    r = fma (r, t,  1.0322052949682532e-005);  
    r = fma (r, t, -3.3555264836704290e-005);  
    r = fma (r, t,  8.4667486930270974e-005);  
    r = fma (r, t, -1.4570926486272249e-004);  
    r = fma (r, t,  7.1877160107951816e-005);  
    r = fma (r, t,  4.9486959714660115e-004);  
    r = fma (r, t, -1.6221099717135142e-003);  
    r = fma (r, t,  1.6425707149019371e-004);  
    r = fma (r, t,  1.9148914196620626e-002);  
    r = fma (r, t, -1.0277918343487556e-001);  
    r = fma (r, t, -6.3661844223699315e-001);  
    r = fma (r, t, -1.2837929411398119e-001);  
    r = fma (r, t, -t);

    return r;
}