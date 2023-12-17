/**
 * FILENAME:     FieldNoise
 * DESCRIPTION:  efficient computation of relative probabilities (for the noise model of the RoboCup 3DSSL)
 * AUTHOR:       Miguel Abreu (m.abreu@fe.up.pt)
 * DATE:         2021
 */

#pragma once
#include "math.h"

class FieldNoise{
    public:

        /**
         * Log probability of real distance d, given noisy radius r
         */
        static double log_prob_r(double d, double r);

        /**
         * Log probability of real horizontal angle h, given noisy angle phi
         */
        static double log_prob_h(double h, double phi);

        /**
         * Log probability of real vertical angle v, given noisy angle theta
         */
        static double log_prob_v(double v, double theta);


    private:

        FieldNoise(){}; //Disable construction

        /**
         * Log probability of normally distributed random variable X from interval1 to interval2:
         * Log Pr[interval1 < X < interval2]
         * @param mean mean of random variable
         * @param std standard deviation of random variable
         * @param interval1 minimum value of random variable
         * @param interval2 maximum value of random variable
         */
        static double log_prob_normal_distribution(double mean, double std, double interval1, double interval2);

        /**
         * This function returns ln(1-sgn(a)*erf(a)), but sgn(a)*erf(a) = |erf(a)|, because sgn(a) == sgn(erf(a))
         * So, it returns: ln(1-|erf(a)|), which is <=0
         * 
         * NOTE: condition to guarantee high precision: |a|>= 1
         * 
         * how to compute erf(a) ?
         * 		erf(a) = sgn(a)(1-e^erf_aux(a))
         * 
         * how to compute erf(a)+erf(b) ? 
         * 		erf(a)+erf(b) = sgn(a)(1-e^erf_aux(a)) + sgn(b)(1-e^erf_aux(b))
         * 		assuming a<0 and b>0:
         * 		              = e^erf_aux(a) -1 + 1 - e^erf_aux(b)
         * 	                  = e^erf_aux(a) - e^erf_aux(b)
         * 
         * 		example: erf(-7)+erf(7.1)
         * 			if we computed it directly:    
         * 				erf(-7)+erf(7.1) = -0.9999999(...) + 0.9999999(...) = -1+1 = 0 (due to lack of precision, even if using double)
         * 			if we use the proposed method: 
         * 				e^erf_aux(-7) - e^erf_aux(7.1) = -1.007340e-23 - -4.183826e-23 = 3.176486E-23
         * 
         * how to compute ln(erf(a)+erf(b)) ?
         * 		assuming a<0 and b>0:
         * 		ln(erf(a)+erf(b)) = ln( exp(erf_aux(a)) - exp(erf_aux(b)) )
         *                        = ln( exp(erf_aux(a)-k) - exp(erf_aux(b)-k) ) + k
         * 		where k = min(erf_aux(a), erf_aux(b)) 
         * 
         * how to compute ln(erf(a)-erf(b)) ? (the difference is just the assumption)
         *      assuming a*b >= 0
         * 
         *      ln(erf(a)-erf(b)) = ln( sgn(a)(1-e^erf_aux(a)) - sgn(a)(1-e^erf_aux(b)) ),   note that sgn(a)=sgn(b)
         * 
         *      rule: log( exp(a) - exp(b) ) = log( exp(a-k) - exp(b-k) ) + k
         * 
         *      if(a>0) 
         *          ln(erf(a)-erf(b)) = ln( 1 - e^erf_aux(a) - 1 + e^erf_aux(b))
         *                            = ln( exp(erf_aux(b)) - exp(erf_aux(a)) )
         *                            = ln( exp(erf_aux(b)-erf_aux(b)) - exp(erf_aux(a)-erf_aux(b)) ) + erf_aux(b)
         *                            = ln( 1 - exp(erf_aux(a)-erf_aux(b)) ) + erf_aux(b)
         *      if(a<0)
         *          ln(erf(a)-erf(b)) = ln( -1 + e^erf_aux(a) + 1 - e^erf_aux(b))
         *                            = ln( exp(erf_aux(a)) - exp(erf_aux(b)) )
         *                            = ln( exp(erf_aux(a)-erf_aux(a)) - exp(erf_aux(b)-erf_aux(a)) ) + erf_aux(a)
         *                            = ln( 1 - exp(erf_aux(b)-erf_aux(a)) ) + erf_aux(a)
         *   
         */
        static double erf_aux(double a);
};